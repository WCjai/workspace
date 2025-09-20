#include "chip.h"
#include "board.h"
#include <stdbool.h>
#include <string.h>

/* ===== WS2812B driver ===== */
#include "ws2812b.h"    /* needs RGB_t, WS2812B, WS2812B_write(...) */

/* ============================================================================
   RX <-> APP protocol replica + WS2812B accessory
   - Upload-Map (connectors)
   - RIT (70 ms) does BOTH:
       • LED keep-alive streaming
       • Slave polling: strict 1..N when no LED jobs active
         (if LED activity interrupts a poll cycle, resume from 1..N)
       • WS2812B flush if any strip changed
   - UART1 RX parses slave replies: 27 27 03 0A <addr> <state> 16
     (state 0x01 idle/not triggered, 0x03 triggered)
   - Heartbeat encodes 0x07 (alive+limit), 0x05 (alive), 0x00 (dead)
   - Immediate limit/heartbeat updates for the *currently-streamed* slave
     (even while general polling is paused)
   - New App command (LEN=0x0C) drives WS2812B: BIN=1(B1=P3.25)/2(B2=P3.26), LED#
   ============================================================================ */

/* -------------------- UART selection -------------------- */
#define UART_APP        LPC_UART0
#define IRQ_APP         UART0_IRQn

#define UART_SLAVE      LPC_UART1       /* Slave bus on P2.0/P2.1 */
#define IRQ_SLAVE       UART1_IRQn

/* -------------------- RX identity -------------------- */
#define RX_ID           0x01

/* -------------------- Slave frames (per captures) -------------------- */
#define SLAVE_LEN_ON      0x97
#define SLAVE_LEN_OFF     0x97
#define SLAVE_LEN_POLL    0x97

/* -------------------- Protocol bytes -------------------- */
enum { SOF = 0x27, END_BYTE = 0x16 };
enum { GRP_APP_TO_RX = 0x85, GRP_RX_TO_APP = 0x00 };
enum { SC_POLL = 0x00, SC_LED_CTRL = 0x02, SC_UPLOAD_MAP = 0x04, SC_LED_RESET = 0x3A, SC_RELAY_SET = 0x06 };
enum { SC_STATUS = 0x0A };

/* -------------------- Limits & buffers -------------------- */
#define MAX_CFG         31
#define RX_LEN_MAX      (4 + 2*MAX_CFG)        /* Upload-Map worst case */
#define TX_FRAME_MAX    (MAX_CFG + 10)         /* heartbeat worst case */

/* -------------------- Config (from Upload-Map) -------------------- */
static uint8_t cfg_conn[MAX_CFG];   /* enabled connector IDs in order */
static uint8_t cfg_count = 0;       /* N connectors => polling goes 1..N by array order */

/* -------------------- Presence & limit-trigger (round snapshots) --------- */
#define CONN_BIT(c) (1u << ((c) - 1))          /* connectors 1..31 -> bits 0..30 */

static volatile uint32_t alive_round_mask     = 0;
static volatile uint32_t triggered_round_mask = 0;

static volatile uint32_t g_alive_mask         = 0;
static volatile uint32_t g_triggered_mask     = 0;

static inline bool is_alive(uint8_t conn) {
    return (conn >= 1 && conn <= 31) && ((g_alive_mask & CONN_BIT(conn)) != 0);
}
static inline bool is_triggered(uint8_t conn) {
    return (conn >= 1 && conn <= 31) && ((g_triggered_mask & CONN_BIT(conn)) != 0);
}

/* -------------------- UART1 TX queue (ISRs -> main) -------------------- */
typedef struct { uint8_t data[9]; uint8_t len; } U1Frame;
#define U1_TXQ_CAP  64

static volatile uint8_t u1_head = 0, u1_tail = 0; /* ISR producer, main consumer */
static U1Frame u1_q[U1_TXQ_CAP];

static inline bool u1q_push_isr(const uint8_t *d, uint8_t n) {
    uint8_t next = (uint8_t)((u1_head + 1) % U1_TXQ_CAP);
    if (next == u1_tail) return false;            /* queue full: drop */
    if (n > sizeof(u1_q[0].data)) n = sizeof(u1_q[0].data);
    memcpy(u1_q[u1_head].data, d, n);
    u1_q[u1_head].len = n;
    u1_head = next;
    return true;
}
static inline bool u1q_pop_main(U1Frame *out) {
    if (u1_tail == u1_head) return false;
    *out = u1_q[u1_tail];
    u1_tail = (uint8_t)((u1_tail + 1) % U1_TXQ_CAP);
    return true;
}

/* Enqueue slave frames (RIT uses these) */
static inline void slave_enqueue_led_on(uint8_t con, uint8_t led) {
    /* 27, 97, 05, 85, con, 02, 01, led, 16 */
    uint8_t f[9] = { 0x27, SLAVE_LEN_ON, 0x05, 0x85, con, 0x02, 0x01, led, 0x16 };
    (void)u1q_push_isr(f, sizeof f);
}
static inline void slave_enqueue_led_off_broadcast(void) {
    /* 27, 97, 04, 85, FF, 03, 00, 16 (0x03 per your trace) */
    uint8_t f[8] = { 0x27, SLAVE_LEN_OFF, 0x04, 0x85, 0xFF, 0x03, 0x00, 0x16 };
    (void)u1q_push_isr(f, sizeof f);
}
static inline void slave_enqueue_poll(uint8_t con) {
    /* 27, 97, 05, 85, con, 00, 00, 00, 16 */
    uint8_t f[9] = { 0x27, SLAVE_LEN_POLL, 0x05, 0x85, con, 0x00, 0x00, 0x00, 0x16 };
    (void)u1q_push_isr(f, sizeof f);
}

/* -------------------- LED keep-alive jobs (RIT @ 70 ms) -------------------- */
#define MAX_LED_JOBS    32

typedef struct {
    uint8_t  con;
    uint8_t  led;
    uint16_t next_allowed_tick;  /* rate-limit per job */
    volatile uint8_t active;     /* write last to "commit" */
} LedJob;

static volatile LedJob g_jobs[MAX_LED_JOBS];
static volatile uint8_t g_off_broadcast_pending = 0;

/* RIT scheduling */
#define RIT_TICK_MS                 70
#define LED_JOB_MIN_PERIOD_TICKS    2   /* send same job at most every 140 ms */

static volatile uint16_t g_tick = 0;    /* increments each RIT tick (wrap ok) */
static uint8_t jobs_rr = 0;             /* round-robin index over job table */

/* Streaming-active flag (read by UART1 IRQ for immediate updates) */
static volatile bool g_led_streaming_active = false;

/* -------------------- App (UART0) heartbeat -------------------- */
static volatile uint8_t g_tx_len = 0;
static uint8_t          g_tx_buf[TX_FRAME_MAX];

static uint8_t build_status_frame(uint8_t *dst, uint8_t cap) {
    const uint8_t N = cfg_count;
    const uint8_t LEN = (uint8_t)(N + 7);
    const uint8_t TOTAL = (uint8_t)(LEN + 3);
    if (TOTAL > cap) return 0;

    uint8_t *p = dst;
    *p++ = SOF; *p++ = LEN; *p++ = GRP_RX_TO_APP; *p++ = RX_ID; *p++ = SC_STATUS;
    *p++ = 0x00; *p++ = N;
    for (uint8_t i = 0; i < N; ++i) {
        uint8_t c = cfg_conn[i];
        uint8_t Si = 0x00;
        if (is_alive(c)) Si = is_triggered(c) ? 0x07 : 0x05; /* 07=alive+limit, 05=alive */
        *p++ = Si;
    }
    *p++ = 0x00; *p++ = 0x00; *p++ = END_BYTE;
    return TOTAL;
}
static inline void send_status(void) {
    uint8_t n = build_status_frame(g_tx_buf, sizeof(g_tx_buf));
    if (n) g_tx_len = n;
}

/* -------------------- WS2812B: two strips on B1=P3.25, B2=P3.26 ----------- */
#define WS_LED_COUNT  96

static RGB_t ws_buf_b1[WS_LED_COUNT];
static RGB_t ws_buf_b2[WS_LED_COUNT];
static WS2812B ws_b1 = { .leds = ws_buf_b1, .num_leds = WS_LED_COUNT };
static WS2812B ws_b2 = { .leds = ws_buf_b2, .num_leds = WS_LED_COUNT };
static volatile bool ws_dirty_b1 = false, ws_dirty_b2 = false;

static inline void ws_clear(RGB_t *buf, uint16_t n) { memset(buf, 0, n * sizeof(RGB_t)); }
static inline void ws_set_red(RGB_t *buf, uint16_t n, uint16_t led1) {
    if (led1 == 0 || led1 > n) return;
    uint16_t idx = (uint16_t)(led1 - 1);
    buf[idx].r = 255; buf[idx].g = 0; buf[idx].b = 0;   /* swap if your driver expects GRB */
}
static inline void ws_flush_if_dirty(void) {
    if (ws_dirty_b1) { WS2812B_write(&ws_b1); ws_dirty_b1 = false; }
    if (ws_dirty_b2) { WS2812B_write(&ws_b2); ws_dirty_b2 = false; }
}

/* Try to parse WS accessory LED command inside SC_LED_CTRL payload:
   Expected payload (pal>=9): 00 00 00 02 <BIN> <LED#> 00 00 00
   BIN: 1=B1(P3.25), 2=B2(P3.26), LED# 1..WS_LED_COUNT
*/
static bool try_handle_ws_accessory(const uint8_t *pay, uint8_t pal) {
    if (pal < 9) return false;
    if (pay[0] != 0x00 || pay[1] != 0x00 || pay[2] != 0x00) return false;
    if (pay[3] != 0x02) return false;
    uint8_t bin = pay[4], led = pay[5];
    if (bin == 1) { ws_set_red(ws_buf_b1, WS_LED_COUNT, led); ws_dirty_b1 = true; return true; }
    if (bin == 2) { ws_set_red(ws_buf_b2, WS_LED_COUNT, led); ws_dirty_b2 = true; return true; }
    return false;
}

/* -------------------- Command handlers (UART0) -------------------- */
static void handle_upload_map(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 1) return;
    const uint8_t N = pay[0];
    if (paylen < (uint8_t)(1 + 2U * N)) return;

    cfg_count = 0;
    for (uint8_t i = 0; i < N && cfg_count < MAX_CFG; ++i) {
        uint8_t c = pay[1 + 2*i + 0];
        uint8_t s = pay[1 + 2*i + 1];
        if (s == 0x01) cfg_conn[cfg_count++] = c;
    }
}

static void handle_led_ctrl(const uint8_t *pay, uint8_t paylen) {
    /* First handle WS accessory command if present */
    if (try_handle_ws_accessory(pay, paylen)) return;

    /* Else: legacy 3-byte LED control for slave boards */
    if (paylen < 3) return;
    uint8_t state = pay[0], con = pay[1], led = pay[2];
    if (state) {
        for (uint8_t i = 0; i < MAX_LED_JOBS; ++i) {
            if (!g_jobs[i].active) {
                g_jobs[i].con = con;
                g_jobs[i].led = led;
                g_jobs[i].next_allowed_tick = g_tick;  /* send immediately */
                __DSB(); __ISB();
                g_jobs[i].active = 1;
                break;
            }
        }
    } else {
        for (uint8_t i = 0; i < MAX_LED_JOBS; ++i) {
            if (g_jobs[i].active && g_jobs[i].con == con && g_jobs[i].led == led) {
                g_jobs[i].active = 0;
                break;
            }
        }
        g_off_broadcast_pending = 1;
        /* Note: WS strips are cleared by LED_RESET, not by per-conn OFF. */
    }
}

static void handle_led_reset(const uint8_t *pay, uint8_t paylen) {
    (void)pay; (void)paylen;
    for (uint8_t i = 0; i < MAX_LED_JOBS; ++i) g_jobs[i].active = 0;
    g_off_broadcast_pending = 1;

    /* Also clear WS2812B accessories */
    ws_clear(ws_buf_b1, WS_LED_COUNT); ws_dirty_b1 = true;
    ws_clear(ws_buf_b2, WS_LED_COUNT); ws_dirty_b2 = true;
}

static void handle_relay_set(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 2) return;
    Board_Relay_Set(pay[0], (pay[1] == 0x01));
}

/* -------------------- UART0 RX parser (App->RX) -------------------- */
typedef enum { RXF_WAIT_SOF=0, RXF_WAIT_LEN, RXF_COLLECT_BODY, RXF_WAIT_END } rx_fsm_t;
static volatile rx_fsm_t rx_state = RXF_WAIT_SOF;
static volatile uint8_t  rx_len   = 0;
static uint8_t           rx_buf[RX_LEN_MAX];
static uint8_t           rx_idx   = 0;

static void dispatch_body(const uint8_t *p, uint8_t len) {
    if (len < 3) return;
    uint8_t group = p[0], id = p[1], sc = p[2];
    const uint8_t *pay = (len > 3) ? &p[3] : NULL;
    uint8_t pal = (len > 3) ? (uint8_t)(len - 3) : 0;

    if (group != GRP_APP_TO_RX || id != RX_ID) return;

    switch (sc) {
        case SC_POLL:        /* payload 00 */                 break;
        case SC_UPLOAD_MAP:  handle_upload_map(pay, pal);     break;
        case SC_LED_CTRL:    handle_led_ctrl(pay, pal);       break;
        case SC_LED_RESET:   handle_led_reset(pay, pal);      break;
        case SC_RELAY_SET:   handle_relay_set(pay, pal);      break;
        default: break;
    }
    /* Always reply with status after any valid command */
    send_status();
}

void UART0_IRQHandler(void) {
    while ((Chip_UART_ReadLineStatus(UART_APP) & UART_LSR_RDR) != 0) {
        uint8_t b = Chip_UART_ReadByte(UART_APP);
        switch (rx_state) {
        case RXF_WAIT_SOF:      if (b == SOF) rx_state = RXF_WAIT_LEN; break;
        case RXF_WAIT_LEN:
            rx_len = b; rx_idx = 0;
            if (!rx_len || rx_len > RX_LEN_MAX) { rx_state = RXF_WAIT_SOF; break; }
            rx_state = RXF_COLLECT_BODY; break;
        case RXF_COLLECT_BODY:
            if (rx_idx < RX_LEN_MAX) {
                rx_buf[rx_idx++] = b;
                if (rx_idx == rx_len) rx_state = RXF_WAIT_END;
            } else rx_state = RXF_WAIT_SOF;
            break;
        case RXF_WAIT_END:
            if (b == END_BYTE) dispatch_body(rx_buf, rx_len);
            rx_state = RXF_WAIT_SOF; break;
        default: rx_state = RXF_WAIT_SOF; break;
        }
    }
}

/* -------------------- UART1 RX parser (Slave->RX) -------------------- */
/* Accepts: 27 27 03 0A <addr> <state> 16
   state: 0x01=idle/not triggered, 0x03=triggered (both imply "alive") */
typedef enum { U1_WAIT_SOF=0, U1_GOT_SOF, U1_WAIT_LEN, U1_COLLECT, U1_WAIT_END } u1_fsm_t;
static volatile u1_fsm_t u1_state = U1_WAIT_SOF;
static volatile uint8_t  u1_len   = 0, u1_idx = 0;
static uint8_t           u1_pay[8];

void UART1_IRQHandler(void) {
    while (Chip_UART_ReadLineStatus(UART_SLAVE) & UART_LSR_RDR) {
        uint8_t b = Chip_UART_ReadByte(UART_SLAVE);
        switch (u1_state) {
        case U1_WAIT_SOF:  if (b == 0x27) u1_state = U1_GOT_SOF; break;
        case U1_GOT_SOF:   if (b == 0x27) u1_state = U1_WAIT_LEN;
                           else { u1_len = b; u1_idx = 0; if (!u1_len || u1_len > sizeof(u1_pay)) { u1_state = U1_WAIT_SOF; break; } u1_state = U1_COLLECT; }
                           break;
        case U1_WAIT_LEN:  u1_len = b; u1_idx = 0;
                           if (!u1_len || u1_len > sizeof(u1_pay)) { u1_state = U1_WAIT_SOF; break; }
                           u1_state = U1_COLLECT; break;
        case U1_COLLECT:   u1_pay[u1_idx++] = b;
                           if (u1_idx == u1_len) u1_state = U1_WAIT_END; break;
        case U1_WAIT_END:
            if (b == 0x16 && u1_len == 3 && u1_pay[0] == 0x0A) {
                uint8_t addr = u1_pay[1], st = u1_pay[2];
                if (addr >= 1 && addr <= 31 && st != 0x00) {
                    uint32_t bit = CONN_BIT(addr);
                    /* Per-round masks (for next commit when polling runs) */
                    alive_round_mask |= bit;
                    if (st == 0x03) triggered_round_mask |= bit; else triggered_round_mask &= ~bit;

                    /* === Immediate update during LED streaming === */
                    if (g_led_streaming_active) {
                        g_alive_mask |= bit;                         /* replying => alive */
                        if (st == 0x03) g_triggered_mask |= bit;
                        else            g_triggered_mask &= ~bit;    /* clear if not triggered */
                    }
                }
            }
            u1_state = U1_WAIT_SOF; break;
        default: u1_state = U1_WAIT_SOF; break;
        }
    }
}

/* -------------------- RIT: 70 ms (LED + polling + WS flush) ------------- */
static volatile uint8_t  poll_rr_idx    = 0;   /* 0..(cfg_count-1) */
static volatile bool     led_active_prev = false;

static inline void commit_and_clear_poll_round(void) {
    g_alive_mask         = alive_round_mask;
    g_triggered_mask     = triggered_round_mask & g_alive_mask; /* trigger only if alive */
    alive_round_mask     = 0;
    triggered_round_mask = 0;
}

void RIT_IRQHandler(void) {
    Chip_RIT_ClearInt(LPC_RITIMER);
    g_tick++;

    /* One-time broadcast OFF if requested */
    if (g_off_broadcast_pending) {
        slave_enqueue_led_off_broadcast();
        g_off_broadcast_pending = 0;
    }

    /* LED streaming takes precedence; pause polling while active */
    bool led_active_now = false;
    for (uint8_t i = 0; i < MAX_LED_JOBS; ++i) { if (g_jobs[i].active) { led_active_now = true; break; } }
    g_led_streaming_active = led_active_now;

    if (led_active_now) {
        led_active_prev = true;

        /* round-robin ONE LED job per tick, with per-job rate limit */
        for (uint8_t k = 0; k < MAX_LED_JOBS; ++k) {
            uint8_t i = (uint8_t)((jobs_rr + k) % MAX_LED_JOBS);
            if (!g_jobs[i].active) continue;
            if ((int16_t)(g_tick - g_jobs[i].next_allowed_tick) >= 0) {
                slave_enqueue_led_on(g_jobs[i].con, g_jobs[i].led);
                g_jobs[i].next_allowed_tick = (uint16_t)(g_tick + LED_JOB_MIN_PERIOD_TICKS);
                jobs_rr = (uint8_t)((i + 1) % MAX_LED_JOBS);
                break;  /* only one job per tick */
            }
        }
        /* Also flush WS strips */
        ws_flush_if_dirty();
        return;    /* no general polling while LEDs stream */
    }

    /* LED mode ended: resume poll from start (1..N); keep snapshot until next commit */
    if (led_active_prev) {
        led_active_prev = false;
        poll_rr_idx = 0;     /* resume at connector #1 */
    }

    /* Polling mode */
    if (cfg_count == 0) { ws_flush_if_dirty(); return; }

    if (poll_rr_idx == 0) {
        commit_and_clear_poll_round();   /* commit previous round, begin new */
    }

    /* Strict 1..N by cfg_conn[] */
    uint8_t con = cfg_conn[poll_rr_idx];
    slave_enqueue_poll(con);

    poll_rr_idx++;
    if (poll_rr_idx >= cfg_count) poll_rr_idx = 0;

    /* Flush WS strips */
    ws_flush_if_dirty();
}

/* -------------------- Main -------------------- */
int main(void) {
    SystemCoreClockUpdate();
    Board_Init();

    /* --- WS2812B pins: P3.25 (B1), P3.26 (B2) as GPIO outputs --- */
    Chip_IOCON_Init(LPC_IOCON);
    Chip_GPIO_Init(LPC_GPIO);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 3, 25, IOCON_FUNC0 | IOCON_MODE_INACT);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 3, 26, IOCON_FUNC0 | IOCON_MODE_INACT);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 3, 25);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 3, 26);
    ws_clear(ws_buf_b1, WS_LED_COUNT); ws_clear(ws_buf_b2, WS_LED_COUNT);
    ws_dirty_b1 = ws_dirty_b2 = true;   /* push initial clear */

    /* UART0 (APP link): 19,200 8N1 */
    Chip_UART_Init(UART_APP);
    Chip_UART_SetBaud(UART_APP, 19200);
    Chip_UART_ConfigData(UART_APP, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_APP, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_APP);

    /* UART1 (SLAVE link): 9600 8N1 (per your setup) */
    Chip_UART_Init(UART_SLAVE);
    Chip_UART_SetBaud(UART_SLAVE, 9600);
    Chip_UART_ConfigData(UART_SLAVE, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_SLAVE, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_SLAVE);

    /* Enable UART IRQs */
    Chip_UART_IntEnable(UART_APP,   UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_APP,   3);
    NVIC_EnableIRQ(IRQ_APP);

    Chip_UART_IntEnable(UART_SLAVE, UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_SLAVE, 2);
    NVIC_EnableIRQ(IRQ_SLAVE);

    /* RIT: 70 ms period does LED publishing, polling, WS flush */
    Chip_RIT_Init(LPC_RITIMER);
    Chip_RIT_SetTimerInterval(LPC_RITIMER, RIT_TICK_MS);
    NVIC_ClearPendingIRQ(RITIMER_IRQn);
    NVIC_SetPriority(RITIMER_IRQn, 1);
    NVIC_EnableIRQ(RITIMER_IRQn);

    for (;;) {
        /* Flush heartbeat (UART0) */
        if (g_tx_len) {
            uint8_t n = g_tx_len;
            Chip_UART_SendBlocking(UART_APP, g_tx_buf, n);
            g_tx_len = 0;
        }
        /* Flush at most one UART1 frame per loop to avoid long blocking bursts */
        U1Frame fr;
        if (u1q_pop_main(&fr)) {
            Chip_UART_SendBlocking(UART_SLAVE, fr.data, fr.len);
        }
        __WFI();
    }
}
