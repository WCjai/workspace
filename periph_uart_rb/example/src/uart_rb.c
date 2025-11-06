#include "chip.h"
#include "board.h"
#include <stdbool.h>
#include <string.h>
#include "ws2812b.h"  /* RGB_t, WS2812B, WS2812B_write(...) */

/* ============================================================================
   RX <-> APP protocol replica + WS2812B accessory
   ---------------------------------------------------------------------------
   - Upload-Map (connectors)
   - RIT (70 ms) does BOTH:
       • LED keep-alive streaming to UART1
       • Slave polling: strict 1..N when no LED jobs active
         (if streaming interrupts a cycle, resume polling at 1 on exit)
       • WS2812B flush if any strip changed (B1=P3.25, B2=P3.26)
   - UART1 RX parses slave replies: 27 27 03 0A <addr> <state> 16
       • state 0x01 => alive/not triggered
       • state 0x03 => alive + triggered
       • both imply “alive”
   - Heartbeat encodes:
       • 0x07 (alive + limit), 0x05 (alive), 0x00 (dead)
   - Immediate limit/heartbeat updates for the *active streaming* slave
     (even while general polling is paused)
   - WS accessory command (SC_LED_CTRL payload, LEN=0x0C pattern):
       00 00 00 02 <BIN> <LED#> 00 00 00
       BIN=1 => B1(P3.25), BIN=2 => B2(P3.26). Lights RED.
   ============================================================================ */

/* ------------------------------ sw 2 pin interrupt ------------------------------ */
#define GPIO_BUTTON_PORT     2
#define GPIO_BUTTON_PIN      3
#define GPIO_IRQ_HANDLER     EINT3_IRQHandler
#define GPIO_NVIC_NAME       EINT3_IRQn

/* ------------------------------ UART selection ------------------------------ */
#define UART_APP   LPC_UART0
#define IRQ_APP    UART0_IRQn

#define UART_SLAVE LPC_UART1   /* Slave bus on P2.0/P2.1 */
#define IRQ_SLAVE  UART1_IRQn

/* ------------------------------ RX identity -------------------------------- */
#define RX_ID 0x01

/* --------------------------- Slave frame “lengths” -------------------------- */
/* These are the “mystery” second byte values observed in your traces. */
#define SLAVE_LEN_ON    0x97
#define SLAVE_LEN_OFF   0x97
#define SLAVE_LEN_POLL  0x97

/* ----------------------------- Protocol bytes ------------------------------- */
enum {
    SOF = 0x27,
    END_BYTE = 0x16
};
enum {
    GRP_APP_TO_RX = 0x85,
    GRP_RX_TO_APP = 0x00
};
enum {
    SC_POLL       = 0x00,
    SC_LED_CTRL   = 0x02,
    SC_UPLOAD_MAP = 0x04,
    SC_LED_RESET  = 0x3A,
    SC_RELAY_SET  = 0x06
};
enum {
    SC_STATUS = 0x0A
};

static volatile uint8_t g_status_ext = 0x00;   /* 0x00 normally, 0x02 when button pressed */

/* App command to clear button flag */
enum { SC_BTNFLAG_RESET = 0x09 };

/* ------------------------------- Limits/sizes ------------------------------- */
#define MAX_CFG      31
#define RX_LEN_MAX   (4 + 2 * MAX_CFG) /* Upload-Map worst case */
#define TX_FRAME_MAX (MAX_CFG + 10)    /* Heartbeat worst case */

/* ------------------------- Upload-Map (connector list) ---------------------- */
static uint8_t cfg_conn[MAX_CFG]; /* enabled connectors in order (1..31 values) */
static uint8_t cfg_count = 0;     /* number of enabled connectors */

/* ---------------------- Presence & limit-trigger snapshots ------------------ */
#define CONN_BIT(c) (1u << ((c) - 1)) /* connectors 1..31 -> bits 0..30 */

/* Per-round accumulators (set in UART1 IRQ on replies) */
static volatile uint32_t round_alive_mask     = 0;
static volatile uint32_t round_triggered_mask = 0;

/* Committed snapshots (used by heartbeat / app view) */
static volatile uint32_t g_alive_mask         = 0;
static volatile uint32_t g_triggered_mask     = 0;

static inline bool is_alive(uint8_t conn) {
    return (conn >= 1 && conn <= 31) && (g_alive_mask & CONN_BIT(conn));
}
static inline bool is_triggered(uint8_t conn) {
    return (conn >= 1 && conn <= 31) && (g_triggered_mask & CONN_BIT(conn));
}

/* ----------------------- UART1 TX ring (ISR -> main) ------------------------ */
typedef struct {
    uint8_t data[9];
    uint8_t len;
} U1Frame;

#define U1_TXQ_CAP 64
static volatile uint8_t u1_head = 0, u1_tail = 0;
static U1Frame u1_q[U1_TXQ_CAP];

static inline bool u1q_push_isr(const uint8_t *d, uint8_t n) {
    const uint8_t next = (uint8_t)((u1_head + 1) % U1_TXQ_CAP);
    if (next == u1_tail) return false;                 /* queue full */
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

/* ------------------------ Helpers: enqueue slave frames --------------------- */
static inline void slave_enqueue_led_on(uint8_t con, uint8_t led) {
    /* 27, 97, 05, 85, con, 02, 01, led, 16 */
    uint8_t f[9] = { SOF, SLAVE_LEN_ON, 0x05, GRP_APP_TO_RX, con, 0x02, 0x01, led, END_BYTE };
    (void)u1q_push_isr(f, sizeof f);
}
static inline void slave_enqueue_led_off_broadcast(void) {
    /* 27, 97, 04, 85, FF, 03, 00, 16   (0x03 seen in your trace) */
    uint8_t f[8] = { SOF, SLAVE_LEN_OFF, 0x04, GRP_APP_TO_RX, 0xFF, 0x03, 0x00, END_BYTE };
    (void)u1q_push_isr(f, sizeof f);
}
static inline void slave_enqueue_poll(uint8_t con) {
    /* 27, 97, 05, 85, con, 00, 00, 00, 16 */
    uint8_t f[9] = { SOF, SLAVE_LEN_POLL, 0x05, GRP_APP_TO_RX, con, 0x00, 0x00, 0x00, END_BYTE };
    (void)u1q_push_isr(f, sizeof f);
}

/* ----------------------------- LED job scheduler ---------------------------- */
/* RIT tick handles:
   - LED streaming (if any job active)
   - or polling (strict 1..N) when no job active
*/
#define MAX_LED_JOBS             32
#define RIT_TICK_MS              70
#define LED_JOB_MIN_PERIOD_TICKS 2  /* each job at most once every ~140 ms */

typedef struct {
    uint8_t  con;
    uint8_t  led;
    uint16_t next_allowed_tick;
    volatile uint8_t active;  /* write last to "commit" */
} LedJob;

static volatile LedJob g_jobs[MAX_LED_JOBS];
static volatile uint8_t g_off_broadcast_pending = 0;

static volatile uint16_t g_tick = 0;  /* increments per RIT tick (wrap ok) */
static uint8_t jobs_rr = 0;           /* round-robin index over job table */

static volatile bool g_led_streaming_active = false; /* read by UART1 IRQ */

/* ------------------------------ App heartbeat ------------------------------ */
static volatile uint8_t g_tx_len = 0;
static uint8_t          g_tx_buf[TX_FRAME_MAX];

static uint8_t build_status_frame(uint8_t *dst, uint8_t cap) {
    const uint8_t N     = cfg_count;
    const uint8_t LEN   = (uint8_t)(N + 7);
    const uint8_t TOTAL = (uint8_t)(LEN + 3);
    if (TOTAL > cap) return 0;

    /* If you already compose “view_*” (OR in round masks), you can keep that here.
       If not, this still works with your committed masks. */
    const uint32_t view_alive = (g_alive_mask | round_alive_mask);
    const uint32_t view_trig  = ( (g_triggered_mask | round_triggered_mask) & view_alive );
    
    uint8_t *p = dst;
    *p++ = SOF;
    *p++ = LEN;
    *p++ = GRP_RX_TO_APP;
    *p++ = RX_ID;
    *p++ = SC_STATUS;
    *p++ = g_status_ext;   /* <—— the byte right after 0x0A (SC_STATUS). 0x00 or 0x02 when button pressed */
    *p++ = N;

    for (uint8_t i = 0; i < N; ++i) {
        const uint8_t  c   = cfg_conn[i];
        const uint32_t bit = (c >= 1 && c <= 31) ? (1u << (c - 1)) : 0;
        uint8_t Si = 0x00;
        if (view_alive & bit) Si = (view_trig & bit) ? 0x07 : 0x05;
        *p++ = Si;
    }
    *p++ = 0x00;   /* reserved */
    *p++ = 0x00;   /* reserved */
    *p++ = END_BYTE;
    return TOTAL;
}

static inline void send_status(void) {
    uint8_t n = build_status_frame(g_tx_buf, sizeof(g_tx_buf));
    if (n) g_tx_len = n;
}

/* ------------------------------ WS2812B strips ------------------------------ */
#define WS_LED_COUNT 96

static RGB_t   ws_buf_b1[WS_LED_COUNT];
static RGB_t   ws_buf_b2[WS_LED_COUNT];
static WS2812B ws_b1 = { .leds = ws_buf_b1, .num_leds = WS_LED_COUNT };
static WS2812B ws_b2 = { .leds = ws_buf_b2, .num_leds = WS_LED_COUNT };

static volatile bool ws_dirty_b1 = false;
static volatile bool ws_dirty_b2 = false;

static inline void ws_clear(RGB_t *buf, uint16_t n) {
    memset(buf, 0, n * sizeof(RGB_t));
}
static inline void ws_set_red(RGB_t *buf, uint16_t n, uint16_t led1) {
    if (led1 == 0 || led1 > n) return;
    const uint16_t idx = (uint16_t)(led1 - 1);
    buf[idx].r = 255; buf[idx].g = 0; buf[idx].b = 0; /* swap if your driver needs GRB */
}
static inline void ws_flush_if_dirty(void) {
    if (ws_dirty_b1) { WS2812B_write(&ws_b1); ws_dirty_b1 = false; }
    if (ws_dirty_b2) { WS2812B_write(&ws_b2); ws_dirty_b2 = false; }
}

/* Try to parse WS accessory command inside SC_LED_CTRL payload:
   Expected payload (pal >= 9):
       00 00 00 02 <BIN> <LED#> 00 00 00
   BIN: 1 => B1 (P3.25), 2 => B2 (P3.26)
*/
static bool try_handle_ws_accessory(const uint8_t *pay, uint8_t pal) {
    if (pal < 9) return false;
    if (pay[0] != 0x00 || pay[1] != 0x00 || pay[2] != 0x00) return false;
    if (pay[3] != 0x02) return false;               /* accessory command marker */
    const uint8_t bin = pay[4];
    const uint8_t led = pay[5];

    if (bin == 1) { ws_set_red(ws_buf_b1, WS_LED_COUNT, led); ws_dirty_b1 = true; return true; }
    if (bin == 2) { ws_set_red(ws_buf_b2, WS_LED_COUNT, led); ws_dirty_b2 = true; return true; }
    return false;
}

/* ------------------------------- App handlers ------------------------------- */
static void handle_upload_map(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 1) return;
    const uint8_t N = pay[0];
    if (paylen < (uint8_t)(1 + 2U * N)) return;

    cfg_count = 0;
    for (uint8_t i = 0; i < N && cfg_count < MAX_CFG; ++i) {
        const uint8_t c = pay[1 + 2*i + 0];
        const uint8_t s = pay[1 + 2*i + 1];
        if (s == 0x01) cfg_conn[cfg_count++] = c;
    }
    g_status_ext = 0x00;
}

static void handle_led_ctrl(const uint8_t *pay, uint8_t paylen) {
    /* First: try WS accessory command */
    if (try_handle_ws_accessory(pay, paylen)) return;

    /* Otherwise: 3B legacy LED (state, con, led) */
    if (paylen < 3) return;
    const uint8_t state = pay[0];
    const uint8_t con   = pay[1];
    const uint8_t led   = pay[2];

    if (state) {
        /* Start/maintain LED streaming job (polling pauses while any job active) */
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
        /* Stop this particular job; one broadcast OFF */
        for (uint8_t i = 0; i < MAX_LED_JOBS; ++i) {
            if (g_jobs[i].active && g_jobs[i].con == con && g_jobs[i].led == led) {
                g_jobs[i].active = 0;
                break;
            }
        }
        g_off_broadcast_pending = 1;
    }
}

static void handle_led_reset(const uint8_t *pay, uint8_t paylen) {
    (void)pay; (void)paylen;
    for (uint8_t i = 0; i < MAX_LED_JOBS; ++i) g_jobs[i].active = 0;
    g_off_broadcast_pending = 1;

    /* Also clear WS accessories */
    ws_clear(ws_buf_b1, WS_LED_COUNT); ws_dirty_b1 = true;
    ws_clear(ws_buf_b2, WS_LED_COUNT); ws_dirty_b2 = true;
}

static void handle_relay_set(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 2) return;
    Board_Relay_Set(pay[0], (pay[1] == 0x01));
}

static void handle_btnflag_reset(const uint8_t *pay, uint8_t pal) {
    /* Your frame is 27 04 85 01 09 00 16 → payload is one byte 0x00 */
    (void)pay; (void)pal;
    g_status_ext = 0x00;     /* clear back to 0 */
}

/* ----------------------------- UART0 RX (App->RX) --------------------------- */
typedef enum { RXF_WAIT_SOF=0, RXF_WAIT_LEN, RXF_COLLECT_BODY, RXF_WAIT_END } rx_fsm_t;

static volatile rx_fsm_t rx_state = RXF_WAIT_SOF;
static volatile uint8_t  rx_len   = 0;
static uint8_t           rx_buf[RX_LEN_MAX];
static uint8_t           rx_idx   = 0;

static void dispatch_app_frame(const uint8_t *p, uint8_t len) {
    if (len < 3) return;
    const uint8_t group = p[0];
    const uint8_t id    = p[1];
    const uint8_t sc    = p[2];
    const uint8_t *pay  = (len > 3) ? &p[3] : NULL;
    const uint8_t pal   = (len > 3) ? (uint8_t)(len - 3) : 0;

    if (group != GRP_APP_TO_RX || id != RX_ID) return;

    switch (sc) {
        case SC_POLL:        /* payload 00 */                 break;
        case SC_UPLOAD_MAP:  handle_upload_map(pay, pal);     break;
        case SC_LED_CTRL:    handle_led_ctrl(pay, pal);       break;
        case SC_LED_RESET:   handle_led_reset(pay, pal);      break;
        case SC_RELAY_SET:   handle_relay_set(pay, pal);      break;
        case SC_BTNFLAG_RESET: handle_btnflag_reset(pay, pal);      break;
        default:                                           break;
    }
    /* Reply heartbeat after any valid command */
    send_status();
}

void UART0_IRQHandler(void) {
    while (Chip_UART_ReadLineStatus(UART_APP) & UART_LSR_RDR) {
        const uint8_t b = Chip_UART_ReadByte(UART_APP);
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
            } else {
                rx_state = RXF_WAIT_SOF;
            }
            break;
        case RXF_WAIT_END:
            if (b == END_BYTE) dispatch_app_frame(rx_buf, rx_len);
            rx_state = RXF_WAIT_SOF; break;
        default:
            rx_state = RXF_WAIT_SOF; break;
        }
    }
}

/* ----------------------------- UART1 RX (Slave->RX) ------------------------- */
/* Accepts: 27 27 03 0A <addr> <state> 16 */
typedef enum { U1_WAIT_SOF=0, U1_GOT_SOF, U1_WAIT_LEN, U1_COLLECT, U1_WAIT_END } u1_fsm_t;

static volatile u1_fsm_t u1_state = U1_WAIT_SOF;
static volatile uint8_t  u1_len   = 0;
static volatile uint8_t  u1_idx   = 0;
static uint8_t           u1_pay[8];

void UART1_IRQHandler(void) {
    while (Chip_UART_ReadLineStatus(UART_SLAVE) & UART_LSR_RDR) {
        const uint8_t b = Chip_UART_ReadByte(UART_SLAVE);
        switch (u1_state) {
        case U1_WAIT_SOF:
            if (b == SOF) u1_state = U1_GOT_SOF;
            break;

        case U1_GOT_SOF:
            if (b == SOF) {
                u1_state = U1_WAIT_LEN;
            } else {
                /* tolerate single SOF lead */
                u1_len = b; u1_idx = 0;
                if (!u1_len || u1_len > sizeof(u1_pay)) { u1_state = U1_WAIT_SOF; break; }
                u1_state = U1_COLLECT;
            }
            break;

        case U1_WAIT_LEN:
            u1_len = b; u1_idx = 0;
            if (!u1_len || u1_len > sizeof(u1_pay)) { u1_state = U1_WAIT_SOF; break; }
            u1_state = U1_COLLECT;
            break;

        case U1_COLLECT:
            u1_pay[u1_idx++] = b;
            if (u1_idx == u1_len) u1_state = U1_WAIT_END;
            break;

        case U1_WAIT_END:
            if (b == END_BYTE && u1_len == 3 && u1_pay[0] == SC_STATUS) {
                const uint8_t addr = u1_pay[1];
                const uint8_t st   = u1_pay[2]; /* 0x01 idle, 0x03 triggered */
                if (addr >= 1 && addr <= 31 && st != 0x00) {
                    const uint32_t bit = CONN_BIT(addr);

                    /* Per-round bitmasks (for next commit in polling) */
                    round_alive_mask |= bit;
                    if (st == 0x03) round_triggered_mask |= bit;
                    else            round_triggered_mask &= ~bit;

                    /* Immediate update during LED streaming */
                    if (g_led_streaming_active) {
                        g_alive_mask |= bit;
                        if (st == 0x03) g_triggered_mask |= bit;
                        else            g_triggered_mask &= ~bit;
                    }
                }
            }
            u1_state = U1_WAIT_SOF;
            break;

        default:
            u1_state = U1_WAIT_SOF;
            break;
        }
    }
}

/* --------------------- RIT: 70 ms (LED, polling, WS flush) ------------------ */
static volatile uint8_t poll_rr_idx     = 0;  /* index in 0..(cfg_count-1) */
static volatile bool    led_active_prev = false;

static inline void commit_and_clear_poll_round(void) {
    /* commit last round snapshot (only trigger if alive) and clear accumulators */
    g_alive_mask     = round_alive_mask;
    g_triggered_mask = round_triggered_mask & g_alive_mask;
    round_alive_mask     = 0;
    round_triggered_mask = 0;
}

void RIT_IRQHandler(void) {
    Chip_RIT_ClearInt(LPC_RITIMER);
    g_tick++;

    /* one-time broadcast OFF if requested */
    if (g_off_broadcast_pending) {
        slave_enqueue_led_off_broadcast();
        g_off_broadcast_pending = 0;
    }

    /* check if any LED streaming job is active */
    bool led_active_now = false;
    for (uint8_t i = 0; i < MAX_LED_JOBS; ++i) {
        if (g_jobs[i].active) { led_active_now = true; break; }
    }
    g_led_streaming_active = led_active_now;

    if (led_active_now) {
        /* mark that we were in LED mode */
        led_active_prev = true;

        /* round-robin ONE LED job per tick, rate-limited per job */
        for (uint8_t k = 0; k < MAX_LED_JOBS; ++k) {
            const uint8_t i = (uint8_t)((jobs_rr + k) % MAX_LED_JOBS);
            if (!g_jobs[i].active) continue;

            const bool time_ok = ((int16_t)(g_tick - g_jobs[i].next_allowed_tick) >= 0);
            if (time_ok) {
                slave_enqueue_led_on(g_jobs[i].con, g_jobs[i].led);
                g_jobs[i].next_allowed_tick = (uint16_t)(g_tick + LED_JOB_MIN_PERIOD_TICKS);
                jobs_rr = (uint8_t)((i + 1) % MAX_LED_JOBS);
                break; /* only one job per tick */
            }
        }

        /* flush WS strips while we’re here */
        ws_flush_if_dirty();
        return; /* no general polling in LED mode */
    }

    /* left LED mode -> resume polling from the beginning; do not clear snapshots */
    if (led_active_prev) {
        led_active_prev = false;
        poll_rr_idx = 0; /* restart at connector #1 */
    }

    /* polling mode: nothing to poll if no Upload-Map yet */
    if (cfg_count == 0) { ws_flush_if_dirty(); return; }

    /* start of a new round => commit previous round’s snapshot */
    if (poll_rr_idx == 0) commit_and_clear_poll_round();

    /* strict 1..N order via cfg_conn[] */
    const uint8_t con = cfg_conn[poll_rr_idx];
    slave_enqueue_poll(con);

    /* advance, wrap to start (next tick will commit previous round again) */
    poll_rr_idx++;
    if (poll_rr_idx >= cfg_count) poll_rr_idx = 0;

    /* flush WS strips */
    ws_flush_if_dirty();
}

void GPIO_IRQ_HANDLER(void) {
    /* Check/clear Port 2, bit 3 interrupt */
    uint32_t stat = Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, GPIOINT_PORT2) |
                    Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, GPIOINT_PORT2);

    if (stat & (1u << GPIO_BUTTON_PIN)) {
        /* Latch flag = 0x02 on any press edge (we use falling by default) */
        g_status_ext = 0x02;

        /* Clear both edges just in case */
        Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIOINT_PORT2, (1u << GPIO_BUTTON_PIN));
    }
}

/* ----------------------------------- Main ---------------------------------- */
int main(void) {
    SystemCoreClockUpdate();
    Board_Init();

    Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_BUTTON_PORT, GPIO_BUTTON_PIN);
    Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, GPIOINT_PORT2, (1u << GPIO_BUTTON_PIN));
    NVIC_ClearPendingIRQ(GPIO_NVIC_NAME);
    NVIC_SetPriority(GPIO_NVIC_NAME, 2);   /* higher prio than UART0 status tx if you want snappy flag */
    NVIC_EnableIRQ(GPIO_NVIC_NAME);

    /* WS pins: P3.25 (B1), P3.26 (B2) as GPIO outputs */
    Chip_IOCON_Init(LPC_IOCON);
    Chip_GPIO_Init(LPC_GPIO);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 3, 25, IOCON_FUNC0 | IOCON_MODE_INACT);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 3, 26, IOCON_FUNC0 | IOCON_MODE_INACT);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 3, 25);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 3, 26);

    /* clear strips once at boot (will flush on first RIT) */
    ws_clear(ws_buf_b1, WS_LED_COUNT); ws_dirty_b1 = true;
    ws_clear(ws_buf_b2, WS_LED_COUNT); ws_dirty_b2 = true;

    /* UART0 (App link): 19,200 8N1 */
    Chip_UART_Init(UART_APP);
    Chip_UART_SetBaud(UART_APP, 19200);
    Chip_UART_ConfigData(UART_APP, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_APP, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_APP);

    /* UART1 (Slave link): 9,600 8N1 */
    Chip_UART_Init(UART_SLAVE);
    Chip_UART_SetBaud(UART_SLAVE, 9600);
    Chip_UART_ConfigData(UART_SLAVE, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_SLAVE, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_SLAVE);

    /* IRQs */
    Chip_UART_IntEnable(UART_APP,   UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_APP,   3);
    NVIC_EnableIRQ(IRQ_APP);

    Chip_UART_IntEnable(UART_SLAVE, UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_SLAVE, 2);
    NVIC_EnableIRQ(IRQ_SLAVE);

    /* RIT: 70 ms tick runs LED streaming / polling / WS flush */
    Chip_RIT_Init(LPC_RITIMER);
    Chip_RIT_SetTimerInterval(LPC_RITIMER, RIT_TICK_MS);
    NVIC_ClearPendingIRQ(RITIMER_IRQn);
    NVIC_SetPriority(RITIMER_IRQn, 1);
    NVIC_EnableIRQ(RITIMER_IRQn);

    /* main loop: flush heartbeat (UART0) and send queued UART1 frames */
    for (;;) {
        if (g_tx_len) {
            const uint8_t n = g_tx_len;
            Chip_UART_SendBlocking(UART_APP, g_tx_buf, n);
            g_tx_len = 0;
        }
        U1Frame fr;
        if (u1q_pop_main(&fr)) {
            Chip_UART_SendBlocking(UART_SLAVE, fr.data, fr.len);
        }
        __WFI();
    }
}
