#include "chip.h"
#include "board.h"
#include <stdbool.h>
#include <string.h>

/* =============================================================================
   RX <-> APP Protocol replica + UART1 slave pinger w/ TTL-based "alive"
   + Limit switch reporting (0x05 normal, 0x07 when limit triggered)
   ============================================================================= */

/* ===================== UART selection & IRQ names ========================= */
#define UART_APP        LPC_UART0     /* App link */
#define IRQ_APP         UART0_IRQn
#define HANDLER_APP     UART0_IRQHandler

#define UART_SLAVE      LPC_UART1     /* Slave LED boards */
#define IRQ_SLAVE       UART1_IRQn
#define HANDLER_SLAVE   UART1_IRQHandler

/* ============================== Identity ================================== */
#define RX_ID           0x01          /* This RX unit's ID */

/* =========================== Protocol bytes =============================== */
enum { SOF = 0x27, END_BYTE = 0x16 };
enum { GRP_APP_TO_RX = 0x85, GRP_RX_TO_APP = 0x00 };
enum { SC_POLL = 0x00, SC_LED_CTRL = 0x02, SC_UPLOAD_MAP = 0x04, SC_LED_RESET = 0x3A, SC_RELAY_SET = 0x06 };
enum { SC_STATUS = 0x0A };

/* ============================ Config limits =============================== */
#define MAX_CFG             31                  /* max connectors supported */

/* Largest incoming LEN (GROUP..payload) to handle on APP RX:
   Upload-Map: LEN = 4 + 2*N  (N=31 => 66) */
#define APP_RX_LEN_MAX      (4 + 2*MAX_CFG)     /* 66 */

/* Largest outgoing status (on-wire): LEN = N + 7 (<= 38), total = LEN + 3 (<= 41) */
#define APP_TX_FRAME_MAX    (MAX_CFG + 10)      /* 41 */

/* ============================ Timing knobs ================================ */
#define APP_BAUD            19200
#define SLAVE_BAUD          9600    /* set to your slave's actual baud */

#define PING_PREMAP_MS      55u     /* broadcast ping period before upload-map */
#define PING_AFTERMAP_MS    50u     /* per-connector ping spacing after map */

/* ======================== Slave frame "length" byte ======================= */
/* If your captures use actual length bytes (common: 0x09/0x08), set them here. */
#define SLAVE_LEN_PING      0x97    /* else 0x09 */
#define SLAVE_LEN_ON        0x97    /* else 0x09 */
#define SLAVE_LEN_OFF       0x97    /* else 0x08 */

/* ============================ State: config =============================== */
static uint8_t cfg_conn[MAX_CFG];   /* enabled connectors (in order) */
static uint8_t cfg_count = 0;       /* count of enabled connectors */

/* ===================== State: time-based "alive" TTL ====================== */
static volatile uint32_t g_ms = 0;          /* 1 ms tick */
static volatile uint32_t last_seen_ms[32];   /* index by connector 1..31; 0 = never */
static uint32_t          alive_ttl_ms = 200; /* computed from cfg_count */

static inline uint32_t per_connector_period_ms(void) {
    return (cfg_count ? (uint32_t)cfg_count * PING_AFTERMAP_MS : PING_PREMAP_MS);
}
static inline void recompute_alive_ttl(void) {
    /* TTL ≈ 2x expected ping interval for that connector + margin */
    uint32_t per = per_connector_period_ms();
    uint32_t ttl = per * 2u + 30u;
    if (ttl < 120u) ttl = 120u;
    if (ttl > 5000u) ttl = 5000u;
    alive_ttl_ms = ttl;
}
static inline void alive_mark_seen(uint8_t con) {
    if (con >= 1 && con <= 31) last_seen_ms[con] = g_ms;
}
static inline bool is_alive(uint8_t con) {
    if (con < 1 || con > 31) return false;
    uint32_t seen = last_seen_ms[con];
    return (seen != 0u) && ((g_ms - seen) <= alive_ttl_ms);
}

/* ======================= State: limit switch per con ====================== */
/* Stores the last reported limit state from the slave for each connector.
   0x01 = NOT TRIGGERED, 0x03 = TRIGGERED (your spec). Others treated as 0x01. */
static volatile uint8_t limit_state[32];   /* index by connector 1..31 */

/* =========================== UART1 (slave) TX ============================= */
static inline void slave_send_ping_broadcast(void) {
    /* 27 97 05 85 00 00 00 00 16 */
    uint8_t f[9] = { 0x27, SLAVE_LEN_PING, 0x05, 0x85, 0x00, 0x00, 0x00, 0x00, 0x16 };
    Chip_UART_SendBlocking(UART_SLAVE, f, sizeof f);
}
static inline void slave_send_ping_addr(uint8_t con) {
    /* 27 97 05 85 <con> 00 00 00 16 */
    uint8_t f[9] = { 0x27, SLAVE_LEN_PING, 0x05, 0x85, con, 0x00, 0x00, 0x00, 0x16 };
    Chip_UART_SendBlocking(UART_SLAVE, f, sizeof f);
}
static inline void slave_send_led_on(uint8_t con, uint8_t led) {
    /* 27 97 05 85 <con> 02 01 <led> 16 */
    uint8_t f[9] = { 0x27, SLAVE_LEN_ON, 0x05, 0x85, con, 0x02, 0x01, led, 0x16 };
    Chip_UART_SendBlocking(UART_SLAVE, f, sizeof f);
}
static inline void slave_send_led_off_broadcast(void) {
    /* 27 97 04 85 FF 00 00 16 */
    uint8_t f[8] = { 0x27, SLAVE_LEN_OFF, 0x04, 0x85, 0xFF, 0x03, 0x00, 0x16 };
    Chip_UART_SendBlocking(UART_SLAVE, f, sizeof f);
}

/* LED actions invoked by App->RX LED control */
static inline void led_on(uint8_t con, uint8_t led)  { slave_send_led_off_broadcast(); slave_send_led_on(con, led); }
static inline void led_off(uint8_t con, uint8_t led) { (void)con; (void)led; slave_send_led_off_broadcast(); }

/* ====================== APP replies: single TX queue ====================== */
static volatile uint8_t g_tx_len = 0;
static uint8_t          g_tx_buf[APP_TX_FRAME_MAX];
static inline void txq_queue(const uint8_t *data, uint8_t n) {
    if (n > sizeof(g_tx_buf)) n = sizeof(g_tx_buf);
    memcpy(g_tx_buf, data, n);
    __DSB(); __ISB();
    g_tx_len = n;
}

/* ======================= Build & queue STATUS frame ======================= */
/* Status: 27 (N+7) 00 RX_ID 0A  flags  N  S1..SN  00 00 16
   Si encoding:
     - 0x00 : dead (no recent reply within TTL)
     - 0x05 : alive, limit NOT triggered (limit_state == 0x01 or default)
     - 0x07 : alive, limit TRIGGERED (limit_state == 0x03)
*/
static uint8_t build_status_frame(uint8_t *dst, uint8_t cap) {
    const uint8_t N = cfg_count;
    const uint8_t LEN   = (uint8_t)(N + 7);
    const uint8_t TOTAL = (uint8_t)(LEN + 3);
    if (TOTAL > cap) return 0;

    uint8_t *p = dst;
    *p++ = SOF;
    *p++ = LEN;
    *p++ = GRP_RX_TO_APP;
    *p++ = RX_ID;
    *p++ = SC_STATUS;
    *p++ = 0x00;          /* flags */
    *p++ = N;
    for (uint8_t i = 0; i < N; ++i) {
        const uint8_t con = cfg_conn[i];
        uint8_t si = 0x00;
        if (is_alive(con)) {
            const uint8_t lim = limit_state[con];
            si = (lim == 0x03) ? 0x07 : 0x05;
        }
        *p++ = si;
    }
    *p++ = 0x00;          /* reserved */
    *p++ = 0x00;          /* reserved */
    *p++ = END_BYTE;
    return TOTAL;
}
static inline void send_status(void) {
    uint8_t n = build_status_frame(g_tx_buf, sizeof(g_tx_buf));
    if (n) g_tx_len = n;
}

/* ========================== App command handlers ========================== */
static void handle_upload_map(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 1) return;
    const uint8_t N = pay[0];
    if (paylen < (uint8_t)(1 + 2U * N)) return;

    cfg_count = 0;
    for (uint8_t i = 0; i < N && cfg_count < MAX_CFG; ++i) {
        const uint8_t con   = pay[1 + 2*i + 0];
        const uint8_t state = pay[1 + 2*i + 1];
        if (state == 0x01) cfg_conn[cfg_count++] = con;
    }
    /* Recompute TTL based on new cfg_count; optionally clear last_seen + limit_state */
    recompute_alive_ttl();
    /* for (int i = 1; i <= 31; ++i) { last_seen_ms[i] = 0; limit_state[i] = 0x01; } */

    send_status();   /* immediate status like the original device */
}
static void handle_led_ctrl(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 3) return;
    const uint8_t state = pay[0];
    const uint8_t con   = pay[1];
    const uint8_t led   = pay[2];
    if (state) led_on(con, led); else led_off(con, led);
}
static void handle_led_reset(const uint8_t *pay, uint8_t paylen) {
    (void)pay; (void)paylen;
    slave_send_led_off_broadcast();
}
static void handle_relay_set(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 2) return;
    const uint8_t relay = pay[0];
    const bool    on    = (pay[1] == 0x01);
    Board_Relay_Set(relay, on);
}

/* ======================== APP RX parser (UART0) =========================== */
typedef enum { RXF_WAIT_SOF=0, RXF_WAIT_LEN, RXF_COLLECT_BODY, RXF_WAIT_END } rx_fsm_t;
static volatile rx_fsm_t app_state = RXF_WAIT_SOF;
static volatile uint8_t  app_len   = 0;
static uint8_t           app_buf[APP_RX_LEN_MAX];
static uint8_t           app_idx   = 0;

static void dispatch_app_body(const uint8_t *p, uint8_t len) {
    if (len < 3) return;                  /* need GROUP, ID, SUBCMD */
    const uint8_t group = p[0];
    const uint8_t id    = p[1];
    const uint8_t sc    = p[2];
    const uint8_t *pay  = (len > 3) ? &p[3] : NULL;
    const uint8_t pal   = (len > 3) ? (uint8_t)(len - 3) : 0;

    if (group != GRP_APP_TO_RX || id != RX_ID) return;

    switch (sc) {
        case SC_POLL:        send_status();               break;
        case SC_UPLOAD_MAP:  handle_upload_map(pay, pal); break;
        case SC_LED_CTRL:    handle_led_ctrl(pay, pal);   break;
        case SC_LED_RESET:   handle_led_reset(pay, pal);  break;
        case SC_RELAY_SET:   handle_relay_set(pay, pal);  break;
        default: break;
    }
}
void HANDLER_APP(void) {
    while ((Chip_UART_ReadLineStatus(UART_APP) & UART_LSR_RDR) != 0) {
        uint8_t b = Chip_UART_ReadByte(UART_APP);
        switch (app_state) {
        case RXF_WAIT_SOF:      if (b == SOF) app_state = RXF_WAIT_LEN; break;
        case RXF_WAIT_LEN:
            app_len = b;
            if (app_len == 0 || app_len > APP_RX_LEN_MAX) { app_state = RXF_WAIT_SOF; break; }
            app_idx = 0; app_state = RXF_COLLECT_BODY; break;
        case RXF_COLLECT_BODY:
            if (app_idx < APP_RX_LEN_MAX) {
                app_buf[app_idx++] = b;
                if (app_idx == app_len) app_state = RXF_WAIT_END;
            } else {
                app_state = RXF_WAIT_SOF;
            }
            break;
        case RXF_WAIT_END:
            if (b == END_BYTE) dispatch_app_body(app_buf, app_len);
            app_state = RXF_WAIT_SOF; break;
        default: app_state = RXF_WAIT_SOF; break;
        }
    }
}

/* ======================= SLAVE RX parser (UART1) ========================== */
/* Expect: 27 27 03 0A <addr> <limit> 16  (double 0x27 then LEN=0x03) */
typedef enum { SLP_WAIT_SOF=0, SLP_GOT_SOF2, SLP_WAIT_LEN, SLP_WAIT_CMD, SLP_WAIT_ADDR, SLP_WAIT_LIMIT, SLP_WAIT_END } slv_fsm_t;
static volatile slv_fsm_t slv_state = SLP_WAIT_SOF;
static volatile uint8_t   slv_addr  = 0;
static volatile uint8_t   slv_limit = 0;  /* last parsed limit byte */

void HANDLER_SLAVE(void) {
    while ((Chip_UART_ReadLineStatus(UART_SLAVE) & UART_LSR_RDR) != 0) {
        uint8_t b = Chip_UART_ReadByte(UART_SLAVE);
        switch (slv_state) {
        case SLP_WAIT_SOF:    if (b == 0x27) slv_state = SLP_GOT_SOF2; break;
        case SLP_GOT_SOF2:    /* second 0x27 in your capture */ slv_state = SLP_WAIT_LEN; break;
        case SLP_WAIT_LEN:    if (b == 0x03) slv_state = SLP_WAIT_CMD; else slv_state = SLP_WAIT_SOF; break;
        case SLP_WAIT_CMD:    if (b == 0x0A) slv_state = SLP_WAIT_ADDR; else slv_state = SLP_WAIT_SOF; break;
        case SLP_WAIT_ADDR:   slv_addr = b; slv_state = SLP_WAIT_LIMIT; break;
        case SLP_WAIT_LIMIT:  slv_limit = b; slv_state = SLP_WAIT_END; break;
        case SLP_WAIT_END:
            if (b == END_BYTE) {
                /* Mark alive and remember limit state for that connector */
                alive_mark_seen(slv_addr);
                if (slv_addr >= 1 && slv_addr <= 31) {
                    limit_state[slv_addr] = slv_limit; /* 0x01 or 0x03 */
                }
            }
            slv_state = SLP_WAIT_SOF; break;
        default: slv_state = SLP_WAIT_SOF; break;
        }
    }
}

/* =============================== Scheduler ================================ */
static volatile uint32_t last_ping_ms = 0;
static uint8_t           rr_index     = 0;    /* round-robin over cfg_conn[] */

static void poll_scheduler_step(void) {
    const uint32_t now = g_ms;

    if (cfg_count == 0) {
        if (now - last_ping_ms >= PING_PREMAP_MS) {
            slave_send_ping_broadcast();
            last_ping_ms = now;
        }
        return;
    }

    if (now - last_ping_ms >= PING_AFTERMAP_MS) {
        uint8_t con = cfg_conn[rr_index];
        slave_send_ping_addr(con);

        rr_index++;
        if (rr_index >= cfg_count) rr_index = 0;

        last_ping_ms = now;
    }
}

/* ============================== Millisecond tick ========================== */
void SysTick_Handler(void) { g_ms++; }

/* ================================= Main =================================== */
int main(void) {
    SystemCoreClockUpdate();
    Board_Init();

    /* UART0 (APP) */
    Chip_UART_Init(UART_APP);
    Chip_UART_SetBaud(UART_APP, APP_BAUD);
    Chip_UART_ConfigData(UART_APP, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_APP, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_APP);

    /* UART1 (SLAVE) */
    Chip_UART_Init(UART_SLAVE);
    Chip_UART_SetBaud(UART_SLAVE, SLAVE_BAUD);
    Chip_UART_ConfigData(UART_SLAVE, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_SLAVE, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_SLAVE);

    /* Enable UART RX IRQs */
    Chip_UART_IntEnable(UART_APP,   UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_APP, 1);
    NVIC_EnableIRQ(IRQ_APP);

    Chip_UART_IntEnable(UART_SLAVE, UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_SLAVE, 2);
    NVIC_EnableIRQ(IRQ_SLAVE);

    /* 1 ms SysTick for scheduler & TTL */
    SysTick_Config(SystemCoreClock / 1000U);
    recompute_alive_ttl();

    for (;;) {
        /* Flush any queued APP reply (status) */
        if (g_tx_len) {
            uint8_t n = g_tx_len;
            Chip_UART_SendBlocking(UART_APP, g_tx_buf, n);
            g_tx_len = 0;
        }

        /* Drive UART1 ping cadence */
        poll_scheduler_step();

        __WFI();
    }
}
