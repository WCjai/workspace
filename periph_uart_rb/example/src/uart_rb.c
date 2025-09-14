#include "chip.h"
#include "board.h"
#include <stdbool.h>
#include <string.h>

/* =============================================================================
   RX <-> APP Protocol replica
   Frame: [0x27][LEN][GROUP][RX_ID][SUBCMD][PAYLOAD...][0x16]
          LEN counts bytes from GROUP through last payload (excludes SOF/LEN/END)
   Groups: 0x85 = App->RX, 0x00 = RX->App
   Subcmd (App->RX): 0x00 poll, 0x02 led_ctrl, 0x04 upload_map, 0x3A led_reset, 0x06 relay_set
   Subcmd (RX->App): 0x0A status
   ============================================================================= */

#define UART_APP        LPC_UART0     /* App link */
#define IRQ_APP         UART0_IRQn
#define HANDLER_APP     UART0_IRQHandler

#define UART_SLAVE      LPC_UART1     /* Slave LED boards (placeholder TX only) */

#define RX_ID           0x01

/* Protocol bytes */
enum { SOF = 0x27, END_BYTE = 0x16 };
enum { GRP_APP_TO_RX = 0x85, GRP_RX_TO_APP = 0x00 };
enum { SC_POLL = 0x00, SC_LED_CTRL = 0x02, SC_UPLOAD_MAP = 0x04, SC_LED_RESET = 0x3A, SC_RELAY_SET = 0x06 };
enum { SC_STATUS = 0x0A };

/* Config capacity (hardware max) */
#define MAX_CFG         31

/* Largest incoming LEN we must support:
   Upload-Map: LEN = 4 + 2*N  (N=31 => 66) */
#define RX_LEN_MAX      (4 + 2*MAX_CFG)

/* Largest outgoing on-wire frame for status:
   LEN = N + 7 (<= 38); on wire total = LEN + 3 (<= 41) */
#define TX_FRAME_MAX    (MAX_CFG + 10)

/* ----------------------------- State & helpers ----------------------------- */

static uint8_t cfg_conn[MAX_CFG];     /* enabled connector IDs (in order) */
static uint8_t cfg_count = 0;         /* N from last upload-map */

/* Reported-alive connectors; rest are 0x00 */
static const uint8_t k_alive_list[] = { 1, 5, 6, 30 };
static const uint8_t k_alive_count  = sizeof(k_alive_list) / sizeof(k_alive_list[0]);

static inline bool is_alive(uint8_t conn) {
    for (uint8_t i = 0; i < k_alive_count; ++i) if (k_alive_list[i] == conn) return true;
    return false;
}

/* LED placeholders (hook to UART1 later if you want) */
static inline void led_on(uint8_t con, uint8_t led)  { (void)con; (void)led; }
static inline void led_off(uint8_t con, uint8_t led) { (void)con; (void)led; }

/* Single TX queue (sent outside ISR) */
static volatile uint8_t g_tx_len = 0;
static uint8_t          g_tx_buf[TX_FRAME_MAX];

static inline void txq_send_now(const uint8_t *data, uint8_t n) {
    if (n > sizeof(g_tx_buf)) n = sizeof(g_tx_buf);
    memcpy(g_tx_buf, data, n);
    __DSB(); __ISB();
    g_tx_len = n;
}

/* ----------------------- Building outgoing status frame -------------------- */

/* Build status into dst; return on-wire byte count, or 0 on error */
static uint8_t build_status_frame(uint8_t *dst, uint8_t cap) {
    const uint8_t N = cfg_count;
    const uint8_t LEN = (uint8_t)(N + 7);          /* bytes from GROUP..payload */
    const uint8_t TOTAL = (uint8_t)(LEN + 3);      /* include SOF, LEN, END_BYTE */

    if (TOTAL > cap) return 0;

    uint8_t *p = dst;
    *p++ = SOF;
    *p++ = LEN;
    *p++ = GRP_RX_TO_APP;
    *p++ = RX_ID;
    *p++ = SC_STATUS;
    *p++ = 0x00;          /* flags */
    *p++ = N;             /* connector count */

    for (uint8_t i = 0; i < N; ++i) {
        const uint8_t conn = cfg_conn[i];
        *p++ = is_alive(conn) ? 0x05 : 0x00;
    }

    *p++ = 0x00;          /* reserved */
    *p++ = 0x00;          /* reserved */
    *p++ = END_BYTE;

    return TOTAL;
}

static inline void send_status(void) {
    uint8_t n = build_status_frame(g_tx_buf, sizeof(g_tx_buf));
    if (n) { g_tx_len = n; }
}

/* ----------------------------- Command handlers --------------------------- */

static void handle_upload_map(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 1) return;
    const uint8_t N = pay[0];
    if (paylen < (uint8_t)(1 + 2U * N)) return;    /* need N pairs */

    cfg_count = 0;
    for (uint8_t i = 0; i < N && cfg_count < MAX_CFG; ++i) {
        const uint8_t conn  = pay[1 + 2*i + 0];
        const uint8_t state = pay[1 + 2*i + 1];
        if (state == 0x01) cfg_conn[cfg_count++] = conn;
    }
    /* Mirror real device: emit status immediately after map */
    send_status();
}

static void handle_led_ctrl(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 3) return;
    const uint8_t state = pay[0];
    const uint8_t conn  = pay[1];
    const uint8_t led   = pay[2];
    if (state) led_on(conn, led); else led_off(conn, led);
}

static void handle_led_reset(const uint8_t *pay, uint8_t paylen) {
    (void)pay; (void)paylen;
    /* Optional: clear tracked LEDs here (no-op placeholder) */
}

static void handle_relay_set(const uint8_t *pay, uint8_t paylen) {
    if (paylen < 2) return;
    const uint8_t relay = pay[0];
    const bool    on    = (pay[1] == 0x01);
    Board_Relay_Set(relay, on);
}

/* ----------------------------- RX frame parser ---------------------------- */

typedef enum {
    RXF_WAIT_SOF = 0,
    RXF_WAIT_LEN,
    RXF_COLLECT_BODY,     /* collect LEN bytes: GROUP..payload */
    RXF_WAIT_END
} rx_fsm_t;

static volatile rx_fsm_t rx_state = RXF_WAIT_SOF;
static volatile uint8_t  rx_len   = 0;            /* expected LEN from header */
static uint8_t           rx_buf[RX_LEN_MAX];      /* GROUP..payload */
static uint8_t           rx_idx   = 0;            /* collected so far */

/* Dispatch a full, validated body (GROUP..payload, length=len) */
static void dispatch_body(const uint8_t *p, uint8_t len) {
    if (len < 3) return;                           /* need GROUP,ID,SUBCMD */

    const uint8_t group = p[0];
    const uint8_t id    = p[1];
    const uint8_t sc    = p[2];
    const uint8_t *pay  = (len > 3) ? &p[3] : NULL;
    const uint8_t  pal  = (len > 3) ? (uint8_t)(len - 3) : 0;

    if (group != GRP_APP_TO_RX || id != RX_ID) return;

    switch (sc) {
        case SC_POLL:        /* payload: 00 */
            send_status();
            break;
        case SC_UPLOAD_MAP:
            handle_upload_map(pay, pal);
            break;
        case SC_LED_CTRL:
            handle_led_ctrl(pay, pal);
            break;
        case SC_LED_RESET:
            handle_led_reset(pay, pal);
            break;
        case SC_RELAY_SET:
            handle_relay_set(pay, pal);
            break;
        default:
            /* ignore unknown subcommands */
            break;
    }
}

/* Byte-ingest state machine (call from UART ISR) */
static inline void rx_ingest(uint8_t b) {
    switch (rx_state) {
    case RXF_WAIT_SOF:
        if (b == SOF) rx_state = RXF_WAIT_LEN;
        break;

    case RXF_WAIT_LEN:
        rx_len = b;
        if (rx_len == 0 || rx_len > RX_LEN_MAX) { rx_state = RXF_WAIT_SOF; break; }
        rx_idx = 0;
        rx_state = RXF_COLLECT_BODY;
        break;

    case RXF_COLLECT_BODY:
        if (rx_idx < RX_LEN_MAX) {
            rx_buf[rx_idx++] = b;
            if (rx_idx == rx_len) rx_state = RXF_WAIT_END;
        } else {
            rx_state = RXF_WAIT_SOF;   /* overflow -> reset */
        }
        break;

    case RXF_WAIT_END:
        if (b == END_BYTE) dispatch_body(rx_buf, rx_len);
        rx_state = RXF_WAIT_SOF;
        break;

    default:
        rx_state = RXF_WAIT_SOF;
        break;
    }
}

/* UART0 ISR: read all pending bytes and feed parser */
void HANDLER_APP(void) {
    while ((Chip_UART_ReadLineStatus(UART_APP) & UART_LSR_RDR) != 0) {
        rx_ingest(Chip_UART_ReadByte(UART_APP));
    }
}

/* ---------------------------------- Main ---------------------------------- */

int main(void) {
    SystemCoreClockUpdate();
    Board_Init();

    /* UART0 (APP link): 19,200 8N1 */
    Chip_UART_Init(UART_APP);
    Chip_UART_SetBaud(UART_APP, 19200);
    Chip_UART_ConfigData(UART_APP, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_APP, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_APP);

    /* UART1 (SLAVE link, placeholder): 19,200 8N1 */
    Chip_UART_Init(UART_SLAVE);
    Chip_UART_SetBaud(UART_SLAVE, 19200);
    Chip_UART_ConfigData(UART_SLAVE, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_SLAVE, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_SLAVE);

    /* Enable UART0 RX interrupts */
    Chip_UART_IntEnable(UART_APP, UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_APP, 1);
    NVIC_EnableIRQ(IRQ_APP);

    /* Idle: flush TX queue if present */
    for (;;) {
        if (g_tx_len) {
            uint8_t n = g_tx_len;
            Chip_UART_SendBlocking(UART_APP, g_tx_buf, n);
            g_tx_len = 0;
        }
        __WFI();
    }
}
