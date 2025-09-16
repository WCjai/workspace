#include "chip.h"
#include "board.h"
#include <stdbool.h>
#include <string.h>

/* ---------------- FreeRTOS 7.x ---------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ============================== Identity & UARTs ============================== */
#define UART_APP        ((LPC_USART_T *)LPC_UART0)
#define IRQ_APP         UART0_IRQn

#define UART_SLAVE      ((LPC_USART_T *)LPC_UART1)
#define IRQ_SLAVE       UART1_IRQn

#define RIT_IRQN        RITIMER_IRQn
/* =========================== Protocol constants ============================== */
enum { SOF = 0x27, END_BYTE = 0x16 };
enum { GRP_APP_TO_RX = 0x85, GRP_RX_TO_APP = 0x00 };
enum { SC_POLL = 0x00, SC_LED_CTRL = 0x02, SC_UPLOAD_MAP = 0x04, SC_LED_RESET = 0x3A, SC_RELAY_SET = 0x06 };
enum { SC_STATUS = 0x0A };

#define RX_ID           0x01
#define APP_BAUD        19200
#define SLAVE_BAUD      9600    /* set to your slave's actual baud */

/* ================================ Timing ===================================== */
/* Replicate original RX device cadence exactly */
#define PING_PREMAP_MS      200u   /* broadcast ping before map upload */
#define PING_AFTERMAP_MS    200u   /* per-connector ping after map     */
#define TTL_MARGIN_MS       100u

/* ================================ Limits ===================================== */
#define MAX_CFG             31
#define APP_RX_LEN_MAX      (4 + 2*MAX_CFG)   /* upload-map cap (N up to 31) */
#define APP_TX_FRAME_MAX    (MAX_CFG + 10)    /* status frame cap */

/* ============================== Slave "lens" ================================= */
/* If your slave uses true lengths (0x09/0x08), change here. You said 0x97. */
#define SLAVE_LEN_PING  0x97
#define SLAVE_LEN_ON    0x97
#define SLAVE_LEN_OFF   0x97

/* ============================ Shared State =================================== */
static uint8_t  cfg_conn[MAX_CFG];         /* enabled connectors in order */
static uint8_t  cfg_count = 0;

static uint32_t last_seen_ms[32];          /* 1..31 valid; 0 means never */
static uint8_t  limit_state[32];           /* 0x01 normal, 0x03 triggered */
static uint32_t alive_ttl_ms = 200;

static xSemaphoreHandle gStateMtx;

/* FreeRTOS 7.x tick helpers */
#define MS_TO_TICKS(ms)   ((portTickType)((ms) / portTICK_RATE_MS))
#define NOW_TICKS()       (xTaskGetTickCount())
#define NOW_MS()          ((uint32_t)(NOW_TICKS() * portTICK_RATE_MS))

/* ============================== RX/TX Queues ================================= */
/* RX byte queues: ISR -> parser tasks */
static xQueueHandle qAppRxBytes;   /* UART0 RX bytes */
static xQueueHandle qSlvRxBytes;   /* UART1 RX bytes */

/* UART1 TX queues */
typedef struct { uint8_t bytes[16]; uint8_t len; } slv_frame_t;
/* ---- RIT IRQ name compatibility (LPCOpen uses RITIMER_IRQn) ---- */


/* LED command (highest priority) */
typedef struct { uint8_t con; uint8_t led; uint8_t state; bool reset_all; } slv_led_cmd_t;
static xQueueHandle qSlvLedCmd;    /* LED commands */

/* Pings (lower priority) */
static xQueueHandle qSlvPing;      /* ping frames */

/* UART0 TX queue: STATUS frames to APP */
typedef struct { uint8_t bytes[APP_TX_FRAME_MAX]; uint8_t len; } app_frame_t;
static xQueueHandle qAppTx;
static xSemaphoreHandle qSlvTxKick;

/* --------- forward declarations for helpers used before their definitions ---- */
static void enqueue_status_frame_urgent_replace(void);
static void enqueue_status_frame_coalesced(void);

/* ============================ Internal helpers =============================== */
static inline uint32_t per_connector_period_ms(void) {
    return (cfg_count ? (uint32_t)cfg_count * PING_AFTERMAP_MS : PING_PREMAP_MS);
}
static void recompute_alive_ttl_locked(void){
    uint32_t per = per_connector_period_ms();
    uint32_t ttl = per * 2u + TTL_MARGIN_MS;
    if (ttl < 120u) ttl = 120u;
    if (ttl > 5000u) ttl = 5000u;
    alive_ttl_ms = ttl;
}
static inline void alive_mark_seen_locked(uint8_t con, uint32_t now_ms){
    if (con >= 1 && con <= 31) last_seen_ms[con] = now_ms;
}
static inline bool is_alive_locked(uint8_t con, uint32_t now_ms){
    if (con < 1 || con > 31) return false;
    uint32_t seen = last_seen_ms[con];
    return (seen != 0u) && ((now_ms - seen) <= alive_ttl_ms);
}

/* UART blocking send for small frames (from *tasks* only) */
static void uart_send_blocking(LPC_USART_T *U, const uint8_t *p, uint32_t n){
    Chip_UART_SendBlocking(U, p, n);
}

/* ========================== Build SLAVE frames =============================== */
static inline void build_slave_ping_broadcast(slv_frame_t *f){
    const uint8_t b[9] = { 0x27, SLAVE_LEN_PING, 0x05, 0x85, 0x00, 0x00, 0x00, 0x00, 0x16 };
    memcpy(f->bytes, b, 9); f->len = 9;
}
static inline void build_slave_ping_addr(slv_frame_t *f, uint8_t con){
    const uint8_t b[9] = { 0x27, SLAVE_LEN_PING, 0x05, 0x85, con, 0x00, 0x00, 0x00, 0x16 };
    memcpy(f->bytes, b, 9); f->len = 9;
}
static inline void build_slave_led_on(slv_frame_t *f, uint8_t con, uint8_t led){
    const uint8_t b[9] = { 0x27, SLAVE_LEN_ON, 0x05, 0x85, con, 0x02, 0x01, led, 0x16 };
    memcpy(f->bytes, b, 9); f->len = 9;
}
static inline void build_slave_led_off_all(slv_frame_t *f){
    /* per your capture: FF 00 00 */
    const uint8_t b[8] = { 0x27, SLAVE_LEN_OFF, 0x04, 0x85, 0xFF, 0x03, 0x00, 0x16 };
    memcpy(f->bytes, b, 8); f->len = 8;
}

/* ============================ Build APP STATUS =============================== */
static uint8_t build_status_locked(uint8_t *dst, uint8_t cap){
    const uint8_t N = cfg_count;
    const uint8_t LEN   = (uint8_t)(N + 7);
    const uint8_t TOTAL = (uint8_t)(LEN + 3);
    if (TOTAL > cap) return 0;

    uint32_t now_ms = NOW_MS();
    uint8_t *p = dst;
    *p++ = SOF;
    *p++ = LEN;
    *p++ = GRP_RX_TO_APP;
    *p++ = RX_ID;
    *p++ = SC_STATUS;
    *p++ = 0x00;        /* flags */
    *p++ = N;

    for (uint8_t i = 0; i < N; ++i){
        uint8_t con = cfg_conn[i];
        uint8_t si  = 0x00;
        if (is_alive_locked(con, now_ms)){
            uint8_t lim = limit_state[con];
            si = (lim == 0x03) ? 0x07 : 0x05;
        }
        *p++ = si;
    }
    *p++ = 0x00; *p++ = 0x00; *p++ = END_BYTE;
    return TOTAL;
}

/* Push newest status; normal (FIFO) */
static void enqueue_status_frame(void){
    app_frame_t f;
    xSemaphoreTake(gStateMtx, portMAX_DELAY);
    uint8_t n = build_status_locked(f.bytes, sizeof(f.bytes));
    xSemaphoreGive(gStateMtx);
    if (n){ f.len = n; (void)xQueueSend(qAppTx, &f, 0); }
}

/* ---- APP TX helpers: "latest-wins" for urgent, coalesced normal ---- */
static inline unsigned portBASE_TYPE app_tx_has_pending(void) {
    return (uxQueueMessagesWaiting(qAppTx) > 0);
}

/* Drop all pending, then enqueue the newest status (never blocks). */
static void enqueue_status_frame_urgent_replace(void){
    app_frame_t f;
    xSemaphoreTake(gStateMtx, portMAX_DELAY);
    uint8_t n = build_status_locked(f.bytes, sizeof(f.bytes));
    xSemaphoreGive(gStateMtx);
    if (!n) return;
    f.len = n;

    taskENTER_CRITICAL();          // avoid race with Task_AppTx dequeue
    xQueueReset(qAppTx);           // drop any regular heartbeats
    (void)xQueueSend(qAppTx, &f, 0);
    taskEXIT_CRITICAL();
}

/* Enqueue a regular heartbeat only if none is pending (coalesce). */
static void enqueue_status_frame_coalesced(void){
    if (app_tx_has_pending()) return;  // drop; an older HB is still waiting
    app_frame_t f;
    xSemaphoreTake(gStateMtx, portMAX_DELAY);
    uint8_t n = build_status_locked(f.bytes, sizeof(f.bytes));
    xSemaphoreGive(gStateMtx);
    if (!n) return;
    f.len = n;
    (void)xQueueSend(qAppTx, &f, 0);   // best-effort; never block
}

static void slv_tx_wake(void){
    if (qSlvTxKick != NULL){
        (void)xSemaphoreGive(qSlvTxKick);
    }
}

/* ========================== App command handlers ============================= */
static void handle_upload_map(const uint8_t *pay, uint8_t pal){
    if (pal < 1) return;
    uint8_t N = pay[0];
    if (pal < (uint8_t)(1 + 2U*N)) return;

    xSemaphoreTake(gStateMtx, portMAX_DELAY);
    cfg_count = 0;
    for (uint8_t i = 0; i < N && cfg_count < MAX_CFG; ++i){
        uint8_t con   = pay[1 + 2*i + 0];
        uint8_t state = pay[1 + 2*i + 1];
        if (state == 0x01) cfg_conn[cfg_count++] = con;
    }
    recompute_alive_ttl_locked();
    xSemaphoreGive(gStateMtx);

    /* Switch RIT cadence now that we know N (post-map = 50 ms) */
    Chip_RIT_SetTimerInterval(LPC_RITIMER, (cfg_count == 0) ? PING_PREMAP_MS : PING_AFTERMAP_MS);

    slv_tx_wake();
}

/* LED commands = highest priority:
   - Drop stale LED frames so we always execute the latest intent.
   - Enqueue OFF-ALL then ON (if state=1). */
static void handle_led_ctrl(const uint8_t *pay, uint8_t pal){
    if (pal < 3) return;
    uint8_t state = pay[0];
    uint8_t con   = pay[1];
    uint8_t led   = pay[2];

    /* Drop pending LED commands so we execute the freshest intent */
    slv_led_cmd_t discard;
    while (xQueueReceive(qSlvLedCmd, &discard, 0) == pdTRUE){}

    slv_led_cmd_t cmd = {
        .con = con,
        .led = led,
        .state = state ? 1u : 0u,
        .reset_all = false
    };
    (void)xQueueSend(qSlvLedCmd, &cmd, 0);

    slv_tx_wake();
}

static void handle_led_reset(void){
    /* Flush any queued LED command and post a broadcast OFF */
    slv_led_cmd_t discard;
    while (xQueueReceive(qSlvLedCmd, &discard, 0) == pdTRUE){}

    slv_led_cmd_t cmd = {
        .con = 0u,
        .led = 0u,
        .state = 0u,
        .reset_all = true
    };
    (void)xQueueSend(qSlvLedCmd, &cmd, 0);
    slv_tx_wake();
}

extern void Board_Relay_Set(uint8_t relay, bool on);
static void handle_relay_set(const uint8_t *pay, uint8_t pal){
    if (pal < 2) return;
    uint8_t relay = pay[0];
    bool on = (pay[1] == 0x01);
    Board_Relay_Set(relay, on);
}

/* ============================= Timer (RIT) =================================== */
/* Use RIT as the deterministic 55ms/50ms ping scheduler. */
static volatile uint8_t rit_rr = 0; /* round-robin index after map */

void RIT_IRQHandler(void){
    portBASE_TYPE hpw = pdFALSE;

    /* Ack the interrupt */
    Chip_RIT_ClearInt(LPC_RITIMER);

    slv_frame_t f;

    /* Snapshot count without lock (8-bit, atomic read). */
    uint8_t n = cfg_count;

    if (n == 0){
        /* Pre-map: broadcast ping every 55 ms */
        build_slave_ping_broadcast(&f);
        (void)xQueueSendFromISR(qSlvPing, &f, &hpw);
    } else {
        /* Post-map: per-connector round-robin every 50 ms */
        if (rit_rr >= n) rit_rr = 0;
        uint8_t con = cfg_conn[rit_rr++];
        build_slave_ping_addr(&f, con);
        (void)xQueueSendFromISR(qSlvPing, &f, &hpw);
    }

    /* Wake the UART1 TX worker */
    if (qSlvTxKick) xSemaphoreGiveFromISR(qSlvTxKick, &hpw);

    portYIELD_FROM_ISR(hpw);
}

static void init_rit(uint32_t period_ms){
    Chip_RIT_Init(LPC_RITIMER);
    Chip_RIT_SetTimerInterval(LPC_RITIMER, period_ms);
    NVIC_SetPriority(RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(RITIMER_IRQn);
}


/* ============================= Tasks & ISRs ================================== */
/* UART0 ISR: push bytes to qAppRxBytes */
void UART0_IRQHandler(void){
    portBASE_TYPE hpw = pdFALSE;
    while (Chip_UART_ReadLineStatus(UART_APP) & UART_LSR_RDR){
        uint8_t b = Chip_UART_ReadByte(UART_APP);
        xQueueSendFromISR(qAppRxBytes, &b, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

/* UART1 ISR: push bytes to qSlvRxBytes */
void UART1_IRQHandler(void){
    portBASE_TYPE hpw = pdFALSE;
    while (Chip_UART_ReadLineStatus(UART_SLAVE) & UART_LSR_RDR){
        uint8_t b = Chip_UART_ReadByte(UART_SLAVE);
        xQueueSendFromISR(qSlvRxBytes, &b, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

/* Task: APP RX parser (UART0) */
static void Task_AppParser(void *arg){
    (void)arg;
    enum { RXF_WAIT_SOF=0, RXF_WAIT_LEN, RXF_COLLECT, RXF_WAIT_END } st = RXF_WAIT_SOF;
    uint8_t len = 0, idx = 0, buf[APP_RX_LEN_MAX];

    for (;;){
        uint8_t b;
        if (xQueueReceive(qAppRxBytes, &b, portMAX_DELAY) != pdTRUE) continue;

        switch (st){
        case RXF_WAIT_SOF:   if (b == SOF) st = RXF_WAIT_LEN; break;
        case RXF_WAIT_LEN:
            len = b;
            if (!len || len > APP_RX_LEN_MAX){ st = RXF_WAIT_SOF; break; }
            idx = 0; st = RXF_COLLECT; break;
        case RXF_COLLECT:
            buf[idx++] = b;
            if (idx == len) st = RXF_WAIT_END;
            break;
        case RXF_WAIT_END:
            if (b == END_BYTE){
                if (len >= 3){
                    uint8_t group = buf[0], id = buf[1], sc = buf[2];
                    const uint8_t *pay = (len > 3) ? &buf[3] : NULL;
                    uint8_t pal = (len > 3) ? (uint8_t)(len - 3) : 0;

                    if (group == GRP_APP_TO_RX && id == RX_ID){
                        switch (sc){
                            case SC_POLL:        break; /* HB below */
                            case SC_UPLOAD_MAP:  handle_upload_map(pay, pal);  break;
                            case SC_LED_CTRL:    handle_led_ctrl(pay, pal);    break;
                            case SC_LED_RESET:   handle_led_reset();           break;
                            case SC_RELAY_SET:   handle_relay_set(pay, pal);   break;
                            default: break;
                        }
                        /* Heartbeat for ANY command (coalesced) */
                        enqueue_status_frame_coalesced();
                    }
                }
            }
            st = RXF_WAIT_SOF;
            break;
        }
    }
}

/* Task: SLAVE RX parser (UART1)
   Expected: 27 27 03 0A <addr> <limit> 16 */
static void Task_SlvRxParser(void *arg){
    (void)arg;
    enum { SLP_WAIT_SOF=0, SLP_GOT_SOF2, SLP_WAIT_LEN, SLP_WAIT_CMD, SLP_WAIT_ADDR, SLP_WAIT_LIMIT, SLP_WAIT_END } st = SLP_WAIT_SOF;
    uint8_t addr=0, lim=0;

    for (;;){
        uint8_t b;
        if (xQueueReceive(qSlvRxBytes, &b, portMAX_DELAY) != pdTRUE) continue;

        switch (st){
        case SLP_WAIT_SOF:    if (b==0x27) st=SLP_GOT_SOF2; break;
        case SLP_GOT_SOF2:    st=SLP_WAIT_LEN; break;
        case SLP_WAIT_LEN:    st = (b==0x03) ? SLP_WAIT_CMD : SLP_WAIT_SOF; break;
        case SLP_WAIT_CMD:    st = (b==0x0A) ? SLP_WAIT_ADDR: SLP_WAIT_SOF; break;
        case SLP_WAIT_ADDR:   addr=b; st=SLP_WAIT_LIMIT; break;
        case SLP_WAIT_LIMIT:  lim=b;  st=SLP_WAIT_END;   break;
        case SLP_WAIT_END:
            if (b == END_BYTE){
                uint32_t now_ms = NOW_MS();
                uint8_t prev_lim;

                xSemaphoreTake(gStateMtx, portMAX_DELAY);
                alive_mark_seen_locked(addr, now_ms);
                prev_lim = (addr>=1 && addr<=31) ? limit_state[addr] : 0x01;
                if (addr>=1 && addr<=31) limit_state[addr] = lim;  /* 0x01 or 0x03 */
                xSemaphoreGive(gStateMtx);

                if (lim != prev_lim) {
                    /* GUARANTEE: replace any pending heartbeats with this updated one */
                    enqueue_status_frame_urgent_replace();
                }
            }
            st = SLP_WAIT_SOF;
            break;
        }
    }
}

/* Task: SLAVE TX (LED queue has priority over ping queue) */
static void Task_SlvTx(void *arg){
    (void)arg;
    slv_frame_t f;
    slv_led_cmd_t led;
    for (;;){
        bool have_led = false;
        while (xQueueReceive(qSlvLedCmd, &led, 0) == pdTRUE){
            have_led = true;
        }
        if (have_led){
            build_slave_led_off_all(&f);                  /* OFF-all first */
            uart_send_blocking(UART_SLAVE, f.bytes, f.len);
            if (!led.reset_all && led.state){
                build_slave_led_on(&f, led.con, led.led); /* then ON if requested */
                uart_send_blocking(UART_SLAVE, f.bytes, f.len);
            }
            continue;
        }

        if (xQueueReceive(qSlvPing, &f, 0) == pdTRUE){
            uart_send_blocking(UART_SLAVE, f.bytes, f.len);
            continue;
        }

        if (qSlvTxKick != NULL){
            (void)xSemaphoreTake(qSlvTxKick, portMAX_DELAY);
        } else {
            taskYIELD();
        }
    }
}

/* Task: APP TX (send STATUS frames) */
static void Task_AppTx(void *arg){
    (void)arg; app_frame_t f;
    for (;;){
        if (xQueueReceive(qAppTx, &f, portMAX_DELAY) == pdTRUE){
            uart_send_blocking(UART_APP, f.bytes, f.len);
        }
    }
}

/* ============================== Hardware init ================================ */
static void init_uart_app(void){
    Chip_UART_Init(UART_APP);
    Chip_UART_SetBaud(UART_APP, APP_BAUD);
    Chip_UART_ConfigData(UART_APP, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_APP, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_APP);
    Chip_UART_IntEnable(UART_APP, UART_IER_RBRINT | UART_IER_RLSINT);
    /* Priorities: must be numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
    NVIC_SetPriority(IRQ_APP, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3);
    NVIC_EnableIRQ(IRQ_APP);
}
static void init_uart_slave(void){
    Chip_UART_Init(UART_SLAVE);
    Chip_UART_SetBaud(UART_SLAVE, SLAVE_BAUD);
    Chip_UART_ConfigData(UART_SLAVE, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_SLAVE, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_SLAVE);
    Chip_UART_IntEnable(UART_SLAVE, UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_SLAVE, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    NVIC_EnableIRQ(IRQ_SLAVE);
}

/* ================================= main() ==================================== */
int main(void){
    SystemCoreClockUpdate();
    Board_Init();

    /* Mutex */
    gStateMtx = xSemaphoreCreateMutex();

    /* Queues */
    qAppRxBytes = xQueueCreate(256, sizeof(uint8_t));
    qSlvRxBytes = xQueueCreate(256, sizeof(uint8_t));
    qSlvLedCmd  = xQueueCreate(4, sizeof(slv_led_cmd_t));  /* LED commands */
    qSlvPing    = xQueueCreate(8, sizeof(slv_frame_t));    /* pings: low-priority */
    qAppTx      = xQueueCreate(6, sizeof(app_frame_t));    /* allow bursts */
    vSemaphoreCreateBinary(qSlvTxKick);   /* v7 API */
    configASSERT(qSlvTxKick);
    configASSERT(qSlvLedCmd);
    /* Make it initially empty (modern behavior) */
    xSemaphoreTake(qSlvTxKick, 0);

    /* Shared-state defaults */
    xSemaphoreTake(gStateMtx, portMAX_DELAY);
    memset(last_seen_ms, 0, sizeof(last_seen_ms));
    for (int i=0;i<32;++i) limit_state[i] = 0x01;
    recompute_alive_ttl_locked();
    xSemaphoreGive(gStateMtx);

    /* UARTs */
    init_uart_app();
    init_uart_slave();

    /* Start RIT for pre-map cadence (55 ms). It will be switched to 50 ms after map. */
    init_rit(PING_PREMAP_MS);

    /* Task priorities:
       - Make APP TX high so urgent limit-change frames go quickly.
       - Keep LED TX high for UART1, then RX parsers. (No pinger task now.) */
    xTaskCreate(Task_AppTx,       (signed char*)"appTx",   configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY+6), NULL);
    xTaskCreate(Task_SlvTx,       (signed char*)"slvTx",   configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY+5), NULL);
    xTaskCreate(Task_AppParser,   (signed char*)"appRx",   configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY+3), NULL);
    xTaskCreate(Task_SlvRxParser, (signed char*)"slvRx",   configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY+3), NULL);

    vTaskStartScheduler();
    for(;;){}
}

/* Board_Relay_Set(...) is provided in your board layer */
