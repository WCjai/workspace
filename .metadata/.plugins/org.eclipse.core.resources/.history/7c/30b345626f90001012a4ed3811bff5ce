#include "chip.h"
#include "board.h"

/* UART selection */
#define UART_SELECTION  LPC_UART0
#define IRQ_SELECTION   UART0_IRQn
#define HANDLER_NAME    UART0_IRQHandler

/* --- Command parser state --- */
static volatile uint8_t st   = 0;
static volatile uint8_t rel  = 0;
static volatile uint8_t val  = 0;


/* --- need to change this command parser later once we able to differenciate between rely commands and heart beat --- */
static inline void feed_parser(uint8_t b)
{
    switch (st) {
    case 0: st = (b == 0x27) ? 1 : 0; break;    /* if 27 jump to next byte and soo on till 5 and 6 th bit recored  */
    case 1: st = (b == 0x05) ? 2 : (b == 0x27 ? 1 : 0); break;
    case 2: st = (b == 0x85) ? 3 : 0; break;
    case 3: st = (b == 0x01) ? 4 : 0; break;
    case 4: st = (b == 0x06) ? 5 : 0; break;
    case 5: rel = b; st = 6; break;
    case 6: val = b; st = 7; break;
    case 7:
        if (b == 0x16) {
            /* Terminal byte OK -> apply relay */
            Board_Relay_Set(rel, (val == 0x01));
        }
        st = 0;
        break;
    default: st = 0; break;
    }
}

/* UART IRQ: read all pending RX bytes and feed the parser */
void HANDLER_NAME(void)
{
    uint32_t lsr;
    while (((lsr = Chip_UART_ReadLineStatus(UART_SELECTION)) & UART_LSR_RDR) != 0) {
        uint8_t b = Chip_UART_ReadByte(UART_SELECTION);
        feed_parser(b);

        // any new parser add here
    }
    /* Clear any interrupt sources if needed (LPCOpen handler usually does this). */
}

int main(void)
{
    SystemCoreClockUpdate();
    Board_Init();

    /* UART0: 19,200 8N1 */
    Chip_UART_Init(UART_SELECTION);
    Chip_UART_SetBaud(UART_SELECTION, 19200);
    Chip_UART_ConfigData(UART_SELECTION, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    Chip_UART_SetupFIFOS(UART_SELECTION, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2);
    Chip_UART_TXEnable(UART_SELECTION);

    /* Enable RX interrupt only (we don't transmit anything) */
    Chip_UART_IntEnable(UART_SELECTION, UART_IER_RBRINT | UART_IER_RLSINT);
    NVIC_SetPriority(IRQ_SELECTION, 1);
    NVIC_EnableIRQ(IRQ_SELECTION);

    /* Idle forever; all work happens in the UART ISR */
    while (1) {
        __WFI(); /* sleep until next interrupt */
    }
}
