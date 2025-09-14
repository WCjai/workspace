#include "board.h"

/* Pin muxing: UART0 on P0.2/P0.3, UART1 on P2.0/P2.1, relay GPIOs on func 0 */
STATIC const PINMUX_GRP_T pinmuxing[] = {
    {0,  2, IOCON_MODE_INACT | IOCON_FUNC1}, /* TXD0 */
    {0,  3, IOCON_MODE_INACT | IOCON_FUNC1}, /* RXD0 */

    {2,  0, IOCON_MODE_INACT | IOCON_FUNC2}, /* TXD1 (verify FUNC on your board) */
    {2,  1, IOCON_MODE_INACT | IOCON_FUNC2}, /* RXD1 (verify FUNC on your board) */

    {4, 28, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 1 */
    {0,  4, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 2 */
    {0,  5, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 3 */
    {0,  6, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 4 */
    {0,  7, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 5 */
    {0,  8, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 6 */
};

void Board_SetupMuxing(void) {
    Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing,
                            sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));
}

void Board_SetupClocking(void) {
    Chip_SetupXtalClocking();
    Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU); /* 100 MHz */
}

void Board_SystemInit(void) {
    Board_SetupMuxing();
    Board_SetupClocking();
}
