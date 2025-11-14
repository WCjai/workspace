#include "board.h"

/* UART0 on P0.2/P0.3, UART1 on P2.0/P2.1, relay GPIOs */
STATIC const PINMUX_GRP_T pinmuxing[] = {
    {0,  2, IOCON_MODE_INACT | IOCON_FUNC1}, /* TXD0 */
    {0,  3, IOCON_MODE_INACT | IOCON_FUNC1}, /* RXD0 */

    {2,  0, IOCON_MODE_INACT | IOCON_FUNC2}, /* TXD1 (verify FUNC for your MCU) */
    {2,  1, IOCON_MODE_INACT | IOCON_FUNC2}, /* RXD1 */
	{2,  8, IOCON_MODE_INACT | IOCON_FUNC2}, /* TXD2 on P2.8 */
	{2,  9, IOCON_MODE_INACT | IOCON_FUNC2}, /* RXD2 on P2.9 */

    {4, 28, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 1 */
    {0,  4, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 2 */
    {0,  5, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 3 */
    {0,  6, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 4 */
    {0,  7, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 5 */
    {0,  8, IOCON_MODE_INACT | IOCON_FUNC0}, /* Relay 6 */

	{3, 26, IOCON_FUNC0 | IOCON_MODE_INACT}, /* addr LED1 */
	{3, 25, IOCON_FUNC0 | IOCON_MODE_INACT}, /* addr LED2 */

	{2, 4, IOCON_FUNC0 | IOCON_MODE_PULLUP}, /* sw1 */
    {2, 3, IOCON_FUNC0 | IOCON_MODE_PULLUP}, /* sw2 */
};

void Board_SetupMuxing(void) {
    Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing,
                            sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));
}

void Board_SetupClocking(void) {
    Chip_SetupXtalClocking();
    Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);
    Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER1, SYSCTL_CLKDIV_1);
    Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER2, SYSCTL_CLKDIV_1);
    Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_UART0, SYSCTL_CLKDIV_1);
    Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_UART1, SYSCTL_CLKDIV_1);
    Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_UART2, SYSCTL_CLKDIV_1);
}

void Board_SystemInit(void) {
    Board_SetupMuxing();
    Board_SetupClocking();
}
