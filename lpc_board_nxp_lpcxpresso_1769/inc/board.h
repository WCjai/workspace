#ifndef __BOARD_H_
#define __BOARD_H_

#include "chip.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Board tag (not used functionally but kept for consistency) */
#define BOARD_NXP_LPCXPRESSO_1769
#define USE_RMII  /* harmless keep-alive; not used here */

/* ---------- Relay helpers (the only board features we export) ---------- */
void Board_Relays_Init(void);
void Board_Relay_Set(uint8_t relay, bool on);

/* ---------- System init (declared here for convenience) ---------- */
void Board_SystemInit(void);
void Board_SetupMuxing(void);
void Board_SetupClocking(void);
void Board_Init(void);

/* NOTE:
   - No LED, buttons, LCD, I2C/SPI/SSP/USB, or debug-UART APIs exposed.
   - UART is configured directly in main.c; we do NOT declare Board_UART_Init().
*/

#include "board_api.h"

#ifdef __cplusplus
}
#endif
#endif /* __BOARD_H_ */
