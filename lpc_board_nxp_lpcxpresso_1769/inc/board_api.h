#ifndef __BOARD_API_H_
#define __BOARD_API_H_

#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------- Board bootstrap (what we actually use) -------- */

/**
 * @brief Setup and initialize hardware prior to main()
 * - Calls Board_SetupMuxing() and Board_SetupClocking()
 */
void Board_SystemInit(void);

/** @brief Setup pin multiplexer per board schematics */
void Board_SetupMuxing(void);

/** @brief Setup system clocking */
void Board_SetupClocking(void);

/** @brief Bring-up used peripherals (GPIO/IOCON + relays) */
void Board_Init(void);

/* -------- DEBUG macros intentionally disabled --------
   No printf redirection, no semihosting. These macros compile away.
*/
#define DEBUGINIT()
#define DEBUGOUT(...)
#define DEBUGSTR(str)
#define DEBUGIN() (int)EOF

#ifdef __cplusplus
}
#endif
#endif /* __BOARD_API_H_ */
