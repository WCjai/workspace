/**
 * @file ws2812b.h
 * @brief Header file for WS2812B LED strip driver.
 *
 * Provides data structures and function prototypes for
 * controlling WS2812B LEDs with LPC microcontrollers.
 *
 * Author: Aadithya (Phoenix Automation)
 */

#ifndef INC_WS2812B_H
#define INC_WS2812B_H

#include <stdint.h>

/*------------------------------------------------------------------------------
 * Data Structures
 *----------------------------------------------------------------------------*/

/**
 * @brief RGB color structure.
 */
typedef struct {
    uint8_t r;   /**< Red channel   (0–255) */
    uint8_t g;   /**< Green channel (0–255) */
    uint8_t b;   /**< Blue channel  (0–255) */
} RGB_t;

/**
 * @brief LED mask to set and clear P3[25] an P3[26]
 */
#define LED_MASK   ((1U << 25) | (1U << 26))

/**
 * @brief WS2812B LED strip object.
 */
typedef struct {
    RGB_t* leds;     /**< Pointer to LED color buffer */
    uint8_t num_leds;/**< Number of LEDs in the strip */
} WS2812B;

/*------------------------------------------------------------------------------
 * Public API Function Prototypes
 *----------------------------------------------------------------------------*/

/**
 * @brief Write LED buffer to WS2812B strip.
 *
 * @param ws2812b Pointer to WS2812B strip object.
 */
void WS2812B_write(WS2812B* ws2812b);

/**
 * @brief Set a given LED to RED (R=255, G=0, B=0).
 *
 * @param ws2812b     Pointer to WS2812B strip object.
 * @param bin_led_num LED index (1-based).
 */
void turnOnRed(WS2812B* ws2812b, int bin_led_num);

#endif /* INC_WS2812B_H */
