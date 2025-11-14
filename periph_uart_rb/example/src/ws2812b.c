#include "ws2812b.h"
#include "chip.h"
#include "board.h"

/*------------------------------------------------------------------------------
 * Low-level WS2812B bit-banging functions (timing critical)
 *----------------------------------------------------------------------------*/

/**
 * @brief Send a logic '0' to WS2812B.
 * Timing: T0H ≈ 400 ns, T0L ≈ 850 ns
 */
static inline void send_zero(void) {

	LPC_GPIO[3].SET = LED_MASK;

    /* High ~420 ns (16 NOPs at 100 MHz) */
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

    LPC_GPIO[3].CLR = LED_MASK;

    /* Low ~800 ns (25 NOPs at 100 MHz) */
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();
}

/**
 * @brief Send a logic '1' to WS2812B.
 * Timing: T1H ≈ 800 ns, T1L ≈ 450 ns
 */
static inline void send_one(void) {

	 LPC_GPIO[3].SET = LED_MASK;

    /* High ~800 ns (36 NOPs at 100 MHz) */
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

    LPC_GPIO[3].CLR = LED_MASK;

    /* Low ~420 ns (8 NOPs at 100 MHz) */
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/**
 * @brief Turn on a given LED with RED color (R=255, G=0, B=0).
 * @param ws2812b Pointer to LED strip structure
 * @param bin_led_num LED index (1-based)
 */
void turnOnRed(WS2812B* ws2812b, int bin_led_num) {
    if (bin_led_num <= 0 || bin_led_num > ws2812b->num_leds) {
        return; // invalid index
    }
    ws2812b->leds[bin_led_num - 1].r = 255;
    ws2812b->leds[bin_led_num - 1].g = 0;
    ws2812b->leds[bin_led_num - 1].b = 0;
}

/**
 * @brief Write the full LED buffer to WS2812B strip.
 * Data is sent in GRB order per WS2812B protocol.
 */
void WS2812B_write(WS2812B* ws2812b) {
    for (uint16_t i = 0; i < ws2812b->num_leds; i++) {
        RGB_t led = ws2812b->leds[i];

        /* Send Green */
        for (uint8_t bit = 0; bit < 8; bit++) {
            (led.g & 0x80) ? send_one() : send_zero();
            led.g <<= 1;
        }

        /* Send Red */
        for (uint8_t bit = 0; bit < 8; bit++) {
            (led.r & 0x80) ? send_one() : send_zero();
            led.r <<= 1;
        }

        /* Send Blue */
        for (uint8_t bit = 0; bit < 8; bit++) {
            (led.b & 0x80) ? send_one() : send_zero();
            led.b <<= 1;
        }
    }

    /* Reset pulse (>50 µs). Approximate using raw NOPs */
    for (uint16_t i = 0; i < 600; i++) {
        __NOP(); __NOP();
    }
}
