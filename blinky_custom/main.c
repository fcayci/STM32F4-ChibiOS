/*
 * Basic blink LED example running on
 * STM32F405RG based custom board
 *
 * External clock (HSE) is disabled by default.
 * Running on internal clock (HSI) at 16MHz.
 * To turn it on open mcuconf.h and change
 * #define STM32_HSE_ENABLED line to TRUE
 * and #define STM32_PLLSRC line to
 * STM32_PLLSRC_HSE
 *
 * Since the custom board also has a 16MHz external
 * clock the STM32_PLLM_VALUE should stay at 16
 */

#include "ch.h"
#include "hal.h"

int main(void) {

	/*
	 * ChibiOS/RT stuff
	 */
	halInit();
	chSysInit();

	/*
	 * Set PD13 as output pin
	 */
	//palSetPadMode(GPIOD, GPIOD_PIN13, PAL_MODE_OUTPUT_PUSHPULL);  /* Orange */

	/* Infinite Loop */
	while (TRUE) {

		/* Toggle the output */
    	palTogglePad(GPIOD, GPIOD_PIN13); /* Orange */

    	/* Sleep for 500 milliseconds */
    	chThdSleepMilliseconds(500);
	}
}
