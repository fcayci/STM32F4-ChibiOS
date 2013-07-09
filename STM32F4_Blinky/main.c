/*
 * Basic blink LED example
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
	//palSetPadMode(GPIOD, GPIO_LED3, PAL_MODE_OUTPUT_PUSHPULL);  /* Orange */

	/* Infinite Loop */
	while (TRUE) {

		/* Toggle the output */
    	palTogglePad(GPIOD, GPIOD_LED3); /* Orange */

    	/* Sleep for 500 milliseconds */
    	chThdSleepMilliseconds(500);
	}
}
