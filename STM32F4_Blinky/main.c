/* Basic blink LED example */

#include "ch.h"
#include "hal.h"

int main(void) {

	halInit();
	chSysInit();

	//palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_OUTPUT_PUSHPULL);  /* Orange */

	while (TRUE) {
    	palTogglePad(GPIOD, GPIOD_LED3); /* Orange */
    	chThdSleepMilliseconds(500);
	}
}
