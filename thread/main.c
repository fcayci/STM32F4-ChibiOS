/* Basic thread example */

#include "ch.h"
#include "hal.h"

/* Periodic thread for led blinking */
static WORKING_AREA(wablink, 32);
static msg_t thBlinker(void *arg){
	(void)arg;
	//palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_OUTPUT_PUSHPULL);  /* Green */
	chRegSetThreadName("blinker");
	while (TRUE){
		palTogglePad(GPIOD, GPIOD_LED4); /* Green */
		chThdSleepMilliseconds(500);
	}
	return 0;
}

int main(void) {

	halInit();
	chSysInit();

	/* Create the thread */
	chThdCreateStatic(wablink, sizeof(wablink), NORMALPRIO, thBlinker, NULL);

	palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_OUTPUT_PUSHPULL);  /* Orange */

	while (TRUE) {
    	palTogglePad(GPIOD, GPIOD_LED3); /* Orange */
    	chThdSleepMilliseconds(1000);
	}
}
