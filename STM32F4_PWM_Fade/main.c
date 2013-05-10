/* Basic PWM example for fading LEDs */

#include "ch.h"
#include "hal.h"

/* Only Channel 1 active */
static PWMConfig pwmCFG = {
  1000000,								/* 1MHz PWM clock frequency  */
  10000,								/* PWM period (in ticks) 10mS (1/1MHz=1uS, 1us*10000 ticks=10mS) */
  NULL,									/* No Callback */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 0 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 1 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 2 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}		/* Enable Channel 3 */
  },
  0  									/* HW dependent part */
};

int main(void) {
	static uint16_t dir = 0, width = 0;

	halInit();
	chSysInit();

	palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));  /* Green */
	palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));  /* Orange */
	palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_ALTERNATE(2));  /* Red */
	palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));  /* Blue */

	pwmStart(&PWMD4, &pwmCFG);

	while (TRUE){
		pwmEnableChannel(&PWMD4, 0, width);
		pwmEnableChannel(&PWMD4, 1, 10000-width);
		pwmEnableChannel(&PWMD4, 2, width);
		pwmEnableChannel(&PWMD4, 3, 10000-width);

		if (dir == 0) {width = (width + 100);}
		else {width = (width - 100);}

		if (width >= 10000) {dir = 1;}
		else if (width == 0) {dir = 0;}
		chThdSleepMilliseconds(10); /* Sleep the processor for 10 milliseconds */
	}
}

