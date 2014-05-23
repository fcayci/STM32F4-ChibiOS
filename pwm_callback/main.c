/*
 * A Simple PWM example for fading LEDs with callback
 */

#include "ch.h"
#include "hal.h"

static void pwmcb(PWMDriver *pwmp);

/* Only Channel 1 active */
static PWMConfig pwmCFG = {
  1000000,								/* 1MHz PWM clock frequency  */
  10000,								/* PWM period (in ticks) 10mS (1/1MHz=1uS, 1us*10000 ticks=10mS) */
  pwmcb,								/* Callback */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 0 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 1 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 2 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}		/* Enable Channel 3 */
  },
  0,  									/* HW dependent part */
  0
};

/*
 * PWM callback.
 * Each time calculate the next duty cycle.
 */
static void pwmcb(PWMDriver *pwmp) {
	(void)pwmp;

	static uint16_t dir = 0, width = 0;

	if (dir == 0) {width = (width + 100);}
	else {width = (width - 100);}

	if (width >= 10000) {dir = 1;}
	else if (width == 0) {dir = 0;}

	pwmEnableChannel(&PWMD4, 0, width);
	pwmEnableChannel(&PWMD4, 1, 10000-width);
	pwmEnableChannel(&PWMD4, 2, width);
	pwmEnableChannel(&PWMD4, 3, 10000-width);
}

int main(void) {

	halInit();
	chSysInit();

	palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));  /* Green */
	palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));  /* Orange */
	palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_ALTERNATE(2));  /* Red */
	palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));  /* Blue */

	pwmStart(&PWMD4, &pwmCFG);

	while (TRUE){
		chThdSleepMilliseconds(500); /* Sleep the processor for 500 milliseconds */
	}
}

