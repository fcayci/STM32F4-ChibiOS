/* Basic PWM example for blinking an LED */

#include "ch.h"
#include "hal.h"

/* Only Channel 1 active */
static PWMConfig pwmCFG = {
  10000,								/* 10kHz PWM clock frequency  */
  10000,								/* PWM period (in ticks) 1S (1/10kHz=0.1mS 0.1ms*10000 ticks=1S) */
  NULL,									/* No Callback */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 0 */
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0  									/* HW dependent part.*/
};

int main(void) {

	halInit();
	chSysInit();

	/*
	 * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
	 * PAL_MODE_ALTERNATE is the value that you pass from Table 8. Alternate function mapping
	 * in DM00037051 - STM32F405xx/STM32F407xx Datasheet
	 */
	palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));

	pwmStart(&PWMD4, &pwmCFG);
	pwmEnableChannel(&PWMD4, 0, 5000); /* Enable channel 0 with 50% duty cycle (5000/10000=50%) */

	while (TRUE) {
    	chThdSleepMilliseconds(1000);
	}
}

