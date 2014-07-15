/*
 * A Simple PWM Example for blinking LED.
 */

#include "ch.h"
#include "hal.h"

static PWMConfig pwmCFG = {
    10000,                              /* 10kHz PWM clock frequency  */
    10000,                              /* PWM period (in ticks) 1S (1/10kHz=0.1mS 0.1ms*10000 ticks=1S) */
    NULL,                               /* No Callback */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Enable Channel 0 */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Enable Channel 1 */
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL}
    },
    0,                                  /* HW dependent part.*/
    0
};

int main(void) {

    halInit();
    chSysInit();

    /*
     * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
     * Check the discovery board schematics to see what pins are connected to the LEDs.
     * PAL_MODE_ALTERNATE is the value that you pass from Table 9. Alternate function mapping
     * in DM00037051 - STM32F405xx/STM32F407xx Datasheet
     */

    palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));

    pwmStart(&PWMD4, &pwmCFG);
    pwmEnableChannel(&PWMD4, 0, 5000); /* Enable channel 0 with 50% duty cycle (5000/10000=50%) */
    pwmEnableChannel(&PWMD4, 1, 5000); /* Enable channel 1 with 50% duty cycle (5000/10000=50%) */

    while (TRUE) {
        chThdSleepMilliseconds(1000);
    }
    return 0;
}