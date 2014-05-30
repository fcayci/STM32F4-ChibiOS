/*
 * A simple static thread example
 * Each thread is used for blinking different
 * leds running at different speed
 */

#include "ch.h"
#include "hal.h"

/* Periodic thread 1 */
static WORKING_AREA(wablink1, 128);
static msg_t blink1(void *arg){
    (void)arg;
    chRegSetThreadName("blink1");
    while (TRUE){
        palTogglePad(GPIOD, GPIOD_LED4);
        chThdSleepMilliseconds(1000);
    }
    return 0;
}

/* Periodic thread 2 */
static WORKING_AREA(wablink2, 128);
static msg_t blink2(void *arg){
    (void)arg;
    chRegSetThreadName("blink2");
    while (TRUE){
        palTogglePad(GPIOD, GPIOD_LED5);
        chThdSleepMilliseconds(500);
    }
    return 0;
}

/* Periodic thread 3 */
static WORKING_AREA(wablink3, 128);
static msg_t blink3(void *arg){
    (void)arg;
    chRegSetThreadName("blink3");
    while (TRUE){
        palTogglePad(GPIOD, GPIOD_LED6);
        chThdSleepMilliseconds(250);
    }
    return 0;
}

int main(void) {

    halInit();
    chSysInit();

    palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_OUTPUT_PUSHPULL);

    /* Create 3 threads for blinking each led */
    chThdCreateStatic(wablink1, sizeof(wablink1), NORMALPRIO, blink1, NULL);
    chThdCreateStatic(wablink2, sizeof(wablink2), NORMALPRIO, blink2, NULL);
    chThdCreateStatic(wablink3, sizeof(wablink3), NORMALPRIO, blink3, NULL);

    /* Main thread blinks the last led*/
    while (TRUE) {
        palTogglePad(GPIOD, GPIOD_LED3);
        chThdSleepMilliseconds(2000);
    }

    return 0;
}
