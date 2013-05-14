/*
 * DragonFly Quadcopter ChibiOS-based firmware
 *
 * 		Furkan Cayci
 *
 */

/*
 * Features:
 *
 * External Oscillator (HSE)	: 16MHz
 * MPU6050 Gyro + Accel + (magn): I2C1 (PB6,PB7)
 * XBee Wireless Tranciever		: USART1 (PA9,PA10)
 * 4 x PWM Motors				: TIM3CH1(PC6), TIM3CH2(PC7), TIM2CH3(PB10), TIM2CH4(PB11)
 * 2 x LEDs						: Green LED (PC4), Blue LED (PC5)
 * Battery Check				: AIN5 (PA5)
 */

#include "ch.h"
#include "hal.h"

/* Heartbeat */
static WORKING_AREA(wahbeat, 32);

static msg_t thBlinker(void *arg){
	(void)arg;
	palSetPadMode(GPIOC, GPIOC_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);  /* Green */
	chRegSetThreadName("blinker");
	while (TRUE){
    	palTogglePad(GPIOC, GPIOC_LED_GREEN);
		chThdSleepMilliseconds(500);
	}
	return 0;
}


int main(void) {

	halInit();
	chSysInit();

	/* Create heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	while (TRUE) {
    	chThdSleepMilliseconds(500);
	}
}
