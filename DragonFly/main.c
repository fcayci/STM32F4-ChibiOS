/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

/* Serial port for debug output */
#define DEBUGPORT SD2

static SerialConfig sd2cfg = {
   115200,									/* 115200 baud rate */
   0,
   USART_CR2_STOP1_BITS | USART_CR2_LINEN,
   0
};

/* I2C config for MPU6050 */
static I2CConfig i2c1cfg = {
	OPMODE_I2C,
	400000,
	FAST_DUTY_CYCLE_16_9
};

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOD, GPIOD_LED3);     /* Orange.  */
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {
   static uint16_t counter=0;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Activate the debug port for serial messages*/
  sdStart(&DEBUGPORT, &sd2cfg);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chprintf((BaseSequentialStream *)&DEBUGPORT, "\r\nInitializing...\r\n") ;
  //sdWrite(&DEBUGPORT, (uint8_t *)"Initializating!\r\n", 17);

  while (TRUE) {
    //if (palReadPad(GPIOA, GPIOA_BUTTON))
    //	sdWrite(&DEBUGPORT, (uint8_t *)"Button Pressed!\r\n", 17);

    chprintf((BaseSequentialStream *)&DEBUGPORT, "Sending...\n\r") ;
    chThdSleepMilliseconds(500);
  }
}
