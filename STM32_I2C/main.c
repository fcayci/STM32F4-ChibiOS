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

void InitSensors(void);
void InitMotors(void);
void InitXBee(void);
void InitDebug(void);
void InitBatteryCheck(void);

static SerialConfig sd2cfg = {
   115200,									/* 230400 baud rate */
   0,
   USART_CR2_STOP1_BITS | USART_CR2_LINEN,
   0
};

/*Config for I2C1 */
static I2CConfig i2c1cfg = {
	OPMODE_I2C,
	400000,
	FAST_DUTY_CYCLE_16_9
};

/*
 * Flash an LED to show it is running
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

void InitDebug(void){

  /* Enable SD1 for ZIGBEE */
  //palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7)) ;
  //palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7)) ;
  //sdStart(&SD1, &sdcfg) ;

  /* Enable SD2 for Debugging
   * TODO: Move this to the external USART on the board
   */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)) ;
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7)) ;
  sdStart(&SD2, &sd2cfg) ;
}

void InitSensors(void){
  palSetPadMode(GPIOB, 6,  PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN) ;
  palSetPadMode(GPIOB, 7,  PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN) ;
  i2cStart(&I2CD3, &i2c1cfg) ;

}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, &sd2cfg);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (TRUE) {
    if (palReadPad(GPIOA, GPIOA_BUTTON))
    	sdWrite(&SD2, (uint8_t *)"Button Pressed!\r\n", 17);
    sdWrite(&SD2, (uint8_t *)"Hello World!\r\n", 14);
    chThdSleepMilliseconds(500);
  }
}
