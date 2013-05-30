/*
 * DragonFly Quadcopter ChibiOS-based firmware
 *
 *		File	: main.c
 * 		Author	: Furkan Cayci
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
 *
 * Sensor Output:
 * 		N:+X
 * E:+Y		W:-Y
 * 		S:-X
 *
 */

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "usbcfg.h"

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define TEST_WA_SIZE    THD_WA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state], (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}


static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

/* Configure Heartbeat */
static WORKING_AREA(wahbeat, 128);

static msg_t thBlinker(void *arg){
	(void)arg;
	palSetPadMode(GPIOC, GPIOC_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOC, GPIOC_LED_BLUE, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(GPIOC, GPIOC_LED_BLUE);
	chRegSetThreadName("blinker");
	while (TRUE){
		palTogglePad(GPIOC, GPIOC_LED_GREEN);
    	palTogglePad(GPIOC, GPIOC_LED_BLUE);
		chThdSleepMilliseconds(1000);
	}
	return 0;
}

int main(void) {
	Thread *shelltp = NULL;
	halInit();
	chSysInit();

	/* Create the heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	  /*
	   * Shell manager initialization.
	   */
	  shellInit();

	  /*
	   * Initializes a serial-over-USB CDC driver.
	   */
	  sduObjectInit(&SDU1);
	  sduStart(&SDU1, &serusbcfg);

	  /*
	   * Activates the USB driver and then the USB bus pull-up on D+.
	   * Note, a delay is inserted in order to not have to disconnect the cable
	   * after a reset.
	   */
	  usbDisconnectBus(serusbcfg.usbp);
	  chThdSleepMilliseconds(1000);
	  usbStart(serusbcfg.usbp, &usbcfg);
	  usbConnectBus(serusbcfg.usbp);

	while (TRUE){
		  if (!shelltp) {
		      if (SDU1.config->usbp->state == USB_ACTIVE) {
		        /* Spawns a new shell.*/
		        shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		      }
		    }
		    else {
		      /* If the previous shell exited.*/
		      if (chThdTerminated(shelltp)) {
		        /* Recovers memory of the previous shell.*/
		        chThdRelease(shelltp);
		        shelltp = NULL;
		      }
		    }
		chThdSleepMilliseconds(500);
	}
}
