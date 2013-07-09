/* Basic USART example for printing */

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#define USART_CR1_9BIT_WORD	(1 << 12)   /* CR1 9 bit word */
#define USART_CR1_PARITY_SET	(1 << 10)	/* CR1 Parity bit enable */
#define USART_CR1_EVEN_PARITY	(0 << 9)   /* CR1 even parity */

static SerialConfig sdcfg = {
	115200,									/* 115200 baud rate */
	USART_CR1_9BIT_WORD | USART_CR1_PARITY_SET | USART_CR1_EVEN_PARITY,
	USART_CR2_STOP1_BITS | USART_CR2_LINEN,
	0
};

/* Periodic thread for reading data */
static WORKING_AREA(waRead, 512);
msg_t thReader(void *p) {
	(void)p;
/*
  EventListener s3EventListener;
  eventmask_t flag;

  chEvtRegisterMask(chnGetEventSource(&SD3), &s3EventListener, 1);

  while (TRUE) {
	chEvtWaitOne(1);
	chSysLock();
	flag = chEvtGetAndClearFlagsI(&s3EventListener);
	chSysUnlock();
	chprintf((BaseSequentialStream *)&SD3, "Serial Event %d\r\n",(uint16_t)flag);

  }
 */
	while (TRUE){
		uint8_t c = chnGetTimeout((BaseChannel *)&SD3, TIME_INFINITE);
		chprintf((BaseSequentialStream *)&SD3, "%s\r\n",c);
	}
}

int main(void) {

	halInit();
	chSysInit();

	sdStart(&SD3, &sdcfg);

	palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(7));

	chThdCreateStatic(waRead, sizeof(waRead), NORMALPRIO, thReader, NULL);

	chprintf((BaseSequentialStream *)&SD3, "Press a key...\r\n");

	while (TRUE){
		chThdSleepMilliseconds(500); /* Sleep the processor for 500 milliseconds */
	}
}
