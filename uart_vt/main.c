/* 
 * A virtual timer example using UART module
 * 115200 Baud - 8N1
 * PC10 - TX (transmits data from uC)
 * PC11 - RX (receives data from PC)
 */

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#define USART_CR1_9BIT_WORD     (1 << 12)   /* CR1 9 bit word */
#define USART_CR1_PARITY_SET    (1 << 10)   /* CR1 Parity bit enable */
#define USART_CR1_EVEN_PARITY   (0 << 9)    /* CR1 even parity */

static VirtualTimer vt3, vt4, vt5;

static const uint8_t message[] = "0123456789ABCD\r\n";
static uint8_t buffer[16];

static void led3off(void *p) {
    (void)p;

    palClearPad(GPIOD, GPIOD_LED3);
}

static void led4off(void *p) {
    (void)p;

    palClearPad(GPIOD, GPIOD_LED4);
}

static void led5off(void *p) {
    (void)p;

    palClearPad(GPIOD, GPIOD_LED5);
}

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
    (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
    (void)uartp;

    palSetPad(GPIOD, GPIOD_LED5); /* Red */
    chSysLockFromIsr();
    if (chVTIsArmedI(&vt5)) chVTResetI(&vt5);
    chVTSetI(&vt5, MS2ST(200), led5off, NULL);
    chSysUnlockFromIsr();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
    (void)uartp;
    (void)e;

    palSetPad(GPIOD, GPIOD_LED6); /* Blue */
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
    (void)uartp;
    (void)c;

    /* Flashing the LED each time a character is received.*/
    palSetPad(GPIOD, GPIOD_LED4); /* Green */
    chSysLockFromIsr();
    if (chVTIsArmedI(&vt4)) chVTResetI(&vt4);
    chVTSetI(&vt4, MS2ST(200), led4off, NULL);
    chSysUnlockFromIsr();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
    (void)uartp;

    /* When the buffer is full, flush it and restart listening */
    uartStopReceive(&UARTD3);
    uartStartSend(&UARTD3, 16, buffer);
    uartStartReceive(&UARTD3, 16, buffer);
    palSetPad(GPIOD, GPIOD_LED3); /* Orange */
    chSysLockFromIsr();
    if (chVTIsArmedI(&vt3)) chVTResetI(&vt3);
    chVTSetI(&vt3, MS2ST(200), led3off, NULL);
    chSysUnlockFromIsr();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uartcfg = {
    txend1,                 /* End of transmission callback */
    txend2,                 /* Physical end of transmission callback */
    rxend,                  /* Receive buffer filled callback */
    rxchar,                 /* Character received while out */
    rxerr,                  /* Receive error callback */
    115200,
    USART_CR1_9BIT_WORD | USART_CR1_PARITY_SET | USART_CR1_EVEN_PARITY,
    USART_CR2_LINEN,
    0
};

int main(void) {

    halInit();
    chSysInit();

    /*
    * PAL_MODE_ALTERNATE is the value that you pass from Table 9. Alternate function mapping
    * in DM00037051 - STM32F405xx/STM32F407xx Datasheet
    */
    uartStart(&UARTD3, &uartcfg);

    palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(7));

    uartStopReceive(&UARTD3);
    uartStopSend(&UARTD3);
    uartStartReceive(&UARTD3, 16, buffer);
    uartStartSend(&UARTD3, 16, message);

    while (TRUE){
	    chThdSleepMilliseconds(500); /* Sleep the processor for 500 milliseconds */
	}
}