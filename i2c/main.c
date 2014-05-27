/* 
* MPU6050 Breakout board attached to STM32F4 Discovery board
* MPU6050 SDA,SCL: PB6,PB7
* UART TX,RX: PA2,PA3
* Needs testing
*/


#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "chprintf.h"
#include "math.h"

#define RAD_TO_DEG 57.2957786

int16_t XaccelRaw, YaccelRaw, ZaccelRaw;
int16_t XgyroRaw, YgyroRaw, ZgyroRaw;

double XaccelAngle, YaccelAngle;

static bool_t initSensors(void);
static void getSensorData(void);

static SerialConfig sd2cfg = {
   115200,									/* 115200 baud rate */
   0,
   USART_CR2_STOP1_BITS | USART_CR2_LINEN,
   0
};


/* Configure I2C for sensors */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2
};

/* Configure Heartbeat */
static WORKING_AREA(wahbeat, 128);

static msg_t thBlinker(void *arg){
	(void)arg;
	chRegSetThreadName("blinker");
	while (TRUE){
    	palTogglePad(GPIOD, GPIOD_LED3); /* Orange */
		chThdSleepMilliseconds(200);
	}
	return 0;
}

int main(void) {

	halInit();
	chSysInit();

	/* Create the heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	sdStart(&SD2, &sd2cfg);
	// sdStart(&SD2, NULL); /* for default configuration */
	palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

	initSensors();

	while (TRUE){
		MPUgetMotion6(&XaccelRaw, &YaccelRaw, &ZaccelRaw, &XgyroRaw, &YgyroRaw, &ZgyroRaw);
		XaccelAngle = (atan2(YaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;
		YaccelAngle = (atan2(XaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;
		chprintf((BaseSequentialStream *)&SD2, "AX: %5d\tAY: %5d\t,AZ: %5d\t,GX: %5d\t,GY: %5d\t,GZ: %5d\tXA: %5f\t,YA: %5f\t\r\n",XaccelRaw,YaccelRaw,ZaccelRaw,XgyroRaw,YgyroRaw,ZgyroRaw,XaccelAngle,YaccelAngle);
		chThdSleepMilliseconds(2);
	}
}

static bool_t initSensors(void){
	bool_t sts = 0;

	/* Disable these for discovery board,
	 * don't think it will be problem on the quad tho */
	//palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(0));
	palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(0));

	i2cStart(&I2CD1, &i2cfg1);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

	chThdSleepMilliseconds(100);

	/* MPU6050, AD0 is connected to VCC */
	MPU6050(MPU6050_ADDRESS_AD0_LOW);

	/* Test connection */
	sts = MPUtestConnection();
	if (!sts) return FALSE;

	MPUreset();
	MPUresetSensors();
	chThdSleepMilliseconds(100);
	MPUinitialize();
	chThdSleepMilliseconds(100);

	MPUgetMotion6(&XaccelRaw, &YaccelRaw, &ZaccelRaw, &XgyroRaw, &YgyroRaw, &ZgyroRaw);

	XaccelAngle = (atan2(YaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;
	YaccelAngle = (atan2(XaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;

	return TRUE;
}

