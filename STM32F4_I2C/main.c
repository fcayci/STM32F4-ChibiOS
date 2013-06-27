/*
 *
 *		File	: main.c
 *		Author	: Furkan Cayci
 *
 */


#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "chprintf.h"
#include "math.h"

/*
 * PWM definitions for the motors. 500 might be increased to 512 to make it 2^9
 */
#define PWM_CLOCK_FREQUENCY 50000000
#define PWM_PERIOD_IN_TICKS 500
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY/PWM_PERIOD_IN_TICKS)

#define MOTOR_EAST			0
#define MOTOR_WEST			1
#define MOTOR_NORTH			2
#define MOTOR_SOUTH			3

#define RAD_TO_DEG 57.2957786

#define DT 				0.002		// Expressed in seconds (for the PID)
#define COMP			0.93		// Variable for complementary filter

#define KP	1		// KP
#define	KI	(1 * DT)	// KI * DT = 1 * 0.002
#define KD	0		// KD / DT = 0 / 0.002

int16_t XaccelRaw, YaccelRaw, ZaccelRaw;
int16_t XgyroRaw, YgyroRaw, ZgyroRaw;

double XaccelAngle, YaccelAngle;
double XgyroRate, YgyroRate;
double XcompAngle, YcompAngle;

static double errorX, integralX, derivativeX, outputX;
static double errorY, integralY, derivativeY, outputY;

static uint16_t mEastSpeed, mWestSpeed, mNorthSpeed, mSouthSpeed;

static bool_t isi2c;

static void initMotors(void);
static void testMotors(void);
static void setMotorSpeed(uint8_t motor_t, uint16_t speed_t);

static void initXBee(void);

static bool_t initSensors(void);
static void getSensorData(void);
static void applyCompFilter(void);

static void stableFlight(void);
static void updatePID(void);
static void updateP(void);
static void updatePI(void);

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

	//XgyroRate = XaccelAngle;
	//YgyroRate = YaccelAngle;
	//XcompAngle = XaccelAngle;
	//YcompAngle = YaccelAngle;

	return TRUE;
}

