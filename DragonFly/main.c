/*
 * DragonFly Quadcopter ChibiOS-based firmware
 *
 *      File    : main.c
 *      Author  : Furkan Cayci
 *
 */

/*
 * DragonFly RevA (May 2013)
 * Configuration:
 *
 * Oscillator (HSE)    : 16MHz
 * MPU6050             : I2C1SCL [PB6], I2C1SDA [PB7], INT [PA2]
 * XBee Wireless Radio : USART1TX [PA9], USART1RX [PA10]
 * 4 x PWM Motors      : TIM3CH1 [PC6] (EAST),
 *                       TIM3CH2 [PC7] (WEST),
 *                       TIM2CH3 [PB10] (NORTH),
 *                       TIM2CH4 [PB11] (SOUTH)
 * 2 x LEDs            : Green LED (PC4), Blue LED (PC5)
 * Battery Check       : AIN5 (PA5)
 */

#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "chprintf.h"
#include "math.h"

#define IMU_DEBUG /* Enable debugging on IMU */

/* USART definitions */
#define USART_CR1_9BIT_WORD   (1 << 12) /* CR1 9 bit word */
#define USART_CR1_PARITY_SET  (1 << 10) /* CR1 Parity bit enable */
#define USART_CR1_EVEN_PARITY (0 << 9)  /* CR1 even parity */

//#define DEBUG_PRINT(x) chprintf((BaseChannel *)&SD1, x);
//#define DEBUG_PRINTF(x, y) chprintf((BaseChannel *)&SD1, x, y);

/*
 * PWM definitions for the motors. 500 might be increased to 512 to make it 2^9
 */
#define PWM_CLOCK_FREQUENCY 50000000
#define PWM_PERIOD_IN_TICKS 500
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY/PWM_PERIOD_IN_TICKS)

#define MOTOR_EAST  0
#define MOTOR_WEST  1
#define MOTOR_NORTH 2
#define MOTOR_SOUTH 3

#define RAD_TO_DEG 57.2957786

#define DT   0.002
#define COMP 0.99

#define KP 1        // KP
#define KI (1 * DT) // KI * DT = 1 * 0.002
#define KD 0        // KD / DT = 0 / 0.002

/* IMU variables */
typedef struct {int16_t x; int16_t y; int16_t z;} axis;

axis accelRaw;
axis gyroRaw;

int16_t XaccelRaw, YaccelRaw, ZaccelRaw;
int16_t XgyroRaw, YgyroRaw, ZgyroRaw;

double XaccelAngle, YaccelAngle;
double XgyroRate, YgyroRate;
double XcompAngle, YcompAngle;

static bool_t isi2c;

/* PID variables */
static double errorX, integralX, derivativeX, outputX;
static double errorY, integralY, derivativeY, outputY;

/* Motor variables */
static uint16_t mEastSpeed, mWestSpeed, mNorthSpeed, mSouthSpeed;

/* XBee functions */
static void initXBee(void);

/* Motor functions*/
static void initMotors(void);
static void testMotors(void);
static void setMotorSpeed(uint8_t motor_t, uint16_t speed_t);
static void stableFlight(void);

/* IMU functions */
static bool_t imuInit(void);
static void imuGetData(void);
static void imuApplyCompFilter(void);
static void imuApplyKalmanFilter(void);

/* PID functions */
static void initPID(void);
static void updateP(void);
static void updatePI(void);
static void updatePID(void);

/* Motor configurations */
static PWMConfig mtrPCFG = {
	PWM_CLOCK_FREQUENCY,
	PWM_PERIOD_IN_TICKS,
	NULL,
	{
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},	// MOTOR_NORTH
		{PWM_OUTPUT_ACTIVE_HIGH, NULL}	// MOTOR_SOUTH
	},
	0
};

static PWMConfig mtrNCFG = {
	PWM_CLOCK_FREQUENCY,
	PWM_PERIOD_IN_TICKS,
	NULL,
	{
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},	// MOTOR_EAST
		{PWM_OUTPUT_ACTIVE_HIGH, NULL}, // MOTOR_WEST
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL}
	},
	0
};

/* XBee configuration */
static SerialConfig sd1cfg = {
	115200,
	USART_CR1_9BIT_WORD | USART_CR1_PARITY_SET | USART_CR1_EVEN_PARITY,
	USART_CR2_STOP1_BITS | USART_CR2_LINEN,
	0
};

/* IMU configuration */
static const I2CConfig i2cfg1 = {
	OPMODE_I2C,
	400000,
	FAST_DUTY_CYCLE_2
};

/* Configure Heartbeat */
static WORKING_AREA(wahbeat, 128);

static msg_t thBlinker(void *arg){
	(void)arg;
	palSetPadMode(GPIOC, GPIOC_PIN4, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOC, GPIOC_PIN5, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(GPIOC, GPIOC_PIN5);
	chRegSetThreadName("blinker");
	while (TRUE){
		palTogglePad(GPIOC, GPIOC_PIN4);
    	palTogglePad(GPIOC, GPIOC_PIN5);
		chThdSleepMilliseconds(200);
	}
	return 0;
}

/* Configure Printing */
static WORKING_AREA(waprint, 128);

static msg_t thPrinter(void *arg){
	(void)arg;
	chRegSetThreadName("printer");
	while (TRUE){
		chprintf((BaseSequentialStream *)&SD1, "%f:%f\n", XcompAngle,YcompAngle);
		//chprintf((BaseSequentialStream *)&SD1, "%f:%f\n", XaccelAngle,XcompAngle);
		//chprintf((BaseSequentialStream *)&SD1, "AX: %6d AY: %6d AZ: %6d ,GX: %6d GY: %6d GZ: %6d\t\r\n", accelRaw.x, accelRaw.y, accelRaw.z, gyroRaw.x, gyroRaw.y, gyroRaw.z);
		//chprintf((BaseSequentialStream *)&SD1, "EX: %f\t EY: %f\t OY: %f\t OY: %f\r\n",XcompAngle,errorY,outputX,outputY);
		chThdSleepMilliseconds(50);
	}
	return 0;
}

int main(void) {

	halInit();
	chSysInit();

	/* Create the heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	//initMotors();
	initXBee();

	chThdSleepMilliseconds(3000);

	isi2c = imuInit();

	//mNorthSpeed = 300;
	mNorthSpeed = 0;
	mSouthSpeed = mNorthSpeed;

	//mEastSpeed = 300;
	mEastSpeed = 0;
	mWestSpeed = mEastSpeed;

	errorX = errorY = 0;
  	integralX = integralY = 0;

  	imuGetData();
	updatePI();
	chThdSleepMilliseconds(1000);

	while (TRUE){
		if(isi2c) {
		  	imuGetData();
			updatePI();
			//stableFlight();
		}
		chThdSleepMilliseconds(2);
	}
}

static void initMotors(void){

	palSetPadMode(GPIOC, GPIOC_PIN6, PAL_MODE_ALTERNATE(2)); /* M1 */
	palSetPadMode(GPIOB, GPIOB_PIN10, PAL_MODE_ALTERNATE(1)); /* M2 */
	palSetPadMode(GPIOC, GPIOC_PIN7, PAL_MODE_ALTERNATE(2)); /* M3 */
	palSetPadMode(GPIOB, GPIOB_PIN11, PAL_MODE_ALTERNATE(1)); /* M4 */

	pwmStart(&PWMD2, &mtrPCFG);
	pwmStart(&PWMD3, &mtrNCFG);
}

static void initXBee(void){
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
	sdStart(&SD1, &sd1cfg);
}

static bool_t imuInit(void){
	bool_t sts = 0;

	i2cStart(&I2CD1, &i2cfg1);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

	chThdSleepMilliseconds(100);

	/* MPU6050, AD0 is connected to VCC */
	MPU6050(MPU6050_ADDRESS_AD0_HIGH);

	/* Test connection */
	sts = MPUtestConnection();
	if (sts) chprintf((BaseSequentialStream *)&SD1, "SENSOR: Testing Connection Success!\r\n");
	else {
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: ERROR: MPU6050 is not found!\r\n");
		return FALSE;
	}
	#if defined(IMU_DEBUG)
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: Reseting...\r\n");
	#endif

	MPUreset();

	#if defined(IMU_DEBUG)
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: Reseting Sensors...\r\n");
	#endif

	MPUresetSensors();
	chThdSleepMilliseconds(100);

	#if defined(IMU_DEBUG)
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: Initializing...\r\n");
	#endif
	MPUinitialize();
	chThdSleepMilliseconds(100);

	#if defined(IMU_DEBUG)
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: Calibrating...\r\n");
	#endif

	MPUgetMotion6(&accelRaw.x, &accelRaw.y, &accelRaw.z, &gyroRaw.x, &gyroRaw.y, &gyroRaw.z);

	XaccelAngle = (atan2(accelRaw.y,accelRaw.z))*RAD_TO_DEG;
	YaccelAngle = (atan2(accelRaw.x,accelRaw.z))*RAD_TO_DEG;

	XgyroRate = XaccelAngle;
	YgyroRate = YaccelAngle;
	XcompAngle = XaccelAngle;
	YcompAngle = YaccelAngle;

	#if defined(IMU_DEBUG)
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: Calibrating Complete...\r\n");
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: Creating Printer thread...\r\n");
		chThdCreateStatic(waprint, sizeof(waprint), NORMALPRIO, thPrinter, NULL);
	#endif
	return TRUE;
}

static void imuGetData(void){

	MPUgetMotion6(&accelRaw.x, &accelRaw.y, &accelRaw.z, &gyroRaw.x, &gyroRaw.y, &gyroRaw.z);

	/*
	 *  Get the atan of X and Y. This value will be between -¹ to ¹ in radians.
	 *  Optionally ¹ can be added to make it between 0 to 2¹. (M_PI).
	 *  Finally convert it to degrees.
	 */
	XaccelAngle = (atan2(accelRaw.y,accelRaw.z))*RAD_TO_DEG;
	YaccelAngle = (atan2(accelRaw.x,accelRaw.z))*RAD_TO_DEG;

	XgyroRate = (double)gyroRaw.x/131.0;
	YgyroRate = -((double)gyroRaw.y/131.0);

	imuApplyCompFilter();
}

static void imuApplyCompFilter(void){
	XcompAngle = (COMP*(XcompAngle+(XgyroRate*DT)))+((1-COMP)*XaccelAngle);
	YcompAngle = (COMP*(YcompAngle+(YgyroRate*DT)))+((1-COMP)*YaccelAngle);
}

/* TODO:
 * 	* Change the (PWM_PER...) variable to something like
 * MAX_MOTOR_SPEED.
 */
static void setMotorSpeed(uint8_t motor_t, uint16_t speed_t){

	if (speed_t > PWM_PERIOD_IN_TICKS) speed_t = PWM_PERIOD_IN_TICKS;

	switch (motor_t) {
	case MOTOR_EAST:
		pwmEnableChannel(&PWMD3, 0, speed_t);
		break;
	case MOTOR_WEST:
		pwmEnableChannel(&PWMD3, 1, speed_t);
		break;
	case MOTOR_NORTH:
		pwmEnableChannel(&PWMD2, 2, speed_t);
		break;
	case MOTOR_SOUTH:
		pwmEnableChannel(&PWMD2, 3, speed_t);
		break;
	}
}

/*
 * Calculate the error in both X and Y coordinates in reference to 180
 * degrees. Get the I value based on 2mS sampling (for now) and find
 * the correction value that will get sent to pwm
 */
static void updatePID(void){
	errorX = 180 - XcompAngle;
  	integralX = integralX + errorX*DT;
  	derivativeX = (errorX - errorX)/DT;
    outputX = KP*errorX + KI*integralX + KD*derivativeX;

	errorY = 180 - YcompAngle;
  	integralY = integralY + errorY*DT;
  	derivativeY = (errorY - errorY)/DT;
    outputY = KP*errorY + KI*integralY + KD*derivativeY;
}
static void updateP(void){
	errorX = 180 - XcompAngle;
    outputX = (int16_t)(KP*errorX);

	errorY = 180 - YcompAngle;
    outputY = (int16_t)(KP*errorY);

    if (outputX > 180) outputX = 180;
    else if (outputX < -180) outputX = -180;
    if (outputY > 180) outputY = 180;
    else if (outputY < -180) outputY = -180;
}
static void updatePI(void){
	errorX = 180 - XcompAngle;
	integralX = integralX + errorX;
    outputX = (int16_t)(KP*errorX + KI*integralX);

	errorY = 180 - YcompAngle;
	integralY = integralY + errorY;
    outputY = (int16_t)(KP*errorY + KI*integralY);

    if (outputX > 180) outputX = 180;
    else if (outputX < -180) outputX = -180;
    if (outputY > 180) outputY = 180;
    else if (outputY < -180) outputY = -180;
}

static void testMotors(void){

	chprintf((BaseSequentialStream *)&SD1, "Testing Motors...");
	setMotorSpeed(MOTOR_EAST, PWM_PERIOD_IN_TICKS/10);
	chThdSleepMilliseconds(5000);
	setMotorSpeed(MOTOR_EAST, 0);
	setMotorSpeed(MOTOR_WEST, PWM_PERIOD_IN_TICKS/10);
	chThdSleepMilliseconds(5000);
	setMotorSpeed(MOTOR_WEST, 0);
	setMotorSpeed(MOTOR_NORTH, PWM_PERIOD_IN_TICKS/10);
	chThdSleepMilliseconds(5000);
	setMotorSpeed(MOTOR_NORTH, 0);
	setMotorSpeed(MOTOR_SOUTH, PWM_PERIOD_IN_TICKS/10);
	chThdSleepMilliseconds(5000);
	setMotorSpeed(MOTOR_SOUTH, 0);
}

static void stableFlight(void){

	//mWestSpeed = mWestSpeed - (int16_t)(outputY);
	//mNorthSpeed = mNorthSpeed + outputX;

	setMotorSpeed(MOTOR_EAST, mEastSpeed);
	setMotorSpeed(MOTOR_WEST, mWestSpeed);
	setMotorSpeed(MOTOR_NORTH, mNorthSpeed);
	setMotorSpeed(MOTOR_SOUTH, mSouthSpeed);
}


