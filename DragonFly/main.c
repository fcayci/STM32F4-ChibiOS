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

#include "mpu6050.h"
#include "chprintf.h"
#include "math.h"

#define TAU_C 			0.075       // Time-constant
#define LOOPTIME 		20         	// Expressed in milli-seconds
#define DT 				0.05		// Expressed in seconds
#define X_ACC_OFFSET 	0
#define Y_ACC_OFFSET 	-1
#define Z_ACC_OFFSET 	16000
#define X_GYRO_OFFSET 	-30
#define Y_GYRO_OFFSET 	-20
#define Z_GYRO_OFFSET 	0
#define X_ACC_SENS_2G 	0.001 		// [g/digit]
#define Y_ACC_SENS_2G 	0.001 		// [g/digit]
#define Z_ACC_SENS_2G 	0.001 		// [g/digit]
#define X_GYRO_SENS 	0.00875 	// [dps/digit]
#define Y_GYRO_SENS 	0.00875 	// [dps/digit]
#define Z_GYRO_SENS 	0.00875 	// [dps/digit]

#define RAD_TO_DEG 57.2957786

#define CHPRINTF_USE_FLOAT          TRUE


int16_t accelXraw, accelYraw, accelZraw;
int16_t gyroXraw, gyroYraw, gyroZraw;

float Roll, Pitch, Yaw;
float tau_c, a_compl;

float x_acc, y_acc, z_acc;
float x_gyro, y_gyro, z_gyro;

int16_t accX, accY, accZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle; // Angle calculate using the gyro

static void initMotors(void);
static void testMotors(void);
static void initXBee(void);
static void initSensors(void);
static void applyCompFilter(void);

/*
static void print(char *p);
static void println(char *p);
static void printn(uint32_t n);
*/

/* Configure Motors */
static PWMConfig mtrPCFG = {
  1000000,								/* 1MHz PWM clock frequency  */
  10000,								/* PWM period (in ticks) 10mS (1/1MHz=1uS, 1us*10000 ticks=10mS) */
  NULL,									/* No Callback */
  {
    {PWM_OUTPUT_DISABLED, NULL},		/* Enable Channel 0 */
    {PWM_OUTPUT_DISABLED, NULL},		/* Enable Channel 1 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 2 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}		/* Enable Channel 3 */
  },
  0  									/* HW dependent part */
};

static PWMConfig mtrNCFG = {
  1000000,								/* 1MHz PWM clock frequency  */
  10000,								/* PWM period (in ticks) 10mS (1/1MHz=1uS, 1us*10000 ticks=10mS) */
  NULL,									/* No Callback */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 0 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 1 */
    {PWM_OUTPUT_DISABLED, NULL},		/* Enable Channel 2 */
    {PWM_OUTPUT_DISABLED, NULL}			/* Enable Channel 3 */
  },
  0  									/* HW dependent part */
};

/* Configure XBee */
static SerialConfig sd1cfg = {
   115200,
   0,
   USART_CR2_STOP1_BITS | USART_CR2_LINEN,
   0
};

/* Configure Sensors */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2
};

/* Configure Heartbeat */
static WORKING_AREA(wahbeat, 128);

static msg_t thBlinker(void *arg){
	(void)arg;
	palSetPadMode(GPIOC, GPIOC_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
	chRegSetThreadName("blinker");
	while (TRUE){
    	palTogglePad(GPIOC, GPIOC_LED_GREEN);
    	palTogglePad(GPIOC, GPIOC_LED_BLUE);
		chThdSleepMilliseconds(200);
	}
	return 0;
}

int main(void) {

	halInit();
	chSysInit();

	palSetPadMode(GPIOC, GPIOC_LED_BLUE, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(GPIOC, GPIOC_LED_BLUE);

	initMotors();
	initXBee();
	initSensors();

	chprintf((BaseSequentialStream *)&SD1, "Testing Motors...");
	testMotors();

	/* Create heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	while (TRUE){

		MPUgetMotion6(&accelXraw, &accelYraw, &accelZraw, &gyroXraw, &gyroYraw, &gyroZraw);
		/* Raw outputs */
		//chprintf((BaseSequentialStream *)&SD1, "accelX: %d\t accelY:%d \r\n", accelX, accelY);

		applyCompFilter();
		/* After complementary filter */
		//chprintf((BaseSequentialStream *)&SD1, "AX: %f \t AY: %f \t AZ: %f \t GX: %f \t GY: %f \t GZ: %f \r\n", x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro);
		chprintf((BaseSequentialStream *)&SD1, "R: %f \t P: %f \t Y: %f \r\n", Roll,Pitch,Yaw);

		chThdSleepMilliseconds(50);
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

static void testMotors(void){
	pwmEnableChannel(&PWMD2, 2, 500); 	/* M1 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD2, 2); 		/* M1 */
	pwmEnableChannel(&PWMD3, 0, 200); 	/* M2 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD3, 0); 		/* M2 */
	pwmEnableChannel(&PWMD2, 3, 200); 	/* M3 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD2, 3); 		/* M3 */
	pwmEnableChannel(&PWMD3, 1, 200); 	/* M4 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD3, 1); 		/* M4 */
}

static void initXBee(void){
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
	sdStart(&SD1, &sd1cfg);
}

static void initSensors(void){
	bool_t sts = 0;

	/* Disable these for discovery board,
	 * don't think it will be problem on the quad tho */
	//palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(0));
	//palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(0));

	i2cStart(&I2CD1, &i2cfg1);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

	chThdSleepMilliseconds(100);

	/* MPU6050, AD0 is connected to VCC */
	MPU6050(MPU6050_ADDRESS_AD0_HIGH);

	/* Test connection */
	sts = MPUtestConnection();
	if (sts) chprintf((BaseSequentialStream *)&SD1, "SENSOR: Testing Connection Success!");
	else chprintf((BaseSequentialStream *)&SD1, "SENSOR: ERROR: MPU6050 is not found!");

	chprintf((BaseSequentialStream *)&SD1, "SENSOR: Reseting...");
	MPUreset();
	chprintf((BaseSequentialStream *)&SD1, "SENSOR: Reseting Sensors...");
	MPUresetSensors();
	chThdSleepMilliseconds(100);
	chprintf((BaseSequentialStream *)&SD1, "SENSOR: Initializing...");
	MPUinitialize();
	chThdSleepMilliseconds(100);
}

/*
static void print(char *p) {
	while (*p) chSequentialStreamPut(&SD1, *p++);
}

static void println(char *p) {
	while (*p) chSequentialStreamPut(&SD1, *p++);
	chSequentialStreamWrite(&SD1, (uint8_t *)"\r\n", 2);
}

static void printn(uint32_t n) {
	char buf[16], *p;

	if (!n) chSequentialStreamPut(&SD1, '0');
	else {
		p = buf;
		while (n) *p++ = (n % 10) + '0', n /= 10;
		while (p > buf) chSequentialStreamPut(&SD1, *--p);
	}
}
*/

static void applyCompFilter(void){

  tau_c = (float) TAU_C;
  a_compl = 0.0;
  a_compl = (float) (TAU_C/(TAU_C+DT)); //0.882352941

  x_acc = ((float)(accelXraw - X_ACC_OFFSET)*X_ACC_SENS_2G*9.8);
  y_acc = ((float)(accelYraw - Y_ACC_OFFSET)*Y_ACC_SENS_2G*9.8);
  z_acc = ((float)(accelZraw - Z_ACC_OFFSET)*Z_ACC_SENS_2G*9.8);
  x_gyro = ((float)(gyroXraw - X_GYRO_OFFSET)*X_GYRO_SENS);
  y_gyro = ((float)(gyroYraw - Y_GYRO_OFFSET)*Y_GYRO_SENS);
  z_gyro = ((float)(gyroZraw - Z_GYRO_OFFSET)*Z_GYRO_SENS);

  Roll = (a_compl)*(Roll + x_gyro*DT) + (1-a_compl)*(x_acc);
  Pitch = (a_compl)*(Pitch + y_gyro*DT) + (1-a_compl)*(y_acc);
  Yaw = (a_compl)*(Yaw + z_gyro*DT) + (1-a_compl)*(z_acc);
}
