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
#define LOOPTIME 		2         	// Expressed in milli-seconds
#define DT 				0.02		// Expressed in seconds
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

const double Q_angle = 0.001;
const double Q_gyroBias = 0.003;
const double R_angle = 0.03;

#define RAD_TO_DEG 57.2957786

#define PWM_CLOCK_FREQUENCY 50000000
#define PWM_PERIOD_IN_TICKS 500
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY/PWM_PERIOD_IN_TICKS)

#define MOTOR_NORTH			2
#define MOTOR_SOUTH			3
#define MOTOR_EAST			0
#define MOTOR_WEST			1

int16_t accelXraw, accelYraw, accelZraw;
int16_t gyroXraw, gyroYraw, gyroZraw;

float Roll, Pitch, Yaw;
uint32_t Roll_i, Pitch_i, Yaw_i;
float tau_c, a_compl;

float x_acc, y_acc, z_acc;
float x_gyro, y_gyro, z_gyro;

int16_t accX, accY, accZ;

double compAngleX, compAngleY;
double gyroXrate, gyroYrate;
double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle; // Angle calculate using the gyro

static int32_t errorX, integralX, derivativeX, outputX;
static int32_t errorY, integralY, derivativeY, outputY;
double Ki, Kd, Kp;
static bool_t isi2c;

static void initMotors(void);
static void testMotors(void);
static void initXBee(void);
static bool_t initSensors(void);
static void applyCompFilter(void);
static void getData(void);
static void testStable(void);
static void testStable2(void);
static void getPID(void);

static void print(char *p);
static void println(char *p);
static void printn(uint32_t n);

/* Configure Motors */
static PWMConfig mtrPCFG = {
	PWM_CLOCK_FREQUENCY,
	PWM_PERIOD_IN_TICKS,
	NULL,
	{
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL}
	},
	0
};

static PWMConfig mtrNCFG = {
	PWM_CLOCK_FREQUENCY,
	PWM_PERIOD_IN_TICKS,
	NULL,
	{
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_DISABLED, NULL}
	},
	0
};

/* Configure serial link for XBee */
static SerialConfig sd1cfg = {
   115200,
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
	palSetPadMode(GPIOC, GPIOC_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOC, GPIOC_LED_BLUE, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(GPIOC, GPIOC_LED_BLUE);
	chRegSetThreadName("blinker");
	while (TRUE){
		palTogglePad(GPIOC, GPIOC_LED_GREEN);
    	palTogglePad(GPIOC, GPIOC_LED_BLUE);
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
		//chprintf((BaseSequentialStream *)&SD1, "PIDY: %d\tAYA: %f\tCYA: %f\r\n",outputY,accYangle,compAngleY);
		chprintf((BaseSequentialStream *)&SD1, "PIDY: %d\tPIDX: %d\r\n",outputY,outputX);
		chThdSleepMilliseconds(200);
	}
	return 0;
}

int main(void) {

	halInit();
	chSysInit();

	initMotors();
	initXBee();

	chThdSleepMilliseconds(3000);

	isi2c = initSensors();

	errorX = 0;
  	integralX = 0;
	errorY = 0;
  	integralY = 0;

	/* Create the heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	Ki=0.05;
	Kp=0.01;
	Kd=0;
	getData();
	getPID();
	chThdSleepMilliseconds(1000);
	while (TRUE){
		if(isi2c) {
			getData();
			getPID();
			//testStable();
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

static void testMotors(void){

	chprintf((BaseSequentialStream *)&SD1, "Testing Motors...");
	pwmEnableChannel(&PWMD2, 2, PWM_PERIOD_IN_TICKS/10); 	/* M1 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD2, 2); 		/* M1 */
	pwmEnableChannel(&PWMD3, 0, PWM_PERIOD_IN_TICKS/10); 	/* M2 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD3, 0); 		/* M2 */
	pwmEnableChannel(&PWMD2, 3, PWM_PERIOD_IN_TICKS/10); 	/* M3 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD2, 3); 		/* M3 */
	pwmEnableChannel(&PWMD3, 1, PWM_PERIOD_IN_TICKS/10); 	/* M4 */
	chThdSleepMilliseconds(5000);
	pwmDisableChannel(&PWMD3, 1); 		/* M4 */
}

static void initXBee(void){
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
	sdStart(&SD1, &sd1cfg);
}

static bool_t initSensors(void){
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
	if (sts) chprintf((BaseSequentialStream *)&SD1, "SENSOR: Testing Connection Success!\r\n");
	else {
		chprintf((BaseSequentialStream *)&SD1, "SENSOR: ERROR: MPU6050 is not found!\r\n");
		return FALSE;
	}

	chprintf((BaseSequentialStream *)&SD1, "SENSOR: Reseting...\r\n");
	MPUreset();
	chprintf((BaseSequentialStream *)&SD1, "SENSOR: Reseting Sensors...\r\n");
	MPUresetSensors();
	chThdSleepMilliseconds(100);
	chprintf((BaseSequentialStream *)&SD1, "SENSOR: Initializing...\r\n");
	MPUinitialize();
	chThdSleepMilliseconds(100);

	chprintf((BaseSequentialStream *)&SD1, "SENSOR: Calibrating...\r\n");
	MPUgetMotion6(&accelXraw, &accelYraw, &accelZraw, &gyroXraw, &gyroYraw, &gyroZraw);
	accXangle = (atan2(accelYraw,accelZraw)+M_PI)*RAD_TO_DEG;
	accYangle = (atan2(accelXraw,accelZraw)+M_PI)*RAD_TO_DEG;
	gyroXrate = accXangle;
	gyroYrate = accYangle;
	compAngleX = compAngleX;
	compAngleY = compAngleY;

	chThdCreateStatic(waprint, sizeof(waprint), NORMALPRIO, thPrinter, NULL);

	return TRUE;
}


static void getPID(void){
	errorX = 180 - compAngleX;
  	integralX = integralX + errorX*0.002;
  	derivativeX = (errorX - errorX)/0.002;
    outputX = Kp*errorX + Ki*integralX + Kd*derivativeX;

	errorY = 180 - compAngleY;
  	integralY = integralY + errorY*0.002;
  	derivativeY = (errorY - errorY)/0.002;
    outputY = Kp*errorY + Ki*integralY + Kd*derivativeY;

}


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


static void getData(void){

	MPUgetMotion6(&accelXraw, &accelYraw, &accelZraw, &gyroXraw, &gyroYraw, &gyroZraw);
	/* Raw outputs */
	//chprintf((BaseSequentialStream *)&SD1, "AXR: %d\t AYR:%d AZR: %d\t GXR:%d GYR: %d\t GZR:%d \r\n", accelXraw, accelYraw, accelZraw, gyroXraw, gyroYraw, gyroZraw);

	//applyCompFilter();
	/* After complementary filter */

	/*
	 *  Get the atan of X and Y. This value will be between -¹ to ¹ in radians.
	 *  Then add ¹ to make it between 0 to 2¹. Finally convert it to degrees...
	 *  Here is the output for an (almost) flat reading:
	 *  accXangle = 178.12345, accYangle = 179.12345
	 */

	accXangle = (atan2(accelYraw,accelZraw)+M_PI)*RAD_TO_DEG;
	accYangle = (atan2(accelXraw,accelZraw)+M_PI)*RAD_TO_DEG;

	gyroXrate = (double)gyroXraw/131.0;
	gyroYrate = -((double)gyroYraw/131.0);

	compAngleX = (0.8*(compAngleX+(gyroXrate*0.02)))+(0.2*accXangle);
	compAngleY = (0.8*(compAngleY+(gyroYrate*0.02)))+(0.2*accYangle);

	//chprintf((BaseSequentialStream *)&SD1, "AX: %f \t AY: %f \t AZ: %f \t GX: %f \t GY: %f \t GZ: %f AngX: %f AngY: %f GX_rate: %d, GY_rate: %d\r\n", x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro,accXangle,accYangle,gyroXrate,gyroYrate);
	//chprintf((BaseSequentialStream *)&SD1, "AaX: %f, AaY: %f, CaX: %f, CaY: %f\r\n", accXangle, accYangle, compAngleX, compAngleY);
	//chprintf((BaseSequentialStream *)&SD1, "%f,%f\r\n", compAngleX,accYangle);
	//chprintf((BaseSequentialStream *)&SD1, "%d,%d\r\n", (int16_t)compAngleX,(int16_t)compAngleY);
	//chprintf((BaseSequentialStream *)&SD1, "R: %f \t P: %f \t Y: %f\r\n", Roll,Pitch,Yaw);

}

static void testStable(void){

	pwmEnableChannel(&PWMD3, MOTOR_EAST, 300+outputY);
	pwmEnableChannel(&PWMD3, MOTOR_WEST, 300-outputY);
	pwmEnableChannel(&PWMD2, MOTOR_NORTH, 300+outputX);
	pwmEnableChannel(&PWMD2, MOTOR_SOUTH, 300-outputX);

}

static void testStable2(void){

	pwmEnableChannel(&PWMD3, MOTOR_WEST, 120+(int16_t)compAngleY);
	pwmEnableChannel(&PWMD3, MOTOR_EAST, 480-(int16_t)compAngleY);
	pwmEnableChannel(&PWMD2, MOTOR_NORTH, 0);//120+compAngleY);
	pwmEnableChannel(&PWMD2, MOTOR_SOUTH, 0);//480-compAngleY);}
}


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
