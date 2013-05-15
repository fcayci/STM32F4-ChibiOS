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
#include "i2cdev_chibi.h"
#include "mpu60x0.h"

static void initMotors(void);
static void testMotors(void);
static void initXBee(void);
static void initSensors(void);
static void testI2Cpins(void);
static void print(char *p);
static void println(char *p);
static void printn(uint32_t n);

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
   115200,						/* FIXME: See if you double this */
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

/* Configure I2C tester */
static WORKING_AREA(watesti2c, 128);

static msg_t thTester(void *arg){
	(void)arg;
	palSetPadMode(GPIOB, GPIOB_SDA, PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, GPIOB_SCL, PAL_STM32_OTYPE_OPENDRAIN);
	palClearPad(GPIOB, GPIOB_SDA);
	chRegSetThreadName("tester");
	while (TRUE){
    	palTogglePad(GPIOB, GPIOB_SDA);
    	palTogglePad(GPIOB, GPIOB_SCL);
		chThdSleepMilliseconds(200);
	}
	return 0;
}

/* Configure Heartbeat */
static WORKING_AREA(wahbeat, 128);

static msg_t thBlinker(void *arg){
	(void)arg;
	palSetPadMode(GPIOC, GPIOC_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
	chRegSetThreadName("blinker");
	while (TRUE){
		//println("toggle2!!...");
    	palTogglePad(GPIOC, GPIOC_LED_GREEN);
    	palTogglePad(GPIOC, GPIOC_LED_BLUE);
		chThdSleepMilliseconds(200);
	}
	return 0;
}

int main(void) {

	halInit();
	chSysInit();

	initMotors();
	initXBee();
	//testI2Cpins();
	initSensors();

	palSetPadMode(GPIOC, GPIOC_LED_BLUE, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(GPIOC, GPIOC_LED_BLUE);

	/* Create heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	pwmEnableChannel(&PWMD2, 2, 2000); /* M1 */
	pwmEnableChannel(&PWMD3, 0, 2000); /* M2 */
	pwmEnableChannel(&PWMD2, 3, 2000); /* M3 */
	pwmEnableChannel(&PWMD3, 1, 2000); /* M4 */

	bool_t sts;


	set_mpu_sample_rate(9);
	set_mpu_config_regsiter(EXT_SYNC_SET0, DLPF_CFG0);
	set_mpu_gyro(XG_ST_DIS, YG_ST_DIS, ZG_ST_DIS, FS_SEL_250);
	set_mpu_accel(XA_ST_DIS, YA_ST_DIS, ZA_ST_DIS, AFS_SEL_2g, ACCEL_HPF0);
	set_mpu_power_mgmt1(DEVICE_RESET_DIS, SLEEP_DIS, CYCLE_DIS, TEMPERATURE_EN, CLKSEL_XG);
	set_mpu_user_control(USER_FIFO_DIS, I2C_MST_DIS, I2C_IF_DIS, FIFO_RESET_DIS, I2C_MST_RESET_DIS, SIG_COND_RESET_DIS);

	  write_mpu_power_mgmt1();
	  write_mpu_gyro();
	  write_mpu_accel();
	  write_mpu_sample_rate();
	  while (TRUE) {
	     mpu_i2c_read_data(0x3B, 14); /* Read accelerometer, temperature and gyro data */
	     chThdSleepMilliseconds(50);
	  }


	while (TRUE) {
		testMotors();
		sts = MPUtestConnection();
		if (sts) println("MPU6050 Initialization Success!!!");
		else println("MPU6050 Initialization FAILED!!!");
		println("toggle!!...");
    	chThdSleepMilliseconds(5000);
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
	println("Activating Motor 1 with %5 duty cycle...");
	pwmEnableChannel(&PWMD2, 2, 500); 	/* M1 */
	chThdSleepMilliseconds(5000);
	println("Deactivating Motor 1...");
	pwmDisableChannel(&PWMD2, 2); 		/* M1 */
	println("Activating Motor 2 with %5 duty cycle...");
	pwmEnableChannel(&PWMD3, 0, 200); 	/* M2 */
	chThdSleepMilliseconds(5000);
	println("Deactivating Motor 2...");
	pwmDisableChannel(&PWMD3, 0); 		/* M2 */
	println("Activating Motor 3 with %5 duty cycle...");
	pwmEnableChannel(&PWMD2, 3, 200); 	/* M3 */
	chThdSleepMilliseconds(5000);
	println("Deactivating Motor 3...");
	pwmDisableChannel(&PWMD2, 3); 		/* M3 */
	println("Activating Motor 4 with %5 duty cycle...");
	pwmEnableChannel(&PWMD3, 1, 200); 	/* M4 */
	chThdSleepMilliseconds(5000);
	println("Deactivating Motor 4...");
	pwmDisableChannel(&PWMD3, 1); 		/* M4 */
}

static void initXBee(void){
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));

	sdStart(&SD1, &sd1cfg);
}

static void initSensors(void){
	bool_t sts;

	i2cStart(&I2CD1, &i2cfg1);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

	chThdSleepMilliseconds(1000);

	//MPU6050(MPU6050_ADDRESS_AD0_HIGH);
	//sts = MPUtestConnection();
	//if (sts) println("MPU6050 Initialization Success!!!");
	//else println("MPU6050 Initialization FAILED!!!");
}

static void testI2Cpins(void){

	/* Create a thread */
	chThdCreateStatic(watesti2c, sizeof(watesti2c), NORMALPRIO, thTester, NULL);

}

static void print(char *p) {

  while (*p) {
    chSequentialStreamPut(&SD1, *p++);
  }
}

static void println(char *p) {

  while (*p) {
    chSequentialStreamPut(&SD1, *p++);
  }
  chSequentialStreamWrite(&SD1, (uint8_t *)"\r\n", 2);
}

static void printn(uint32_t n) {
  char buf[16], *p;

  if (!n)
    chSequentialStreamPut(&SD1, '0');
  else {
    p = buf;
    while (n)
      *p++ = (n % 10) + '0', n /= 10;
    while (p > buf)
      chSequentialStreamPut(&SD1, *--p);
  }
}
