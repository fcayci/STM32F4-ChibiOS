#include <stdlib.h>
#include <string.h>
//#include "chprintf.h"
#include "ch.h"
#include "hal.h"

/* Accelerometer Registers */
#define LIS302DL_ADDRESS 				0x1C //0b0011100
#define LIS302DL_WHO_AM_I               0x0F
#define LIS302DL_CTRL_REG1              0x20
#define LIS302DL_CTRL_REG2              0x21
#define LIS302DL_CTRL_REG3              0x22
#define LIS302DL_STATUS_REG             0x27
#define LIS302DL_OUTX                   0x29
#define LIS302DL_OUTY                   0x2B
#define LIS302DL_OUTZ                   0x2D

/* currently only one I2C bus is possible */
#define I2C_MPU 			I2CD1
#define MPU_INT_PORT		GPIOB
#define MPU_INT_PIN			4


#define I2CDEV_DEFAULT_READ_TIMEOUT     1000
#define I2CDEV_BUFFER_LENGTH			64

/* buffers depth */
#define ACCEL_RX_DEPTH 6
#define ACCEL_TX_DEPTH 4

static uint8_t rxbuf[ACCEL_RX_DEPTH];
static uint8_t txbuf[ACCEL_TX_DEPTH];
static i2cflags_t errors = 0;
static int16_t acceleration_x, acceleration_y, acceleration_z;


int8_t I2CdevreadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout);
int8_t I2CdevreadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout);
int8_t I2CdevreadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout);
int8_t I2CdevreadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout);

bool_t I2CdevwriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool_t I2CdevwriteBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
bool_t I2CdevwriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool_t I2CdevwriteBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
bool_t I2CdevwriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
bool_t I2CdevwriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
bool_t I2CdevwriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
bool_t I2CdevwriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);


/**
 *
 */
static void print(char *p) {

  while (*p) {
    sdPut(&SD2, *p++);
  }
}

/**
 *
 */
static void println(char *p) {

  while (*p) {
    sdPut(&SD2, *p++);
  }
  sdWriteTimeout(&SD2, (uint8_t *)"\r\n", 2, TIME_INFINITE);
}

/**
 *
 */
static void printn(int16_t n) {
  char buf[16], *p;

  if (n > 0)
    sdPut(&SD2, '+');
  else{
    sdPut(&SD2, '-');
    n = abs(n);
  }

  if (!n)
    sdPut(&SD2, '0');
  else {
    p = buf;
    while (n)
      *p++ = (n % 10) + '0', n /= 10;
    while (p > buf)
      sdPut(&SD2, *--p);
  }
}

/**
 * Converts data from 2complemented representation to signed integer
 */
int16_t complement2signed(uint8_t msb, uint8_t lsb){
  uint16_t word = 0;
  word = (msb << 8) + lsb;
  if (msb > 0x7F){
    return -1 * ((int16_t)((~word) + 1));
  }
  return (int16_t)word;
}

/* I2C interface #1 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

/*
 * Application entry point.
 */
int main(void) {
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  sdStart(&SD2, NULL); /* Default 115200 8N1 */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /* Select I2C for LIS302DL Accelerometer */
  //palSetPadMode(GPIOE, 3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOE,GPIOE_CS_SPI);
  /* if SDO pad is connected to ground,
  * LSb value is Ô0Õ (address 0011100b)
  * GPIOA6 -> 0 */
  //palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOA,GPIOA_SDO);
  /* Route:
   * GPIOA5 - SCL -> B6
   * GPIOA7 - SDA -> B9 */
  i2cStart(&I2CD1, &i2cfg1);
  //palSetPadMode(GPIOA, 5, PAL_STM32_MODE_INPUT);
  //palSetPadMode(GPIOA, 7, PAL_STM32_MODE_INPUT);
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4));
  chThdSleepMilliseconds(100);
  if (palReadPad(GPIOA, 5))
	  print("A5 is HIGH\r\n");
  else
	  print("A5 is LOW\r\n");
  if (palReadPad(GPIOA, 7))
  	  print("A7 is HIGH\r\n");
  else
	  print("A7 is LOW\r\n");

  /**
   * Prepares the accelerometer
   */

  print("Sending: ");
  status =  I2CdevwriteByte(LIS302DL_ADDRESS, LIS302DL_CTRL_REG1, 0xC7);
  //status =  I2CdevreadByte(LIS302DL_ADDRESS, LIS302DL_ADDRESS, rxbuf, tmo);
  //status = i2cMasterReceiveTimeout(&I2CD1, LIS302DL_ADDRESS, txbuf, 1, tmo);
  //status = i2cMasterTransmitTimeout(&I2CD1, LIS302DL_ADDRESS, txbuf, 1, rxbuf, 1, tmo);
  //i2cReleaseBus(&I2CD1);
  print("sent... \r\n");
  if (status == FALSE){
    errors = i2cGetErrors(&I2CD1);
    print("FALSE1!\r\n");
  }
  status =  I2CdevwriteByte(LIS302DL_ADDRESS, LIS302DL_CTRL_REG2, 0x00);
  if (status == FALSE){
    errors = i2cGetErrors(&I2CD1);
    print("FALSE2!\r\n");
  }
  status =  I2CdevwriteByte(LIS302DL_ADDRESS, LIS302DL_CTRL_REG1, 0x00);
  if (status == FALSE){
    errors = i2cGetErrors(&I2CD1);
    print("FALSE3!\r\n");
  }

  /*
   * Normal main() thread activity, nothing in this test.
   */
  while (TRUE) {

    chThdSleepMilliseconds(100);

    //txbuf[0] = LIS302DL_OUTX;
    //i2cAcquireBus(&I2CD1);
    status =  I2CdevreadByte(LIS302DL_ADDRESS, LIS302DL_OUTX, rxbuf, tmo);
   // status = i2cMasterTransmitTimeout(&I2CD1, LIS302DL_ADDRESS, txbuf, 1, rxbuf, 1, tmo);
    //status = i2cMasterReceiveTimeout(&I2CD1, LIS302DL_ADDRESS, rxbuf, 1, tmo);
    //i2cReleaseBus(&I2CD1);

    //acceleration_x = complement2signed(rxbuf[0], rxbuf[1]);
    acceleration_x = (int8_t)rxbuf[0];
    acceleration_y = complement2signed(rxbuf[2], rxbuf[3]);
    acceleration_z = complement2signed(rxbuf[4], rxbuf[5]);

    print("x: ");
    printn(acceleration_x);
    print(" y: ");
    printn(acceleration_y);
    print(" z: ");
    printn(status);
    println("");

  }
}


int8_t I2CdevreadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
	//uint8_t mpu_txbuf[1], mpu_rxbuf[I2CDEV_BUFFER_LENGTH], i;
	msg_t rdymsg;
	if(length > I2CDEV_BUFFER_LENGTH) {
		return FALSE;
	}
	i2cAcquireBus(&I2C_MPU);
	rdymsg = i2cMasterTransmitTimeout(&I2C_MPU, devAddr, &regAddr, 1, data, length, MS2ST(timeout));
	i2cReleaseBus(&I2C_MPU);
	if(rdymsg == RDY_TIMEOUT || rdymsg == RDY_RESET) {
		return FALSE;
	}
	return TRUE;
}

bool_t I2CdevwriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
	uint8_t mpu_txbuf[I2CDEV_BUFFER_LENGTH], mpu_rxbuf[1];
	msg_t rdymsg;
	if((length + 1)> I2CDEV_BUFFER_LENGTH) {
		return FALSE;
	}
	mpu_txbuf[0] = regAddr;
	memcpy(mpu_txbuf + sizeof(uint8_t), data, sizeof(uint8_t) * length);

	i2cAcquireBus(&I2C_MPU);
	rdymsg = i2cMasterTransmit(&I2C_MPU, devAddr, mpu_txbuf, length + 1, mpu_rxbuf, 0);
	i2cReleaseBus(&I2C_MPU);
	if(rdymsg == RDY_TIMEOUT || rdymsg == RDY_RESET) {
		print("Burda kaldin di mi siskooOoO!!!/r/n");
		return FALSE;
	}
	return TRUE;
}

bool_t I2CdevwriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
	return I2CdevwriteBytes(devAddr, regAddr, 1, &data);
}

int8_t I2CdevreadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
	return I2CdevreadBytes(devAddr, regAddr, 1, data, timeout);
}
