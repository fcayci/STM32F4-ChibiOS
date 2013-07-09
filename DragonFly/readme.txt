*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 based Quadcopter.                     **
*****************************************************************************

** TARGET **

ARM-Cortex-M4
64 PIN - STM32F405RG

** REVISION **

DragonFly Quadcopter Board RevA (May 2013)

** CONFIGURATION ** 

Oscillator (HSE)    : 16MHz
MPU6050             : I2C1SCL [PB6], I2C1SDA [PB7], INT [PA2]
XBee Wireless Radio : USART1TX [PA9], USART1RX [PA10]
4 x PWM Motors      : TIM3CH1 [PC6] (EAST),
                      TIM3CH2 [PC7] (WEST),
                      TIM2CH3 [PB10] (NORTH),
                      TIM2CH4 [PB11] (SOUTH)
2 x LEDs            : Green LED (PC4), Blue LED (PC5)
Battery Check       : AIN5 (PA5)