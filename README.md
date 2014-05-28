## STM32F4-ChibiOS

ChibiOS/RT examples running on STM32F4 Discovery board

### Required tools

* `GNU ARM Toolchain` from [https://launchpad.net/gcc-arm-embedded]()
* `ChibiOS/RT` from [https://github.com/mabl/ChibiOS]()
* `ST-LINK` from [https://github.com/texane/stlink]()

#### Notes

* st-flash usage from a project directory:

  ```
  $ st-flash write build/ch.bin 0x08000000
  ```

* st-flash arguments for eclipse:

  ```
  write ${selected_resource_loc}/build/ch.bin 0x08000000
  ```

#### TODO

* I2C code test
* UART example test
* Serial read example
* Shell over serial line example
* ADC example
* DAC example
