ChibiProjects
=============

My ChibiOS/RT projects running mostly on STM32F4 Discovery board and my STM32F405RG based custom quadcopter board.

How to setup:

Get GNU Tools for ARM Embedded Processors from [https://launchpad.net/gcc-arm-embedded]. Pre-build ones should be fine. Extract it to some directory and add it to your PATH variable. Make sure there is no spaces in the directory path.

Checkout ChibiOS/RT from [https://github.com/mabl/ChibiOS]
Checkout ChibiProjects. Place them (ChibiOS/RT and ChibiProjects) so that they are under the same directory.

At this point you should be able to build the projects. You browse into one of the projects and execute 'make clean && make'. If it gives an error, you are probably missing something. Figure out what it is from the error output message. If it all goes well, you should see a 'Done' message at the end.

Get Eclipse IDE for C/C++ Developers, and set it up according to ChibiOS pages from [http://www.chibios.org/dokuwiki/doku.php?id=chibios:guides:eclipse1] and [http://www.chibios.org/dokuwiki/doku.php?id=chibios:guides:eclipse2].
The most important section is 'Preparing your Workspace' section.

For debugging get either stlink from [https://github.com/texane/stlink] or openocd to flash the board. You can also use [https://github.com/ArchMage/stm32loader] to flash your board using bootloader over USART port.


st-flash usage from a proejct directory:
st-flash write build/ch.bin 0x08000000

st-flash arguments for eclipse:
write ${selected_resource_loc}/build/ch.bin 0x08000000
