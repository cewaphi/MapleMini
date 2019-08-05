
MapleMini as OpenTestJig
========================

Use the LeafLabs MapleMini as a lightweight testjig controller.
USB-connected MapleMini can be controlled via a command line interface
on the USB-serial port

Supported features
------------------

* read and write GPIO pins
* read and write UART port
* read PWM statistics
* set PWM frequency
* read temperature- and humidity-sensors on i2c bus (ADT7410 and SI7021-A20)
* read ADC pins
* control different linear and rotary motors (via mode [direction/home] and stepping pulses)

Firmware
--------

ChibiOS stable 16.1.x is used as RTOS.
The firmware is originally based on the MapleMini example of ChibiOS.
It should be flashed via DFU using the original MapleMini bootloader.
See below.

Build Firmware
--------------

You will need the following dependencies:
* GNU Make
* arm-none-eabi-gcc

Then just go into the firmware/ subdirectory and call `make`.

Update Firmware
---------------

To flash the firmware to the maplemini, an additional Make target has been
added. It requies dfu-util. While the MapleMini is connected via USB and
still in bootloader mode (first few seconds after pressing the reset-button),
just run `make flash_usb` .

