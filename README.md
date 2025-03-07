# Bot-Hoven Pico Firmware

## Requirements
1. VSCode with the [Raspberry Pi Pico Project](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) installed.
    1. Ensure that the extension installs the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).
    2. Ensure that the extension installs the [ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`gcc-arm-none-eabi`) and [CMake](https://cmake.org/)

## Importing the Project
1. Open the repository folder in VSCode.
2. From the sidebar select the Raspberry Pi Pico Project button.
3. Select Import Project, and then choose the folder where you cloned the repository.
4. Import with default settings (this can take over 5 minutes if it is the first time).

## Flashing the Pico
1. Disconnect the Pico from power, then hold down the `BOOTSEL` button and re-connect the power, wait until that hard-drive is mounted to your PC to release the button.
2. From the Raspberry Pi Pico Project sidebar, select `Run project (USB)` and wait while the project compiles and then uploads.