# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Pico_2_Stepper C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Pico_2_Stepper 
    src/main.cpp
    src/MotorController.cpp
    src/SPIInterface.cpp
    src/TrapezoidalStepper.cpp
)

pico_set_program_name(Pico_2_Stepper "Pico_2_Stepper")
pico_set_program_version(Pico_2_Stepper "0.1")

# # Generate PIO header
# pico_generate_pio_header(Pico_2_Stepper ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Pico_2_Stepper 0)
pico_enable_stdio_usb(Pico_2_Stepper 1)

# Add the standard library to the build
target_link_libraries(Pico_2_Stepper
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Pico_2_Stepper PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Pico_2_Stepper 
        hardware_spi
        hardware_pio
        # hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_irq
        pico_stdlib
        )

pico_add_extra_outputs(Pico_2_Stepper)

