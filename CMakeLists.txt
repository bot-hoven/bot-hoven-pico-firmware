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
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pi_pico_stepper C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add all source files
add_executable(pi_pico_stepper 
    src/pi_pico_stepper.c
    src/spi_comm.c
)

# Project metadata
pico_set_program_name(pi_pico_stepper "pi_pico_stepper")
pico_set_program_version(pi_pico_stepper "0.1")

# Enable USB output, disable UART
pico_enable_stdio_uart(pi_pico_stepper 0)
pico_enable_stdio_usb(pi_pico_stepper 1)

# Add required libraries
target_link_libraries(pi_pico_stepper
    pico_stdlib
    hardware_spi
    hardware_watchdog
)

# Include directories
target_include_directories(pi_pico_stepper PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Add additional output formats
pico_add_extra_outputs(pi_pico_stepper)

