#ifndef PI_PICO_STEPPER_H
#define PI_PICO_STEPPER_H

#include <string.h>
#include <math.h>
#include <stdio.h>

#include <hardware/spi.h>
#include <hardware/watchdog.h>
#include <hardware/sync.h>

#include <pico/stdlib.h>
#include "pico/binary_info.h"

#include "pi_pico_commands.h"
#include "spi_comm.h"

// Configuration Constants
#define POSITION_FORMAT "%.3f"
#define RESPONSE_SIZE 7
#define BUFFER_SIZE 256
#define POSITION_UPDATE_INTERVAL_US 10000
#define WATCHDOG_TIMEOUT_MS 3000

// SPI Configuration
#define SPI_CLOCK_SPEED_HZ (500 * 1000)
#define SPI_DATA_BITS 8
#define SPI_MODE_POLARITY SPI_CPOL_0
#define SPI_MODE_PHASE SPI_CPHA_1
#define SPI_BIT_ORDER SPI_MSB_FIRST

// Motor Control Structure
typedef struct {
    volatile double current_position;
    volatile double desired_position;
    const double resolution;
} StepperMotor;

/**
 * @brief Initialize SPI peripheral in slave mode
 * @param spi_instance SPI instance (e.g., spi_default)
 */
void spi_slave_init(spi_inst_t* spi_instance);

/**
 * @brief Update motor position towards desired position
 * @param motor Pointer to StepperMotor structure
 */
void update_position(StepperMotor* motor);

/**
 * @brief Handle incoming SPI commands
 * @param command Received command byte from master
 */
void handle_spi_command(uint8_t command);


#endif // PI_PICO_STEPPER_H