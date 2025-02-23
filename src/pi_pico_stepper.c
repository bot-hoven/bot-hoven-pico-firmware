#include "include/pi_pico_stepper.h"

// Motor instances
StepperMotor l_stepper = {0.0, 0.0, 0.1};
StepperMotor r_stepper = {0.0, 0.0, 0.1};

// SPI buffers
uint8_t recv_buffer[BUFFER_SIZE] = {0};
char response_buffer[32] = {0};

void spi_slave_init(spi_inst_t* spi_instance) {
    spi_init(spi_instance, SPI_CLOCK_SPEED_HZ);
    spi_set_format(spi_instance, SPI_DATA_BITS, SPI_MODE_POLARITY, SPI_MODE_PHASE, SPI_BIT_ORDER);
    spi_set_slave(spi_instance, true);

    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
}

void update_position(StepperMotor* motor) {
    double position_change = motor->desired_position - motor->current_position;
    if (fabs(position_change) >= motor->resolution) {
        motor->current_position += (position_change > 0) ? motor->resolution : -motor->resolution;
    }
}

void handle_position_update(uint8_t motor) {
    uint8_t byte = 0xFF;
    uint8_t recv_index = 0;

    while (byte != 0x00 && recv_index < BUFFER_SIZE - 1) {
        byte = spi_sub_read_8_blocking(spi_default);
        recv_buffer[recv_index++] = byte;
    }
    recv_buffer[recv_index] = '\0';

    double position;
    if (sscanf((char*)recv_buffer, "%lf", &position) == 1) {
        StepperMotor* target = (motor == 'l') ? &l_stepper : &r_stepper;
        target->desired_position = position;
        printf("Set %c stepper to %.3f\n", motor, position);
    }
}

void handle_position_request(uint8_t motor) {
    StepperMotor* target = (motor == 'l') ? &l_stepper : &r_stepper;
    int written = snprintf(response_buffer, sizeof(response_buffer), POSITION_FORMAT, target->current_position);

    // Include null terminator + dummy byte space
    spi_sub_write_8_n_blocking(spi_default, (uint8_t*)response_buffer, written + 2);
}

void handle_spi_command(uint8_t command) {
    switch (command) {
    case CMD_CALIBRATE: {
        spi_sub_read_8_blocking(spi_default);  // Read parameter byte
        printf("Calibration started\n");
        break;
    }

    case CMD_POSITION_UPDATE: {
        uint8_t motor = spi_sub_read_8_blocking(spi_default);
        if (IS_VALID_MOTOR(motor))
            handle_position_update(motor);
        break;
    }

    case CMD_POSITION_REQUEST: {
        uint8_t motor = spi_sub_read_8_blocking(spi_default);
        if (IS_VALID_MOTOR(motor))
            handle_position_request(motor);
        break;
    }

    case CMD_NULL: {
        printf("Null-Terminator received.\n");
        break;
    }

    case CMD_DUMMY: {
        printf("Extra dummy byte received.\n");
        break;
    }

    default: {
        printf("Unknown command received: %x\n", command);
        break;
    }
    }
}

int main() {
    stdio_init_all();
    spi_slave_init(spi_default);
    sleep_ms(2000);  // Wait for serial monitor to open
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    absolute_time_t last_update = get_absolute_time();

    while (true) {
        // Handle the incoming SPI commands
        if (spi_is_readable(spi_default)) {
            uint8_t command = spi_sub_read_8_blocking(spi_default);
            handle_spi_command(command);
            command = 0xFF;  // Reset the byte variable to a non-command reliant value
        }

        // Pet the dog
        watchdog_update();

        // Simulate the motors moving
        if (absolute_time_diff_us(last_update, get_absolute_time()) > POSITION_UPDATE_INTERVAL_US) {
            update_position(&l_stepper);
            update_position(&r_stepper);
            last_update = get_absolute_time();
        }
    }
}

// #include <hardware/sync.h>
// #include <math.h>
// #include <stdio.h>
// #include <string.h>

// #include "hardware/spi.h"
// #include "hardware/watchdog.h"
// #include "pi_pico_commands.h"
// #include "pico/binary_info.h"
// #include "pico/stdlib.h"
// #include "include/spi_slave.h"

// #define POSITION_FORMAT "%.3f"  // 3 decimal places precision
// #define RESPONSE_SIZE 7
// #define BYTES_TO_PURGE 1
// #define BUFFER_SIZE 256

// #define SPI_CLOCK_SPEED_HZ (500 * 1000)  // 500kHz
// #define SPI_DATA_BITS 8                  // 8 bits/word
// #define SPI_MODE_POLARITY SPI_CPOL_0     // Clock polarity
// #define SPI_MODE_PHASE SPI_CPHA_1        // Clock phase
// #define SPI_BIT_ORDER SPI_MSB_FIRST      // MSB first

// // Motor control
// volatile double l_currentPosition = 0.0;
// volatile double l_desiredPosition = 0.0;
// volatile double r_currentPosition = 0.0;
// volatile double r_desiredPosition = 0.0;
// const double cl42tResolution = 0.1;

// // Global variables for response handling
// char response_buffer[32] = {0};   // Increased buffer size for flexibility
// int response_length = 0;          // Total bytes to send (including null terminator)
// volatile int response_index = 0;  // Current position in response buffer
// uint32_t status;

// void update_position(unsigned char stepper_side) {
//     volatile double* current = (stepper_side == 'l') ? &l_currentPosition : &r_currentPosition;
//     volatile double* desired = (stepper_side == 'l') ? &l_desiredPosition : &r_desiredPosition;
//     double position_change = *desired - *current;

//     if (fabs(position_change) >= cl42tResolution) {
//         *current += (position_change > 0) ? cl42tResolution : -cl42tResolution;
//     }
// }

// int main() {
//     stdio_init_all();

//     spi_init(spi_default, SPI_CLOCK_SPEED_HZ);

//     spi_set_format(spi_default, 8, SPI_MODE_POLARITY, SPI_MODE_PHASE, SPI_BIT_ORDER);
//     spi_set_slave(spi_default, true);
//     gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
//     gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
//     gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
//     gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);

//     // Make the SPI pins available to picotool
//     bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN,
//                                PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

//     // Hold here for 2s to allow the serial monitor to open
//     sleep_ms(2000);

//     printf("SPI slave waiting for data\n");

//     unsigned char command = 0;

//     unsigned char dummywritedata = 0xAA;
//     unsigned char motor = 'l';
//     double position = 0.000;
//     unsigned char byte = 0xFF;
//     uint8_t recv_buffer[BUFFER_SIZE] = {0};
//     uint8_t recv_index = 0;

//     watchdog_enable(3000, 1);

//     while (true) {
//         // check if there's something to read
//         if (spi_is_readable(spi_default)) {
//             command = spi_sub_read_8_blocking(spi_default);
//             // printf("Command is %s\n", command);
//             switch (command) {
//             case CMD_CALIBRATE: {
//                 byte = spi_sub_read_8_blocking(spi_default);
//                 printf("Starting calibration routine...\n");
//                 break;
//             }
//             case CMD_POSITION_UPDATE: {
//                 motor = spi_sub_read_8_blocking(spi_default);
//                 recv_index = 0;

//                 // Atomic read of all position bytes
//                 status = save_and_disable_interrupts();
//                 while (byte != 0x00) {
//                     if (spi_is_readable) {
//                         byte = spi_sub_read_8_blocking(spi_default);
//                         recv_buffer[recv_index++] = byte;
//                     }
//                 }
//                 restore_interrupts(status);

//                 // Calculate the length of the numeric substring
//                 if (recv_index == 0) {
//                     printf("No numeric data received.\n");
//                     break;
//                 }

//                 printf("Length of value: %zu\n", recv_index);  // %zu for size_t
//                 // The recv_buffer should probably be volatile and then copied into a local buffer, which would be
//                 // converted to the string Convert the string to a double
//                 if (sscanf(recv_buffer, "%lf", &position) != 1) {  // Check return value
//                     printf("Data is not a valid numerical value\n");
//                 } else {
//                     printf("Setting %c stepper to %.3f\n", motor, position);
//                     if (motor == 'l') {
//                         l_desiredPosition = position;
//                     } else {
//                         r_desiredPosition = position;
//                     }
//                 }
//                 break;
//             }
//             case CMD_POSITION_REQUEST: {
//                 motor = spi_sub_read_8_blocking(spi_default);
//                 if (!IS_VALID_MOTOR(motor)) {
//                     printf("Stepper motor %c does not exist.\n", motor);
//                     break;
//                 }

//                 printf("Position of stepper %c requested\n", motor);

//                 // Atomic read of position
//                 status = save_and_disable_interrupts();
//                 position = (motor == 'l') ? l_currentPosition : r_currentPosition;
//                 restore_interrupts(status);

//                 // Format position into string with null terminator
//                 int written = snprintf(response_buffer, sizeof(response_buffer), "%.3f", position);

//                 // Ensure null termination and calculate full length
//                 response_buffer[sizeof(response_buffer) - 1] = '\0';  // Force null terminator

//                 // Includes null-terminator and one extra dummy byte that the Pi will try to send
//                 response_length = strlen(response_buffer) + 2;

//                 // Atomic write of all bytes
//                 status = save_and_disable_interrupts();
//                 if (spi_is_writable) {
//                     spi_sub_write_8_n_blocking(spi_default, (uint8_t*)&response_buffer, response_length);
//                 }
//                 restore_interrupts(status);

//                 // Debug output
//                 printf("Prepared response: '");
//                 for (int i = 0; i < response_length; i++) {
//                     printf("%c", response_buffer[i]);
//                 }
//                 printf("\\0\n");
//                 break;
//             }
//             case CMD_NULL: {
//                 printf("Null-Terminator received.\n");
//                 break;
//             }
//             case CMD_DUMMY: {
//                 printf("Extra dummy byte received.\n");
//                 break;
//             }
//             default: {
//                 printf("Unknown command received: %x\n", command);
//                 break;
//             }
//             }
//             byte = 0xFF;  // Reset the byte variable to a non-command reliant value
//         }

//         static absolute_time_t last_update;
//         if (absolute_time_diff_us(last_update, get_absolute_time()) > 10000) {
//             update_position('l');
//             update_position('r');
//             last_update = get_absolute_time();
//         }

//         // pet the watchdog
//         watchdog_update();
//     }
// }