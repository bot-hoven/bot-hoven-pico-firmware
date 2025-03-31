#include "include/SPIInterface.h"

#include <cstdio>
#include <cstring>

SPIInterface::SPIInterface(spi_inst_t* spi_port, uint8_t miso_pin, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin,
                           uint32_t clock_speed_hz, uint8_t data_bits)
    : spi_port_(spi_port),
      miso_pin_(miso_pin),
      cs_pin_(cs_pin),
      sck_pin_(sck_pin),
      mosi_pin_(mosi_pin),
      clock_speed_hz_(clock_speed_hz),
      data_bits_(data_bits),
      calibration_callback_(nullptr),
      position_command_callback_(nullptr),
      position_state_callback_(nullptr),
      home_callback_(nullptr) {
    // Initialize buffers
    memset(recv_buffer_, 0, BUFFER_SIZE);
    memset(response_buffer_, 0, POSITION_BUFFER_SIZE);
}

void SPIInterface::init() {
    // Initialize SPI in slave mode
    spi_init(spi_port_, clock_speed_hz_);
    spi_set_format(spi_port_, data_bits_, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(spi_port_, true);

    // Set pin functions
    gpio_set_function(miso_pin_, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin_, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin_, GPIO_FUNC_SPI);
    gpio_set_function(cs_pin_, GPIO_FUNC_SPI);
}

bool SPIInterface::dataAvailable() { return spi_is_readable(spi_port_); }

void SPIInterface::processCommands() {
    if (!dataAvailable()) {
        return;
    }

    // Read command byte
    uint8_t command = spiReadByte();

    // Process command
    switch (command) {
    case CALIBRATE:
        handleCalibrationCommand();
        break;

    case COMMAND_POSITION:
        handlePositionCommand();
        break;

    case STATE_POSITION:
        handlePositionStateRequest();
        break;

    case HOME_POSITION:
        handleHomeCommand();
        break;

    case NULL_BYTE:
        // printf("Null-terminator received.\n");
        break;

    case DUMMY:
        // printf("Dummy byte received.\n");
        break;

    default:
        // printf("Unknown command received: 0x%02x\n", command);
        break;
    }
}

void SPIInterface::handleCalibrationCommand() {
    // Skip the motor identifier byte if present
    spiReadByte();

    // printf("Calibration command received\n");

    // Call the callback if registered
    if (calibration_callback_) {
        calibration_callback_();
    }
}

void SPIInterface::handlePositionCommand() {
    // Read the motor identifier (l or r)
    uint8_t motor = spiReadByte();

    if (!IS_VALID_MOTOR(motor)) {
        printf("Invalid motor identifier: 0x%02x\n", motor);
        return;
    }

    // Read the position value until null terminator
    uint8_t byte = 0xFF;
    uint8_t recv_index = 0;

    while (byte != NULL_BYTE && recv_index < BUFFER_SIZE - 1) {
        byte = spiReadByte();
        recv_buffer_[recv_index++] = byte;
    }

    // Null-terminate the received string
    recv_buffer_[recv_index - 1] = '\0';

    // Parse the position
    float position;
    if (sscanf((char*)recv_buffer_, "%f", &position) == 1) {
        // printf("Position command: motor=%c, position=%.3f\n", motor, position);

        // Call the callback if registered
        if (position_command_callback_) {
            position_command_callback_(static_cast<char>(motor), position);
        }
    } else {
        printf("Failed to parse position value: %s\n", recv_buffer_);
    }
}

void SPIInterface::handlePositionStateRequest() {
    // Read the motor identifier (l or r)
    uint8_t motor = spiReadByte();

    // Skip the null terminator
    spiReadByte();

    if (!IS_VALID_MOTOR(motor)) {
        printf("Invalid motor identifier: 0x%02x\n", motor);
        return;
    }

    // Get the current position from the callback
    float position = 0.0f;
    if (position_state_callback_) {
        position = position_state_callback_(static_cast<char>(motor));
    }

    // Format the response
    int written = snprintf(response_buffer_, POSITION_BUFFER_SIZE, "%.3f", position);

    // Send the response with null terminator
    spiWriteBuffer(response_buffer_, written + 1);

    // printf("Position state response: motor=%c, position=%.3f\n", motor, position);
}

// Add a new handler method:
void SPIInterface::handleHomeCommand() {
    // Skip the motor identifier byte if present
    spiReadByte();

    printf("Home command received\n");

    // Call the callback if registered
    if (home_callback_) {
        home_callback_();
    }
}

uint8_t SPIInterface::spiReadByte() {
    uint8_t byte;
    spi_read_blocking(spi_port_, DUMMY, &byte, 1);
    return byte;
}

void SPIInterface::spiWriteBuffer(const char* buffer, size_t length) {
    spi_write_blocking(spi_port_, (const uint8_t*)buffer, length);
}

void SPIInterface::setCalibrationCallback(CalibrationCallback callback) { calibration_callback_ = callback; }

void SPIInterface::setPositionCommandCallback(PositionCommandCallback callback) {
    position_command_callback_ = callback;
}

void SPIInterface::setPositionStateCallback(PositionStateCallback callback) { position_state_callback_ = callback; }

void SPIInterface::setHomeCallback(HomeCallback callback) { home_callback_ = callback; }














// #include "include/SPIInterface.h"

// #include <cstdio>
// #include <cstring>

// // Debug mode flag - set to 1 to enable debug prints, 0 to disable
// #define DEBUG_MODE 0

// /**
//  * @brief Constructor for SPIInterface
//  * 
//  * @param spi_port SPI port to use (e.g., spi0, spi1)
//  * @param miso_pin MISO pin number
//  * @param cs_pin Chip select pin number
//  * @param sck_pin SCK pin number
//  * @param mosi_pin MOSI pin number
//  * @param clock_speed_hz Clock speed in Hz
//  * @param data_bits Number of data bits per transfer
//  */
// SPIInterface::SPIInterface(spi_inst_t* spi_port, uint8_t miso_pin, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin,
//                            uint32_t clock_speed_hz, uint8_t data_bits)
//     : spi_port_(spi_port),
//       miso_pin_(miso_pin),
//       cs_pin_(cs_pin),
//       sck_pin_(sck_pin),
//       mosi_pin_(mosi_pin),
//       clock_speed_hz_(clock_speed_hz),
//       data_bits_(data_bits),
//       calibration_callback_(nullptr),
//       position_command_callback_(nullptr),
//       position_state_callback_(nullptr),
//       home_callback_(nullptr) {
//     // Initialize buffers
//     memset(recv_buffer_, 0, BUFFER_SIZE);
//     memset(response_buffer_, 0, POSITION_BUFFER_SIZE);
// }

// /**
//  * @brief Initialize the SPI interface in slave mode
//  * 
//  * Sets up SPI with the configured parameters and puts it in slave mode.
//  */
// void SPIInterface::init() {
//     // Initialize SPI in slave mode
//     spi_init(spi_port_, clock_speed_hz_);
//     spi_set_format(spi_port_, data_bits_, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
//     spi_set_slave(spi_port_, true);

//     // Set pin functions
//     gpio_set_function(miso_pin_, GPIO_FUNC_SPI);
//     gpio_set_function(sck_pin_, GPIO_FUNC_SPI);
//     gpio_set_function(mosi_pin_, GPIO_FUNC_SPI);
//     gpio_set_function(cs_pin_, GPIO_FUNC_SPI);
// }

// /**
//  * @brief Check if data is available to read from SPI
//  * 
//  * @return true if data is available, false otherwise
//  */
// bool SPIInterface::dataAvailable() { return spi_is_readable(spi_port_); }

// /**
//  * @brief Process incoming SPI commands
//  * 
//  * Reads command byte and dispatches to appropriate handler function.
//  */
// void SPIInterface::processCommands() {
//     if (!dataAvailable()) {
//         return;
//     }

//     // Read command byte
//     uint8_t command = spiReadByte();

//     // Process command
//     switch (command) {
//     case CALIBRATE:
//         handleCalibrationCommand();
//         break;

//     case COMMAND_POSITION:
//         handlePositionCommand();
//         break;

//     case STATE_POSITION:
//         handlePositionStateRequest();
//         break;

//     case HOME_POSITION:
//         handleHomeCommand();
//         break;

//     case NULL_BYTE:
//         // Null terminator received - no action needed
//         break;

//     case DUMMY:
//         // Dummy byte received - no action needed
//         break;

//     default:
//         if (DEBUG_MODE) {
//             printf("Unknown command received: 0x%02x\n", command);
//         }
//         break;
//     }
// }

// /**
//  * @brief Handle calibration command
//  * 
//  * Skips the motor identifier byte and calls the calibration callback.
//  */
// void SPIInterface::handleCalibrationCommand() {
//     // Skip the motor identifier byte if present
//     spiReadByte();

//     if (DEBUG_MODE) {
//         printf("Calibration command received\n");
//     }

//     // Call the callback if registered
//     if (calibration_callback_) {
//         calibration_callback_();
//     }
// }

// /**
//  * @brief Handle position command
//  * 
//  * Reads motor identifier and position value, then calls the position command callback.
//  */
// void SPIInterface::handlePositionCommand() {
//     // Read the motor identifier (l or r)
//     uint8_t motor = spiReadByte();

//     if (!IS_VALID_MOTOR(motor)) {
//         printf("Invalid motor identifier: 0x%02x\n", motor);
//         return;
//     }

//     // Read the position value until null terminator
//     uint8_t byte = 0xFF;
//     uint8_t recv_index = 0;

//     while (byte != NULL_BYTE && recv_index < BUFFER_SIZE - 1) {
//         byte = spiReadByte();
//         recv_buffer_[recv_index++] = byte;
//     }

//     // Null-terminate the received string
//     recv_buffer_[recv_index - 1] = '\0';

//     // Parse the position
//     float position;
//     if (sscanf((char*)recv_buffer_, "%f", &position) == 1) {
//         if (DEBUG_MODE) {
//             printf("Position command: motor=%c, position=%.3f\n", motor, position);
//         }

//         // Call the callback if registered
//         if (position_command_callback_) {
//             position_command_callback_(static_cast<char>(motor), position);
//         }
//     } else {
//         printf("Failed to parse position value: %s\n", recv_buffer_);
//     }
// }

// /**
//  * @brief Handle position state request
//  * 
//  * Reads motor identifier and returns current position via callback.
//  */
// void SPIInterface::handlePositionStateRequest() {
//     // Read the motor identifier (l or r)
//     uint8_t motor = spiReadByte();

//     // Skip the null terminator
//     spiReadByte();

//     if (!IS_VALID_MOTOR(motor)) {
//         printf("Invalid motor identifier: 0x%02x\n", motor);
//         return;
//     }

//     // Get the current position from the callback
//     float position = 0.0f;
//     if (position_state_callback_) {
//         position = position_state_callback_(static_cast<char>(motor));
//     }

//     // Format the response
//     int written = snprintf(response_buffer_, POSITION_BUFFER_SIZE, "%.3f", position);

//     // Send the response with null terminator
//     spiWriteBuffer(response_buffer_, written + 1);

//     if (DEBUG_MODE) {
//         printf("Position state response: motor=%c, position=%.3f\n", motor, position);
//     }
// }

// /**
//  * @brief Handle home command
//  * 
//  * Skips the motor identifier byte and calls the home callback.
//  */
// void SPIInterface::handleHomeCommand() {
//     // Skip the motor identifier byte if present
//     spiReadByte();

//     if (DEBUG_MODE) {
//         printf("Home command received\n");
//     }

//     // Call the callback if registered
//     if (home_callback_) {
//         home_callback_();
//     }
// }

// /**
//  * @brief Read a single byte from SPI
//  * 
//  * @return The byte read from SPI
//  */
// uint8_t SPIInterface::spiReadByte() {
//     uint8_t byte;
//     spi_read_blocking(spi_port_, DUMMY, &byte, 1);
//     return byte;
// }

// /**
//  * @brief Write a buffer to SPI
//  * 
//  * @param buffer The buffer to write
//  * @param length The length of the buffer
//  */
// void SPIInterface::spiWriteBuffer(const char* buffer, size_t length) {
//     spi_write_blocking(spi_port_, (const uint8_t*)buffer, length);
// }

// /**
//  * @brief Set the calibration callback
//  * 
//  * @param callback The callback function to register
//  */
// void SPIInterface::setCalibrationCallback(CalibrationCallback callback) { calibration_callback_ = callback; }

// /**
//  * @brief Set the position command callback
//  * 
//  * @param callback The callback function to register
//  */
// void SPIInterface::setPositionCommandCallback(PositionCommandCallback callback) {
//     position_command_callback_ = callback;
// }

// /**
//  * @brief Set the position state callback
//  * 
//  * @param callback The callback function to register
//  */
// void SPIInterface::setPositionStateCallback(PositionStateCallback callback) { position_state_callback_ = callback; }

// /**
//  * @brief Set the home callback
//  * 
//  * @param callback The callback function to register
//  */
// void SPIInterface::setHomeCallback(HomeCallback callback) { home_callback_ = callback; }