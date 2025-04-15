#include "include/SPIInterface.h"
#include <cstring>
#include <cstdio>

SPIInterface::SPIInterface(
    spi_inst_t* spi_port,
    uint8_t miso_pin,
    uint8_t cs_pin,
    uint8_t sck_pin,
    uint8_t mosi_pin,
    uint32_t clock_speed_hz,
    uint8_t data_bits
) : spi_port_(spi_port),
    miso_pin_(miso_pin),
    cs_pin_(cs_pin),
    sck_pin_(sck_pin),
    mosi_pin_(mosi_pin),
    clock_speed_hz_(clock_speed_hz),
    data_bits_(data_bits),
    calibration_callback_(nullptr),
    position_command_callback_(nullptr),
    position_state_callback_(nullptr),
    pid_tune_callback_(nullptr) {
    
    // Initialize buffers
    memset(recv_buffer_, 0, BUFFER_SIZE);
    memset(response_buffer_, 0, POSITION_BUFFER_SIZE);
    error_flag_ = false;
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

bool SPIInterface::dataAvailable() {
    return spi_is_readable(spi_port_);
}

void SPIInterface::setErrorFlag(bool error_state) {
    error_flag_ = error_state;
}

void SPIInterface::processCommands() {
    // This function always deals with FIRST byte of message.
    // Handlers process the rest.
        // If in error state, return error for any command
        if (error_flag_) {
            // Check if data is available to respond with error
            if (dataAvailable()) {
                // Read the command byte (but ignore it)
                uint8_t command = spiReadByte();
                
                // Send error response
                spiWriteBuffer("e", 2); // "e" + null terminator
            }
            return;
        }

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
        case PID_TUNE:
            handlePidTuneCommand();
            break;
        case NULL_BYTE:
            // printf("Null-terminator received.\n");
            break;
        case DUMMY:
            // printf("Dummy byte received.\n");
            break;
        default:
            printf("Unknown command received: 0x%02x\n", command);
            break;
    }
}

void SPIInterface::handleCalibrationCommand() {
    // // Skip the null
    // spiReadByte();
    
    // // printf("Calibration command received\n");
    
    // // Call the callback if registered
    // if (calibration_callback_) {
    //     calibration_callback_();
    // }

    // Skip the null
    spiReadByte();
    
    // Check calibration status and respond
    bool isCalibrated = false;
    if (calibration_callback_) {
        isCalibrated = calibration_callback_();
    }
    
    if (isCalibrated) {
        // Send 'c' + null terminator to indicate calibration complete
        char calibrated_response[2] = {'c', '\0'};
        spiWriteBuffer(calibrated_response, 2);
        printf("Sending calibration complete response\n");
    } else {
        // Send dummy byte to indicate in progress
        uint8_t response = DUMMY;
        spi_write_blocking(spi_port_, &response, 1);
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
    // This is for case when buffer size is reached.
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
        // printf("Failed to parse position value: %s\n", recv_buffer_);
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

void SPIInterface::handlePidTuneCommand() {
    // Read the motor identifier (l or r)
    uint8_t motor = spiReadByte();
    
    if (!IS_VALID_MOTOR(motor)) {
        printf("Invalid motor identifier: 0x%02x\n", motor);
        return;
    }
    
    // Read the PID parameters until null terminator
    uint8_t byte = 0xFF;
    uint8_t recv_index = 0;
    
    while (byte != NULL_BYTE && recv_index < BUFFER_SIZE - 1) {
        byte = spiReadByte();
        recv_buffer_[recv_index++] = byte;
    }
    
    // Null-terminate the received string
    recv_buffer_[recv_index - 1] = '\0';
    
    // Parse the PID parameters (format: "kp,ki,kd")
    float kp, ki, kd;
    if (sscanf((char*)recv_buffer_, "%f,%f,%f", &kp, &ki, &kd) == 3) {
        printf("PID tune command: motor=%c, kp=%.2f, ki=%.2f, kd=%.2f\n", 
               static_cast<char>(motor), kp, ki, kd);
        
        // Call the callback if registered
        if (pid_tune_callback_) {
            pid_tune_callback_(static_cast<char>(motor), kp, ki, kd);
        }
    } else {
        printf("Failed to parse PID parameters: %s\n", recv_buffer_);
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

void SPIInterface::setCalibrationCallback(CalibrationCallback callback) {
    calibration_callback_ = callback;
}

void SPIInterface::setPositionCommandCallback(PositionCommandCallback callback) {
    position_command_callback_ = callback;
}

void SPIInterface::setPositionStateCallback(PositionStateCallback callback) {
    position_state_callback_ = callback;
}

void SPIInterface::setPidTuneCallback(PidTuneCallback callback) {
    pid_tune_callback_ = callback;
}