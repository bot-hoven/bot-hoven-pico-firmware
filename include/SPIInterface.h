#pragma once

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <functional>
#include <string>

// SPI Command definitions
#define CALIBRATE 0x63       // 'c'
#define COMMAND_POSITION 0x70 // 'p'
#define STATE_POSITION 0x72  // 'r'
#define PID_TUNE 0x74        // 't' for tune
#define ERROR 0x65           // 'e'
#define NULL_BYTE 0x00
#define DUMMY 0xFF
#define LEFT_STEPPER_MOTOR 0x6C  // 'l'
#define RIGHT_STEPPER_MOTOR 0x72 // 'r'
#define IS_VALID_MOTOR(m) ((m) == LEFT_STEPPER_MOTOR || (m) == RIGHT_STEPPER_MOTOR)

class SPIInterface {
public:
    // Constructor
    SPIInterface(
        spi_inst_t* spi_port,
        uint8_t miso_pin,
        uint8_t cs_pin,
        uint8_t sck_pin,
        uint8_t mosi_pin,
        uint32_t clock_speed_hz = 500 * 1000,
        uint8_t data_bits = 8
    );
    
    // Initialize SPI as slave
    void init();
    
    // Check if data is available to read
    bool dataAvailable();
    
    // Process incoming commands (call this regularly from main loop)
    void processCommands();
    
    // Response formatting
    static const int POSITION_BUFFER_SIZE = 32;
    
    // Callback function types
    using CalibrationCallback = std::function<void()>;
    using PositionCommandCallback = std::function<bool(char motor, float position)>;
    using PositionStateCallback = std::function<float(char motor)>;
    using PidTuneCallback = std::function<bool(char motor, float kp, float ki, float kd)>;
    
    // Set callbacks for various commands
    void setCalibrationCallback(CalibrationCallback callback);
    void setPositionCommandCallback(PositionCommandCallback callback);
    void setPositionStateCallback(PositionStateCallback callback);
    void setPidTuneCallback(PidTuneCallback callback);
    
private:
    // SPI configuration
    spi_inst_t* spi_port_;
    uint8_t miso_pin_;
    uint8_t cs_pin_;
    uint8_t sck_pin_;
    uint8_t mosi_pin_;
    uint32_t clock_speed_hz_;
    uint8_t data_bits_;
    
    // Buffers
    static const int BUFFER_SIZE = 256;
    uint8_t recv_buffer_[BUFFER_SIZE];
    char response_buffer_[POSITION_BUFFER_SIZE];
    
    // Handle specific commands
    void handleCalibrationCommand();
    void handlePositionCommand();
    void handlePositionStateRequest();
    void handlePidTuneCommand();
    
    // Callbacks for processing commands
    CalibrationCallback calibration_callback_;
    PositionCommandCallback position_command_callback_;
    PositionStateCallback position_state_callback_;
    PidTuneCallback pid_tune_callback_;
    
    // SPI utility functions
    uint8_t spiReadByte();
    void spiWriteBuffer(const char* buffer, size_t length);
};