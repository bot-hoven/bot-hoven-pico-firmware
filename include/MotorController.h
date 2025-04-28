#pragma once

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "SimplifiedStepper.h"
#include "SPIInterface.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

// LED Configuration
#define LED_PIN PICO_DEFAULT_LED_PIN
#define BLINK_INTERVAL_US 500 * 1000

// System update intervals
#define POSITION_UPDATE_INTERVAL_US 10 * 1000

enum CalibrationState {
    CALIBRATION_STATE_UNCALIBRATED = 0,
    CALIBRATION_STATE_IN_PROGRESS = 1,
    CALIBRATION_STATE_COMPLETED = 2
};

static volatile uint32_t g_emergency_stop_atomic;

// Emergency stop callback
void limit_switch_callback(uint gpio, uint32_t events);

class MotorController {
public:
    // Constructor
    MotorController();
    
    // Initialization
    void init();
    
    // Core management
    static void core1_entry_static();
    void core1_entry();
    void update_core0();
    
    // Calibration functions
    bool onCalibrationCommand();
    bool calibrateLeftMotor();
    bool calibrateRightMotor();
    
    // Emergency stop functions
    void initEmergencyStop();
    void emergencyStop();
    bool isEmergencyStopActive() const;
    void clearEmergencyStop();
    
    // Calibration status check - used by limit switch ISR
    bool isCalibrationComplete() const;
    
    static MotorController* instance;
    
private:
    // Hardware components
    SimplifiedStepper left_stepper_;
    SimplifiedStepper right_stepper_;
    SPIInterface spi_interface_;
    
    // Timing variables
    absolute_time_t last_position_update_;
    absolute_time_t last_blink_;
    bool blink_state_;

    // Multicore control
    volatile bool core1_ready;
    volatile bool motor_command_pending;
    volatile char motor_command_target;
    volatile float motor_command_position;
    volatile CalibrationState calibration_state;
    volatile bool calibration_complete;
    spin_lock_t *left_motor_spin_lock;
    
    // Emergency stop state
    volatile bool emergency_stop_active;
    spin_lock_t *emergency_spin_lock;
    
    // LED control
    void initLED();
    void updateLED();
    
    // SPI command handlers
    bool onPositionCommand(char motor, float position);
    float onPositionStateRequest(char motor);
    bool onPidTuneCommand(char motor, float kp, float ki, float kd);
    
    // Friend declarations
    friend void limit_switch_callback(uint gpio, uint32_t events);
};