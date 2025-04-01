#pragma once

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "TrapezoidalStepper.h"
#include "SPIInterface.h"

// LED Configuration
#define LED_PIN PICO_DEFAULT_LED_PIN
#define BLINK_INTERVAL_US 500 * 1000

// System update intervals
#define POSITION_UPDATE_INTERVAL_US 10 * 1000

class MotorController {
public:
    // Constructor
    MotorController();
    
    // Initialization
    void init();
    
    // Main loop (call this in a loop from main())
    void update();
    
private:
    // Hardware components
    TrapezoidalStepper left_stepper_;
    // TrapezoidalStepper right_stepper_;
    SPIInterface spi_interface_;
    
    // Timing variables
    absolute_time_t last_position_update_;
    absolute_time_t last_blink_;
    bool blink_state_;
    
    // LED control
    void initLED();
    void updateLED();
    
    // SPI command handlers
    void onCalibrationCommand();
    bool onPositionCommand(char motor, float position);
    float onPositionStateRequest(char motor);
    
    // Motor update function
    void monitorMotors();
};