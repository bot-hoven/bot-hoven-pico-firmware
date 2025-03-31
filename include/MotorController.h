#pragma once

#include "SPIInterface.h"
#include "TrapezoidalStepper.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

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

    // Core1 entry point
    static void core1_entry_static();
    void core1_entry();
    void update_core0();

    // Static instance pointer for core1 access
    static MotorController* instance;

private:
    // Hardware components
    TrapezoidalStepper left_stepper_;
    TrapezoidalStepper right_stepper_;
    SPIInterface spi_interface_;

    // Multicore control
    bool core1_ready;
    bool motor_command_pending;
    char motor_command_target;
    float motor_command_position;

    // Timing variables
    absolute_time_t last_position_update_;
    absolute_time_t last_blink_;
    bool blink_state_;

    // LED control
    void initLED();
    void updateLED();

    // SPI command handlers
    void onCalibrationCommand();
    void onHomeCommand();
    bool onPositionCommand(char motor, float position);
    float onPositionStateRequest(char motor);

    // Motor update function
    void updateMotors();
};