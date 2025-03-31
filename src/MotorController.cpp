#include "include/MotorController.h"

#include <cstdio>

// Hardware Pin definitions - Left Stepper
#define LEFT_PULSE_PIN 10
#define LEFT_DIR_PIN 11
#define LEFT_ENABLE_PIN -1
#define LEFT_LIMIT_LEFT_PIN 2
#define LEFT_LIMIT_RIGHT_PIN 3

// Hardware Pin definitions - Right Stepper
#define RIGHT_PULSE_PIN 12
#define RIGHT_DIR_PIN 13
#define RIGHT_ENABLE_PIN -1
#define RIGHT_LIMIT_LEFT_PIN 4
#define RIGHT_LIMIT_RIGHT_PIN 5

// SPI Configuration
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define SPI_PORT spi0
#define SPI_CLOCK_SPEED_HZ (500 * 1000)

// Rail configuration (in meters)
#define TOTAL_RAIL_LENGTH 1.219f
#define LEFT_HAND_WIDTH 0.127f
#define RIGHT_HAND_WIDTH 0.127f
#define HOME_OFFSET 0.0f

// Initial estimate for calibration (will be recalculated)
#define INITIAL_STEPS_PER_METER 10000

// Motion parameters
#define MAX_VELOCITY 48000.0f
#define ACCELERATION 24000.0f
#define DECELERATION 24000.0f

MotorController* MotorController::instance = nullptr;
TrapezoidalStepper* TrapezoidalStepper::stepper_instances[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

static void core1_entry_wrapper() { MotorController::instance->core1_entry(); }

MotorController::MotorController()
    : left_stepper_(LEFT_PULSE_PIN, LEFT_DIR_PIN, LEFT_ENABLE_PIN, LEFT_LIMIT_LEFT_PIN, LEFT_LIMIT_RIGHT_PIN,
                    INITIAL_STEPS_PER_METER, TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET),
      right_stepper_(RIGHT_PULSE_PIN, RIGHT_DIR_PIN, RIGHT_ENABLE_PIN, RIGHT_LIMIT_LEFT_PIN, RIGHT_LIMIT_RIGHT_PIN,
                     INITIAL_STEPS_PER_METER, TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET),
      spi_interface_(SPI_PORT, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI, SPI_CLOCK_SPEED_HZ),
      blink_state_(false) {
    instance = this;
    core1_ready = false;
    motor_command_pending = false;
}

void MotorController::init() {
    // Initialize stdio for debugging output
    stdio_init_all();
    sleep_ms(2000);  // Wait for serial connection to stabilize

    // Initialize LED
    initLED();

    // Configure the stepper motors
    left_stepper_.setMaxVelocity(MAX_VELOCITY);
    left_stepper_.setAcceleration(ACCELERATION);
    left_stepper_.setDeceleration(DECELERATION);
    left_stepper_.initTimerIRQ(0, 0);  // Use TIMER_IRQ_0 for left stepper

    right_stepper_.setMaxVelocity(MAX_VELOCITY);
    right_stepper_.setAcceleration(ACCELERATION);
    right_stepper_.setDeceleration(DECELERATION);
    right_stepper_.initTimerIRQ(1, 0);  // Use TIMER_IRQ_1 for right stepper

    // Initialize SPI interface
    spi_interface_.init();

    // Register SPI callbacks
    spi_interface_.setCalibrationCallback([this]() { this->onCalibrationCommand(); });
    spi_interface_.setPositionCommandCallback(
        [this](char motor, float position) { return this->onPositionCommand(motor, position); });
    spi_interface_.setPositionStateCallback([this](char motor) { return this->onPositionStateRequest(motor); });
    spi_interface_.setHomeCallback([this]() { this->onHomeCommand(); });

    // Initialize timing variables
    last_position_update_ = get_absolute_time();
    last_blink_ = get_absolute_time();

    // // Start core1 for motor control
    // multicore_launch_core1(core1_entry_wrapper);

    // // Wait for core1 to be ready
    // while (!core1_ready) {
    //     tight_loop_contents();
    // }

    printf("Initialization complete. Ready to receive commands.\n");
}

void MotorController::update_core0() {
    // Process SPI commands
    spi_interface_.processCommands();

    // Update LED
    updateLED();
}

void MotorController::core1_entry_static() {
    // Ensure instance is set correctly
    if (instance != nullptr) {
        instance->core1_entry();
    } else {
        printf("ERROR: MotorController instance not set!\n");
        while(true) { tight_loop_contents(); }
    }
}

void MotorController::core1_entry() {
    printf("Core 1 started\n");
    
    // Signal that core1 is ready
    core1_ready = true;
    
    // Main loop for core1
    while (true) {
        if (motor_command_pending) {
            // Handle motor commands
            // ...
        }
        sleep_ms(1000);
    }
}

// void MotorController::core1_entry() {
//     // Core 1 initialization
//     printf("Core 1: Motor control system initializing...\n");

//     // Signal that core1 is ready
//     core1_ready = true;

//     // Main loop for core1
//     while (true) {
//         if (motor_command_pending) {
//             // Process pending motor command
//             if (motor_command_target == 'l') {
//                 left_stepper_.moveTo(motor_command_position);
//                 while (!left_stepper_.isMotionComplete()) {
//                     sleep_us(50);  // Small delay
//                 }
//             } else if (motor_command_target == 'r') {
//                 right_stepper_.moveTo(motor_command_position);
//                 while (!right_stepper_.isMotionComplete()) {
//                     sleep_us(50);  // Small delay
//                 }
//             }

//             // Mark command as processed
//             motor_command_pending = false;
//         }

//         // Small delay to prevent tight looping
//         sleep_us(100);
//     }
// }

// void MotorController::update() {
//     // Process SPI commands
//     spi_interface_.processCommands();

//     // // Update motors at a fixed rate
//     // if (absolute_time_diff_us(last_position_update_, get_absolute_time()) > POSITION_UPDATE_INTERVAL_US) {
//     //     updateMotors();
//     //     last_position_update_ = get_absolute_time();
//     // }

//     // Update LED status (blink to show system is alive)
//     updateLED();
// }

void MotorController::initLED() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
}

void MotorController::updateLED() {
    if (absolute_time_diff_us(last_blink_, get_absolute_time()) > BLINK_INTERVAL_US) {
        blink_state_ = !blink_state_;
        gpio_put(LED_PIN, blink_state_);
        last_blink_ = get_absolute_time();
    }
}

void MotorController::onCalibrationCommand() {
    // printf("Calibration command received. Calibrating both motors...\n");

    // Calibrate left stepper
    // printf("Calibrating left stepper...\n");
    if (!left_stepper_.calibrate()) {
        printf("ERROR: Left stepper calibration failed: %s\n", left_stepper_.getErrorMessage());
        return;
    }

    // Calibrate right stepper
    // printf("Calibrating right stepper...\n");
    // if (!right_stepper_.calibrate()) {
    //     printf("ERROR: Right stepper calibration failed: %s\n", right_stepper_.getErrorMessage());
    //     return;
    // }

    // printf("Calibration complete for both motors.\n");
}

// bool MotorController::onPositionCommand(char motor, float position) {
//     bool result = false;

//     if (motor == 'l') {
//         // printf("Moving left stepper to position %.3f\n", position);
//         result = left_stepper_.moveTo(position);

//         if (!result) {
//             printf("ERROR: Failed to move left stepper: %s\n", left_stepper_.getErrorMessage());
//         }

//         // Wait for motion to complete before returning
//         // while (!left_stepper_.isMotionComplete()) {
//             // Small delay to prevent tight looping
//             // sleep_us(50);
//         // }

//     } else if (motor == 'r') {
//         result = true;
//         // printf("Moving right stepper to position %.3f\n", position);
//         // result = right_stepper_.moveTo(position);
//         // if (!result) {
//         //     printf("ERROR: Failed to move right stepper: %s\n", right_stepper_.getErrorMessage());
//         // }
//     }

//     return result;
// }

bool MotorController::onPositionCommand(char motor, float position) {
    printf("Position command received: motor=%c, position=%.3f\n", motor, position);

    // Validate motor
    if (motor != 'l' && motor != 'r') {
        return false;
    }

    // Set up command for core1
    motor_command_target = motor;
    motor_command_position = position;
    motor_command_pending = true;

    // Wait for command to be picked up (or use a timeout)
    uint32_t timeout_ms = 5000;  // 5 second timeout
    absolute_time_t timeout_time = make_timeout_time_ms(timeout_ms);

    while (motor_command_pending) {
        // Check for timeout
        if (absolute_time_diff_us(get_absolute_time(), timeout_time) <= 0) {
            printf("ERROR: Command timeout\n");
            motor_command_pending = false;
            return false;
        }

        sleep_us(50);  // Small delay
    }

    return true;
}

float MotorController::onPositionStateRequest(char motor) {
    if (motor == 'l') {
        return left_stepper_.getCurrentPosition();
    } else if (motor == 'r') {
        // return right_stepper_.getCurrentPosition();
    }

    return 0.0f;
}

void MotorController::onHomeCommand() {
    // printf("Home command received. Homing both motors...\n");

    // Home left stepper
    // printf("Homing left stepper...\n");
    if (!left_stepper_.homePosition()) {
        printf("ERROR: Left stepper homing failed: %s\n", left_stepper_.getErrorMessage());
        return;
    }

    // Home right stepper
    printf("Homing right stepper...\n");
    if (!right_stepper_.homePosition()) {
        printf("ERROR: Right stepper homing failed: %s\n", right_stepper_.getErrorMessage());
        return;
    }

    // printf("Homing complete for both motors.\n");
}

void MotorController::updateMotors() {
    // This function is intentionally left empty as the TrapezoidalStepper class
    // handles motion updates through its own alarm callbacks.
    //
    // If we needed additional motor state updates or monitoring, we would
    // implement them here.
}









// #include "include/MotorController.h"

// #include <cstdio>

// // Debug mode flag - set to 1 to enable debug prints, 0 to disable
// #define DEBUG_MODE 0

// // Hardware Pin definitions - Left Stepper
// #define LEFT_PULSE_PIN 10
// #define LEFT_DIR_PIN 11
// #define LEFT_ENABLE_PIN -1
// #define LEFT_LIMIT_LEFT_PIN 2
// #define LEFT_LIMIT_RIGHT_PIN 3

// // Hardware Pin definitions - Right Stepper
// #define RIGHT_PULSE_PIN 12
// #define RIGHT_DIR_PIN 13
// #define RIGHT_ENABLE_PIN -1
// #define RIGHT_LIMIT_LEFT_PIN 4
// #define RIGHT_LIMIT_RIGHT_PIN 5

// // SPI Configuration
// #define PIN_MISO 16
// #define PIN_CS 17
// #define PIN_SCK 18
// #define PIN_MOSI 19
// #define SPI_PORT spi0
// #define SPI_CLOCK_SPEED_HZ (500 * 1000)

// // Rail configuration (in meters)
// #define TOTAL_RAIL_LENGTH 1.219f
// #define LEFT_HAND_WIDTH 0.127f
// #define RIGHT_HAND_WIDTH 0.127f
// #define HOME_OFFSET 0.0f

// // Initial estimate for calibration (will be recalculated)
// #define INITIAL_STEPS_PER_METER 10000

// // Motion parameters
// #define MAX_VELOCITY 48000.0f
// #define ACCELERATION 24000.0f
// #define DECELERATION 24000.0f

// MotorController* MotorController::instance = nullptr;
// TrapezoidalStepper* TrapezoidalStepper::stepper_instances[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

// /**
//  * @brief Static entry point for core1 execution
//  * 
//  * This is a wrapper that calls the core1_entry method of the MotorController instance.
//  */
// static void core1_entry_wrapper() { MotorController::instance->core1_entry(); }

// /**
//  * @brief Constructor for MotorController
//  * 
//  * Initializes the hardware components including both stepper motors and the SPI interface.
//  * Sets up static instance pointer for access from core1.
//  */
// MotorController::MotorController()
//     : left_stepper_(LEFT_PULSE_PIN, LEFT_DIR_PIN, LEFT_ENABLE_PIN, LEFT_LIMIT_LEFT_PIN, LEFT_LIMIT_RIGHT_PIN,
//                     INITIAL_STEPS_PER_METER, TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET),
//       right_stepper_(RIGHT_PULSE_PIN, RIGHT_DIR_PIN, RIGHT_ENABLE_PIN, RIGHT_LIMIT_LEFT_PIN, RIGHT_LIMIT_RIGHT_PIN,
//                      INITIAL_STEPS_PER_METER, TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET),
//       spi_interface_(SPI_PORT, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI, SPI_CLOCK_SPEED_HZ),
//       blink_state_(false) {
//     instance = this;
//     core1_ready = false;
//     motor_command_pending = false;
// }

// /**
//  * @brief Initializes the MotorController
//  * 
//  * Sets up stdio, LED, stepper motors, and SPI interface.
//  * Registers callbacks for SPI commands and initializes timing variables.
//  */
// void MotorController::init() {
//     // Initialize stdio for debugging output
//     stdio_init_all();
//     sleep_ms(2000);  // Wait for serial connection to stabilize

//     // Initialize LED
//     initLED();

//     // Configure the stepper motors
//     left_stepper_.setMaxVelocity(MAX_VELOCITY);
//     left_stepper_.setAcceleration(ACCELERATION);
//     left_stepper_.setDeceleration(DECELERATION);
//     left_stepper_.initTimerIRQ(0, 0);  // Use TIMER_IRQ_0 for left stepper

//     right_stepper_.setMaxVelocity(MAX_VELOCITY);
//     right_stepper_.setAcceleration(ACCELERATION);
//     right_stepper_.setDeceleration(DECELERATION);
//     right_stepper_.initTimerIRQ(1, 0);  // Use TIMER_IRQ_1 for right stepper

//     // Initialize SPI interface
//     spi_interface_.init();

//     // Register SPI callbacks
//     spi_interface_.setCalibrationCallback([this]() { this->onCalibrationCommand(); });
//     spi_interface_.setPositionCommandCallback(
//         [this](char motor, float position) { return this->onPositionCommand(motor, position); });
//     spi_interface_.setPositionStateCallback([this](char motor) { return this->onPositionStateRequest(motor); });
//     spi_interface_.setHomeCallback([this]() { this->onHomeCommand(); });

//     // Initialize timing variables
//     last_position_update_ = get_absolute_time();
//     last_blink_ = get_absolute_time();

//     if (DEBUG_MODE) {
//         printf("Initialization complete. Ready to receive commands.\n");
//     }
// }

// /**
//  * @brief Update function for core0
//  * 
//  * Processes SPI commands and updates the LED status.
//  * Should be called regularly from the main loop in core0.
//  */
// void MotorController::update_core0() {
//     // Process SPI commands
//     spi_interface_.processCommands();

//     // Update LED
//     updateLED();
// }

// /**
//  * @brief Static entry point for core1
//  * 
//  * Ensures the MotorController instance is set and calls its core1_entry method.
//  */
// void MotorController::core1_entry_static() {
//     // Ensure instance is set correctly
//     if (instance != nullptr) {
//         instance->core1_entry();
//     } else {
//         printf("ERROR: MotorController instance not set!\n");
//         while(true) { tight_loop_contents(); }
//     }
// }

// /**
//  * @brief Core1 entry point
//  * 
//  * Handles motor commands on core1. This is currently a placeholder
//  * that prints a message and sets core1_ready.
//  */
// void MotorController::core1_entry() {
//     if (DEBUG_MODE) {
//         printf("Core 1 started\n");
//     }
    
//     // Signal that core1 is ready
//     core1_ready = true;
    
//     // Main loop for core1
//     while (true) {
//         if (motor_command_pending) {
//             if (motor_command_target == 'l')
//                 left_stepper_.moveTo(motor_command_position);
//             // else {
//             //     result = right_stepper_.moveTo(motor_command_position);
//             // }
//             motor_command_pending = false;
//         }
//         sleep_ms(1000);
//     }
// }

// /**
//  * @brief Initializes the onboard LED
//  * 
//  * Sets up the LED pin as output and initializes it to off.
//  */
// void MotorController::initLED() {
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);
//     gpio_put(LED_PIN, 0);
// }

// /**
//  * @brief Updates the LED status
//  * 
//  * Blinks the LED at the specified interval to indicate system status.
//  */
// void MotorController::updateLED() {
//     if (absolute_time_diff_us(last_blink_, get_absolute_time()) > BLINK_INTERVAL_US) {
//         blink_state_ = !blink_state_;
//         gpio_put(LED_PIN, blink_state_);
//         last_blink_ = get_absolute_time();
//     }
// }

// /**
//  * @brief Handles calibration commands
//  * 
//  * Calibrates the left stepper. Right stepper calibration is commented out.
//  */
// void MotorController::onCalibrationCommand() {
//     if (!left_stepper_.calibrate()) {
//         printf("ERROR: Left stepper calibration failed: %s\n", left_stepper_.getErrorMessage());
//         return;
//     }
// }

// /**
//  * @brief Handles position commands
//  * 
//  * Sets up a position command for execution by core1.
//  * Uses a timeout to prevent blocking indefinitely.
//  * 
//  * @param motor The motor identifier ('l' for left, 'r' for right)
//  * @param position The target position in meters
//  * @return true if command was accepted, false otherwise
//  */
// bool MotorController::onPositionCommand(char motor, float position) {
//     if (DEBUG_MODE) {
//         printf("Position command received: motor=%c, position=%.3f\n", motor, position);
//     }

//     // Validate motor
//     if (motor != 'l' && motor != 'r') {
//         return false;
//     }

//     // Set up command for core1
//     motor_command_target = motor;
//     motor_command_position = position;
//     motor_command_pending = true;

//     // Wait for command to be picked up (or use a timeout)
//     uint32_t timeout_ms = 5000;  // 5 second timeout
//     absolute_time_t timeout_time = make_timeout_time_ms(timeout_ms);

//     while (motor_command_pending) {
//         // Check for timeout
//         if (absolute_time_diff_us(get_absolute_time(), timeout_time) <= 0) {
//             printf("ERROR: Command timeout\n");
//             motor_command_pending = false;
//             return false;
//         }

//         sleep_us(50);  // Small delay
//     }

//     return true;
// }

// /**
//  * @brief Handles position state requests
//  * 
//  * Returns the current position of the specified motor.
//  * 
//  * @param motor The motor identifier ('l' for left, 'r' for right)
//  * @return The current position in meters
//  */
// float MotorController::onPositionStateRequest(char motor) {
//     if (motor == 'l') {
//         return left_stepper_.getCurrentPosition();
//     } else if (motor == 'r') {
//         // TODO: Implement right stepper position state
//         return 0.0f;
//     }

//     return 0.0f;
// }

// /**
//  * @brief Handles home commands
//  * 
//  * Homes both the left and right stepper motors.
//  */
// void MotorController::onHomeCommand() {
//     if (!left_stepper_.homePosition()) {
//         printf("ERROR: Left stepper homing failed: %s\n", left_stepper_.getErrorMessage());
//         return;
//     }

//     if (DEBUG_MODE) {
//         printf("Homing right stepper...\n");
//     }
    
//     if (!right_stepper_.homePosition()) {
//         printf("ERROR: Right stepper homing failed: %s\n", right_stepper_.getErrorMessage());
//         return;
//     }
// }

// void MotorController::updateMotors() {
//     // This function is intentionally left empty as the TrapezoidalStepper class
//     // handles motion updates through its own alarm callbacks.
//     //
//     // If we needed additional motor state updates or monitoring, we would
//     // implement them here.
// }
















