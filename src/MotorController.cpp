#include "include/MotorController.h"

#include <cstdio>

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

// Hardware Pin definitions - Left Stepper
#define LEFT_PULSE_PIN 10
#define LEFT_DIR_PIN 11
#define LEFT_ENABLE_PIN 12
#define LEFT_LIMIT_LEFT_PIN 2
#define LEFT_LIMIT_RIGHT_PIN 3

// Hardware Pin definitions - Right Stepper
#define RIGHT_PULSE_PIN 6
#define RIGHT_DIR_PIN 7
#define RIGHT_ENABLE_PIN 8
#define RIGHT_LIMIT_LEFT_PIN 4
#define RIGHT_LIMIT_RIGHT_PIN 5

// SPI Configuration
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define SPI_PORT spi0
#define SPI_CLOCK_SPEED_HZ (500 * 1000)

// Error message for SPI
#define ERROR_MESSAGE "e"

// Limit switch interrupt callback
void limit_switch_callback(uint gpio, uint32_t events) {
    // Only trigger if instance exists and calibration is complete
    if (MotorController::instance == nullptr) {
        return;  // Exit early if instance isn't set
    }

    // Use a simple bool check rather than calling a method which might be complex
    if (MotorController::instance->calibration_state == CALIBRATION_STATE_COMPLETED) {
        // Set global emergency flag - implementation in controller happens later
        g_emergency_stop_atomic = true;
    }
}

// Rail configuration (in meters)
#define TOTAL_RAIL_LENGTH 1.235075f
#define LEFT_HAND_WIDTH 0.2032f
#define RIGHT_HAND_WIDTH 0.2032f
#define HOME_OFFSET 0.164f

// Initial estimate for calibration (will be recalculated)
#define INITIAL_STEPS_PER_METER 10000

// PID parameters
#define PID_KP 0.75f  // Proportional gain
#define PID_KI 0.0f   // Integral gain
#define PID_KD 0.03f  // Derivative gain

MotorController* MotorController::instance = nullptr;

// Global variables to signal between cores
volatile bool g_calibrate_right_motor = false;
volatile bool g_right_motor_calibration_complete = false;

MotorController::MotorController()
    : left_stepper_(LEFT_PULSE_PIN, LEFT_DIR_PIN, LEFT_ENABLE_PIN, LEFT_LIMIT_LEFT_PIN, LEFT_LIMIT_RIGHT_PIN,
                    INITIAL_STEPS_PER_METER, TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET, 50000,
                    2, LEFT_STEPPER_ALARM_NUM),
      right_stepper_(RIGHT_PULSE_PIN, RIGHT_DIR_PIN, RIGHT_ENABLE_PIN, RIGHT_LIMIT_LEFT_PIN, RIGHT_LIMIT_RIGHT_PIN,
                     INITIAL_STEPS_PER_METER, TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET, 50000,
                     2, RIGHT_STEPPER_ALARM_NUM),
      spi_interface_(SPI_PORT, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI, SPI_CLOCK_SPEED_HZ),
      blink_state_(false) {
    instance = this;
    core1_ready = false;
    motor_command_pending = false;
    // calibration_complete = false; // Core 1 won't control right motor until calibration is done
    calibration_state = CALIBRATION_STATE_UNCALIBRATED;
    emergency_stop_active = false;
    g_emergency_stop_atomic = 0;
}

// In MotorController.cpp
void MotorController::init() {
    // Initialize LED
    printf("Initializing system...\n");
    initLED();

    // Initialize spin lock
    uint spin_lock_num = next_striped_spin_lock_num();
    emergency_spin_lock = spin_lock_init(spin_lock_num);

    // Configure left stepper first
    printf("Configuring left stepper...\n");
    left_stepper_.setPidParameters(PID_KP, PID_KI, PID_KD);

    // Signal with LED that left stepper is initialized
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);

    // Initialize SPI interface
    printf("Initializing SPI interface...\n");
    spi_interface_.init();

    // Signal with LED that SPI is initialized
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);

    // Configure right stepper with delay between steps
    printf("Configuring right stepper...\n");
    sleep_ms(100);  // Pause before configuring right stepper
    right_stepper_.setPidParameters(PID_KP, PID_KI, PID_KD);

    // Signal with LED that right stepper is initialized
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);

    // Register SPI callbacks
    printf("Setting up SPI callbacks...\n");
    spi_interface_.setCalibrationCallback([this]() { return this->onCalibrationCommand(); });
    spi_interface_.setPositionCommandCallback(
        [this](char motor, float position) { return this->onPositionCommand(motor, position); });
    spi_interface_.setPositionStateCallback([this](char motor) { return this->onPositionStateRequest(motor); });
    spi_interface_.setPidTuneCallback(
        [this](char motor, float kp, float ki, float kd) { return this->onPidTuneCommand(motor, kp, ki, kd); });

    // Initialize GPIO for limit switches, but don't enable interrupts yet
    printf("Setting up limit switch GPIO pins...\n");
    gpio_set_function(LEFT_LIMIT_LEFT_PIN, GPIO_FUNC_SIO);
    gpio_set_function(LEFT_LIMIT_RIGHT_PIN, GPIO_FUNC_SIO);
    gpio_set_function(RIGHT_LIMIT_LEFT_PIN, GPIO_FUNC_SIO);  // Fixed from NULL
    gpio_set_function(RIGHT_LIMIT_RIGHT_PIN, GPIO_FUNC_SIO);

    gpio_set_dir(LEFT_LIMIT_LEFT_PIN, GPIO_IN);
    gpio_set_dir(LEFT_LIMIT_RIGHT_PIN, GPIO_IN);
    gpio_set_dir(RIGHT_LIMIT_LEFT_PIN, GPIO_IN);  // Fixed from NULL
    gpio_set_dir(RIGHT_LIMIT_RIGHT_PIN, GPIO_IN);

    // Set up the callback without enabling interrupts
    gpio_set_irq_callback(&limit_switch_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Initialize timing variables
    last_blink_ = get_absolute_time();

    left_motor_spin_lock = spin_lock_init(next_striped_spin_lock_num());

    printf("Initialization complete. Ready to receive commands.\n");
}

void MotorController::update_core0() {
    // Check for emergency stop condition
    if (g_emergency_stop_atomic && !emergency_stop_active) {
        emergencyStop();
    }

    // Process SPI commands
    spi_interface_.processCommands();

    // Update left stepper only if not in emergency stop
    if (!emergency_stop_active) {
        static absolute_time_t last_update = get_absolute_time();
        absolute_time_t now = get_absolute_time();

        // if (absolute_time_diff_us(last_update, now) > 5000) {
            uint32_t save = spin_lock_blocking(left_motor_spin_lock);
            left_stepper_.update();
            spin_unlock(left_motor_spin_lock, save);
            sleep_us(50);
        //     last_update = now;
        // }
    }

    // Update LED
    updateLED();
}

void MotorController::core1_entry_static() {
    // Ensure instance is set correctly
    if (instance != nullptr) {
        instance->core1_entry();
    } else {
        printf("ERROR: MotorController instance not set!\n");
        while (true) {
            tight_loop_contents();
        }
    }
}

void MotorController::core1_entry() {
    printf("Core 1 started\n");

    // Signal that core1 is ready
    core1_ready = true;

    // Main loop for core1
    while (true) {
        // Check for emergency stop condition
        if (g_emergency_stop_atomic && !emergency_stop_active) {
            // Emergency stop is handled by core0
            sleep_ms(10);
            continue;
        } else {
            if (calibration_state == CALIBRATION_STATE_COMPLETED) {
                // Process any position commands for right motor
                if (motor_command_pending && motor_command_target == 'r') {
                    bool result = false;
                    printf("Moving right stepper to position %.3f\n", motor_command_position);
                    result = right_stepper_.moveTo(motor_command_position);
                    if (!result) {
                        printf("ERROR: Failed to move right stepper: %s\n", right_stepper_.getErrorMessage());
                    }
                    motor_command_pending = false;
                }
                right_stepper_.update();
            } else {
                sleep_ms(10);
                continue;
            }
        }
    }
}

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

bool MotorController::onCalibrationCommand() {
    printf("Calibration command received.\n");

    // If already calibrated, return true without restarting
    if (calibration_state == CALIBRATION_STATE_COMPLETED) {
        printf("Already calibrated.\n");
        return true;
    }

    // If already in progress, return false to indicate not yet complete
    if (calibration_state == CALIBRATION_STATE_IN_PROGRESS) {
        printf("Calibration in progress.\n");
        return false;
    }

    // If we reach here, we need to start calibration
    printf("Starting coordinated calibration sequence...\n");
    calibration_state = CALIBRATION_STATE_IN_PROGRESS;

    // Disable emergency stop interrupts during calibration
    gpio_set_irq_enabled(LEFT_LIMIT_LEFT_PIN, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(LEFT_LIMIT_RIGHT_PIN, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(RIGHT_LIMIT_LEFT_PIN, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(RIGHT_LIMIT_RIGHT_PIN, GPIO_IRQ_EDGE_RISE, false);

    // Stage 1: First move right motor to its right limit
    printf("1. Moving right motor to its right limit...\n");
    if (!right_stepper_.moveToLimit(RIGHT_LIMIT_RIGHT_PIN, SimplifiedStepper::CW)) {
        printf("ERROR: Right stepper failed to reach right limit: %s\n", right_stepper_.getErrorMessage());
        calibration_state = CALIBRATION_STATE_UNCALIBRATED;
        return false;
    }
    printf("Right motor at right limit position.\n");

    // Stage 2: Calibrate left motor completely (left limit → right limit → left limit)
    printf("2. Starting left motor calibration sequence...\n");
    if (!calibrateLeftMotor()) {
        printf("ERROR: Calibration failed during left stepper calibration sequence.");
        calibration_state = CALIBRATION_STATE_UNCALIBRATED;
        return false;
    }

    // Stage 3: Calibrate right motor (left limit → right limit)
    // printf("3. Starting right motor calibration sequence...\n");
    // if (!calibrateRightMotor()) {
    //     printf("ERROR: Calibration failed during right stepper calibration sequence.");
    //     calibration_state = CALIBRATION_STATE_UNCALIBRATED;
    //     return false;
    // }

    printf("Calibration sequence complete for both motors, moving to home positions.\n");

    float leftHomePos = left_stepper_.getLeftBoundary()  + HOME_OFFSET * 2.5;  // From left boundary (which is 0)
    // float rightHomePos = right_stepper_.getRightBoundary() - HOME_OFFSET;  // Start at left boundary
    
    printf("Moving left motor to home position (%.3f m from left boundary)...\n", leftHomePos);
    left_stepper_.moveTo(leftHomePos);
    while (!left_stepper_.isMotionComplete()) {
        left_stepper_.update();
        sleep_us(35);
    }
    printf("Left motor at home position\n");
    
    // printf("Moving right motor to home position (%.3f m from left boundary)...\n", rightHomePos);
    // right_stepper_.moveTo(rightHomePos);
    // while (!right_stepper_.isMotionComplete()) {
    //     right_stepper_.update();
    //     sleep_us(35);
    // }
    // printf("Right motor at home position\n");

    // Re-enable emergency stop interrupts now that calibration is complete
    // initEmergencyStop();

    // Set state to completed after calibration finishes
    calibration_state = CALIBRATION_STATE_COMPLETED;
    printf("Calibration complete.\n");

    return (calibration_state == CALIBRATION_STATE_COMPLETED);
}

bool MotorController::calibrateLeftMotor() {
    printf("Calibrating left stepper...\n");
    
    // 1. Move to the left limit switch
    printf("Moving left stepper to left limit...\n");
    if (!left_stepper_.moveToLimit(LEFT_LIMIT_LEFT_PIN, SimplifiedStepper::CCW)) {
        printf("ERROR: Left stepper failed to reach left limit: %s\n", left_stepper_.getErrorMessage());
        return false;
    }
    
    // Reset step counter at left limit
    // left_stepper_.setCurrentPosition(0.0f);
    left_stepper_.resetStepCounter(); // Make sure steps are reset as well
    printf("Left stepper at left limit. Position reset to 0.0.\n");
    
    // Set left boundary to 0 (left limit)
    left_stepper_.setLeftBoundary(left_stepper_.getCurrentPosition());
    
    // 2. Move to right limit, counting steps
    printf("Moving left stepper to right limit...\n");
    long stepsToRight = left_stepper_.moveToLimit(LEFT_LIMIT_RIGHT_PIN, SimplifiedStepper::CW);
    if (stepsToRight < 0) {
        printf("ERROR: Left stepper failed to reach right limit: %s\n", left_stepper_.getErrorMessage());
        return false;
    }
    
    // Calculate the expected travel (should be less than the rail length minus right hand width)
    float expectedMaxTravel = TOTAL_RAIL_LENGTH - RIGHT_HAND_WIDTH;
    
    // Calculate actual travel based on steps and update steps/meter
    uint32_t actualStepsPerMeter = (uint32_t)(stepsToRight / expectedMaxTravel);
    printf("Left stepper moved %ld steps over expected %.3f meters\n", stepsToRight, expectedMaxTravel);
    printf("Left stepper steps per meter updated to: %lu (from %lu)\n", 
           actualStepsPerMeter, left_stepper_.getStepsPerMeter());
    
    // Update steps per meter with our new calibrated value
    left_stepper_.setStepsPerMeter(actualStepsPerMeter);
    
    // Now update the current position based on the new steps per meter
    float leftRightLimitPos = static_cast<float>(stepsToRight) / actualStepsPerMeter;
    // left_stepper_.setCurrentPosition(left_stepper_.getCurrentPosition());
    printf("Left stepper at right limit. Recalculated position: %.3f meters\n", left_stepper_.getCurrentPosition());
    
    // Set right boundary
    left_stepper_.setRightBoundary(left_stepper_.getCurrentPosition());
    
    // 3. Move back to left limit
    printf("Moving left stepper back to left limit...\n");
    if (!left_stepper_.moveToLimit(LEFT_LIMIT_LEFT_PIN, SimplifiedStepper::CCW)) {
        printf("ERROR: Left stepper failed to reach left limit: %s\n", left_stepper_.getErrorMessage());
        return false;
    }
    printf("Left stepper at left limit. Recalculated position: %.3f meters\n", left_stepper_.getCurrentPosition());
    
    // Reset position to 0 again at left limit
    left_stepper_.setCurrentPosition(left_stepper_.getCurrentPosition());
    // left_stepper_.resetStepCounter(); // Reset steps again
    // printf("Left stepper back at left limit. Position reset to 0.0.\n");
    
    printf("Left stepper calibration complete. Total travel: %.3f meters (expected: %.3f m)\n", 
        left_stepper_.getRightBoundary(), expectedMaxTravel);

    return true;
}

bool MotorController::calibrateRightMotor() {
    printf("Calibrating right stepper...\n");
    
    // 1. First measure the total rail distance by moving to left limit
    printf("Moving right stepper to left limit...\n");
    if (!right_stepper_.moveToLimit(LEFT_LIMIT_RIGHT_PIN, SimplifiedStepper::CCW)) {
        printf("ERROR: Right stepper failed to reach left limit: %s\n", right_stepper_.getErrorMessage());
        return false;
    }

    right_stepper_.resetStepCounter(); // Make sure steps are reset
    
    // Set left boundary for right stepper (left limit + left hand width)
    float rightLeftBoundary = LEFT_HAND_WIDTH + RIGHT_HAND_WIDTH;
    right_stepper_.setLeftBoundary(right_stepper_.getCurrentPosition());
    printf("Right stepper at left limit. Position set to %.3f meters.\n", right_stepper_.getLeftBoundary());
    
    // Reset step counter and set position to left boundary
    // right_stepper_.setCurrentPosition(rightLeftBoundary);
    
    // 2. Move to right limit, counting steps
    printf("Moving right stepper to right limit...\n");
    long stepsToRight = right_stepper_.moveToLimit(RIGHT_LIMIT_RIGHT_PIN, SimplifiedStepper::CW);
    if (stepsToRight < 0) {
        printf("ERROR: Right stepper failed to reach right limit: %s\n", right_stepper_.getErrorMessage());
        return false;
    }
    
    // Calculate the expected travel distance
    float expectedMaxTravel = TOTAL_RAIL_LENGTH - LEFT_HAND_WIDTH;
    
    // Calculate actual steps per meter based on expected travel
    uint32_t actualStepsPerMeter = (uint32_t)(stepsToRight / expectedMaxTravel);
    printf("Right stepper moved %ld steps over expected %.3f meters\n", stepsToRight, expectedMaxTravel);
    printf("Right stepper steps per meter updated to: %lu (from %lu)\n", 
           actualStepsPerMeter, right_stepper_.getStepsPerMeter());
    
    // Update steps per meter with our new calibrated value
    right_stepper_.setStepsPerMeter(actualStepsPerMeter);
    
    // Now calculate the current position based on new steps per meter
    float actualTravel = static_cast<float>(stepsToRight) / actualStepsPerMeter;
    float rightRightLimitPos = rightLeftBoundary + actualTravel;
    right_stepper_.setCurrentPosition(right_stepper_.getCurrentPosition());
    
    // Set right boundary for both steppers
    right_stepper_.setRightBoundary(right_stepper_.getCurrentPosition());
    printf("Right stepper at right limit. Recalculated position: %.3f meters\n", right_stepper_.getRightBoundary());
    
    printf("Right stepper calibration complete. Total travel: %.3f meters (expected: %.3f m)\n", 
           actualTravel, expectedMaxTravel);
    printf("Total rail length measured: %.3f meters (defined as: %.3f m)\n", 
        right_stepper_.getRightBoundary(), TOTAL_RAIL_LENGTH);

    return true;
}

bool MotorController::onPositionCommand(char motor, float position) {
    // Don't accept position commands when in emergency stop
    if (emergency_stop_active) {
        printf("ERROR: Cannot move motors during emergency stop\n");
        return false;
    }

    bool result = false;

    if (calibration_state == CALIBRATION_STATE_COMPLETED) {
        if (motor == 'l') {
            printf("COMMAND: Move left stepper to position %.3f\n", position);
            uint32_t save = spin_lock_blocking(left_motor_spin_lock);
            result = left_stepper_.moveTo(position);
            spin_unlock(left_motor_spin_lock, save);
            if (!result) {
                printf("ERROR: Failed to move left stepper: %s\n", left_stepper_.getErrorMessage());
            }
        } else if (motor == 'r') {
            printf("COMMAND: Move right stepper to position %.3f\n", position);

            if (core1_ready) {
                // Send command to core1
                motor_command_target = motor;
                motor_command_position = position;
                motor_command_pending = true;
                result = true;
            } else {
                printf("ERROR: Core 1 not ready to accept commands\n");
                result = false;
            }
        }
    }

    return result;
}

float MotorController::onPositionStateRequest(char motor) {
    // During emergency stop, still report positions
    if (motor == 'l') {
        return left_stepper_.getCurrentPosition();
    } else if (motor == 'r') {
        return right_stepper_.getCurrentPosition();
    }
    return 0.0f;
}

void MotorController::initEmergencyStop() {
    printf("Initializing emergency stop system...\n");

    // Configure limit switch GPIO interrupts for all limit switches
    // Left stepper limit switches
    gpio_set_irq_enabled(LEFT_LIMIT_LEFT_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(LEFT_LIMIT_RIGHT_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Right stepper limit switches
    gpio_set_irq_enabled(RIGHT_LIMIT_LEFT_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(RIGHT_LIMIT_RIGHT_PIN, GPIO_IRQ_EDGE_RISE, true);

    printf("Emergency stop system initialized.\n");
}

void MotorController::emergencyStop() {
    // Use our class member spin lock
    uint32_t save = spin_lock_blocking(emergency_spin_lock);

    if (!emergency_stop_active) {
        emergency_stop_active = true;
        // g_emergency_stop = true;

        printf("EMERGENCY STOP ACTIVATED!\n");

        // Stop both motors immediately
        left_stepper_.emergencyStop();
        right_stepper_.emergencyStop();
    }

    spin_unlock(emergency_spin_lock, save);
}

bool MotorController::isEmergencyStopActive() const { return emergency_stop_active; }

void MotorController::clearEmergencyStop() {
    if (!emergency_stop_active) {
        return;  // Not in emergency stop
    }

    printf("Clearing emergency stop condition\n");

    // Clear emergency stop flags
    emergency_stop_active = false;
    g_emergency_stop_atomic = false;

    // Reset PID controllers
    left_stepper_.resetPid();
    right_stepper_.resetPid();

    // Return LED to normal blinking
    blink_state_ = false;

    printf("Emergency stop cleared\n");
}

bool MotorController::isCalibrationComplete() const {
    // This method is safe to call from an ISR
    // It checks if both motors are calibrated and the calibration sequence is done
    return calibration_state == CALIBRATION_STATE_COMPLETED;
}

bool MotorController::onPidTuneCommand(char motor, float kp, float ki, float kd) {
    bool result = false;

    if (motor == 'l') {
        printf("Setting PID parameters for left stepper: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
        left_stepper_.setPidParameters(kp, ki, kd);
        result = true;
    } else if (motor == 'r') {
        printf("Setting PID parameters for right stepper: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
        right_stepper_.setPidParameters(kp, ki, kd);
        result = true;
    }

    return result;
}