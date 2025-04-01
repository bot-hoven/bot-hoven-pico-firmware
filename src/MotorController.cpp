#include "include/MotorController.h"
#include <cstdio>

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
#define PIN_MISO 16 // THIS SHOULD BE OPPOSITE OF SCHEMATIC
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19 // THIS SHOULD BE OPPOSITE OF SCHEMATIC
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

MotorController::MotorController() 
    : left_stepper_(
        LEFT_PULSE_PIN, LEFT_DIR_PIN, LEFT_ENABLE_PIN,
        LEFT_LIMIT_LEFT_PIN, LEFT_LIMIT_RIGHT_PIN,
        INITIAL_STEPS_PER_METER,
        TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET
      ),
    //   right_stepper_(
    //     RIGHT_PULSE_PIN, RIGHT_DIR_PIN, RIGHT_ENABLE_PIN,
    //     RIGHT_LIMIT_LEFT_PIN, RIGHT_LIMIT_RIGHT_PIN,
    //     INITIAL_STEPS_PER_METER,
    //     TOTAL_RAIL_LENGTH, LEFT_HAND_WIDTH, RIGHT_HAND_WIDTH, HOME_OFFSET
    //   ),
      spi_interface_(
        SPI_PORT, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI, SPI_CLOCK_SPEED_HZ
      ),
      blink_state_(false) {
}

void MotorController::init() {
    // Initialize stdio for debugging output
    stdio_init_all();
    sleep_ms(2000); // Wait for serial connection to stabilize
    
    // Initialize LED
    initLED();
    
    // Configure the stepper motors
    left_stepper_.setMaxVelocity(MAX_VELOCITY);
    left_stepper_.setAcceleration(ACCELERATION);
    left_stepper_.setDeceleration(DECELERATION);
    
    // right_stepper_.setMaxVelocity(MAX_VELOCITY);
    // right_stepper_.setAcceleration(ACCELERATION);
    // right_stepper_.setDeceleration(DECELERATION);
    
    // Initialize SPI interface
    spi_interface_.init();
    
    // Register SPI callbacks
    spi_interface_.setCalibrationCallback([this]() { this->onCalibrationCommand(); });
    spi_interface_.setPositionCommandCallback([this](char motor, float position) { 
        return this->onPositionCommand(motor, position);
    });
    spi_interface_.setPositionStateCallback([this](char motor) {
        return this->onPositionStateRequest(motor);
    });
    
    // Initialize timing variables
    // last_position_update_ = get_absolute_time();
    last_blink_ = get_absolute_time();
    
    // printf("Initialization complete. Ready to receive commands.\n");
}

void MotorController::update() {
    // Process SPI commands
    spi_interface_.processCommands();

    
    // Monitor motors at a fixed rate if desired.
    // if (absolute_time_diff_us(last_position_update_, get_absolute_time()) > POSITION_UPDATE_INTERVAL_US) {
    //     monitorMotors();
    //     last_position_update_ = get_absolute_time();
    // }
    
    // Update LED status (blink to show system is alive)
    updateLED();
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

bool MotorController::onPositionCommand(char motor, float position) {
    bool result = false;
    
    if (motor == 'l') {
        // printf("Moving left stepper to position %.3f\n", position);
        result = left_stepper_.moveTo(position);
        if (!result) {
            printf("ERROR: Failed to move left stepper: %s\n", left_stepper_.getErrorMessage());
        }
    } else if (motor == 'r') {
        // printf("Moving right stepper to position %.3f\n", position);
        // result = right_stepper_.moveTo(position);
        // if (!result) {
        //     printf("ERROR: Failed to move right stepper: %s\n", right_stepper_.getErrorMessage());
        // }
    }
    
    return result;
}

float MotorController::onPositionStateRequest(char motor) {
    if (motor == 'l') {
        return left_stepper_.getCurrentPosition();
    } else if (motor == 'r') {
        // return right_stepper_.getCurrentPosition();
    }
    return 0.0f;
}

void MotorController::monitorMotors() {
    // This function is intentionally left empty as the TrapezoidalStepper class
    // handles motion updates through its own alarm callbacks.
    // 
    // If we needed additional motor state updates or monitoring, we would
    // implement them here.
}