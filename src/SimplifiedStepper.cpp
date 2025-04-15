#include "include/SimplifiedStepper.h"

#include <stdio.h>

#include <cmath>
#include <cstring>

#include "hardware/irq.h"
#include "hardware/timer.h"

// Define timer IRQs and alarms


// Default PID parameters
#define DEFAULT_KP 0.75f            // Proportional gain
#define DEFAULT_KI 0.0f             // Integral gain
#define DEFAULT_KD 0.03f            // Derivative gain
#define MIN_PID_OUTPUT 0.0f         // Minimum PID output (steps/second)
#define MAX_ERROR_INTEGRAL 1000.0f  // Maximum accumulated error
#define MAX_POSITION_ERROR 0.001f   // Error threshold for position reached (1mm)
#define MAX_STEP_INTERVAL_US 50000  // Maximum step interval (20 steps/sec minimum speed)

// Pin definitions for left and right steppers
#define LEFT_PULSE_PIN 10
#define RIGHT_PULSE_PIN 6

// Static variable initialization
SimplifiedStepper* SimplifiedStepper::left_instance = nullptr;
SimplifiedStepper* SimplifiedStepper::right_instance = nullptr;

// IRQ handlers for left and right steppers
void left_stepper_irq(void) {
    // Clear the interrupt
    hw_clear_bits(&LEFT_STEPPER_TIMER->intr, 1u << LEFT_STEPPER_ALARM_NUM);
    
    // Call step on the left instance
    if (SimplifiedStepper::left_instance != nullptr) {
        SimplifiedStepper::left_instance->step();
    }
}

void right_stepper_irq(void) {
    // Clear the interrupt
    hw_clear_bits(&RIGHT_STEPPER_TIMER->intr, 1u << RIGHT_STEPPER_ALARM_NUM);
    
    // Call step on the right instance
    if (SimplifiedStepper::right_instance != nullptr) {
        SimplifiedStepper::right_instance->step();
    }
}

SimplifiedStepper::SimplifiedStepper(uint8_t pulsePin, uint8_t dirPin, int8_t enablePin, uint8_t leftLimitPin,
                                     uint8_t rightLimitPin, uint32_t stepsPerMeter, float totalRailLength,
                                     float leftHandWidth, float rightHandWidth, float homeOffset, uint32_t maxStepRate,
                                     uint32_t minPulseWidth, uint8_t alarmNum)
    : pulsePin(pulsePin),
      dirPin(dirPin),
      enablePin(enablePin),
      leftLimitPin(leftLimitPin),
      rightLimitPin(rightLimitPin),
      stepsPerMeter(stepsPerMeter),
      totalRailLength(totalRailLength),
      leftHandWidth(leftHandWidth),
      rightHandWidth(rightHandWidth),
      homeOffset(homeOffset),
      minPulseWidth(minPulseWidth),
      maxStepRate(maxStepRate),
      totalTravelMeters(0.0f),
      leftLimitPosition(0.0f),
      rightLimitPosition(0.0f),
      leftBoundary(0.0f),
      rightBoundary(totalRailLength),
    //   isCalibrated(false),
      // Initialize PID controller parameters
      kp(DEFAULT_KP),
      ki(DEFAULT_KI),
      kd(DEFAULT_KD),
      errorIntegral(0.0f),
      lastError(0.0f),
      lastErrorTime(0),
      minOutput(MIN_PID_OUTPUT),
      maxOutput(maxStepRate),
      // Initialize state variables
      currentPosition(0.0f),
      targetPosition(0.0f),
      absoluteStepPos(0),
      direction(true),
      isMoving(false),
      lastStepTime(0),
      minStepInterval(1000000 / maxStepRate),
      currentVelocity(0.0f),
      alarmNum(alarmNum) {
    // Initialize error message buffer
    errorMessage[0] = '\0';


    // Initialize GPIO
    gpio_init(pulsePin);
    gpio_init(dirPin);
    gpio_set_dir(pulsePin, GPIO_OUT);
    gpio_set_dir(dirPin, GPIO_OUT);
    gpio_put(pulsePin, 0);
    gpio_put(dirPin, 0);

    // Initialize enable pin if it's used
    if (enablePin >= 0) {
        gpio_init(enablePin);
        gpio_set_dir(enablePin, GPIO_OUT);
        gpio_put(enablePin, 0);  // Start with motor disabled
    }

    // Initialize limit switch pins
    gpio_init(leftLimitPin);
    gpio_init(rightLimitPin);
    gpio_set_dir(leftLimitPin, GPIO_IN);
    gpio_set_dir(rightLimitPin, GPIO_IN);

    // Initialize timing
    lastErrorTime = time_us_64();

    // In SimplifiedStepper.cpp constructor:

    // In SimplifiedStepper.cpp constructor:
    // Setup hardware timer IRQ based on which stepper this is
    if (pulsePin == LEFT_PULSE_PIN) {
        left_instance = this;
        
        // Configure timer0 for left stepper
        irq_set_enabled(TIMER0_IRQ, false);
        hw_clear_bits(&LEFT_STEPPER_TIMER->inte, 1u << LEFT_STEPPER_ALARM_NUM);
        
        irq_set_exclusive_handler(TIMER0_IRQ, left_stepper_irq);
        hw_set_bits(&LEFT_STEPPER_TIMER->inte, 1u << LEFT_STEPPER_ALARM_NUM);
        irq_set_enabled(TIMER0_IRQ, true);
        
        printf("Left stepper IRQ configured on timer0\n");
    } else if (pulsePin == RIGHT_PULSE_PIN) {
        right_instance = this;
        
        // Configure timer1 for right stepper
        irq_set_enabled(TIMER1_IRQ, false);
        hw_clear_bits(&RIGHT_STEPPER_TIMER->inte, 1u << RIGHT_STEPPER_ALARM_NUM);
        
        irq_set_exclusive_handler(TIMER1_IRQ, right_stepper_irq);
        hw_set_bits(&RIGHT_STEPPER_TIMER->inte, 1u << RIGHT_STEPPER_ALARM_NUM);
        irq_set_enabled(TIMER1_IRQ, true);
        
        printf("Right stepper IRQ configured on timer1\n");
    }
}

// Destructor to clean up IRQ handlers
SimplifiedStepper::~SimplifiedStepper() {
    // Disable IRQ and clear instance pointer
    if (this == left_instance) {
        hw_clear_bits(&timer_hw->inte, 1u << LEFT_STEPPER_ALARM_NUM);
        left_instance = nullptr;
    } else if (this == right_instance) {
        hw_clear_bits(&timer_hw->inte, 1u << RIGHT_STEPPER_ALARM_NUM);
        right_instance = nullptr;
    }
}

uint32_t SimplifiedStepper::calculatePidStepInterval() {
    // Calculate velocity using PID controller
    float velocity = updatePid();

    // Save current velocity
    currentVelocity = velocity;

    // If velocity is very small, return maximum interval
    if (velocity < 0.1f) {
        return MAX_STEP_INTERVAL_US;
    }

    // Convert to time interval (microseconds)
    uint32_t interval = static_cast<uint32_t>(1000000.0f / velocity);

    // Ensure interval is not too small based on driver specs
    if (interval < minStepInterval) {
        interval = minStepInterval;
    }

    return interval;
}

float SimplifiedStepper::updatePid() {
    // Calculate error (in meters)
    float error = targetPosition - currentPosition;

    // If error is very small, consider position reached
    if (fabs(error) < MAX_POSITION_ERROR / stepsPerMeter) {
        // If we're at the target position, stop moving
        isMoving = false;
        return 0.0f;  // Return zero velocity
    }

    // Calculate time step since last update (in seconds)
    uint64_t now = time_us_64();
    float dt = (now - lastErrorTime) / 1000000.0f;

    // Prevent division by zero or very small dt
    if (dt < 0.00001f) {
        dt = 0.00001f;
    }

    // Calculate derivative term (change in error over time)
    float errorDerivative = (error - lastError) / dt;

    // Update integral term with anti-windup
    errorIntegral += error * dt;

    // Limit integral term to prevent windup
    if (errorIntegral > MAX_ERROR_INTEGRAL) {
        errorIntegral = MAX_ERROR_INTEGRAL;
    } else if (errorIntegral < -MAX_ERROR_INTEGRAL) {
        errorIntegral = -MAX_ERROR_INTEGRAL;
    }

    // PID formula
    float output = kp * error + ki * errorIntegral + kd * errorDerivative;

    // Convert output from meters to steps per second
    output *= stepsPerMeter;

    // Keep track of direction
    bool newDirection = (output >= 0);

    // Use absolute value for velocity
    output = fabs(output);

    // Clamp output to valid range
    if (output < minOutput) {
        output = minOutput;
    } else if (output > maxOutput) {
        output = maxOutput;
    }

    // Update state for next iteration
    lastError = error;
    lastErrorTime = now;

    // Update direction if needed
    if (direction != newDirection) {
        direction = newDirection;
        setDirection(direction);
    }

    return output;
}

void SimplifiedStepper::armTimerForNextStep(uint32_t delay_us) {
    uint64_t target;
    
    if (this == left_instance) {
        target = LEFT_STEPPER_TIMER->timerawl + delay_us;
        LEFT_STEPPER_TIMER->alarm[LEFT_STEPPER_ALARM_NUM] = (uint32_t)target;
    } else if (this == right_instance) {
        target = RIGHT_STEPPER_TIMER->timerawl + delay_us;
        RIGHT_STEPPER_TIMER->alarm[RIGHT_STEPPER_ALARM_NUM] = (uint32_t)target;
    }
}

void SimplifiedStepper::step() {
    // If we're not supposed to be moving, don't step
    if (!isMoving) {
        return;
    }

    // Generate step pulse
    generateStep();

    // Update absolute step position based on direction
    if (direction) {
        absoluteStepPos++;
    } else {
        absoluteStepPos--;
    }

    // Update current position in meters
    updatePositionFromSteps();

    // Record step time
    lastStepTime = time_us_64();

    // Calculate next step interval
    uint32_t interval = calculatePidStepInterval();

    // If interval is MAX_STEP_INTERVAL_US, movement is complete
    if (interval >= MAX_STEP_INTERVAL_US) {
        isMoving = false;
    } else {
        // Schedule next step
        armTimerForNextStep(interval);
    }
}

bool SimplifiedStepper::update() {
    // This method is now primarily for PID calculations and checking motion status
    // The actual stepping is done by the IRQ handler

    // If we're not supposed to be moving, don't do anything
    if (!isMoving) {
        return false;
    }

    // Calculate a new PID output
    calculatePidStepInterval();

    return true;
}

void SimplifiedStepper::generateStep() {
    gpio_put(pulsePin, 1);
    busy_wait_us(minPulseWidth);  // Minimum 1μs pulse width per datasheet
    gpio_put(pulsePin, 0);
}

void SimplifiedStepper::setDirection(bool isClockwise) {
    // Check if this is the right motor and invert direction if needed
    bool actualDirection = isClockwise;
    if (pulsePin == RIGHT_PULSE_PIN) {
        actualDirection = !isClockwise;
    }

    gpio_put(dirPin, actualDirection ? 0 : 1);
    busy_wait_us(4);  // Minimum 2μs delay per datasheet
}

void SimplifiedStepper::updatePositionFromSteps() {
    currentPosition = static_cast<float>(absoluteStepPos) / stepsPerMeter;
}

void SimplifiedStepper::setError(const char* message) {
    strncpy(errorMessage, message, sizeof(errorMessage) - 1);
    errorMessage[sizeof(errorMessage) - 1] = '\0';
    printf("ERROR: %s\n", errorMessage);
}

void SimplifiedStepper::setPidParameters(float p, float i, float d) {
    // Update PID gains
    kp = p;
    ki = i;
    kd = d;

    // Reset integral term when parameters change
    errorIntegral = 0.0f;
}

void SimplifiedStepper::resetPid() {
    // Reset PID controller state
    errorIntegral = 0.0f;
    lastError = 0.0f;
    lastErrorTime = time_us_64();
}

void SimplifiedStepper::enableMotor(bool enable) {
    if (enablePin >= 0) {
        gpio_put(enablePin, enable ? 1 : 0);
    }
}

void SimplifiedStepper::setLeftBoundary(float position) { leftBoundary = position; }
void SimplifiedStepper::setRightBoundary(float position) { rightBoundary = position; }
float SimplifiedStepper::getLeftBoundary() const { return leftBoundary; }
float SimplifiedStepper::getRightBoundary() const { return rightBoundary; }

long SimplifiedStepper::getStepCount() const {
    // Return the absolute step count from the last movement
    // This is used during calibration to measure distances
    return abs(absoluteStepPos);
}
long SimplifiedStepper::getStepsPerMeter() const { return stepsPerMeter; }

float SimplifiedStepper::getCurrentPosition() const { return currentPosition; }

bool SimplifiedStepper::isMotionComplete() const { return !isMoving; }

float SimplifiedStepper::getCurrentVelocity() const { return currentVelocity; }

void SimplifiedStepper::setCurrentPosition(float position) {
    if (!isMoving) {
        currentPosition = position;
        absoluteStepPos = static_cast<int32_t>(position * stepsPerMeter);
    }
}

bool SimplifiedStepper::moveTo(float position) {
    // Bounds check against physical boundaries
    if (position < leftBoundary) {
        char errorMsg[100];
        snprintf(errorMsg, sizeof(errorMsg), "Requested position %.3f is less than left boundary %.3f", 
                position, leftBoundary);
        setError(errorMsg);
        return false;
    }
    
    if (position > rightBoundary) {
        char errorMsg[100];
        snprintf(errorMsg, sizeof(errorMsg), "Requested position %.3f exceeds right boundary %.3f", 
                position, rightBoundary);
        setError(errorMsg);
        return false;
    }
    
    // Set new target
    targetPosition = position;
    
    // Calculate movement parameters
    float distance = targetPosition - currentPosition;
    
    // Update direction based on target position
    bool newDirection = (distance >= 0) ? SimplifiedStepper::CW : SimplifiedStepper::CCW;
    
    char dirString[100];
    snprintf(dirString, sizeof(dirString), "Old direction is %s.", 
        direction == SimplifiedStepper::CW ? "CW" : "CCW");
    printf("%s\n", dirString);

    // If direction changed, set it
    if (direction != newDirection) {
        direction = newDirection;
        setDirection(direction);
    }

    snprintf(dirString, sizeof(dirString), "New direction is %s.", 
        direction == SimplifiedStepper::CW ? "CW" : "CCW");
    printf("%s\n", dirString);

    
    // If we're not already moving and the distance is significant, start moving
    if (!isMoving && fabs(distance) > MAX_POSITION_ERROR / stepsPerMeter) {
        isMoving = true;
        
        // Reset PID state for new movement
        resetPid();
        
        // Reset step timing
        lastStepTime = 0;
        
        // Schedule first step using timer
        uint32_t interval = calculatePidStepInterval();
        armTimerForNextStep(interval);
    }
    
    return true;
}

void SimplifiedStepper::stop() {
    // Set target to current position to stop movement
    targetPosition = currentPosition;
    // Don't immediately cancel isMoving flag - let PID handle it
}

void SimplifiedStepper::emergencyStop() {
    // Immediately stop the motor
    isMoving = false;
    // Set target to current position
    targetPosition = currentPosition;
    // Reset PID state
    resetPid();
}

bool SimplifiedStepper::isLimitSwitchTriggered() const {
    // Check if either limit switch is triggered
    return (gpio_get(leftLimitPin) == 1 || gpio_get(rightLimitPin) == 1);
}

const char* SimplifiedStepper::getErrorMessage() const { return errorMessage; }

// bool SimplifiedStepper::getIsCalibrated() const { return isCalibrated; }

long SimplifiedStepper::moveToLimit(uint8_t limitSwitchPin, bool direction, float slowSpeed) {
    // Make sure we're not already at the limit
    if (gpio_get(limitSwitchPin) == 1) {  // Switches are active HIGH
        // Move away from the switch first (opposite direction)
        long backoffSteps = 2000;
        if (!moveSteps(backoffSteps, !direction)) {
            return -1;
        }

        // Wait for motion to complete
        while (isMoving) {
            update();
            sleep_us(50);
        }
    }

    // Set movement direction
    setDirection(direction);
    sleep_us(4);  // Allow time for direction change

    long steps = 0;
    absolute_time_t startTime = get_absolute_time();

    // Keep stepping until switch is triggered or timeout
    while (gpio_get(limitSwitchPin) != 1) {  // Switches are active HIGH
        generateStep();
        steps++;

        // Update step counter based on direction
        if (direction == CW) {
            absoluteStepPos++;
        } else {
            absoluteStepPos--;
        }

        // Check for timeout
        if (absolute_time_diff_us(startTime, get_absolute_time()) > (CALIBRATION_TIMEOUT_MS * 1000)) {
            setError("Timeout while moving to limit switch");
            return -1;
        }

        // Use a reasonable delay for calibration moves
        busy_wait_us(35);  // Slower speed for calibration (2000 steps/sec)
    }

    // Update position based on steps
    updatePositionFromSteps();

    return steps;
}

bool SimplifiedStepper::moveSteps(long numSteps, bool direction) {
    if (numSteps <= 0)
        return true;  // Nothing to do

    setDirection(direction);
    sleep_us(4);  // Allow time for direction change

    // For direct stepping without IRQ for calibration
    // This ensures we have precise control during calibration
    isMoving = false;  // Make sure IRQ doesn't interfere

    for (long i = 0; i < numSteps; i++) {
        generateStep();

        // Update step counter based on direction
        if (direction == CW) {
            absoluteStepPos++;
        } else {
            absoluteStepPos--;
        }

        // Basic delay - not using motion profile for calibration/homing
        sleep_us(35);  // 2000 steps/sec
    }

    // Update position after steps
    updatePositionFromSteps();

    return true;
}

// bool SimplifiedStepper::calibrate() {
//     // 1. Move to the left limit switch
//     if (moveToLimit(leftLimitPin, CCW) < 0) {
//         return false;  // Error already set in moveToLimit
//     }

//     // Reset step counter at left limit
//     absoluteStepPos = 0;

//     // 2. Move to the right limit switch, counting steps
//     long stepsToRight = moveToLimit(rightLimitPin, CW);
//     if (stepsToRight < 0) {
//         return false;  // Error already set
//     }

//     // Calculate the mechanical parameters
//     totalTravelMeters = totalRailLength - leftHandWidth - rightHandWidth;  // tracks centre of hand
//     stepsPerMeter = static_cast<uint32_t>(stepsToRight / totalTravelMeters);

//     // Calculate limit positions relative to homed position
//     float railCenterMeters = totalRailLength / 2.0f;
//     leftLimitPosition = -railCenterMeters + leftHandWidth / 2.0f - homeOffset;
//     rightLimitPosition = (railCenterMeters - rightHandWidth - leftHandWidth / 2.0f) - homeOffset;

//     // Set calibration flag
//     isCalibrated = true;

//     // 3. Perform homing
//     return homePosition();
// }

bool SimplifiedStepper::homePosition() {
    // if (!isCalibrated) {
    //     setError("System not calibrated, cannot home");
    //     return false;
    // }

    // Calculate steps from current position to home position
    float railCenterMeters = totalRailLength / 2.0f;
    float homePositionStepsFromLeft = (railCenterMeters + homeOffset - leftHandWidth / 2.0f) * stepsPerMeter;

    long stepsToHome = homePositionStepsFromLeft - absoluteStepPos;
    bool direction = (stepsToHome > 0) ? CW : CCW;

    // Move to home position
    if (!moveSteps(abs(stepsToHome), direction)) {
        return false;
    }

    // Reset position to zero at home position
    currentPosition = 0.0f;
    targetPosition = 0.0f;
    currentVelocity = 0.0f;
    absoluteStepPos = 0;  // Reset step counter at home position

    // Reset PID controller state
    resetPid();

    return true;
}

float SimplifiedStepper::getLeftLimitPosition() const { return leftLimitPosition; }

float SimplifiedStepper::getRightLimitPosition() const { return rightLimitPosition; }

uint32_t SimplifiedStepper::getStepsTaken() const {
    return abs(absoluteStepPos);
}

void SimplifiedStepper::resetStepCounter() {
    absoluteStepPos = 0;
    currentPosition = 0.0f;
}

long SimplifiedStepper::getAbsoluteStepPos() const {
    return absoluteStepPos;
}

void SimplifiedStepper::setStepsPerMeter(uint32_t steps) {
    if (steps > 0) {
        stepsPerMeter = steps;
        updatePositionFromSteps();
    }
}