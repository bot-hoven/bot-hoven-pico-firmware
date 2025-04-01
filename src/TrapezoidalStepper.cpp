#include "include/TrapezoidalStepper.h"
#include <cmath>
#include <cstring>
#include <stdio.h>
#include "hardware/irq.h"
#include "hardware/timer.h"

// Define timer IRQs and alarms
#define LEFT_STEPPER_ALARM_NUM 2  // Use timer alarm 2 for left stepper (leaving 0/1 for SDK)
#define RIGHT_STEPPER_ALARM_NUM 3 // Use timer alarm 3 for right stepper
#define TIMER_IRQ_2 TIMER0_IRQ_2
#define TIMER_IRQ_3 TIMER0_IRQ_3

// Pin definitions from MotorController.cpp for use in constructor
#define LEFT_PULSE_PIN 10
#define RIGHT_PULSE_PIN 6

// Static variable initialization
TrapezoidalStepper* TrapezoidalStepper::left_instance = nullptr;
TrapezoidalStepper* TrapezoidalStepper::right_instance = nullptr;

// IRQ handlers for left and right steppers
void left_stepper_irq(void) {
    // Clear the interrupt
    hw_clear_bits(&timer_hw->intr, 1u << LEFT_STEPPER_ALARM_NUM);
    
    // Call step on the left instance
    if (TrapezoidalStepper::left_instance != nullptr) {
        TrapezoidalStepper::left_instance->step();
    }
}

void right_stepper_irq(void) {
    // Clear the interrupt
    hw_clear_bits(&timer_hw->intr, 1u << RIGHT_STEPPER_ALARM_NUM);
    
    // Call step on the right instance
    if (TrapezoidalStepper::right_instance != nullptr) {
        TrapezoidalStepper::right_instance->step();
    }
}

TrapezoidalStepper::TrapezoidalStepper(uint8_t pulsePin, uint8_t dirPin, int8_t enablePin,
                                    uint8_t leftLimitPin, uint8_t rightLimitPin,
                                    uint32_t stepsPerMeter,
                                    float totalRailLength, float leftHandWidth, float rightHandWidth, float homeOffset,
                                    uint32_t maxStepRate, uint32_t minPulseWidth)
    : pulsePin(pulsePin), dirPin(dirPin), enablePin(enablePin),
      leftLimitPin(leftLimitPin), rightLimitPin(rightLimitPin),
      stepsPerMeter(stepsPerMeter),
      totalRailLength(totalRailLength), leftHandWidth(leftHandWidth), rightHandWidth(rightHandWidth),
      homeOffset(homeOffset),
      minPulseWidth(minPulseWidth), maxStepRate(maxStepRate),
      totalTravelMeters(0.0f), leftLimitPosition(0.0f), rightLimitPosition(0.0f), isCalibrated(false),
      maxVelocity(10000.0f), acceleration(20000.0f), deceleration(20000.0f),
      currentPosition(0.0f), targetPosition(0.0f),
      totalSteps(0), currentStep(0), absoluteStepPos(0),
      accelSteps(0), constVelSteps(0), decelSteps(0),
      direction(true), isMoving(false),
      lastStepTime(0), minStepInterval(1000000 / maxStepRate),
      currentVelocity(0.0f), entryVelocity(0.0f), highestVelocity(0.0f) {
    
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
    lastUpdateTime = get_absolute_time();
    
    // Setup hardware timer IRQ based on which stepper this is
    if (pulsePin == LEFT_PULSE_PIN) {
        left_instance = this;
        
        // Setup IRQ handler for left stepper
        hw_set_bits(&timer_hw->inte, 1u << LEFT_STEPPER_ALARM_NUM);
        irq_set_exclusive_handler(TIMER_IRQ_2, left_stepper_irq);
        irq_set_enabled(TIMER_IRQ_2, true);
    } else if (pulsePin == RIGHT_PULSE_PIN) {
        right_instance = this;
        
        // Setup IRQ handler for right stepper
        hw_set_bits(&timer_hw->inte, 1u << RIGHT_STEPPER_ALARM_NUM);
        irq_set_exclusive_handler(TIMER_IRQ_3, right_stepper_irq);
        irq_set_enabled(TIMER_IRQ_3, true);
    }
}

// Destructor to clean up IRQ handlers
TrapezoidalStepper::~TrapezoidalStepper() {
    // Disable IRQ and clear instance pointer
    if (this == left_instance) {
        hw_clear_bits(&timer_hw->inte, 1u << LEFT_STEPPER_ALARM_NUM);
        left_instance = nullptr;
    } else if (this == right_instance) {
        hw_clear_bits(&timer_hw->inte, 1u << RIGHT_STEPPER_ALARM_NUM);
        right_instance = nullptr;
    }
}

void TrapezoidalStepper::armTimerForNextStep(uint32_t delay_us) {
    uint64_t target = timer_hw->timerawl + delay_us;
    
    if (this == left_instance) {
        timer_hw->alarm[LEFT_STEPPER_ALARM_NUM] = (uint32_t) target;
    } else if (this == right_instance) {
        timer_hw->alarm[RIGHT_STEPPER_ALARM_NUM] = (uint32_t) target;
    }
}

uint32_t TrapezoidalStepper::calculateStepInterval() {
    /*
    Compute timing between steps (i.e., usec/step) based on desired velocity.
    Effectively what we're doing here is computing the new velocity based on
    how far we've travelled so far, subject to our initial velocity and acceleration.
    */
    float velocity = 0.0f;
    
    if (currentStep < accelSteps) {
        // Acceleration phase
        velocity = sqrt(entryVelocity * entryVelocity + 2.0f * acceleration * currentStep);
    } else if (currentStep == accelSteps) {
        // Exactly at transition point - ensure we get exactly maxVelocity
        velocity = maxVelocity;
    } else if (currentStep < accelSteps + constVelSteps) {
        // Constant velocity phase
        velocity = maxVelocity;
    } else if (currentStep == accelSteps + constVelSteps) {
        // At transition to deceleration - ensure we get exactly highestVelocity
        velocity = highestVelocity;
    } else if (currentStep < totalSteps) {
        // Deceleration phase
        int32_t decelStart = accelSteps + constVelSteps;
        int32_t distanceInDecelPhase = currentStep - decelStart;
        float vDecStart = highestVelocity;
        
        float decelTerm = (vDecStart * vDecStart) - 2.0f * deceleration * static_cast<float>(distanceInDecelPhase);
        if (decelTerm < 0.0f) { 
            decelTerm = 0.0f;
        }

        velocity = sqrt(decelTerm);
    } else {
        // Motion complete
        isMoving = false;
        return UINT32_MAX;
    }
    
    // Clamp velocity to valid range
    if (velocity < 1.0f) {
        velocity = 1.0f;
    } else if (velocity > maxStepRate) {
        velocity = maxStepRate;
    }
    
    currentVelocity = velocity;
    
    // Convert to time interval (microseconds)
    uint32_t interval = static_cast<uint32_t>(1000000.0f / velocity);
    
    // Ensure interval is not too small based on driver specs
    if (interval < minStepInterval) {
        interval = minStepInterval;
    }
    
    return interval;
}

void TrapezoidalStepper::step() {
    // Skip if we've already completed the move
    if (currentStep >= totalSteps) {
        isMoving = false;
        return;
    }
    
    // Generate step pulse
    generateStep();
    
    // Update position
    currentStep++;
    if (direction) {
        absoluteStepPos++;
    } else {
        absoluteStepPos--;
    }
    
    // Update current position in meters
    updatePositionFromSteps();
    
    // Record step time
    lastStepTime = time_us_64();
    
    // Schedule next step if still moving
    if (currentStep < totalSteps) {
        uint32_t interval = calculateStepInterval();
        armTimerForNextStep(interval);
    } else {
        isMoving = false;
    }
}

void TrapezoidalStepper::generateStep() {
    gpio_put(pulsePin, 1);
    busy_wait_us(minPulseWidth);   // Minimum 1μs pulse width per datasheet
    gpio_put(pulsePin, 0);
}

void TrapezoidalStepper::setDirection(bool isClockwise) {
    gpio_put(dirPin, isClockwise ? 0 : 1);
    busy_wait_us(2);  // Minimum 2μs delay per datasheet
}

void TrapezoidalStepper::updatePositionFromSteps() {
    currentPosition = static_cast<float>(absoluteStepPos) / stepsPerMeter;
}

void TrapezoidalStepper::setError(const char* message) {
    strncpy(errorMessage, message, sizeof(errorMessage) - 1);
    errorMessage[sizeof(errorMessage) - 1] = '\0';
    // printf("ERROR: %s\n", errorMessage);
}

void TrapezoidalStepper::setMaxVelocity(float velocity) {
    if (velocity > 0 && velocity <= maxStepRate) {
        maxVelocity = velocity;
    }
}

void TrapezoidalStepper::setAcceleration(float accel) {
    if (accel > 0) {
        acceleration = accel;
    }
}

void TrapezoidalStepper::setDeceleration(float decel) {
    if (decel > 0) {
        deceleration = decel;
    }
}

void TrapezoidalStepper::enableMotor(bool enable) {
    if (enablePin >= 0) {
        gpio_put(enablePin, enable ? 1 : 0);
    }
}

float TrapezoidalStepper::getCurrentPosition() const {
    return currentPosition;
}

bool TrapezoidalStepper::isMotionComplete() const {
    return !isMoving;
}

float TrapezoidalStepper::getCurrentVelocity() const {
    return currentVelocity;
}

void TrapezoidalStepper::setCurrentPosition(float position) {
    /*
    This function only updates the current position variables if we know where we are.
    Thus, it is only set if we are not moving (at some stationary known position)
    */
    if (!isMoving) {
        currentPosition = position;
        absoluteStepPos = static_cast<int32_t>(position * stepsPerMeter);
    }
}

bool TrapezoidalStepper::moveTo(float position, float startVelocity) {
    // Bounds check if calibrated
    if (isCalibrated) {
        if (position < leftLimitPosition || position > rightLimitPosition) {
            setError("Requested position is outside the valid range");
            return false;
        }
    }
    
    // If already moving, cancel the current move
    if (isMoving) {
        isMoving = false;
    }

    // Set new target
    targetPosition = position;
    
    // Calculate movement parameters
    float distance = targetPosition - currentPosition;
    direction = (distance >= 0);
    totalSteps = static_cast<int32_t>(fabs(distance) * stepsPerMeter);
    
    // If no movement needed, return
    if (totalSteps == 0) return false;
    
    // Set direction pin according to datasheet timing requirements
    setDirection(direction);
    
    // Set entry velocity (clamped to safe value)
    entryVelocity = fmin(startVelocity, maxVelocity);
    
    // Calculate motion profile
    float accelDistance = (maxVelocity * maxVelocity - entryVelocity * entryVelocity) / (2.0f * acceleration);
    accelSteps = static_cast<int32_t>(accelDistance + 0.5f);
    
    float decelDistance = (maxVelocity * maxVelocity) / (2.0f * deceleration);
    decelSteps = static_cast<int32_t>(decelDistance + 0.5f);
    
    // Check if we can reach max velocity
    if (accelSteps + decelSteps > totalSteps) {
        // Triangular profile - calculate crossover point
        float peakVelocity = sqrt(
            (entryVelocity * entryVelocity * deceleration + 
             2.0f * acceleration * deceleration * totalSteps) / 
            (acceleration + deceleration)
        );
        
        // Calculate new accel and decel steps
        accelSteps = static_cast<int32_t>((peakVelocity * peakVelocity - entryVelocity * entryVelocity) / 
                                          (2.0f * acceleration));
        decelSteps = totalSteps - accelSteps;
        highestVelocity = peakVelocity;
        constVelSteps = 0;
    } else {
        // Trapezoidal profile
        constVelSteps = totalSteps - accelSteps - decelSteps;
        highestVelocity = maxVelocity;
    }
    
    // Initialize state
    currentStep = 0;
    currentVelocity = entryVelocity;
    isMoving = true;
    lastStepTime = time_us_64();
    
    // Schedule first step using hardware timer
    uint32_t interval = calculateStepInterval();
    armTimerForNextStep(interval);
    
    return true;
}

void TrapezoidalStepper::stop() {
    if (!isMoving) return;
    
    // Calculate steps needed to stop from current velocity
    float stepsToStop = (currentVelocity * currentVelocity) / (2.0f * deceleration);
    
    // Calculate new target position
    float stopDistance = stepsToStop / stepsPerMeter;
    if (direction) {
        targetPosition = currentPosition + stopDistance;
    } else {
        targetPosition = currentPosition - stopDistance;
    }
    
    // Set up a new move to the stop position
    totalSteps = currentStep + static_cast<int32_t>(stepsToStop);
    accelSteps = currentStep;  // No more acceleration
    constVelSteps = 0;         // No constant velocity phase
    decelSteps = static_cast<int32_t>(stepsToStop);
    
    // The existing step() function will handle the deceleration
    // No need to explicitly schedule the next step as it's already been scheduled
}

void TrapezoidalStepper::emergencyStop() {
    if (!isMoving) return;
    
    // Immediately stop by setting isMoving to false
    // The step() function will check this and not schedule new steps
    isMoving = false;
}

const char* TrapezoidalStepper::getErrorMessage() const {
    return errorMessage;
}

bool TrapezoidalStepper::getIsCalibrated() const {
    return isCalibrated;
}

long TrapezoidalStepper::moveToLimit(uint8_t limitSwitchPin, bool direction, float slowSpeed) {
    // Save current settings
    float savedMaxVelocity = maxVelocity;
    float savedAcceleration = acceleration;
    float savedDeceleration = deceleration;
    
    // Set slower speed for limit approach
    float limitVelocity = maxVelocity * slowSpeed;
    float limitAccel = acceleration * slowSpeed;
    setMaxVelocity(limitVelocity);
    setAcceleration(limitAccel);
    setDeceleration(limitAccel);
    
    // Make sure we're not already at the limit
    if (gpio_get(limitSwitchPin) == 1) { // Switches are active HIGH
        // Move away from the switch first (opposite direction)
        long backoffSteps = 100;
        if (!moveSteps(backoffSteps, !direction)) {
            // Restore original parameters
            setMaxVelocity(savedMaxVelocity);
            setAcceleration(savedAcceleration);
            setDeceleration(savedDeceleration);
            return -1;
        }
        
        // Wait for motion to complete
        while (!isMotionComplete()) {
            sleep_ms(10);
        }
    }
    
    // Set movement direction
    setDirection(direction);
    sleep_ms(5); // Allow time for direction change
    
    long steps = 0;
    absolute_time_t startTime = get_absolute_time();
    
    // Keep stepping until switch is triggered or timeout
    while (gpio_get(limitSwitchPin) != 1) { // Switches are active HIGH
        generateStep();
        steps++;
        
        // Update step counter based on direction
        if (direction == CW) {
            absoluteStepPos++;
        } else {
            absoluteStepPos--;
        }
        
        // Check for timeout
        if (absolute_time_diff_us(startTime, get_absolute_time()) > 
            (CALIBRATION_TIMEOUT_MS * 1000)) {
            setError("Timeout while moving to limit switch");
            // Restore original parameters
            setMaxVelocity(savedMaxVelocity);
            setAcceleration(savedAcceleration);
            setDeceleration(savedDeceleration);
            return -1;
        }
        
        // Use a reasonable delay for calibration moves
        busy_wait_us(63); // Slower speed for calibration
    }
    
    // Update position based on steps
    updatePositionFromSteps();
    
    // Restore original parameters
    setMaxVelocity(savedMaxVelocity);
    setAcceleration(savedAcceleration);
    setDeceleration(savedDeceleration);
    
    return steps;
}

bool TrapezoidalStepper::moveSteps(long numSteps, bool direction) {
    if (numSteps <= 0) return true; // Nothing to do
    
    setDirection(direction);
    sleep_ms(5); // Allow time for direction change
    
    for (long i = 0; i < numSteps; i++) {
        generateStep();
        
        // Update step counter based on direction
        if (direction == CW) {
            absoluteStepPos++;
        } else {
            absoluteStepPos--;
        }
        
        // Basic delay - not using motion profile for calibration/homing
        sleep_us(63);
    }
    
    // Update position after steps
    updatePositionFromSteps();
    return true;
}

bool TrapezoidalStepper::calibrate() {
    // 1. Move to the left limit switch
    if (moveToLimit(leftLimitPin, CCW) < 0) {
        return false; // Error already set in moveToLimit
    }
    
    // Reset step counter at left limit
    absoluteStepPos = 0;
    
    // 2. Move to the right limit switch, counting steps
    long stepsToRight = moveToLimit(rightLimitPin, CW);
    if (stepsToRight < 0) {
        return false; // Error already set
    }
    
    // Calculate the mechanical parameters
    // Want to track center of hand. Using left hand e.g., we need to account for left half of that hand
    // and then also the right half of the left hand on the right on of the rail: leftwidth/2 + leftwidth/2 = leftwidth
    // Need to also account for the entire width of the right hand, a space which the left hand cannot occupy
    totalTravelMeters = totalRailLength - leftHandWidth - rightHandWidth; // tracks centre of hand
    stepsPerMeter = static_cast<uint32_t>(stepsToRight / totalTravelMeters);
    
    // Calculate limit positions relative to homed position
    // Use totalRailLength for calculating the center of the rail
    float railCenterMeters = totalRailLength / 2.0f;
    leftLimitPosition = -railCenterMeters + leftHandWidth/2.0f - homeOffset;
    rightLimitPosition = (railCenterMeters - rightHandWidth - leftHandWidth/2.0f) - homeOffset;
    
    // Set calibration flag
    isCalibrated = true;
    
    // 3. Perform homing
    return homePosition();
}

bool TrapezoidalStepper::homePosition() {
    if (!isCalibrated) {
        setError("System not calibrated, cannot home");
        return false;
    }
    
    // Calculate steps from current position to home position
    float railCenterMeters = totalRailLength / 2.0f;
    float homePositionStepsFromLeft = (railCenterMeters + homeOffset - leftHandWidth/2.0f) * stepsPerMeter;
    
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
    absoluteStepPos = 0; // Reset step counter at home position
    
    // Reset the update time to prevent large initial time deltas
    lastUpdateTime = get_absolute_time();
    
    return true;
}

float TrapezoidalStepper::getLeftLimitPosition() const {
    return leftLimitPosition;
}

float TrapezoidalStepper::getRightLimitPosition() const {
    return rightLimitPosition;
}