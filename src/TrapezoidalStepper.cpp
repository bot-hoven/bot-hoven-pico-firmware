#include "include/TrapezoidalStepper.h"
#include <cmath>
#include <cstring>
#include <stdio.h>

// Static alarm callback function
static int64_t stepper_alarm_callback(alarm_id_t id, void *user_data) {
    TrapezoidalStepper *stepper = static_cast<TrapezoidalStepper*>(user_data);
    stepper->step();
    return 0;  // One-shot timer
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
      direction(true), isMoving(false), alarmId(0),
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
        uint32_t interval = calculateStepInterval(); // get time between pulse just sent and the next one
        alarmId = add_alarm_in_us(interval, stepper_alarm_callback, this, true);
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
        cancel_alarm(alarmId);
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
    
    // Steps needed to accelerate from entryVelocity to maxVelocity
    // d_acc = (v_max^2 - vo^2) / 2a_acc
    float accelDistance = (maxVelocity * maxVelocity - entryVelocity * entryVelocity) / (2.0f * acceleration);
    accelSteps = static_cast<int32_t>(accelDistance + 0.5f);
    
    // Steps needed to decelerate from maxVelocity to zero
    // d_dec = v_max^2 / 2a_dec
    float decelDistance = (maxVelocity * maxVelocity) / (2.0f * deceleration);
    decelSteps = static_cast<int32_t>(decelDistance + 0.5f);

    // printf("Total steps: %d\n", totalSteps);
    // printf("Sum steps: %d\tAccel Steps: %d\tDecel Steps:%d\n", accelSteps+decelSteps, accelSteps, decelSteps);
    // printf("\n");
    
    // Check if we can reach max velocity
    if (accelSteps + decelSteps > totalSteps) {
        // Triangular profile - calculate crossover point
        
        // Find peak velocity we can reach before we need to start decelerating
        // Below equation comes from D_total = d_acc + d_dec (then solve for v_peak)
        // v_peak = sqrt[ (vo^2*a_dec + 2*a_acc*a_dec*dist) / (a_acc + a_dec) ]
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
    
    // Schedule first step
    uint32_t interval = calculateStepInterval();
    alarmId = add_alarm_in_us(interval, stepper_alarm_callback, this, true);
    
    return true;
}

void TrapezoidalStepper::stop() {
    if (!isMoving) return;
    
    // Cancel the current alarm
    cancel_alarm(alarmId);
    
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
    
    // Schedule next step
    uint32_t interval = calculateStepInterval();
    alarmId = add_alarm_in_us(interval, stepper_alarm_callback, this, true);
}

void TrapezoidalStepper::emergencyStop() {
    if (isMoving) {
        cancel_alarm(alarmId);
        isMoving = false;
    }
}

const char* TrapezoidalStepper::getErrorMessage() const {
    return errorMessage;
}

bool TrapezoidalStepper::getIsCalibrated() const {
    return isCalibrated;
}

long TrapezoidalStepper::moveToLimit(uint8_t limitSwitchPin, bool direction, float slowSpeed) {
    // // printf("Moving to limit switch %u in direction %s\n", 
        //    limitSwitchPin, direction == CCW ? "CCW (left)" : "CW (right)");
    
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
        // // printf("Already at limit switch, moving away slightly\n");
        
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
    
    // // printf("Reached limit switch after %ld steps\n", steps);
    return steps;
}

bool TrapezoidalStepper::moveSteps(long numSteps, bool direction) {
    if (numSteps <= 0) return true; // Nothing to do
    
    // printf("Moving %ld steps in direction %s\n", 
        //    numSteps, direction == CCW ? "CCW (left)" : "CW (right)");
    
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
    // printf("Starting calibration sequence\n");
    
    // 1. Move to the left limit switch
    if (moveToLimit(leftLimitPin, CCW) < 0) {
        return false; // Error already set in moveToLimit
    }
    
    // Reset step counter at left limit
    absoluteStepPos = 0;
    // printf("Left limit reached. Step counter reset to 0.\n");
    
    // Move slightly away from the switch to ensure it's not triggered
    // if (!moveSteps(100, CW)) {
    //     return false;
    // }
    
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
    
    // printf("Calibration complete. Total travel: %.3f meters, %.6f meters per step (%u steps/m)\n",
        //    totalTravelMeters, 1.0f/stepsPerMeter, stepsPerMeter);
    
    // Calculate limit positions relative to homed position
    // Use totalRailLength for calculating the center of the rail
    float railCenterMeters = totalRailLength / 2.0f;
    leftLimitPosition = -railCenterMeters + leftHandWidth/2.0f - homeOffset;
    rightLimitPosition = (railCenterMeters - rightHandWidth - leftHandWidth/2.0f) - homeOffset;
    // rightLimitPosition = (railCenterMeters - rightHandWidth/2.0f) - homeOffset; // TODO: need to consider that limit is between lh and rh (above line does this)
    
    // printf("Limit positions: Left=%.3f m, Right=%.3f m (relative to home)\n",
        //    leftLimitPosition, rightLimitPosition);
    
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
    
    // printf("Homing to position at offset %.3f m from center\n", homeOffset);
    
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
    
    // printf("Homing complete. Current position set to 0.0 meters\n");
    return true;
}

float TrapezoidalStepper::getLeftLimitPosition() const {
    return leftLimitPosition;
}

float TrapezoidalStepper::getRightLimitPosition() const {
    return rightLimitPosition;
}