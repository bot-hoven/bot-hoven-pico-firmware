/**
 * TrapezoidalStepper.h
 * A class for controlling stepper motors with a trapezoidal velocity profile
 * using the CL42T(V4.1) Closed Loop Stepper Driver.
 */

 #pragma once

 #include "pico/stdlib.h"
 #include "hardware/timer.h"
 #include "hardware/gpio.h"
 #include "hardware/irq.h"
 
 // Forward declarations for IRQ handlers
 void left_stepper_irq(void);
 void right_stepper_irq(void);
 
 class TrapezoidalStepper {
 private:
     // Hardware configuration
     uint8_t pulsePin;           // GPIO pin for pulse (step) signal
     uint8_t dirPin;             // GPIO pin for direction signal
     uint8_t enablePin;          // GPIO pin for enable signal (optional)
     uint8_t leftLimitPin;       // GPIO pin for left limit switch
     uint8_t rightLimitPin;      // GPIO pin for right limit switch
     uint32_t stepsPerMeter;     // Conversion factor: steps per meter of linear movement
     uint32_t minPulseWidth;     // Minimum pulse width in microseconds
     uint32_t maxStepRate;       // Maximum step rate in steps/second
 
     // Calibration parameters
     float totalRailLength;      // Total length of rail in meters
     float leftHandWidth;        // Width of left-side actuator in meters
     float rightHandWidth;       // Width of right-side actuator in meters
     float homeOffset;           // Offset from center for home position in meters
     
     // Calibration derived values
     float totalTravelMeters;    // Actual travel distance in meters
     float leftLimitPosition;    // Position of left limit in meters (relative to home)
     float rightLimitPosition;   // Position of right limit in meters (relative to home)
     bool isCalibrated;          // Flag indicating if the system is calibrated
     
     // Motion parameters
     float maxVelocity;          // Maximum velocity in steps/second
     float acceleration;         // Acceleration in steps/second^2
     float deceleration;         // Deceleration in steps/second^2
     
     // State variables
     float currentPosition;      // Current position in meters
     float targetPosition;       // Target position in meters
     int32_t totalSteps;         // Total steps to move
     int32_t currentStep;        // Current step in the current move
     int32_t absoluteStepPos;    // Absolute step position
     int32_t accelSteps;         // Number of steps in acceleration phase
     int32_t constVelSteps;      // Number of steps in constant velocity phase
     int32_t decelSteps;         // Number of steps in deceleration phase
     bool direction;             // Movement direction (true = CW, false = CCW)
     bool isMoving;              // Flag indicating if the motor is currently moving
     char errorMessage[100];     // Error message buffer
     
     // Timing variables
     uint64_t lastStepTime;      // Time of the last step in microseconds
     uint32_t minStepInterval;   // Minimum step interval in microseconds
     float currentVelocity;      // Current velocity in steps/second
     float entryVelocity;        // Velocity at the start of the move
     float highestVelocity;      // Top velocity reached during move (peak or max)
     absolute_time_t lastUpdateTime; // Time of the last position update
 
     /**
      * Calculate the step interval for the current step
      * @return Step interval in microseconds
      */
     uint32_t calculateStepInterval();
 
     /**
      * Set error message
      * @param message Error message to set
      */
     void setError(const char* message);
     
     /**
      * Generate a single step pulse
      */
     void generateStep();
     
     /**
      * Set motor direction
      * @param isClockwise true for clockwise, false for counter-clockwise
      */
     void setDirection(bool isClockwise);
     
     /**
      * Update the position value based on steps taken
      */
     void updatePositionFromSteps();
     
     /**
      * Arm the hardware timer for the next step
      * @param delay_us Delay in microseconds until the next step
      */
     void armTimerForNextStep(uint32_t delay_us);
 
     // Friend declarations for IRQ handlers
     friend void left_stepper_irq(void);
     friend void right_stepper_irq(void);
 
 public:
     // Static instances for IRQ handler access
     static TrapezoidalStepper* left_instance;
     static TrapezoidalStepper* right_instance;
 
     // Direction constants
     static const bool CW = true;    // Clockwise direction
     static const bool CCW = false;  // Counter-clockwise direction
     
     // Timeout constants
     static const uint32_t CALIBRATION_TIMEOUT_MS = 60000; // 1 minute timeout for calibration
     
     /**
      * Constructor
      * @param pulsePin GPIO pin for pulse signal (connected to PUL+ on driver)
      * @param dirPin GPIO pin for direction signal (connected to DIR+ on driver)
      * @param enablePin GPIO pin for enable signal (connected to ENA+ on driver), -1 if not used
      * @param leftLimitPin GPIO pin for left limit switch
      * @param rightLimitPin GPIO pin for right limit switch
      * @param stepsPerMeter Conversion factor: steps per meter of linear movement, will be recalculated during calibration
      * @param totalRailLength Total physical length of the rail in meters
      * @param leftHandWidth Width of the left-side actuator in meters
      * @param rightHandWidth Width of the right-side actuator in meters
      * @param homeOffset Offset from center for home position in meters
      * @param maxStepRate Maximum step rate in steps/second (default 50000)
      * @param minPulseWidth Minimum pulse width in microseconds (default 2)
      */
     TrapezoidalStepper(uint8_t pulsePin, uint8_t dirPin, int8_t enablePin,
                      uint8_t leftLimitPin, uint8_t rightLimitPin,
                      uint32_t stepsPerMeter,
                      float totalRailLength, float leftHandWidth, float rightHandWidth, float homeOffset,
                      uint32_t maxStepRate = 50000, uint32_t minPulseWidth = 4);
     
     /**
      * Destructor to clean up IRQ handlers
      */
     ~TrapezoidalStepper();
 
     /**
      * Perform a single step - called from IRQ handler
      */
     void step();
 
     /**
      * Set the maximum velocity
      * @param velocity Maximum velocity in steps/second
      */
     void setMaxVelocity(float velocity);
 
     /**
      * Set the acceleration rate
      * @param accel Acceleration in steps/second^2
      */
     void setAcceleration(float accel);
 
     /**
      * Set the deceleration rate
      * @param decel Deceleration in steps/second^2
      */
     void setDeceleration(float decel);
 
     /**
      * Enable or disable the motor
      * @param enable True to enable, false to disable
      */
     void enableMotor(bool enable);
 
     /**
      * Get the current position
      * @return Current position in meters
      */
     float getCurrentPosition() const;
 
     /**
      * Check if motion is complete
      * @return True if motion is complete, false otherwise
      */
     bool isMotionComplete() const;
 
     /**
      * Get the current velocity
      * @return Current velocity in steps/second
      */
     float getCurrentVelocity() const;
 
     /**
      * Set current position without moving the motor
      * @param position New current position in meters
      */
     void setCurrentPosition(float position);
 
     /**
      * Move to a specific position with trapezoidal velocity profile
      * @param position Target position in meters
      * @param startVelocity Initial velocity in steps/second (default 0)
      * @return False if move wasn't started, true otherwise
      */
     bool moveTo(float position, float startVelocity = 0.0f);
 
     /**
      * Stop the motor with controlled deceleration
      */
     void stop();
 
     /**
      * Emergency stop (immediate halt without deceleration)
      */
     void emergencyStop();
     
     /**
      * Get the last error message
      * @return Error message string
      */
     const char* getErrorMessage() const;
     
     /**
      * Check if the system is calibrated
      * @return True if calibrated, false otherwise
      */
     bool getIsCalibrated() const;
     
     /**
      * Move to a limit switch, return steps taken or -1 on error
      * @param limitSwitchPin GPIO pin for the limit switch
      * @param direction Movement direction (true for CW, false for CCW)
      * @param slowSpeed Speed reduction factor for careful approach (0.1-1.0)
      * @return Number of steps taken or -1 on error
      */
     long moveToLimit(uint8_t limitSwitchPin, bool direction, float slowSpeed = 0.25f);
     
     /**
      * Move a specific number of steps in a direction at calibration speed
      * @param numSteps Number of steps to move
      * @param direction Movement direction (true for CW, false for CCW)
      * @return True if successful, false on error
      */
     bool moveSteps(long numSteps, bool direction);
     
     /**
      * Calibrate the system using limit switches
      * @return True if calibration was successful, false otherwise
      */
     bool calibrate();
     
     /**
      * Home the system to the defined home position
      * @return True if homing was successful, false otherwise
      */
     bool homePosition();
     
     /**
      * Get the left limit position
      * @return Left limit position in meters (relative to home)
      */
     float getLeftLimitPosition() const;
     
     /**
      * Get the right limit position
      * @return Right limit position in meters (relative to home)
      */
     float getRightLimitPosition() const;
 };