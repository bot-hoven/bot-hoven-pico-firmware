#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

// Define timer IRQs and alarms
#define LEFT_STEPPER_TIMER timer0_hw
#define RIGHT_STEPPER_TIMER timer1_hw
#define LEFT_STEPPER_ALARM_NUM 0   // Use alarm 0 on timer0
#define RIGHT_STEPPER_ALARM_NUM 0  // Use alarm 0 on timer1
#define TIMER0_IRQ TIMER0_IRQ_0 // The specific IRQ for timer0, alarm 0
#define TIMER1_IRQ TIMER1_IRQ_0 // The specific IRQ for timer1, alarm 0

// Forward declarations for IRQ handlers
void left_stepper_irq(void);
void right_stepper_irq(void);

class SimplifiedStepper {
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
    float leftBoundary;  // Left-most position this stepper can move to
    float rightBoundary; // Right-most position this stepper can move to
    
    // PID controller parameters
    float kp;                   // Proportional gain
    float ki;                   // Integral gain
    float kd;                   // Derivative gain
    float errorIntegral;        // Accumulated error (for integral term)
    float lastError;            // Previous error (for derivative term)
    uint64_t lastErrorTime;     // Time of last error calculation (microseconds)
    float minOutput;            // Minimum PID output (steps/second)
    float maxOutput;            // Maximum PID output (steps/second)
    
    // State variables
    float currentPosition;      // Current position in meters
    float targetPosition;       // Target position in meters
    int32_t absoluteStepPos;    // Absolute step position
    bool direction;             // Movement direction (true = CW, false = CCW)
    bool isMoving;              // Flag indicating if the motor is currently moving
    char errorMessage[100];     // Error message buffer
    
    // Movement variables
    float currentVelocity;      // Current velocity in steps/second
    uint64_t lastStepTime;      // Time of the last step in microseconds
    uint32_t minStepInterval;   // Minimum step interval in microseconds
    uint8_t alarmNum;           // Timer alarm number for this stepper

    /**
     * Calculate the step interval using PID control
     * @return Step interval in microseconds
     */
    uint32_t calculatePidStepInterval();

    /**
     * Update the PID controller
     * @return Velocity in steps/second
     */
    float updatePid();

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
    
    /**
     * Handle a timer interrupt - called from IRQ handler
     */
    void handleIrq();
    
    // Friend declarations for IRQ handlers
    friend void left_stepper_irq(void);
    friend void right_stepper_irq(void);

public:
    // Static instances for IRQ handler access
    static SimplifiedStepper* left_instance;
    static SimplifiedStepper* right_instance;
    
    // Direction constants
    static const bool CW = true;    // Clockwise direction
    static const bool CCW = false;  // Counter-clockwise direction
    
    // Timeout constants
    static const uint32_t CALIBRATION_TIMEOUT_MS = 60000; // 1 minute timeout for calibration
    
    /**
     * Constructor
     * @param pulsePin GPIO pin for pulse signal
     * @param dirPin GPIO pin for direction signal
     * @param enablePin GPIO pin for enable signal, -1 if not used
     * @param leftLimitPin GPIO pin for left limit switch
     * @param rightLimitPin GPIO pin for right limit switch
     * @param stepsPerMeter Conversion factor: steps per meter of linear movement
     * @param totalRailLength Total physical length of the rail in meters
     * @param leftHandWidth Width of the left-side actuator in meters
     * @param rightHandWidth Width of the right-side actuator in meters
     * @param homeOffset Offset from center for home position in meters
     * @param maxStepRate Maximum step rate in steps/second (default 50000)
     * @param minPulseWidth Minimum pulse width in microseconds (default 2)
     * @param alarmNum Timer alarm number to use (2 for left, 3 for right)
     */
    SimplifiedStepper(uint8_t pulsePin, uint8_t dirPin, int8_t enablePin,
                    uint8_t leftLimitPin, uint8_t rightLimitPin,
                    uint32_t stepsPerMeter,
                    float totalRailLength, float leftHandWidth, float rightHandWidth, float homeOffset,
                    uint32_t maxStepRate = 50000, uint32_t minPulseWidth = 4,
                    uint8_t alarmNum = 2);
    
    /**
     * Destructor to clean up IRQ handlers
     */
    ~SimplifiedStepper();
    
    /**
     * Perform a single update step for the motor
     * This is now used mainly for PID calculations and non-IRQ operations
     * @return true if motor moved, false otherwise
     */
    bool update();
    
    /**
     * Step function called from IRQ handler
     */
    void step();

    /**
     * Set PID controller parameters
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     */
    void setPidParameters(float p, float i, float d);

    /**
     * Reset the PID controller state
     */
    void resetPid();

    /**
     * Enable or disable the motor
     * @param enable True to enable, false to disable
     */
    void enableMotor(bool enable);

    void setLeftBoundary(float position);
    void setRightBoundary(float position);
    float getLeftBoundary() const;
    float getRightBoundary() const;
    long getStepCount() const;
    long getStepsPerMeter() const;

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
     * Move to a specific position using PID control
     * @param position Target position in meters
     * @return False if move wasn't started, true otherwise
     */
    bool moveTo(float position);

    /**
     * Stop the motor
     */
    void stop();
    
    /**
     * Emergency stop - immediately stop the motor
     */
    void emergencyStop();
    
    /**
     * Get the current limit switch status
     * @return true if any limit switch is triggered
     */
    bool isLimitSwitchTriggered() const;
    
    /**
     * Get the last error message
     * @return Error message string
     */
    const char* getErrorMessage() const;
    
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
     * Get the left limit position
     * @return Left limit position in meters (relative to home)
     */
    float getLeftLimitPosition() const;
    
    /**
     * Get the right limit position
     * @return Right limit position in meters (relative to home)
     */
    float getRightLimitPosition() const;

    uint32_t getStepsTaken() const;
    void resetStepCounter();
    long getAbsoluteStepPos() const;
    void setStepsPerMeter(uint32_t steps);
};