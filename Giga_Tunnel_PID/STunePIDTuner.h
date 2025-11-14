/*
 * STunePIDTuner.h
 * 
 * Wrapper class for sTune library integration with Wind Tunnel PID controller
 * Provides a simplified interface for autotuning PID gains using sTune's
 * inflection point method.
 * 
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef STUNE_PID_TUNER_H
#define STUNE_PID_TUNER_H

#include <Arduino.h>
#include <sTune.h>

class STunePIDTuner {
public:
    // Tuning status states
    enum TuningStatus {
        IDLE,           // Not tuning
        SETTLING,       // Settling before test
        TESTING,        // Active tuning test
        COMPLETED,      // Tuning completed successfully
        TIMEOUT,        // Tuning timed out
        ERROR           // Tuning error occurred
    };

    /**
     * Constructor
     * @param input Pointer to process variable (measured airspeed)
     * @param output Pointer to control output (PWM value)
     * @param tuningMethod sTune tuning method (default: NoOvershoot_PID)
     */
    STunePIDTuner(float *input, float *output, sTune::TuningMethod tuningMethod = sTune::NoOvershoot_PID);

    /**
     * Destructor
     */
    ~STunePIDTuner();

    /**
     * Configure and start tuning process
     * @param setpoint Target airspeed for tuning
     * @param inputSpan Maximum input range (e.g., 30.0 m/s)
     * @param outputSpan Maximum output range (e.g., 255 for PWM)
     * @param outputStart Initial output value before step
     * @param outputStep Step change in output for test
     * @param testTimeSec Estimated test duration (seconds)
     * @param settleTimeSec Settling time before test (seconds)
     * @param samples Number of samples for test
     * @return true if configuration successful
     */
    bool startTuning(float setpoint, float inputSpan = 30.0, float outputSpan = 255.0,
                     float outputStart = 0.0, float outputStep = 100.0,
                     uint32_t testTimeSec = 60, uint32_t settleTimeSec = 5,
                     uint16_t samples = 300);

    /**
     * Update the tuning process - call this in main loop during tuning
     * @return Current tuning status
     */
    TuningStatus update();

    /**
     * Check if tuning is finished
     * @return true if tuning completed (success or timeout/error)
     */
    bool isFinished();

    /**
     * Get the tuned PID gains
     * @param kp Output: Proportional gain
     * @param ki Output: Integral gain
     * @param kd Output: Derivative gain
     * @return true if tuning completed and gains are valid
     */
    bool getTunedGains(float &kp, float &ki, float &kd);

    /**
     * Reset tuner to idle state
     */
    void reset();

    /**
     * Get current tuning status
     * @return Current status
     */
    TuningStatus getStatus() const { return _status; }

    /**
     * Get status as string for debugging
     * @return Status string
     */
    const char* getStatusString() const;

    /**
     * Set emergency stop temperature/speed limit
     * @param limit Maximum safe value
     */
    void setEmergencyStop(float limit);

    /**
     * Set the tuning method
     * @param method sTune tuning method
     */
    void setTuningMethod(sTune::TuningMethod method);

    /**
     * Get the current tuning method name as string
     * @return Tuning method name
     */
    const char* getTuningMethodName() const;

    /**
     * Print tuning results to Serial
     */
    void printResults();

    /**
     * Get process characteristics
     */
    float getProcessGain() const;
    float getDeadTime() const;
    float getTau() const;

private:
    sTune *_tuner;              // sTune instance
    float *_input;              // Pointer to input variable
    float *_output;             // Pointer to output variable
    float _setpoint;            // Target setpoint for tuning
    TuningStatus _status;       // Current tuning status
    unsigned long _startTime;   // Tuning start time
    unsigned long _timeout;     // Timeout in milliseconds
    float _kp, _ki, _kd;        // Tuned gains
    bool _gainsValid;           // Flag indicating if gains are valid
    sTune::TuningMethod _tuningMethod;  // Selected tuning method
    
    // Configuration storage
    float _inputSpan;
    float _outputSpan;
    float _outputStart;
    float _outputStep;
    uint32_t _testTimeSec;
    uint32_t _settleTimeSec;
    uint16_t _samples;
};

#endif // STUNE_PID_TUNER_H
