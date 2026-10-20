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
     * @param tuningMethod sTune tuning method (e.g. sTune::Mixed_PID)
     */
    STunePIDTuner(float *input, float *output, sTune::TuningMethod tuningMethod);

    /**
     * Destructor
     */
    ~STunePIDTuner();

    // Not copyable: _tuner points into this object's own storage.
    STunePIDTuner(const STunePIDTuner &) = delete;
    STunePIDTuner &operator=(const STunePIDTuner &) = delete;

    /**
     * Configure and start tuning process
     * @param setpoint Target airspeed for tuning
     * @param inputSpan Maximum input range (e.g., 30.0 m/s)
     * @param outputSpan Maximum output range (e.g., 255 for PWM)
     * @param outputStart Output held while settling, before the step (PWM)
     * @param outputStep Output during the step (PWM): the step goes from
     *                   outputStart to outputStep (sTune's name; it is not
     *                   the size of the step)
     * @param testTimeSec Estimated test duration (seconds)
     * @param settleTimeSec Settling time before test (seconds)
     * @param samples Number of samples for test
     * @return true if configuration successful
     */
    bool startTuning(float setpoint, float inputSpan, float outputSpan,
                     float outputStart, float outputStep,
                     uint32_t testTimeSec, uint32_t settleTimeSec, uint16_t samples);

    /**
     * Update the tuning process - call this in main loop during tuning
     * @return Current tuning status
     */
    TuningStatus update();

    /**
     * Get the tuned PID gains, converted to QuickPID's parallel form in raw
     * units (output per input unit, e.g. PWM per m/s; see convertGains())
     * @param kp Output: Proportional gain
     * @param ki Output: Integral gain
     * @param kd Output: Derivative gain
     * @return true if tuning completed with a usable step response
     */
    bool getTunedGains(float &kp, float &ki, float &kd);

    /**
     * Reset tuner to idle state
     */
    void reset();

    /**
     * Get status as string for debugging
     * @return Status string
     */
    const char* getStatusString() const;

    /**
     * Set the emergency stop limit (airspeed). If the input exceeds it during
     * a run, update() returns ERROR so the caller can stop the fan. The check
     * is done here, not by sTune: sTune's own check latches for the life of
     * the object and only resets its output to outputStart.
     * @param limit Maximum safe value (0 = inputSpan of the run)
     */
    void setEmergencyStop(float limit);

    /**
     * Get the current tuning method name as string
     * @return Tuning method name
     */
    const char* getTuningMethodName() const;

    /**
     * Print how the last run ended: the process model and gains (COMPLETED),
     * or why it ended early (TIMEOUT, ERROR). update() prints nothing when a
     * run ends, so the caller can stop the fan first. Call it before reset().
     */
    void printReport();

    /**
     * Get process characteristics as sTune reports them: the process gain is
     * normalised to the spans (% input per % output, sTune.cpp line 194),
     * times in seconds
     */
    float getProcessGain() const;
    float getDeadTime() const;
    float getTau() const;

private:
    // sTune 2.4.0 leaves members uninitialised (pvInst is compared with the
    // emergency stop on the first Run(); its _outputStart is written to *output)
    // and never clears its emergency-stop latch. So each run gets a new sTune
    // object built in zero-filled storage inside this wrapper (no separate
    // heap block).
    void createTuner();
    void convertGains(float kpSpan, float invTi, float invTd);
    bool resultUsable() const;

    alignas(sTune) unsigned char _tunerStorage[sizeof(sTune)];
    sTune *_tuner;              // sTune instance (lives in _tunerStorage)
    float *_input;              // Pointer to input variable
    float *_output;             // Pointer to output variable
    float _setpoint;            // Target setpoint for tuning
    TuningStatus _status;       // Current tuning status
    unsigned long _startTime;   // Tuning start time
    unsigned long _timeout;     // Safety net: settle + test time + 50 %, in ms
    float _kp, _ki, _kd;        // Tuned gains
    bool _gainsValid;           // Flag indicating if gains are valid
    sTune::TuningMethod _tuningMethod;  // Selected tuning method
    
    // Configuration of the present run (set by startTuning())
    float _inputSpan;
    float _outputSpan;
    uint32_t _settleTimeSec;
    float _eStop;               // emergency stop limit (0 = inputSpan of the run)
    float _eStopInput;          // input that tripped the emergency stop
};

#endif // STUNE_PID_TUNER_H
