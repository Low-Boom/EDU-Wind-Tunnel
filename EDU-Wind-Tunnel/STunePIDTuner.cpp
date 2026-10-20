/*
 * STunePIDTuner.cpp
 * 
 * Wrapper class implementation for sTune library integration
 * 
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include "STunePIDTuner.h"
#include "PlatformConfig.h" // serialHasRoom()
#include <new>      // placement new
#include <string.h> // memset, strlen
#include <math.h>   // INFINITY, isfinite

static bool isPositiveFinite(float x) {
    return isfinite(x) && x > 0.0;
}

// Progress messages are optional (the sketch's data lines show the phase as
// well), so one that would block the loop is skipped (see serialHasRoom()).
static void printProgress(const char *line) {
    if (serialHasRoom((int)strlen(line) + 2)) Serial.println(line);   // + CR LF
}

STunePIDTuner::STunePIDTuner(float *input, float *output, sTune::TuningMethod tuningMethod)
    : _tuner(nullptr),
      _input(input),
      _output(output),
      _setpoint(0.0),
      _status(IDLE),
      _startTime(0),
      _timeout(120000), // Default 120 second timeout
      _kp(0.0),
      _ki(0.0),
      _kd(0.0),
      _gainsValid(false),
      _tuningMethod(tuningMethod),
      _inputSpan(30.0),
      _outputSpan(255.0),
      _outputStart(0.0),
      _outputStep(100.0),
      _testTimeSec(60),
      _settleTimeSec(5),
      _samples(300),
      _eStop(0.0),
      _eStopInput(0.0) {
    createTuner();
}

STunePIDTuner::~STunePIDTuner() {
    if (_tuner != nullptr) {
        _tuner->~sTune();
        _tuner = nullptr;
    }
}

// Build a new sTune object in zero-filled storage, so nothing is left over
// from an earlier run or from old RAM contents: pvInst starts at 0 and the
// emergency-stop latch (eStopAbort) is clear. Called for every run.
void STunePIDTuner::createTuner() {
    float savedOutput = *_output;   // the constructor writes *output; keep the caller's
    if (_tuner != nullptr) {
        _tuner->~sTune();
    }
    memset(_tunerStorage, 0, sizeof(_tunerStorage));
    // Compiler barrier: the zero fill must really be written before the
    // constructor runs (GCC's -flifetime-dse may otherwise drop stores to an
    // object's storage that happen before its constructor).
    __asm__ __volatile__("" : : "r"(_tunerStorage) : "memory");
    // printOFF: sTune's own summary prints its gains in its own form (see
    // convertGains()); printReport() prints the converted ones instead.
    _tuner = new (_tunerStorage) sTune(_input, _output, _tuningMethod,
                                       sTune::directIP, sTune::printOFF);
    *_output = savedOutput;
}

bool STunePIDTuner::startTuning(float setpoint, float inputSpan, float outputSpan,
                                 float outputStart, float outputStep,
                                 uint32_t testTimeSec, uint32_t settleTimeSec,
                                 uint16_t samples) {
    
    if (setpoint <= 0.0) {
        Serial.println("[sTune] ERROR: Invalid setpoint (must be > 0)");
        _status = ERROR;
        return false;
    }

    // Store configuration
    _setpoint = setpoint;
    _inputSpan = inputSpan;
    _outputSpan = outputSpan;
    _outputStart = outputStart;
    _outputStep = outputStep;
    _testTimeSec = testTimeSec;
    _settleTimeSec = settleTimeSec;
    _samples = samples;
    
    // Reset state
    _gainsValid = false;
    _kp = 0.0;
    _ki = 0.0;
    _kd = 0.0;
    
    // Fresh sTune object for this run, then configure it.
    // Memory: Configure() -> sTan::begin() allocates a new float[samples * 0.06]
    // buffer for the sliding tangent on every call and never frees the old
    // one (4 floats = 16 bytes of heap per sTune run with 75 samples). The
    // buffer belongs to the global sTan object in sTune.cpp, not to the sTune
    // object, so the wrapper cannot free it without changing the library.
    // Configure() is called exactly once per run.
    createTuner();
    _tuner->Configure(inputSpan, outputSpan, outputStart, outputStep, 
                      testTimeSec, settleTimeSec, samples);
    // Configure() starts with Reset(), which writes the old start value (0 in
    // a new object) to *output; the fan must hold outputStart instead.
    *_output = outputStart;
    // The emergency stop is checked in update(). sTune's own check is turned
    // off (an infinite limit can never trip), because it latches for the
    // life of the object, holds the output at outputStart without telling
    // the caller, and leaves the run to end only by the timeout.
    _tuner->SetEmergencyStop(INFINITY);
    
    // Calculate timeout: settle time + test time + 50% margin
    _timeout = (settleTimeSec + testTimeSec) * 1500; // milliseconds
    _startTime = millis();
    _status = SETTLING;
    
    Serial.println("\n=====================================");
    Serial.println("[sTune] AUTOTUNING STARTED");
    Serial.println("=====================================");
    Serial.print("[sTune] Target setpoint: "); Serial.print(_setpoint, 2); Serial.println(" m/s");
    Serial.print("[sTune] Method: Inflection Point ("); Serial.print(getTuningMethodName()); Serial.println(")");
    Serial.println("\n[sTune] Configuration:");
    Serial.print("        Input span: "); Serial.print(inputSpan, 1); Serial.println(" m/s");
    Serial.print("        Output span: "); Serial.print(outputSpan, 0); Serial.println(" PWM");
    Serial.print("        Output start: "); Serial.print(outputStart, 0); Serial.println(" PWM");
    Serial.print("        Output step: "); Serial.print(outputStep, 0); Serial.println(" PWM");
    Serial.print("        Test time: "); Serial.print(testTimeSec); Serial.println(" sec");
    Serial.print("        Settle time: "); Serial.print(settleTimeSec); Serial.println(" sec");
    Serial.print("        Samples: "); Serial.println(samples);
    Serial.print("        Timeout: "); Serial.print(_timeout / 1000); Serial.println(" sec");
    Serial.println("=====================================");
    Serial.println("[sTune] Starting settling phase...");
    
    return true;
}

// When a run ends (COMPLETED, TIMEOUT, ERROR) nothing is printed here: the
// caller stops the fan or hands it to the PID first and then calls
// printReport(), because printing can block (see serialHasRoom()).
STunePIDTuner::TuningStatus STunePIDTuner::update() {
    // Check timeout
    if (_status != IDLE && _status != COMPLETED && _status != ERROR) {
        if (millis() - _startTime > _timeout) {
            _status = TIMEOUT;
            return _status;
        }
    }

    // Emergency stop: input above the limit during a run ends it with ERROR,
    // so the caller stops the fan at once.
    float eStopLimit = (_eStop > 0.0) ? _eStop : _inputSpan;
    if ((_status == SETTLING || _status == TESTING) && *_input > eStopLimit) {
        _eStopInput = *_input;
        _status = ERROR;
        return _status;
    }

    // sTune returns "sample" while settling too, so the switch to TESTING is
    // based on the configured settle time, not on the first sample.
    if (_status == SETTLING && millis() - _startTime >= _settleTimeSec * 1000UL) {
        _status = TESTING;
        printProgress("[sTune] Settling complete - starting test");
        printProgress("[sTune] Monitoring inflection point...");
    }

    // Run sTune state machine
    if (_status == SETTLING || _status == TESTING) {
        uint8_t tuneResult = _tuner->Run();
        
        switch (tuneResult) {

            case sTune::tunings: {
                // The test has ended. sTune also ends it when the airspeed
                // never rose clearly (fan not reacting, hose off, step too
                // small), so check the result before offering the gains.
                float kpSpan, invTi, invTd;
                _tuner->GetAutoTunings(&kpSpan, &invTi, &invTd);
                convertGains(kpSpan, invTi, invTd);
                _gainsValid = resultUsable();
                _status = COMPLETED;
                break;
            }
                
            case sTune::runPid:
                // This shouldn't happen in our use case
                break;
        }
    }
    
    return _status;
}

bool STunePIDTuner::isFinished() {
    return (_status == COMPLETED || _status == TIMEOUT || _status == ERROR);
}

bool STunePIDTuner::getTunedGains(float &kp, float &ki, float &kd) {
    if (!_gainsValid || _status != COMPLETED) {
        return false;
    }
    
    kp = _kp;
    ki = _ki;
    kd = _kd;
    return true;
}

// Back to IDLE. The sTune object is left alone (every run builds a new one),
// so the caller's output variable keeps its value: sTune's own Reset() would
// overwrite it with the step's start value.
void STunePIDTuner::reset() {
    _status = IDLE;
    _gainsValid = false;
    _kp = 0.0;
    _ki = 0.0;
    _kd = 0.0;
    _setpoint = 0.0;
    _startTime = 0;
}

const char* STunePIDTuner::getStatusString() const {
    switch (_status) {
        case IDLE: return "IDLE";
        case SETTLING: return "SETTLING";
        case TESTING: return "TESTING";
        case COMPLETED: return "COMPLETED";
        case TIMEOUT: return "TIMEOUT";
        case ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void STunePIDTuner::setEmergencyStop(float limit) {
    _eStop = limit;   // checked in update(); 0 = use the run's inputSpan
}

// sTune 2.4.0 does not give its gains in QuickPID's form (sTune.cpp):
// - Kp is per unit of span: the process gain is normalised to the input and
//   output spans (line 194), so every Kp rule is "% output per % input";
// - GetKi() returns 1/Ti, e.g. Ziegler-Nichols 1/(2 td) (line 429);
// - GetKd() returns 1/Td, e.g. 1/(0.5 td) (line 452), and 0 for PI methods.
// QuickPID is parallel form in raw units (PWM, m/s):
//   output = Kp e + Ki integral(e) + Kd de/dt, with Ki = Kp/Ti, Kd = Kp Td.
void STunePIDTuner::convertGains(float kpSpan, float invTi, float invTd) {
    _kp = kpSpan * _outputSpan / _inputSpan;           // PWM per m/s
    _ki = _kp * invTi;                                 // Kp / Ti
    _kd = (invTd == 0.0) ? 0.0 : _kp / invTd;          // Kp * Td (PI: no D)
}

// A step test without a clear response still "completes" in sTune, with a
// zero (or nan) dead time, time constant or process gain; its formulas then
// divide by zero and give nan/inf gains. Only a complete model is usable.
bool STunePIDTuner::resultUsable() const {
    return isPositiveFinite(_kp) && isPositiveFinite(_ki) && isfinite(_kd) && _kd >= 0.0 &&
           isPositiveFinite(getTau()) && isPositiveFinite(getDeadTime()) &&
           isPositiveFinite(getProcessGain());
}

// How the last run ended: the process model and, if usable, the PID gains
// (COMPLETED), or why it stopped early (TIMEOUT, ERROR).
void STunePIDTuner::printReport() {
    if (_status == TIMEOUT) {
        Serial.println("[sTune] TIMEOUT!");
        Serial.println("[sTune] Tuning took too long - aborting");
        Serial.print("[sTune] Duration: "); Serial.print((millis() - _startTime) / 1000);
        Serial.println(" seconds");
        Serial.println("[sTune] Troubleshooting:");
        Serial.println("        - Check if system responds to output step");
        Serial.println("        - Verify sensors are working correctly");
        Serial.println("        - Try larger output step");
        Serial.println("        - Increase test time estimate");
        return;
    }
    if (_status == ERROR) {
        Serial.print("[sTune] ABORT: airspeed "); Serial.print(_eStopInput, 2);
        Serial.print(" m/s above the limit of "); Serial.print((_eStop > 0.0) ? _eStop : _inputSpan, 2);
        Serial.println(" m/s");
        return;
    }
    if (_status != COMPLETED) return;    // stopped by the user: nothing to add

    Serial.println("\n=====================================");
    Serial.println(_gainsValid ? "[sTune] AUTOTUNING COMPLETE" : "[sTune] NO USABLE STEP RESPONSE");
    Serial.println("=====================================");
    Serial.print("[sTune] Duration: "); Serial.print((millis() - _startTime) / 1000);
    Serial.println(" seconds");
    Serial.println("[sTune] Process model from the step response:");
    Serial.print("        Process gain: "); Serial.print(getProcessGain() * _inputSpan / _outputSpan, 4);
    Serial.println(" m/s per PWM");
    Serial.print("        Dead time: "); Serial.print(getDeadTime(), 2); Serial.println(" s");
    Serial.print("        Time constant (Tau): "); Serial.print(getTau(), 2); Serial.println(" s");
    if (_gainsValid) {
        Serial.print("[sTune] PID gains ("); Serial.print(getTuningMethodName()); Serial.println("):");
        Serial.print("        Kp = "); Serial.println(_kp, 4);
        Serial.print("        Ki = "); Serial.println(_ki, 4);
        Serial.print("        Kd = "); Serial.println(_kd, 4);
    } else {
        Serial.println("[sTune] The airspeed did not rise clearly after the step, so no gains");
        Serial.println("        can be computed. Check fan power, the PWM wiring and the pitot");
        Serial.println("        hoses, or set a higher target (larger step), then try again.");
    }
    Serial.println("=====================================\n");
}

float STunePIDTuner::getProcessGain() const {
    if (_tuner != nullptr) {
        return _tuner->GetProcessGain();
    }
    return 0.0;
}

float STunePIDTuner::getDeadTime() const {
    if (_tuner != nullptr) {
        return _tuner->GetDeadTime();
    }
    return 0.0;
}

float STunePIDTuner::getTau() const {
    if (_tuner != nullptr) {
        return _tuner->GetTau();
    }
    return 0.0;
}

void STunePIDTuner::setTuningMethod(sTune::TuningMethod method) {
    _tuningMethod = method;
    if (_tuner != nullptr) {
        _tuner->SetTuningMethod(method);
    }
}

const char* STunePIDTuner::getTuningMethodName() const {
    switch (_tuningMethod) {
        case sTune::ZN_PID:
            return "Ziegler-Nichols PID";
        case sTune::DampedOsc_PID:
            return "Damped Oscillation PID";
        case sTune::NoOvershoot_PID:
            return "No Overshoot PID";
        case sTune::CohenCoon_PID:
            return "Cohen-Coon PID";
        case sTune::Mixed_PID:
            return "Mixed PID";
        case sTune::ZN_PI:
            return "Ziegler-Nichols PI";
        case sTune::DampedOsc_PI:
            return "Damped Oscillation PI";
        case sTune::NoOvershoot_PI:
            return "No Overshoot PI";
        case sTune::CohenCoon_PI:
            return "Cohen-Coon PI";
        case sTune::Mixed_PI:
            return "Mixed PI";
        default:
            return "Unknown";
    }
}
