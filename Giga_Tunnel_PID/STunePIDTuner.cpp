/*
 * STunePIDTuner.cpp
 * 
 * Wrapper class implementation for sTune library integration
 * 
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include "STunePIDTuner.h"

STunePIDTuner::STunePIDTuner(float *input, float *output, sTune::TuningMethod tuningMethod)
    : _input(input),
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
      _samples(300) {
    
    // Create sTune instance with selected tuning method
    _tuner = new sTune(input, output, _tuningMethod, sTune::directIP, sTune::printSUMMARY);
}

STunePIDTuner::~STunePIDTuner() {
    if (_tuner != nullptr) {
        delete _tuner;
        _tuner = nullptr;
    }
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
    
    // Configure sTune
    _tuner->Configure(inputSpan, outputSpan, outputStart, outputStep, 
                      testTimeSec, settleTimeSec, samples);
    
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

STunePIDTuner::TuningStatus STunePIDTuner::update() {
    // Check timeout
    if (_status != IDLE && _status != COMPLETED && _status != ERROR) {
        if (millis() - _startTime > _timeout) {
            Serial.println("\n[sTune] TIMEOUT!");
            Serial.println("[sTune] Tuning took too long - aborting");
            Serial.print("[sTune] Duration: "); Serial.print((millis() - _startTime) / 1000);
            Serial.println(" seconds");
            Serial.println("[sTune] Troubleshooting:");
            Serial.println("        - Check if system responds to output step");
            Serial.println("        - Verify sensors are working correctly");
            Serial.println("        - Try larger output step");
            Serial.println("        - Increase test time estimate");
            _status = TIMEOUT;
            return _status;
        }
    }

    // Run sTune state machine
    if (_status == SETTLING || _status == TESTING) {
        uint8_t tuneResult = _tuner->Run();
        
        switch (tuneResult) {
            case sTune::sample:
                // Active sampling during test
                if (_status == SETTLING) {
                    _status = TESTING;
                    Serial.println("[sTune] Settling complete - starting test");
                    Serial.println("[sTune] Monitoring inflection point...");
                }
                break;
                
            case sTune::tunings: {
                // Tuning completed successfully
                _tuner->GetAutoTunings(&_kp, &_ki, &_kd);
                _gainsValid = true;
                _status = COMPLETED;
                
                unsigned long duration = (millis() - _startTime) / 1000;
                
                Serial.println("\n=====================================");
                Serial.println("[sTune] AUTOTUNING COMPLETE!");
                Serial.println("=====================================");
                Serial.print("[sTune] Duration: "); Serial.print(duration); Serial.println(" seconds");
                Serial.println("[sTune] Tuned PID Gains:");
                Serial.print("        Kp = "); Serial.println(_kp, 4);
                Serial.print("        Ki = "); Serial.println(_ki, 4);
                Serial.print("        Kd = "); Serial.println(_kd, 4);
                Serial.println("[sTune] Process Characteristics:");
                Serial.print("        Process Gain: "); Serial.println(getProcessGain(), 4);
                Serial.print("        Dead Time: "); Serial.print(getDeadTime(), 2); Serial.println(" s");
                Serial.print("        Time Constant (Tau): "); Serial.print(getTau(), 2); Serial.println(" s");
                Serial.print("[sTune] Tuning method: "); Serial.println(getTuningMethodName());
                Serial.println("[sTune] Gains ready to apply");
                Serial.println("=====================================\n");
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

void STunePIDTuner::reset() {
    _status = IDLE;
    _gainsValid = false;
    _kp = 0.0;
    _ki = 0.0;
    _kd = 0.0;
    _setpoint = 0.0;
    _startTime = 0;
    
    if (_tuner != nullptr) {
        _tuner->Reset();
    }
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
    if (_tuner != nullptr) {
        _tuner->SetEmergencyStop(limit);
    }
}

void STunePIDTuner::printResults() {
    if (_tuner != nullptr) {
        _tuner->printResults();
    }
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
