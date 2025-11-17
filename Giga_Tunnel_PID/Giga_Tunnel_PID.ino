// Basic Arduino includes
#include <Arduino.h>
#include <Wire.h>

// Sensor libraries
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <QuickPID.h>
#include "ms4525do.h"
#include "math.h"

// sTune library and wrapper (optional - for advanced autotuning)
#include <sTune.h>
#include "STunePIDTuner.h"

// I2C Scanner utility
#include "I2CScanner.h"

// sTune Configuration - User Selectable Tuning Method
// Available methods:
//   sTune::ZN_PID           - Ziegler-Nichols (moderate overshoot, fast response)
//   sTune::DampedOsc_PID    - Damped Oscillation (balanced)
//   sTune::NoOvershoot_PID  - No Overshoot (conservative, stable) [DEFAULT]
//   sTune::CohenCoon_PID    - Cohen-Coon (good for systems with delay)
//   sTune::Mixed_PID        - Mixed (average of all methods)
// For PI control, use: ZN_PI, DampedOsc_PI, NoOvershoot_PI, CohenCoon_PI, Mixed_PI
const sTune::TuningMethod STUNE_METHOD = sTune::Mixed_PID;

// MBED support
// #include "mbed.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define DPS_ADDRESS 0x28
#define BMP_I2C_ADDRESS 0x77

// PID gains - Full PID control
float Kp = 25.0, Ki = 10.0, Kd = 15.0;

// Pin assignments
const int PWM_pin = 9;    // Motor PWM control output
const int TACH_pin = 2;   // Tachometer pulse input
const int ENC_PPR = 2;
const float kI = 1;

const int MAX_PRESSURE = 14745.0;
const float MIN_PRESSURE = 1638.0;

const float MAX_AIRSPEED = 30.0;
const float MIN_AIRSPEED = 0.0;
const float Specific_gas_constant = 287.058;

// Pressure sensor calibration
float pressureOffset = 0.0;
bool calibrated = false;
int calibrationSamples = 50;  // Default calibration samples (adjustable)
const int MIN_CALIB_SAMPLES = 5;
const int MAX_CALIB_SAMPLES = 100;

// Sensor oversampling configuration
const int PRESSURE_OVERSAMPLES = 5;  // Read sensor 5 times per measurement

// Pressure sensor averaging configuration
int pressureAverageSamples = 20;  // Default: 20 samples
int savedAverageSamples = 20;     // Saved value during auto-tune
const int MAX_AVG_SAMPLES = 50;   // Maximum: 50 samples
const int MIN_AVG_SAMPLES = 1;
float pressureBuffer[MAX_AVG_SAMPLES];
int pressureBufferIndex = 0;
bool pressureBufferFilled = false;

// Temperature correction
float lastAirDensity = 1.225;  // Cache for display
float lastTemperature = 15.0;  // Cache for display

// Timing
const unsigned long updateInterval = 200;  // 200ms update interval
unsigned long lastUpdate = 0;
unsigned long startTime = 0;

unsigned int LastValidRPM = 0;
unsigned long LastRPMUpdateTime = 0;
const unsigned long RPM_TIMEOUT_MS = 1000;

// PWM Rate Limiting (disabled during tuning)
float lastPWMOutput = 0;
const float MAX_PWM_CHANGE_PER_CYCLE = 25.0;

// PID and control variables
float currPWM = 0;
float desiredAirSpeed = 0;
float currentAirSpeed = 0;

// Relay Auto-tuning configuration
const float MIN_TUNE_SETPOINT = 2.0;
bool autoTuning = false;
float tuneSetpoint = 0.0;
uint32_t tuneStartTime = 0;

// Relay tuning state
enum RelayState { RELAY_HIGH, RELAY_LOW };
RelayState relayState = RELAY_HIGH;
float relayOutputHigh = 0;
float relayOutputLow = 0;
float relayHysteresis = 0.5;  // m/s hysteresis band

// Oscillation detection
float peakHigh = -999;
float peakLow = 999;
unsigned long lastCrossingTime = 0;
unsigned long oscillationPeriod = 0;
int cycleCount = 0;
const int REQUIRED_CYCLES = 3;  // Need 3 complete cycles
bool firstCrossing = true;

// Accumulated measurements
float sumPeriods = 0;
float sumAmplitudes = 0;
int validCycles = 0;

// Debug
unsigned long debugCounter = 0;

// sTune autotuning (optional alternative to relay tuning)
STunePIDTuner *sTuner = nullptr;
bool sTuneActive = false;

// Forward declarations
void OnTachPulse();
void initializeBarometer();
void checkForUserAirspeedUpdate();
void calibratePressureSensor(bool skipWait = false, int numSamples = -1);
float readPressureSensorRawUncalibrated();
float readPressureSensorRaw();
float readPressureSensorOversampled();
float readPressureSensorAveraged();
float readBarometerSensor();
float calcCurrentAirSpeed(float pressure, float AirDensity);
unsigned int RPMcalc();
void startRelayTune(int pwmLow = -1, int pwmHigh = -1);
void runRelayTune();
void setPressureAverageSamples(int samples);
void startSTune();
void runSTune();

QuickPID myPID(&currentAirSpeed, &currPWM, &desiredAirSpeed);

bfs::Ms4525do pres;
Adafruit_BMP3XX bmp;

volatile unsigned long LastPulseTime = 0;
volatile unsigned long PulseInterval = 0;
unsigned long PrevPulseInterval = 0;
bool changed = false;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    while(Serial.available()) Serial.read();
    
    Serial.println("\n\n");
    Serial.println("=====================================");
    Serial.println("   EDU Wind Tunnel PID Controller v1.1");
    Serial.println("   Build: 2025-11-09 00:08:20 UTC");
    Serial.println("   User: Low-Boom");
    Serial.println("=====================================");
    Serial.flush();
    delay(100);

    // Initialize pressure buffer
    for (int i = 0; i < MAX_AVG_SAMPLES; i++) {
        pressureBuffer[i] = 0.0;
    }

    // Initialize I2C
    Serial.print("[INIT] I2C bus...");
    Wire.begin();
    Wire.setClock(400000);
    Serial.println(" OK (400kHz)");
    delay(100);

    // Scan all I2C buses for connected devices
    scanAllI2CBuses(Serial);

    // Configure pressure sensor
    Serial.print("[INIT] MS4525DO pressure sensor...");
    pres.Config(&Wire, DPS_ADDRESS, 1.0f, -1.0f);
    
    if (!pres.Begin()) {
        Serial.println(" FAILED!");
        Serial.println("[ERROR] Cannot communicate with pressure sensor");
        Serial.println("[ERROR] Check I2C address 0x28");
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    Serial.println(" OK");
    Serial.print("        Oversampling: "); Serial.print(PRESSURE_OVERSAMPLES); 
    Serial.println("x per reading");
    Serial.print("        Averaging: "); Serial.print(pressureAverageSamples); 
    Serial.println(" samples");
    delay(100);

    // Initialize barometer
    initializeBarometer();

    // Initialize hardware
    Serial.println("[INIT] Motor & Tachometer...");
    Serial.print("        PWM Pin: "); Serial.println(PWM_pin);
    Serial.print("        TACH Pin: "); Serial.println(TACH_pin);
    
    pinMode(PWM_pin, OUTPUT);
    analogWrite(PWM_pin, 0);  // Start at 0
    
    pinMode(TACH_pin, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(TACH_pin), OnTachPulse, FALLING);
    
    Serial.println("        OK");
    delay(100);

    // Calibrate pressure sensor with 30 second wait (startup only)
    calibratePressureSensor(false);

    // Configure QuickPID for full PID control
    Serial.print("[INIT] QuickPID Controller (Full PID)...");
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetOutputLimits(0, 255);
    myPID.SetSampleTimeUs(updateInterval * 1000);
    myPID.SetMode(QuickPID::Control::automatic);
    
    // OPTIMIZED SETTINGS FOR LAGGY SYSTEMS - Full PID
    myPID.SetProportionalMode(QuickPID::pMode::pOnMeas);      // P on Measurement
    myPID.SetDerivativeMode(QuickPID::dMode::dOnMeas);        // D on Measurement
    myPID.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition); // Conditional integration
    
    Serial.println(" OK");
    Serial.print("        Update Rate: "); Serial.print(updateInterval); Serial.println(" ms");
    Serial.print("        Kp="); Serial.print(Kp, 1);
    Serial.print(", Ki="); Serial.print(Ki, 1);
    Serial.print(", Kd="); Serial.println(Kd, 1);
    Serial.println("        P-on-Measurement: ENABLED");
    Serial.println("        D-on-Measurement: ENABLED");
    Serial.println("        Conditional Anti-Windup: ENABLED");
    Serial.print("        PWM Rate Limit: ±"); 
    Serial.print(MAX_PWM_CHANGE_PER_CYCLE); 
    Serial.println(" per cycle");
    delay(100);

    // Initialize sTune wrapper (optional advanced autotuning)
    Serial.print("[INIT] sTune wrapper...");
    sTuner = new STunePIDTuner(&currentAirSpeed, &currPWM, STUNE_METHOD);
    sTuner->setEmergencyStop(MAX_AIRSPEED);
    Serial.println(" OK");
    Serial.print("        Tuning Method: ");
    Serial.println(sTuner->getTuningMethodName());
    Serial.println("        Advanced autotuning available via 'stune' command");
    delay(100);

    startTime = millis();
    
    Serial.println("=====================================");
    Serial.println("[READY] System initialized!");
    Serial.println("");
    Serial.println("HARDWARE:");
    Serial.print("  PWM Output: Pin "); Serial.println(PWM_pin);
    Serial.print("  Tachometer: Pin "); Serial.println(TACH_pin);
    Serial.println("");
    Serial.println("CONTROL:");
    Serial.println("  Type airspeed value (m/s) to set target");
    Serial.println("  Example: '10' sets target to 10 m/s");
    Serial.println("");
    Serial.println("COMMANDS:");
    Serial.println("  tune                    - Auto PWM range");
    Serial.println("  tune <low> <high>       - Manual PWM range");
    Serial.println("    Example: 'tune 40 100'");
    Serial.println("  tune <Kp> <Ki> <Kd>     - Manual PID gains");
    Serial.println("    Example: 'tune 20 8 12'");
    Serial.println("  recal [N]               - Recalibrate sensor");
    Serial.println("  avg N                   - Set averaging samples");
    Serial.println("  stune                   - sTune autotuning (advanced)");
    Serial.println("");
    Serial.println("AUTO-TUNE:");
    Serial.println("  Method: Relay + Aggressive Tuning Rules");
    Serial.println("  Tuning: 'Some Overshoot' for fast response");
    Serial.println("  1. Set target airspeed (min 2 m/s)");
    Serial.println("  2. Type 'tune' or 'tune <low> <high>'");
    Serial.println("  3. System oscillates 3-4 cycles");
    Serial.println("  4. Aggressive PID gains calculated");
    Serial.println("  Duration: ~30-60 seconds");
    Serial.println("  Tip: Start conservative (e.g., tune 40 100)");
    Serial.println("");
    Serial.println("STUNE AUTO-TUNE (ADVANCED):");
    Serial.println("  Method: Inflection Point (Set Mode at Compile)");
    Serial.println("  Tuning: Stable response, minimal overshoot");
    Serial.println("  1. Set target airspeed (min 2 m/s)");
    Serial.println("  2. Type 'stune'");
    Serial.println("  3. System applies step and monitors response");
    Serial.println("  4. Conservative PID gains calculated");
    Serial.println("  Duration: ~30-60 seconds");
    Serial.println("  Benefits: More stable than relay tuning");
    Serial.println("");
    Serial.println("MANUAL TUNING:");
    Serial.println("  tune <Kp> <Ki> <Kd> - Directly set gains");
    Serial.println("  Example: tune 20 8 12");
    Serial.println("  Ranges: Kp(1-100), Ki(0.1-50), Kd(0.1-50)");
    Serial.println("");
    Serial.println("DATA FORMAT:");
    Serial.println("  T(s) | V | Target | PWM | P(Pa) | Temp(°C) | Err");
    Serial.println("=====================================\n");
    
    lastUpdate = millis();
}

void loop() {
    unsigned long now = millis();
    
    if (now - lastUpdate >= updateInterval) {
        lastUpdate = now;
        debugCounter++;
        
        // Read sensors with advanced processing
        float currPressure = readPressureSensorAveraged();   // Oversampled + Averaged
        float currAirDensity = readBarometerSensor();        // Temperature compensated
        currentAirSpeed = calcCurrentAirSpeed(currPressure, currAirDensity);

        if (sTuneActive) {
            // sTune autotuning mode
            runSTune();
        } else if (!autoTuning) {
            checkForUserAirspeedUpdate();
            
            // Normal PID control
            myPID.Compute();
            
            // Apply PWM with rate limiting
            int pwmOut = constrain((int)round(currPWM), 0, 255);
            float pwmChange = pwmOut - lastPWMOutput;
            if (abs(pwmChange) > MAX_PWM_CHANGE_PER_CYCLE) {
                pwmChange = (pwmChange > 0) ? MAX_PWM_CHANGE_PER_CYCLE : -MAX_PWM_CHANGE_PER_CYCLE;
                pwmOut = lastPWMOutput + pwmChange;
            }
            lastPWMOutput = pwmOut;
            analogWrite(PWM_pin, pwmOut);
            
        } else {
            // Relay auto-tuning mode - DIRECT CONTROL
            runRelayTune();
        }

        unsigned long elapsed = (now - startTime) / 1000;
        float error = desiredAirSpeed - currentAirSpeed;

        // Print data
        if (sTuneActive) {
            Serial.print("[sTune] ");
            Serial.print(sTuner->getStatusString()); Serial.print(" | ");
        } else if (autoTuning) {
            Serial.print("[TUNE] ");
            uint32_t tuneElapsed = (millis() - tuneStartTime) / 1000;
            Serial.print(tuneElapsed); Serial.print("s | ");
            Serial.print("Cycle:"); Serial.print(cycleCount); Serial.print(" | ");
        } else {
            Serial.print(elapsed); Serial.print("s | ");
        }
        Serial.print("V:"); Serial.print(currentAirSpeed, 2); Serial.print(" | ");
        Serial.print("T:"); Serial.print(desiredAirSpeed, 2); Serial.print(" | ");
        Serial.print("PWM:"); Serial.print((int)round(currPWM)); Serial.print(" | ");
        Serial.print("P:"); Serial.print(currPressure, 2); Serial.print(" | ");
        Serial.print("T:"); Serial.print(lastTemperature, 1); Serial.print("°C | ");
        Serial.print("Err:"); Serial.print(error, 2);
        
        // Show PID values and averaging every 20 readings
        if (debugCounter % 20 == 0 && !autoTuning && !sTuneActive) {
            Serial.print(" | Kp:"); Serial.print(Kp, 1);
            Serial.print(" Ki:"); Serial.print(Ki, 1);
            Serial.print(" Kd:"); Serial.print(Kd, 1);
            Serial.print(" Avg:"); Serial.print(pressureAverageSamples);
        }
        
        Serial.println();
    }
}

void startRelayTune(int pwmLow, int pwmHigh) {
    if (desiredAirSpeed < MIN_TUNE_SETPOINT) {
        Serial.println("\n[TUNE] ERROR: Target too low for auto-tune");
        Serial.print("[TUNE] Minimum target: ");
        Serial.print(MIN_TUNE_SETPOINT, 2);
        Serial.println(" m/s");
        Serial.println("[TUNE] Set a higher target speed first (e.g., 10)");
        Serial.println("[TUNE] Then type 'tune' to start\n");
        return;
    }
    
    // SAVE CURRENT AVERAGING BEFORE CHANGING IT
    savedAverageSamples = pressureAverageSamples;
    
    // REDUCE AVERAGING FOR FASTER TUNING RESPONSE
    // setPressureAverageSamples(3);  // Keep some averaging for noise reduction
    
    Serial.println("\n=====================================");
    Serial.println("[RELAY TUNE] AUTO-TUNE STARTING");
    Serial.println("=====================================");
    Serial.print("[RELAY] Target airspeed: ");
    Serial.print(desiredAirSpeed, 2);
    Serial.println(" m/s");
    Serial.println("[RELAY] Method: Åström-Hägglund Relay");
    Serial.println("[RELAY] Tuning Rules: AGGRESSIVE (Some Overshoot)");
    // Serial.print("[RELAY] Averaging temporarily set to 3 (was ");
    // Serial.print(savedAverageSamples);
    Serial.println(")");
    
    // Calculate relay outputs
    if (pwmLow > 0 && pwmHigh > 0) {
        // Manual PWM range provided
        relayOutputLow = constrain(pwmLow, 0, 255);
        relayOutputHigh = constrain(pwmHigh, 0, 255);
        
        if (relayOutputLow >= relayOutputHigh) {
            Serial.println("[ERROR] PWM Low must be less than PWM High!");
            Serial.println("        Example: tune 40 100");
            setPressureAverageSamples(savedAverageSamples);
            return;
        }
        
        Serial.println("[RELAY] PWM Range: MANUAL");
    } else {
        float estimatedPWM = desiredAirSpeed * 8; // Ball park sensitivity
        relayOutputHigh = constrain(round(estimatedPWM) + 20, 40, 245);
        relayOutputLow = constrain(round(estimatedPWM) - 20, 20, 150);
        
        Serial.println("[RELAY] PWM Range: AUTO-ESTIMATED");
        Serial.println("        Tip: If tuning fails, try manual range");
        Serial.println("        Example: tune 40 100");
    }
    
    Serial.println("[RELAY] Configuration:");
    Serial.print("        PWM High: "); Serial.println((int)relayOutputHigh);
    Serial.print("        PWM Low: "); Serial.println((int)relayOutputLow);
    Serial.print("        PWM Delta: "); Serial.println((int)(relayOutputHigh - relayOutputLow));
    Serial.print("        Hysteresis: ±"); Serial.print(relayHysteresis); Serial.println(" m/s");
    Serial.print("        Required cycles: "); Serial.println(REQUIRED_CYCLES);
    Serial.println("[RELAY] Process:");
    Serial.println("        1. Oscillate PWM between high/low");
    Serial.println("        2. Measure oscillation period & amplitude");
    Serial.println("        3. Calculate AGGRESSIVE PID gains");
    Serial.println("        Expected: Faster response, some overshoot");
    Serial.println("        Duration: ~30-60 seconds");
    Serial.println("=====================================");
    
    // Initialize relay tuning state
    tuneSetpoint = desiredAirSpeed;
    tuneStartTime = millis();
    relayState = RELAY_HIGH;
    
    peakHigh = -999;
    peakLow = 999;
    cycleCount = 0;
    firstCrossing = true;
    sumPeriods = 0;
    sumAmplitudes = 0;
    validCycles = 0;
    
    // Set PWM to LOW and start
    relayState = RELAY_LOW;
    currPWM = relayOutputLow;
    analogWrite(PWM_pin, (int)relayOutputLow);
    delay(500);
    // Verify it was written
    Serial.print("[RELAY] currPWM variable = ");
    Serial.println(currPWM, 1);
    Serial.println("[RELAY] Tuning active");
    
    autoTuning = true;
}

void runRelayTune() {
    // Update peak tracking
    if (currentAirSpeed > peakHigh) peakHigh = currentAirSpeed;
    if (currentAirSpeed < peakLow) peakLow = currentAirSpeed;
    
    // DIRECTLY WRITE PWM every cycle to ensure it's applied
    int pwmToWrite = (int)round(currPWM);
    analogWrite(PWM_pin, pwmToWrite);
    
    // Relay logic with hysteresis
    if (relayState == RELAY_HIGH) {
        // Switch to LOW if airspeed goes above target + hysteresis
        if (currentAirSpeed > tuneSetpoint + relayHysteresis) {
            relayState = RELAY_LOW;
            currPWM = relayOutputLow;
            analogWrite(PWM_pin, (int)relayOutputLow);
            
            Serial.print("[RELAY] Switched to LOW (PWM ");
            Serial.print((int)relayOutputLow);
            Serial.print(") at V=");
            Serial.print(currentAirSpeed, 2);
            Serial.println(" m/s");
            
            // Record crossing
            if (!firstCrossing) {
                unsigned long now = millis();
                oscillationPeriod = now - lastCrossingTime;
                lastCrossingTime = now;
                
                float amplitude = (peakHigh - peakLow) / 2.0;
                
                // Valid measurement if amplitude is reasonable
                if (amplitude > 0.5 && oscillationPeriod > 2000) {
                    sumPeriods += oscillationPeriod;
                    sumAmplitudes += amplitude;
                    validCycles++;
                    cycleCount++;
                    
                    Serial.print("[RELAY] Cycle "); Serial.print(cycleCount);
                    Serial.print(" | Period: "); Serial.print(oscillationPeriod / 1000.0, 1); Serial.print("s");
                    Serial.print(" | Amplitude: "); Serial.print(amplitude, 2); Serial.print(" m/s");
                    Serial.print(" | Peak High: "); Serial.print(peakHigh, 2);
                    Serial.print(" | Peak Low: "); Serial.println(peakLow, 2);
                    
                    // Reset peaks for next cycle
                    peakHigh = currentAirSpeed;
                    peakLow = currentAirSpeed;
                }
            } else {
                firstCrossing = false;
                lastCrossingTime = millis();
            }
        }
    } else {
        // Switch to HIGH if airspeed goes below target - hysteresis
        if (currentAirSpeed < tuneSetpoint - relayHysteresis) {
            relayState = RELAY_HIGH;
            currPWM = relayOutputHigh;
            analogWrite(PWM_pin, (int)relayOutputHigh);
            
            Serial.print("[RELAY] Switched to HIGH (PWM ");
            Serial.print((int)relayOutputHigh);
            Serial.print(") at V=");
            Serial.print(currentAirSpeed, 2);
            Serial.println(" m/s");
        }
    }
    
    // Check if we have enough cycles
    if (validCycles >= REQUIRED_CYCLES) {
        // Calculate average period and amplitude
        float avgPeriod = sumPeriods / validCycles / 1000.0;  // seconds
        float avgAmplitude = sumAmplitudes / validCycles;      // m/s
        
        // Calculate ultimate gain (Ku) and period (Tu)
        float outputStep = relayOutputHigh - relayOutputLow;
        float Ku = (4.0 * outputStep) / (PI * avgAmplitude * 255.0);  // Normalize by 255 for 0-1 range
        float Tu = avgPeriod;
        
        // AGGRESSIVE TUNING RULES - "Some Overshoot" method
        // Based on Tyreus-Luyben modified for faster response
        // More aggressive than conservative ZN rules
        // Kp = 0.45 * Ku / 0.9;  // ~0.50 * Ku (more aggressive than 0.6)
        // Ki = 0.54 * Ku / (2.2 * Tu);  // Faster integral action
        // Kd = 0.15 * Ku * Tu;  // Stronger derivative (2x standard)
        
        // Alternative calculation for comparison
        Kp = 0.7 * Ku;  // Even more aggressive
        Ki = 1.75 * Ku / Tu;
        Kd = 0.15 * Ku * Tu;
        
        Serial.println("\n[RELAY] Tuning calculations:");
        Serial.print("  Standard (Some Overshoot): Kp="); Serial.print(Kp, 2);
        Serial.print(", Ki="); Serial.print(Ki, 2);
        Serial.print(", Kd="); Serial.println(Kd, 2);
        // Serial.print("  Extra Aggressive option: Kp="); Serial.print(Kp_aggressive, 2);
        // Serial.print(", Ki="); Serial.print(Ki_aggressive, 2);
        // Serial.print(", Kd="); Serial.println(Kd_aggressive, 2);
        // Serial.println("  Using: Some Overshoot (balanced aggressive)");
        
        // Tuning Bounds
        Kp = constrain(Kp, 10.0, 100.0);
        Ki = constrain(Ki, 1.0, 10.0);
        Kd = constrain(Kd, 1.0, 10.0);
        
        myPID.SetTunings(Kp, Ki, Kd);
        
        // RESTORE PREVIOUS AVERAGING SETTING
        setPressureAverageSamples(savedAverageSamples);
        
        // Stop motor
        currPWM = 0;
        analogWrite(PWM_pin, 0);
        
        Serial.println("\n=====================================");
        Serial.println("[RELAY] AUTO-TUNE COMPLETE!");
        Serial.println("=====================================");
        Serial.println("        Measured Parameters:");
        Serial.print("          Period (Tu): "); Serial.print(Tu, 2); Serial.println(" s");
        Serial.print("          Amplitude: "); Serial.print(avgAmplitude, 2); Serial.println(" m/s");
        Serial.print("          Ultimate Gain (Ku): "); Serial.println(Ku, 4);
        Serial.print("          PWM Range: "); Serial.print((int)relayOutputLow);
        Serial.print(" - "); Serial.println((int)relayOutputHigh);
        Serial.println("        Calculated PID Gains (AGGRESSIVE):");
        Serial.print("          Kp = "); Serial.println(Kp, 4);
        Serial.print("          Ki = "); Serial.println(Ki, 4);
        Serial.print("          Kd = "); Serial.println(Kd, 4);
        Serial.println("        Tuning Method: Some Overshoot");
        Serial.println("        Expected: Fast response, minor overshoot");
        Serial.print("        Averaging restored to ");
        Serial.print(savedAverageSamples);
        Serial.println(" samples");
        Serial.println("        PID updated and ready");
        Serial.println("        ");
        Serial.println("        If too aggressive, reduce Kp by 20-30%");
        Serial.println("        If too slow, increase all gains by 20%");
        Serial.print("        Or manually set: tune ");
        Serial.print(Kp, 1); Serial.print(" ");
        Serial.print(Ki, 1); Serial.print(" ");
        Serial.println(Kd, 1);
        Serial.println("=====================================\n");
        
        autoTuning = false;
        desiredAirSpeed = tuneSetpoint;
        lastPWMOutput = 0;
        myPID.Reset();
    }
    
    // Timeout after 120 seconds
    if ((millis() - tuneStartTime) > 120000) {
        Serial.println("\n[RELAY] AUTO-TUNE TIMEOUT!");
        Serial.println("        Not enough oscillations detected");
        Serial.print("        Valid cycles: "); Serial.print(validCycles);
        Serial.print(" / "); Serial.println(REQUIRED_CYCLES);
        Serial.println("        Troubleshooting:");
        Serial.println("        - Check motor/ESC connection and power");
        Serial.println("        - Verify PWM pin wiring");
        Serial.println("        - Try different PWM range");
        Serial.println("        - Ensure target airspeed is achievable");
        Serial.print("        - Current range: "); Serial.print((int)relayOutputLow);
        Serial.print(" - "); Serial.println((int)relayOutputHigh);
        Serial.print("        - Peak airspeed seen: "); Serial.print(peakHigh, 2);
        Serial.println(" m/s");
        
        setPressureAverageSamples(savedAverageSamples);
        autoTuning = false;
        currPWM = 0;
        analogWrite(PWM_pin, 0);
    }
}

void setPressureAverageSamples(int samples) {
    samples = constrain(samples, MIN_AVG_SAMPLES, MAX_AVG_SAMPLES);
    
    if (samples != pressureAverageSamples) {
        pressureAverageSamples = samples;
        
        // Reset averaging buffer
        pressureBufferIndex = 0;
        pressureBufferFilled = false;
        for (int i = 0; i < MAX_AVG_SAMPLES; i++) {
            pressureBuffer[i] = 0.0;
        }
        
        Serial.print("[CONFIG] Pressure averaging set to ");
        Serial.print(pressureAverageSamples);
        Serial.println(" samples");
        
        if (pressureAverageSamples == 1) {
            Serial.println("         (No averaging - raw oversampled readings)");
        } else {
            Serial.print("         Effective lag: ~");
            Serial.print(pressureAverageSamples * updateInterval);
            Serial.println(" ms");
        }
    }
}

void calibratePressureSensor(bool skipWait, int numSamples) {
    // Use provided numSamples, or default if not specified
    int samples = (numSamples > 0) ? numSamples : calibrationSamples;
    samples = constrain(samples, MIN_CALIB_SAMPLES, MAX_CALIB_SAMPLES);
    
    Serial.println("\n[CALIB] Pressure Sensor Calibration");
    Serial.println("=====================================");
    Serial.println("        **CRITICAL - READ CAREFULLY**");
    Serial.println("        1. Ensure NO airflow in tunnel");
    Serial.println("        2. Both pitot ports open to atmosphere");
    Serial.println("        3. Fan must be completely OFF");
    
    if (!skipWait) {
        Serial.println("        4. Let system settle for thermal stability");
        Serial.println("");
        Serial.println("        Waiting 15 seconds...");
        Serial.println("=====================================");
        
        for (int i = 15; i > 0; i--) {
            Serial.print("        ");
            if (i < 10) Serial.print(" ");
            Serial.print(i);
            Serial.println(" seconds remaining...");
            delay(1000);
        }
    } else {
        Serial.println("        4. Waiting 3 seconds...");
        Serial.println("=====================================");
        delay(3000);
    }
    
    Serial.println("\n        Starting calibration measurements...");
    Serial.print("        Using "); Serial.print(samples); 
    Serial.println(" uncalibrated oversampled readings");
    
    float sum = 0;
    int validSamples = 0;
    float minVal = 999999;
    float maxVal = -999999;
    
    Serial.print("        Taking "); Serial.print(samples); Serial.print(" samples: ");
    
    // Take calibration samples WITHOUT applying offset (use raw uncalibrated function)
    for (int i = 0; i < samples; i++) {
        float sum_oversample = 0.0;
        int validReads = 0;
        
        // Oversample the raw sensor
        for (int j = 0; j < PRESSURE_OVERSAMPLES; j++) {
            float reading = readPressureSensorRawUncalibrated();
            if (reading >= 0.0) {
                sum_oversample += reading;
                validReads++;
            }
            delayMicroseconds(500);
        }
        
        if (validReads > 0) {
            float avgReading = sum_oversample / validReads;
            sum += avgReading;
            validSamples++;
            minVal = min(minVal, avgReading);
            maxVal = max(maxVal, avgReading);
            Serial.print(".");
        } else {
            Serial.print("X");
        }
        delay(100);
    }
    Serial.println();
    
    if (validSamples > samples / 2) {
        pressureOffset = sum / validSamples;
        calibrated = true;
        
        Serial.println("\n        Calibration Results:");
        Serial.print("        Valid samples: "); Serial.print(validSamples); 
        Serial.print(" / "); Serial.println(samples);
        Serial.print("        Min reading: "); Serial.print(minVal, 4); Serial.println(" Pa");
        Serial.print("        Max reading: "); Serial.print(maxVal, 4); Serial.println(" Pa");
        Serial.print("        Range: "); Serial.print(maxVal - minVal, 4); Serial.println(" Pa");
        Serial.print("        Average (offset): "); Serial.print(pressureOffset, 4); Serial.println(" Pa");
        Serial.print("        Std Dev est: "); 
        Serial.print((maxVal - minVal) / sqrt(validSamples), 4); Serial.println(" Pa");
        Serial.println("        Status: SUCCESS ✓");
        Serial.println("=====================================\n");
        
        // Reset averaging buffer after calibration
        pressureBufferIndex = 0;
        pressureBufferFilled = false;
        for (int i = 0; i < MAX_AVG_SAMPLES; i++) {
            pressureBuffer[i] = 0.0;
        }
        
    } else {
        Serial.println("\n        [ERROR] Calibration FAILED!");
        Serial.print("        Valid samples: "); Serial.print(validSamples); 
        Serial.print(" / "); Serial.println(samples);
        Serial.println("        Using zero offset (uncalibrated mode)");
        Serial.println("        WARNING: Readings may be inaccurate!");
        Serial.println("=====================================\n");
        pressureOffset = 0.0;
        calibrated = false;
    }
    delay(500);
}

float readBarometerSensor() { 
    if (!bmp.performReading()) {
        return lastAirDensity;
    }
    
    float temp_c = bmp.temperature;
    float temp_k = temp_c + 273.15;
    float pressure_pa = bmp.pressure;
    float density = pressure_pa / (Specific_gas_constant * temp_k);
    
    lastAirDensity = density;
    lastTemperature = temp_c;
    
    return density;
}

float readPressureSensorRawUncalibrated() {
    float pressure = 0.0;
    
    if (pres.Read()) {
        pressure = pres.pres_pa();
        if (pressure < 0.0) pressure = 0.0;
    }
    
    return pressure;
}

float readPressureSensorRaw() { 
    float pressure = 0.0;
    
    if (pres.Read()) {
        pressure = pres.pres_pa();
        
        if (calibrated) {
            pressure -= pressureOffset;
        }
        
        if (pressure < 0.0) pressure = 0.0;
    }
    
    return pressure;
}

float readPressureSensorOversampled() {
    float sum = 0.0;
    int validReads = 0;
    
    for (int i = 0; i < PRESSURE_OVERSAMPLES; i++) {
        float reading = readPressureSensorRaw();
        if (reading >= 0.0) {
            sum += reading;
            validReads++;
        }
        delayMicroseconds(500);
    }
    
    if (validReads > 0) {
        return sum / validReads;
    }
    
    return 0.0;
}

float readPressureSensorAveraged() {
    float singleReading = readPressureSensorOversampled();
    
    pressureBuffer[pressureBufferIndex] = singleReading;
    pressureBufferIndex++;
    
    if (pressureBufferIndex >= pressureAverageSamples) {
        pressureBufferIndex = 0;
        pressureBufferFilled = true;
    }
    
    float sum = 0;
    int samplesToAverage = pressureBufferFilled ? pressureAverageSamples : pressureBufferIndex;
    
    for (int i = 0; i < samplesToAverage; i++) {
        sum += pressureBuffer[i];
    }
    
    return (samplesToAverage > 0) ? (sum / samplesToAverage) : 0.0;
}

float calcCurrentAirSpeed(float pressure, float AirDensity) { 
    if (pressure < 0.0) pressure = 0.0;
    if (AirDensity < 0.1) AirDensity = 1.225;
    
    float airSpeed = sqrt((2.0 * pressure) / AirDensity);
    
    return airSpeed;
}

void checkForUserAirspeedUpdate() {
    if (Serial.available() > 0) {
        String inputVal = Serial.readStringUntil('\n');
        inputVal.trim();
        
        if (inputVal.equalsIgnoreCase("stune")) {
            startSTune();
            return;
        }
        
        if (inputVal.equalsIgnoreCase("tune")) {
            startRelayTune();  // Auto PWM
            return;
        }
        
        if (inputVal.startsWith("tune ") || inputVal.startsWith("TUNE ")) {
            // Parse "tune" command with parameters
            // Could be: tune <low> <high> (PWM range) OR tune <Kp> <Ki> <Kd> (manual PID)
            
            int firstSpace = inputVal.indexOf(' ');
            int secondSpace = inputVal.indexOf(' ', firstSpace + 1);
            int thirdSpace = inputVal.indexOf(' ', secondSpace + 1);
            
            if (thirdSpace > 0) {
                // THREE parameters: tune <Kp> <Ki> <Kd>
                String kpStr = inputVal.substring(firstSpace + 1, secondSpace);
                String kiStr = inputVal.substring(secondSpace + 1, thirdSpace);
                String kdStr = inputVal.substring(thirdSpace + 1);
                
                float newKp = kpStr.toFloat();
                float newKi = kiStr.toFloat();
                float newKd = kdStr.toFloat();
                
                if (newKp > 0 && newKi >= 0 && newKd >= 0) {
                    // Apply constraints
                    newKp = constrain(newKp, 1.0, 100.0);
                    newKi = constrain(newKi, 0.1, 50.0);
                    newKd = constrain(newKd, 0.1, 50.0);
                    
                    Kp = newKp;
                    Ki = newKi;
                    Kd = newKd;
                    
                    myPID.SetTunings(Kp, Ki, Kd);
                    myPID.Reset();
                    
                    Serial.println("\n[PID] Manual PID gains set:");
                    Serial.print("      Kp = "); Serial.println(Kp, 4);
                    Serial.print("      Ki = "); Serial.println(Ki, 4);
                    Serial.print("      Kd = "); Serial.println(Kd, 4);
                    Serial.println("      PID controller reset and ready\n");
                } else {
                    Serial.println("[ERROR] Invalid PID values");
                    Serial.println("        Usage: tune <Kp> <Ki> <Kd>");
                    Serial.println("        Example: tune 20 8 12");
                    Serial.println("        Ranges: Kp(1-100), Ki(0.1-50), Kd(0.1-50)");
                }
                
            } else if (secondSpace > 0) {
                // TWO parameters: tune <low> <high> (PWM range)
                String lowStr = inputVal.substring(firstSpace + 1, secondSpace);
                String highStr = inputVal.substring(secondSpace + 1);
                
                int pwmLow = lowStr.toInt();
                int pwmHigh = highStr.toInt();
                
                if (pwmLow > 0 && pwmHigh > 0) {
                    startRelayTune(pwmLow, pwmHigh);
                } else {
                    Serial.println("[ERROR] Invalid PWM values");
                    Serial.println("        Usage: tune <low> <high>");
                    Serial.println("        Example: tune 40 100");
                }
            } else {
                Serial.println("[ERROR] Missing parameters");
                Serial.println("        For relay auto-tune: tune <low> <high>");
                Serial.println("          Example: tune 40 100");
                Serial.println("        For manual PID: tune <Kp> <Ki> <Kd>");
                Serial.println("          Example: tune 20 8 12");
            }
            return;
        }
        
        if (inputVal.startsWith("recal") || inputVal.startsWith("RECAL")) {
            Serial.println("\n[CMD] Recalibrating pressure sensor...");
            Serial.println("        Stop the fan and ensure no airflow!");
            
            int spaceIndex = inputVal.indexOf(' ');
            if (spaceIndex > 0) {
                int numSamples = inputVal.substring(spaceIndex + 1).toInt();
                if (numSamples >= MIN_CALIB_SAMPLES && numSamples <= MAX_CALIB_SAMPLES) {
                    calibratePressureSensor(true, numSamples);
                } else {
                    Serial.print("[ERROR] Calibration samples must be between ");
                    Serial.print(MIN_CALIB_SAMPLES); Serial.print(" and ");
                    Serial.println(MAX_CALIB_SAMPLES);
                }
            } else {
                calibratePressureSensor(true);
            }
            return;
        }
        
        if (inputVal.startsWith("avg ") || inputVal.startsWith("AVG ")) {
            int newSamples = inputVal.substring(4).toInt();
            if (newSamples >= MIN_AVG_SAMPLES && newSamples <= MAX_AVG_SAMPLES) {
                setPressureAverageSamples(newSamples);
            } else {
                Serial.print("[ERROR] Averaging samples must be between ");
                Serial.print(MIN_AVG_SAMPLES); Serial.print(" and ");
                Serial.println(MAX_AVG_SAMPLES);
            }
            return;
        }
        
        float inputAirSpeed = inputVal.toFloat();
        
        if (inputAirSpeed < 0) {
            Serial.println("[ERROR] Negative airspeed not allowed");
            return;
        }
        
        if (inputAirSpeed > MAX_AIRSPEED) {
            Serial.print("[WARN] Capping at "); Serial.print(MAX_AIRSPEED); Serial.println(" m/s");
            inputAirSpeed = MAX_AIRSPEED;
        }
        
        desiredAirSpeed = inputAirSpeed;
        myPID.Reset();
        
        Serial.print("[SERIAL] New target: "); Serial.print(desiredAirSpeed, 2); Serial.println(" m/s");
    }
}

unsigned int RPMcalc() { 
    noInterrupts();
    unsigned long interval = PulseInterval;
    interrupts();

    if (interval > 0) {
        unsigned int rpm = (60UL * 1000000UL) / (interval * 2);
        LastValidRPM = rpm;
        LastRPMUpdateTime = millis();
        return rpm;
    }

    if ((millis() - LastRPMUpdateTime) > RPM_TIMEOUT_MS) {
        return 0;
    }
    
    return LastValidRPM;
}

void OnTachPulse() { 
    unsigned long now = micros();
    PulseInterval = now - LastPulseTime;
    LastPulseTime = now;
}

void initializeBarometer() { 
    Serial.print("[INIT] BMP3XX barometer (I2C)...");
    
    if (!bmp.begin_I2C(BMP_I2C_ADDRESS)) {
        Serial.println(" trying 0x76...");
        if (!bmp.begin_I2C(0x76)) {
            Serial.println(" FAILED!");
            Serial.println("[ERROR] BMP3 not found at 0x76 or 0x77");
            while (1) {
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                delay(500);
            }
        }
        Serial.println(" OK (0x76)");
    } else {
        Serial.println(" OK (0x77)");
    }
    
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    
    if (bmp.performReading()) {
        Serial.print("        T:"); Serial.print(bmp.temperature, 1); Serial.print("°C ");
        Serial.print("P:"); Serial.print(bmp.pressure/100.0, 1); Serial.println("hPa");
        Serial.println("        Temperature compensation: ENABLED");
    }
    delay(100);
}

// sTune autotuning functions
void startSTune() {
    if (desiredAirSpeed < MIN_TUNE_SETPOINT) {
        Serial.println("\n[sTune] ERROR: Target too low for auto-tune");
        Serial.print("[sTune] Minimum target: ");
        Serial.print(MIN_TUNE_SETPOINT, 2);
        Serial.println(" m/s");
        Serial.println("[sTune] Set a higher target speed first (e.g., 10)");
        Serial.println("[sTune] Then type 'stune' to start\n");
        return;
    }
    
    // Save current averaging for potential restoration
    savedAverageSamples = pressureAverageSamples;
    
    // Start sTune with appropriate parameters
    // Parameters: setpoint, inputSpan, outputSpan, outputStart, outputStep, testTime, settleTime, samples
    float outputStep = desiredAirSpeed * 8.0; // Estimate based on typical response
    outputStep = constrain(outputStep, 60.0, 200.0);
    
    if (sTuner->startTuning(desiredAirSpeed, MAX_AIRSPEED, 255.0, 0.0, outputStep, 60, 5, 300)) {
        sTuneActive = true;
    }
}

void runSTune() {
    // Update sTune state machine
    STunePIDTuner::TuningStatus status = sTuner->update();
    
    // Write PWM output (sTuner manages the output value via currPWM pointer)
    analogWrite(PWM_pin, (int)round(currPWM));
    
    if (status == STunePIDTuner::COMPLETED) {
        // Get tuned gains and apply to PID
        float newKp, newKi, newKd;
        if (sTuner->getTunedGains(newKp, newKi, newKd)) {
            Kp = newKp;
            Ki = newKi;
            Kd = newKd;
            
            myPID.SetTunings(Kp, Ki, Kd);
            myPID.Reset();
            
            Serial.println("[sTune] Gains applied to PID controller");
            Serial.println("[sTune] Resuming normal operation");
        }
        
        // Restore averaging setting
        setPressureAverageSamples(savedAverageSamples);
        
        sTuneActive = false;
        sTuner->reset();
        lastPWMOutput = 0;
        
    } else if (status == STunePIDTuner::TIMEOUT || status == STunePIDTuner::ERROR) {
        Serial.println("[sTune] Tuning failed - keeping previous gains");
        
        // Restore averaging setting
        setPressureAverageSamples(savedAverageSamples);
        
        sTuneActive = false;
        sTuner->reset();
        currPWM = 0;
        analogWrite(PWM_pin, 0);
        lastPWMOutput = 0;
    }
}