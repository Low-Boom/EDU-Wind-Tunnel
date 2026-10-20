// Basic Arduino includes
#include <Arduino.h>
#include <Wire.h>

// Board-specific pins and settings (ESP32-P4, Arduino Uno R4 Minima / WiFi)
#include "PlatformConfig.h"

// Sensor libraries
#include "DFRobot_BMP58X.h"
#include <DFRobot_LWLP.h>
#include <QuickPID.h>
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

#define SEALEVELPRESSURE_HPA (1013.25)
#define DPS_ADDRESS 0x00   // DFRobot LWLP5000 (SEN0343) — fixed I2C address
#define BMP_I2C_ADDRESS 0x47  // DFRobot BMP585 (SEN0665) — default I2C address (alt: 0x46)

// PID gains - Full PID control
float Kp = 25.0, Ki = 10.0, Kd = 15.0;

// Pins (FAN_PWM_PIN, STATUS_LED_PIN, I2C pins) are set in PlatformConfig.h
const int ENC_PPR = 2;
const float kI = 1;

const int MAX_PRESSURE = 14745.0;
const float MIN_PRESSURE = 1638.0;

// Highest target. The LWLP5000 is rated to 500 Pa, which is 28.6 m/s at
// sea-level air density (1.225 kg/m3); above that the reading is not valid.
const float MAX_AIRSPEED = 28.0;
const float MIN_AIRSPEED = 0.0;
const float Specific_gas_constant = 287.058;

// Pressure sensor calibration (zero offset, measured with the fan off)
float pressureOffset = 0.0;              // Pa, subtracted from every reading
const int DEFAULT_CALIB_SAMPLES = 50;
const int CALIBRATION_SETTLE_S = 15;     // fan run-down + thermal settling before zeroing
const unsigned long CALIB_SAMPLE_SPACING_MS = 100;
const int MIN_CALIB_SAMPLES = 5;
const int MAX_CALIB_SAMPLES = 100;

// Sensor oversampling: readings per update. Each LWLP read blocks for about
// 60 ms (the library waits for the conversion), so more than one read would
// not fit into the 200 ms update interval; the moving average does the
// smoothing instead.
const int PRESSURE_OVERSAMPLES = 1;

// Pressure sensor averaging configuration
int pressureAverageSamples = 20;  // Default: 20 samples
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
unsigned long lastSlowPassWarning = 0;
const unsigned long SLOW_PASS_WARNING_INTERVAL_MS = 10000;
unsigned long startTime = 0;

unsigned int LastValidRPM = 0;
unsigned long LastRPMUpdateTime = 0;
const unsigned long RPM_TIMEOUT_MS = 1000;

// PWM Rate Limiting (disabled during tuning)
int fanPwm = -1;                       // PWM last written to the fan (-1 = never)
const int MAX_PWM_CHANGE_PER_CYCLE = 25;

// PID and control variables
float currPWM = 0;
float desiredAirSpeed = 0;
float currentAirSpeed = 0;

// What loop() does with the fan: closed-loop PID or one of the auto-tuners.
// Only one mode is active at a time, so tuning and PID never fight over PWM.
enum ControlMode { MODE_PID, MODE_RELAY_TUNE, MODE_STUNE };
ControlMode controlMode = MODE_PID;

// Serial command input: one line is collected without blocking loop()
const size_t SERIAL_LINE_SIZE = 64;   // buffer size incl. the terminating '\0'
char serialLine[SERIAL_LINE_SIZE];
size_t serialLineLength = 0;
bool serialLineTooLong = false;
const int MAX_COMMAND_WORDS = 4;      // longest command: tune <Kp> <Ki> <Kd>

// Auto-tuning (relay and sTune)
const float MIN_TUNE_SETPOINT = 2.0;     // m/s; below this the pitot signal is too small

// PID gain limits, shared by manual entry and both auto-tuners
const float KP_MIN = 1.0, KP_MAX = 100.0;
const float KI_MIN = 0.1, KI_MAX = 50.0;
const float KD_MIN = 0.1, KD_MAX = 50.0;

// Relay auto-tune (Astrom-Hagglund): switch the PWM between a low and a high
// level whenever the airspeed leaves a band around the target. The resulting
// steady oscillation gives the ultimate gain Ku and period Tu.
const float RELAY_HYSTERESIS = 0.5;               // m/s band around the target
const int REQUIRED_CYCLES = 3;                    // valid cycles to average
const float MIN_CYCLE_AMPLITUDE = 0.5;            // m/s; smaller swings are noise
const unsigned long MIN_CYCLE_PERIOD_MS = 2000;   // shorter "cycles" are noise
const unsigned long RELAY_TIMEOUT_MS = 120000;
bool relayHigh = false;                // true while the high PWM level is applied
int relayPwmLow = 0;
int relayPwmHigh = 0;
unsigned long tuneStartTime = 0;
unsigned long lastCrossingTime = 0;    // last HIGH->LOW switch
bool firstCrossing = true;
float peakHigh = 0.0;                  // airspeed extremes of the current cycle
float peakLow = 0.0;
int relayCycles = 0;                   // valid cycles measured so far
float sumPeriodsS = 0.0;
float sumAmplitudes = 0.0;

// Debug
unsigned long debugCounter = 0;

// sTune autotuning (optional alternative to relay tuning)
STunePIDTuner *sTuner = nullptr;

// Forward declarations
void haltWithError(const char *msg);
void OnTachPulse();
void initializeBarometer();
void pollSerialCommands();
void handleCommand(char *line);
int splitWords(char *line, char *words[], int maxWords);
bool wordIs(const char *word, const char *keyword);
bool parseDecimal(const char *text, float &value);
bool parseWholeNumber(const char *text, int minValue, int maxValue, int &value);
void setTargetAirspeed(float target);
void setManualGains(const char *kpText, const char *kiText, const char *kdText);
void stopFan();
void pidBumplessStart();
void setFanPwm(int pwm);
void abortTuning(const char *reason);
void finishTuning(float newKp, float newKi, float newKd);
void applyGains(float newKp, float newKi, float newKd);
float clampGain(const char *name, float value, float minValue, float maxValue);
bool tuneTargetOk();
void setRelay(bool high);
void recordRelayCycle();
void completeRelayTune();
void runPidControl();
void printDataLine(unsigned long now, float pressure);
void printHelp();
void warnIfSlowPass(unsigned long passMs);
void calibratePressureSensor(int numSamples);
float readPressureRaw();
float readPressureCorrected();
float readPressureAveraged();
void resetPressureAverage();
float readBarometerSensor();
float calcCurrentAirSpeed(float pressure, float AirDensity);
unsigned int RPMcalc();
void startRelayTune(int pwmLow = -1, int pwmHigh = -1);
void runRelayTune();
void setPressureAverageSamples(int samples);
void startSTune();
void runSTune();

QuickPID myPID(&currentAirSpeed, &currPWM, &desiredAirSpeed);

DFRobot_LWLP lwlp;
DFRobot_BMP58X_I2C *bmp58x = nullptr;

volatile unsigned long LastPulseTime = 0;
volatile unsigned long PulseInterval = 0;
unsigned long PrevPulseInterval = 0;
bool changed = false;

void setup() {
    // Drive the fan off before anything else: until pinMode() runs the PWM pin
    // floats and some fan drivers read a floating input as "full speed".
    pinMode(FAN_PWM_PIN, OUTPUT);
    digitalWrite(FAN_PWM_PIN, LOW);
    if (STATUS_LED_PIN >= 0) {
        pinMode(STATUS_LED_PIN, OUTPUT);
        digitalWrite(STATUS_LED_PIN, LOW);
    }

    Serial.begin(115200);
    delay(2000);
    
    while(Serial.available()) Serial.read();
    
    Serial.println("\n\n");
    Serial.println("=====================================");
    Serial.println("   EDU Wind Tunnel PID Controller v1.1");
    Serial.println("   Sensors: DFRobot SEN0665 (BMP585) + SEN0343 (LWLP5000)");
    Serial.println("   Build: 2025-11-09 00:08:20 UTC");
    Serial.println("   User: Low-Boom");
    Serial.println("=====================================");
    Serial.flush();
    delay(100);

    // Initialize I2C
    Serial.print("[INIT] I2C bus...");
    #if I2C_SDA_PIN >= 0
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // ESP32: explicit SDA/SCL
    #else
        Wire.begin();                            // Uno R4: board default SDA/SCL
    #endif
    // Keep the core default clock (100 kHz standard mode): both sensors
    // support it and it is more forgiving of long sensor wires than 400 kHz.
    Serial.print(" OK (");
    #if defined(ARDUINO_ARCH_ESP32)
        Serial.print(Wire.getClock() / 1000);
    #else
        Serial.print(100);   // Uno R4: Wire.begin() always sets 100 kHz (no getter)
    #endif
    Serial.println(" kHz)");
    delay(100);

    // Scan all I2C buses for connected devices
    scanAllI2CBuses(Serial);
    
    // Configure differential pressure sensor (DFRobot SEN0343 / LWLP5000)
    // The LWLP5000 uses I2C address 0x00 (fixed), which is not scanned by the
    // standard I2C bus scanner.  Initialize it directly on Wire.
    Serial.print("[INIT] LWLP5000 differential pressure sensor (DFRobot SEN0343)...");
    if (lwlp.begin() != 0) {
        Serial.println(" FAILED!");
        Serial.println("[ERROR] Cannot communicate with differential pressure sensor");
        Serial.println("[ERROR] Check SEN0343 connections to Wire (SDA/SCL)");
        haltWithError("Differential pressure sensor not responding");
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
    Serial.println("[INIT] Fan PWM...");
    Serial.print("        PWM Pin: "); Serial.println(FAN_PWM_PIN);
    Serial.print("        Frequency: ");
    if (FAN_PWM_FREQUENCY_HZ > 0) {
        Serial.print(FAN_PWM_FREQUENCY_HZ); Serial.println(" Hz");
    } else {
        Serial.println("core default");
    }
    setFanPwm(0);
    
    Serial.println("        OK");
    delay(100);

    // Zero the pressure sensor (same routine as the 'recal' command)
    calibratePressureSensor(DEFAULT_CALIB_SAMPLES);

    // Configure QuickPID for full PID control
    Serial.print("[INIT] QuickPID Controller (Full PID)...");
    // loop() calls Compute() exactly once per update interval, so QuickPID
    // runs in timer mode: it computes on every call, using the configured
    // sample time for the I and D scaling.
    myPID.SetOutputLimits(0, 255);
    myPID.SetSampleTimeUs(updateInterval * 1000);
    // P and D on measurement: a setpoint step does not kick the output.
    // Conditional anti-windup: the integral stops growing while saturated.
    myPID.SetTunings(Kp, Ki, Kd, QuickPID::pMode::pOnMeas, QuickPID::dMode::dOnMeas,
                     QuickPID::iAwMode::iAwCondition);
    pidBumplessStart();
    
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
    Serial.print("  PWM Output: Pin "); Serial.println(FAN_PWM_PIN);
    Serial.println("");
    printHelp();
    Serial.println("=====================================\n");
    
    lastUpdate = millis();
}

void loop() {
    // sTune keeps its own sample timing and needs two Run() calls per
    // processed sample, so it is serviced on every pass, not only every 200 ms.
    if (controlMode == MODE_STUNE) runSTune();

    unsigned long now = millis();
    if (now - lastUpdate < updateInterval) return;
    lastUpdate = now;
    debugCounter++;

    // 1. Sensors: fresh airspeed for both the commands and the controller
    float currPressure = readPressureAveraged();         // Oversampled + Averaged
    float currAirDensity = readBarometerSensor();        // Temperature compensated
    currentAirSpeed = calcCurrentAirSpeed(currPressure, currAirDensity);

    // 2. User commands, accepted in every mode (stop must always work)
    ControlMode modeBeforeCommands = controlMode;
    unsigned long commandStart = millis();
    pollSerialCommands();
    unsigned long commandTime = millis() - commandStart;   // e.g. recal blocks on purpose

    // 3. Control. If a command just started or ended a tune, the new mode has
    //    already set the fan; let it begin cleanly on the next pass instead of
    //    running the old controller once more with a different mode's output.
    if (controlMode == modeBeforeCommands) {
        switch (controlMode) {
            case MODE_PID:        runPidControl(); break;
            case MODE_RELAY_TUNE: runRelayTune();  break;
            case MODE_STUNE:      break;             // serviced at the top of loop()
        }
    }

    printDataLine(now, currPressure);
    warnIfSlowPass(millis() - now - commandTime);
}

// The PID assumes one Compute() per update interval. If a pass takes longer
// (slow sensor, long serial output) the loop runs slower than configured, so
// say so - at most every 10 s to keep the data stream readable.
void warnIfSlowPass(unsigned long passMs) {
    if (passMs <= updateInterval) return;
    if (millis() - lastSlowPassWarning < SLOW_PASS_WARNING_INTERVAL_MS) return;
    lastSlowPassWarning = millis();
    Serial.print("[WARN] Update pass took "); Serial.print(passMs);
    Serial.print(" ms (interval "); Serial.print(updateInterval);
    Serial.println(" ms) - control runs slower than configured");
}

// Normal closed-loop operation: PID output with a PWM slew-rate limit.
void runPidControl() {
    // Target 0 means "fan off": hold PWM at 0 instead of computing. While the
    // fan coasts down, the falling airspeed would otherwise make the D and
    // P-on-measurement terms push PWM up again. stopFan() also keeps the PID
    // initialised, so a new target starts smoothly from 0 PWM.
    if (desiredAirSpeed <= 0) {
        stopFan();
        return;
    }

    // PWM rate limit: the output may move at most MAX_PWM_CHANGE_PER_CYCLE
    // from the present PWM. Doing this through the PID's output limits (not
    // after Compute()) also clamps the integral, so it cannot wind up while
    // the fan is still ramping.
    myPID.SetOutputLimits(constrain(fanPwm - MAX_PWM_CHANGE_PER_CYCLE, 0, 255),
                          constrain(fanPwm + MAX_PWM_CHANGE_PER_CYCLE, 0, 255));
    myPID.Compute();
    setFanPwm((int)round(currPWM));
}

void printDataLine(unsigned long now, float pressure) {
    unsigned long elapsed = (now - startTime) / 1000;
    float error = desiredAirSpeed - currentAirSpeed;

    if (controlMode == MODE_STUNE) {
        Serial.print("[sTune] ");
        Serial.print(sTuner->getStatusString()); Serial.print(" | ");
    } else if (controlMode == MODE_RELAY_TUNE) {
        Serial.print("[TUNE] ");
        uint32_t tuneElapsed = (millis() - tuneStartTime) / 1000;
        Serial.print(tuneElapsed); Serial.print("s | ");
        Serial.print("Cycle:"); Serial.print(relayCycles); Serial.print(" | ");
    } else {
        Serial.print(elapsed); Serial.print("s | ");
    }
    Serial.print("V:"); Serial.print(currentAirSpeed, 2); Serial.print(" | ");
    Serial.print("T:"); Serial.print(desiredAirSpeed, 2); Serial.print(" | ");
    Serial.print("PWM:"); Serial.print((int)round(currPWM)); Serial.print(" | ");
    Serial.print("P:"); Serial.print(pressure, 2); Serial.print(" | ");
    Serial.print("T:"); Serial.print(lastTemperature, 1); Serial.print("°C | ");
    Serial.print("Err:"); Serial.print(error, 2);

    // Show PID values and averaging every 20 readings
    if (debugCounter % 20 == 0 && controlMode == MODE_PID) {
        Serial.print(" | Kp:"); Serial.print(Kp, 1);
        Serial.print(" Ki:"); Serial.print(Ki, 1);
        Serial.print(" Kd:"); Serial.print(Kd, 1);
        Serial.print(" Avg:"); Serial.print(pressureAverageSamples);
    }

    Serial.println();
}

// ---------------------------------------------------------------------------
// Relay auto-tune
// ---------------------------------------------------------------------------

// Start a relay tune around the present target. pwmLow/pwmHigh = -1 means
// "estimate the PWM levels from the target".
void startRelayTune(int pwmLow, int pwmHigh) {
    if (!tuneTargetOk()) return;

    bool autoRange = (pwmLow < 0 || pwmHigh < 0);
    if (autoRange) {
        int estimatedPwm = (int)round(desiredAirSpeed * 8);   // rough fan sensitivity
        pwmHigh = constrain(estimatedPwm + 20, 40, 245);
        pwmLow = constrain(estimatedPwm - 20, 20, 150);
    } else if (pwmLow >= pwmHigh) {
        Serial.println("[ERROR] PWM low must be less than PWM high (e.g. tune 40 100)");
        return;
    }
    relayPwmLow = pwmLow;
    relayPwmHigh = pwmHigh;

    Serial.println("\n=====================================");
    Serial.println("[RELAY] AUTO-TUNE STARTING");
    Serial.println("=====================================");
    Serial.print("[RELAY] Target airspeed: "); Serial.print(desiredAirSpeed, 2); Serial.println(" m/s");
    Serial.println("[RELAY] Method: Astrom-Hagglund relay, Pessen integral rule");
    Serial.print("[RELAY] PWM range: "); Serial.print(relayPwmLow);
    Serial.print(" - "); Serial.print(relayPwmHigh);
    Serial.println(autoRange ? " (estimated; if tuning fails try e.g. tune 40 100)" : " (manual)");
    Serial.print("[RELAY] Hysteresis: +/-"); Serial.print(RELAY_HYSTERESIS, 2);
    Serial.print(" m/s, cycles needed: "); Serial.print(REQUIRED_CYCLES);
    Serial.print(", timeout: "); Serial.print(RELAY_TIMEOUT_MS / 1000); Serial.println(" s");
    Serial.println("[RELAY] Type 0 or stop to abort");
    Serial.println("=====================================");

    tuneStartTime = millis();
    firstCrossing = true;
    peakHigh = currentAirSpeed;
    peakLow = currentAirSpeed;
    relayCycles = 0;
    sumPeriodsS = 0.0;
    sumAmplitudes = 0.0;
    setRelay(false);                   // start low; the first switch is LOW->HIGH
    controlMode = MODE_RELAY_TUNE;
}

// One relay step per update interval.
void runRelayTune() {
    if (currentAirSpeed > peakHigh) peakHigh = currentAirSpeed;
    if (currentAirSpeed < peakLow) peakLow = currentAirSpeed;

    if (!relayHigh && currentAirSpeed < desiredAirSpeed - RELAY_HYSTERESIS) {
        setRelay(true);
    } else if (relayHigh && currentAirSpeed > desiredAirSpeed + RELAY_HYSTERESIS) {
        setRelay(false);
        recordRelayCycle();
        if (relayCycles >= REQUIRED_CYCLES) {
            completeRelayTune();
            return;
        }
    }

    if (millis() - tuneStartTime > RELAY_TIMEOUT_MS) {
        Serial.print("\n[RELAY] Not enough oscillation: ");
        Serial.print(relayCycles); Serial.print(" of "); Serial.print(REQUIRED_CYCLES);
        Serial.println(" valid cycles");
        Serial.println("        - Check fan power and the PWM wiring");
        Serial.println("        - Make sure the target lies between the speeds of the two PWM levels");
        Serial.println("        - Try a manual range, e.g. tune 40 100");
        abortTuning("relay tune timeout");
    }
}

// Apply the low or high relay level.
void setRelay(bool high) {
    relayHigh = high;
    int pwm = high ? relayPwmHigh : relayPwmLow;
    currPWM = pwm;
    setFanPwm(pwm);
    Serial.print("[RELAY] Switched to "); Serial.print(high ? "HIGH" : "LOW");
    Serial.print(" (PWM "); Serial.print(pwm);
    Serial.print(") at V="); Serial.print(currentAirSpeed, 2); Serial.println(" m/s");
}

// Called at every HIGH->LOW switch: one full oscillation cycle has ended.
void recordRelayCycle() {
    unsigned long now = millis();
    if (!firstCrossing) {
        unsigned long periodMs = now - lastCrossingTime;
        float amplitude = (peakHigh - peakLow) / 2.0;
        if (amplitude > MIN_CYCLE_AMPLITUDE && periodMs > MIN_CYCLE_PERIOD_MS) {
            relayCycles++;
            sumPeriodsS += periodMs / 1000.0;
            sumAmplitudes += amplitude;
            Serial.print("[RELAY] Cycle "); Serial.print(relayCycles);
            Serial.print(" | Period: "); Serial.print(periodMs / 1000.0, 1); Serial.print(" s");
            Serial.print(" | Amplitude: "); Serial.print(amplitude, 2); Serial.println(" m/s");
        }
    }
    // Every HIGH->LOW switch (the first one too) starts a new cycle window,
    // so the peaks always belong to exactly one cycle.
    firstCrossing = false;
    lastCrossingTime = now;
    peakHigh = currentAirSpeed;
    peakLow = currentAirSpeed;
}

// Enough cycles: compute Ku and Tu, then the PID gains.
void completeRelayTune() {
    float tu = sumPeriodsS / relayCycles;              // ultimate period, s
    float amplitude = sumAmplitudes / relayCycles;     // a, m/s
    // Describing function of a relay with output amplitude d = (high-low)/2:
    // Ku = 4d / (pi a) = 2 (high - low) / (pi a), in PWM per m/s.
    float ku = 2.0 * (relayPwmHigh - relayPwmLow) / (PI * amplitude);

    // Pessen integral rule: Kp = 0.7 Ku, Ti = 0.4 Tu, Td = 0.15 Tu
    float newKp = 0.7 * ku;
    float newKi = 1.75 * ku / tu;       // Kp / Ti
    float newKd = 0.105 * ku * tu;      // Kp * Td

    Serial.println("\n=====================================");
    Serial.println("[RELAY] AUTO-TUNE COMPLETE");
    Serial.println("=====================================");
    Serial.print("        Period Tu: "); Serial.print(tu, 2); Serial.println(" s");
    Serial.print("        Amplitude: "); Serial.print(amplitude, 2); Serial.println(" m/s");
    Serial.print("        Ultimate gain Ku: "); Serial.print(ku, 3); Serial.println(" PWM per m/s");
    Serial.print("        Pessen integral rule: Kp="); Serial.print(newKp, 3);
    Serial.print(" Ki="); Serial.print(newKi, 3);
    Serial.print(" Kd="); Serial.println(newKd, 3);
    finishTuning(newKp, newKi, newKd);
}

void setPressureAverageSamples(int samples) {
    samples = constrain(samples, MIN_AVG_SAMPLES, MAX_AVG_SAMPLES);
    
    if (samples != pressureAverageSamples) {
        pressureAverageSamples = samples;
        resetPressureAverage();
        
        Serial.print("[CONFIG] Pressure averaging set to ");
        Serial.print(pressureAverageSamples);
        Serial.println(" samples");
        
        if (pressureAverageSamples == 1) {
            Serial.println("         (No averaging - raw oversampled readings)");
        } else {
            // A plain N-sample average delays the signal by (N-1)/2 samples
            Serial.print("         Effective lag: ~");
            Serial.print((pressureAverageSamples - 1) * updateInterval / 2);
            Serial.println(" ms");
        }
    }
}

// Zero the differential pressure sensor: the mean of numSamples raw readings
// taken in still air becomes the offset. The fan is always stopped first,
// because any airflow during calibration would be stored as "zero" and make
// every later airspeed reading wrong.
void calibratePressureSensor(int numSamples) {
    stopFan();

    Serial.println("\n[CALIB] Pressure Sensor Calibration");
    Serial.println("=====================================");
    Serial.println("        Fan stopped. Keep the tunnel still:");
    Serial.println("        - no airflow, both pitot ports open to the room");
    Serial.print  ("        Waiting "); Serial.print(CALIBRATION_SETTLE_S);
    Serial.println(" s for the fan to stop and readings to settle...");
    for (int i = CALIBRATION_SETTLE_S; i > 0; i--) {
        Serial.print("        ");
        if (i < 10) Serial.print(" ");
        Serial.print(i);
        Serial.println(" seconds remaining...");
        delay(1000);
    }

    Serial.print("        Taking "); Serial.print(numSamples); Serial.print(" samples: ");
    double sum = 0.0;
    double sumSquares = 0.0;
    float minVal = 0.0;
    float maxVal = 0.0;
    for (int i = 0; i < numSamples; i++) {
        float reading = readPressureRaw();
        sum += reading;
        sumSquares += (double)reading * reading;
        if (i == 0 || reading < minVal) minVal = reading;
        if (i == 0 || reading > maxVal) maxVal = reading;
        Serial.print(".");
        delay(CALIB_SAMPLE_SPACING_MS);
    }
    Serial.println();

    float mean = sum / numSamples;
    float variance = sumSquares / numSamples - (double)mean * mean;
    if (variance < 0) variance = 0;              // rounding can give tiny negatives
    pressureOffset = mean;
    resetPressureAverage();                      // old samples used the old offset

    Serial.println("\n        Calibration Results:");
    Serial.print("        Min reading: "); Serial.print(minVal, 4); Serial.println(" Pa");
    Serial.print("        Max reading: "); Serial.print(maxVal, 4); Serial.println(" Pa");
    Serial.print("        Offset (mean): "); Serial.print(pressureOffset, 4); Serial.println(" Pa");
    Serial.print("        Noise (std dev): "); Serial.print(sqrt(variance), 4); Serial.println(" Pa");
    Serial.println("=====================================\n");
}

float readBarometerSensor() { 
    float temp_c = bmp58x->readTempC();
    float temp_k = temp_c + 273.15;
    float pressure_pa = bmp58x->readPressPa();
    
    // Sanity-check: fall back to cached values on implausible readings
    if (temp_k < 173.15 || temp_k > 373.15 || pressure_pa < 50000.0 || pressure_pa > 120000.0) {
        return lastAirDensity;
    }
    
    float density = pressure_pa / (Specific_gas_constant * temp_k);
    
    lastAirDensity = density;
    lastTemperature = temp_c;
    
    return density;
}

// One raw sensor reading in Pa. The LWLP library already subtracts the
// reading it took at power-on; calibration removes what is left of the zero
// error. Blocks for about 60 ms (the library waits for the conversion).
float readPressureRaw() {
    return lwlp.getData().presure;   // "presure" is the library's spelling
}

// Oversampled reading minus the calibration offset. Kept signed on purpose:
// at zero airspeed the noise swings both ways, and clamping negative values
// here would bias the moving average (and the airspeed) upward.
float readPressureCorrected() {
    float sum = 0.0;
    for (int i = 0; i < PRESSURE_OVERSAMPLES; i++) {
        sum += readPressureRaw();
    }
    return sum / PRESSURE_OVERSAMPLES - pressureOffset;
}

// Moving average over the last pressureAverageSamples corrected readings.
float readPressureAveraged() {
    pressureBuffer[pressureBufferIndex] = readPressureCorrected();
    pressureBufferIndex++;
    if (pressureBufferIndex >= pressureAverageSamples) {
        pressureBufferIndex = 0;
        pressureBufferFilled = true;
    }

    int count = pressureBufferFilled ? pressureAverageSamples : pressureBufferIndex;
    float sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += pressureBuffer[i];
    }
    return sum / count;      // count >= 1: one sample was just stored
}

// Forget old samples (after a new offset or a new averaging length).
void resetPressureAverage() {
    pressureBufferIndex = 0;
    pressureBufferFilled = false;
}

// Pitot equation v = sqrt(2 q / rho). Negative pressure (noise around zero
// airspeed) is the only place that is clamped, because sqrt needs q >= 0.
float calcCurrentAirSpeed(float pressure, float AirDensity) { 
    if (pressure < 0.0) pressure = 0.0;
    if (AirDensity < 0.1) AirDensity = 1.225;
    
    float airSpeed = sqrt((2.0 * pressure) / AirDensity);
    
    return airSpeed;
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------

// Collect typed characters without blocking and run each complete line.
// A line ends with LF, CR or CR+LF, so every serial monitor setting works.
void pollSerialCommands() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialLineTooLong) {
                Serial.print("[ERROR] Line too long (max ");
                Serial.print(SERIAL_LINE_SIZE - 1);
                Serial.println(" characters) - ignored");
            } else if (serialLineLength > 0) {
                serialLine[serialLineLength] = '\0';
                handleCommand(serialLine);
            }
            serialLineLength = 0;
            serialLineTooLong = false;
        } else if (serialLineLength < SERIAL_LINE_SIZE - 1) {
            serialLine[serialLineLength++] = c;
        } else {
            serialLineTooLong = true;   // discard the rest of this line
        }
    }
}

// Run one command line. Keywords are case-insensitive.
void handleCommand(char *line) {
    char *words[MAX_COMMAND_WORDS];
    int count = splitWords(line, words, MAX_COMMAND_WORDS);
    if (count == 0) return;                       // blank line
    if (count > MAX_COMMAND_WORDS) {
        Serial.println("[ERROR] Too many values - type 'help' for the command list");
        return;
    }

    const char *cmd = words[0];
    float number = 0.0;
    bool isNumber = parseDecimal(cmd, number);

    // Stop works in every mode, including during a tune.
    if (count == 1 && (wordIs(cmd, "stop") || (isNumber && number == 0.0))) {
        if (controlMode != MODE_PID) {
            abortTuning("stopped by user");
        } else {
            stopFan();
            Serial.println("[STOP] Fan stopped, target 0 m/s");
        }
        return;
    }

    // While a tuner drives the fan, nothing else may change its state.
    if (controlMode != MODE_PID) {
        Serial.println("[BUSY] Tuning in progress - type 0 or stop to abort");
        return;
    }

    if (wordIs(cmd, "help")) {
        printHelp();
    } else if (wordIs(cmd, "tune")) {
        if (count == 1) {
            startRelayTune();                     // automatic PWM range
        } else if (count == 3) {
            int pwmLow, pwmHigh;
            if (parseWholeNumber(words[1], 1, 255, pwmLow) &&
                parseWholeNumber(words[2], 1, 255, pwmHigh)) {
                startRelayTune(pwmLow, pwmHigh);
            } else {
                Serial.println("[ERROR] PWM values must be whole numbers 1-255");
                Serial.println("        Usage: tune <low> <high>   Example: tune 40 100");
            }
        } else if (count == 4) {
            setManualGains(words[1], words[2], words[3]);
        } else {
            Serial.println("[ERROR] Usage: tune | tune <low> <high> | tune <Kp> <Ki> <Kd>");
        }
    } else if (wordIs(cmd, "stune") && count == 1) {
        startSTune();
    } else if (wordIs(cmd, "recal")) {
        int samples = DEFAULT_CALIB_SAMPLES;
        if (count > 2 || (count == 2 &&
            !parseWholeNumber(words[1], MIN_CALIB_SAMPLES, MAX_CALIB_SAMPLES, samples))) {
            Serial.print("[ERROR] Calibration samples must be between ");
            Serial.print(MIN_CALIB_SAMPLES); Serial.print(" and ");
            Serial.println(MAX_CALIB_SAMPLES);
            return;
        }
        calibratePressureSensor(samples);
    } else if (wordIs(cmd, "avg")) {
        int samples;
        if (count == 2 && parseWholeNumber(words[1], MIN_AVG_SAMPLES, MAX_AVG_SAMPLES, samples)) {
            setPressureAverageSamples(samples);
        } else {
            Serial.print("[ERROR] Averaging samples must be between ");
            Serial.print(MIN_AVG_SAMPLES); Serial.print(" and ");
            Serial.println(MAX_AVG_SAMPLES);
        }
    } else if (isNumber && count == 1) {
        setTargetAirspeed(number);
    } else {
        Serial.print("[ERROR] Unknown command or wrong number of values: '");
        Serial.print(cmd);
        Serial.println("' - type 'help' for the command list");
    }
}

// Split a line in place into words separated by spaces or tabs.
// Returns the number of words, or maxWords + 1 if there are more.
int splitWords(char *line, char *words[], int maxWords) {
    int count = 0;
    char *p = line;
    while (true) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') return count;
        if (count == maxWords) return maxWords + 1;
        words[count++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') p++;
        if (*p != '\0') *p++ = '\0';
    }
}

// Case-insensitive match of a typed word against a lower-case keyword.
bool wordIs(const char *word, const char *keyword) {
    while (*word != '\0' && *keyword != '\0') {
        if (tolower((unsigned char)*word) != *keyword) return false;
        word++;
        keyword++;
    }
    return *word == *keyword;
}

// Parse a plain decimal number such as "12", "-3" or "7.5". The whole word
// must be a number, so "10abc", "1e3", "nan" or "inf" are rejected and a typo
// can never turn into a fan speed.
bool parseDecimal(const char *text, float &value) {
    const char *p = text;
    if (*p == '+' || *p == '-') p++;
    bool hasDigit = false;
    bool hasPoint = false;
    for (; *p != '\0'; p++) {
        if (isdigit((unsigned char)*p)) {
            hasDigit = true;
        } else if (*p == '.' && !hasPoint) {
            hasPoint = true;
        } else {
            return false;
        }
    }
    if (!hasDigit) return false;
    value = (float)atof(text);
    return isfinite(value);        // very long digit strings overflow to inf
}

// Parse a whole number and check that it lies in [minValue, maxValue].
bool parseWholeNumber(const char *text, int minValue, int maxValue, int &value) {
    float f;
    if (!parseDecimal(text, f) || f != floorf(f) || f < minValue || f > maxValue) {
        return false;
    }
    value = (int)f;
    return true;
}

void setTargetAirspeed(float target) {
    if (target < 0) {
        Serial.println("[ERROR] Negative airspeed not allowed");
        return;
    }
    if (target > MAX_AIRSPEED) {
        Serial.print("[WARN] Capping at "); Serial.print(MAX_AIRSPEED); Serial.println(" m/s");
        target = MAX_AIRSPEED;
    }
    // Only the setpoint changes. The PID keeps its integral, so the fan moves
    // smoothly from its present PWM (Reset() would zero the integral and drop
    // the output to about 0 first).
    desiredAirSpeed = target;
    Serial.print("[SERIAL] New target: "); Serial.print(desiredAirSpeed, 2); Serial.println(" m/s");
}

// tune <Kp> <Ki> <Kd>: set the PID gains by hand.
void setManualGains(const char *kpText, const char *kiText, const char *kdText) {
    float newKp, newKi, newKd;
    if (!parseDecimal(kpText, newKp) || !parseDecimal(kiText, newKi) ||
        !parseDecimal(kdText, newKd) || newKp <= 0 || newKi < 0 || newKd < 0) {
        Serial.println("[ERROR] Invalid PID values");
        Serial.println("        Usage: tune <Kp> <Ki> <Kd>   Example: tune 20 8 12");
        return;
    }
    applyGains(newKp, newKi, newKd);   // takes effect on the next Compute(), no bump
}

// Clamp the gains to the allowed ranges (reporting any that were clamped)
// and give them to the PID. Used by manual entry and both auto-tuners.
void applyGains(float newKp, float newKi, float newKd) {
    Kp = clampGain("Kp", newKp, KP_MIN, KP_MAX);
    Ki = clampGain("Ki", newKi, KI_MIN, KI_MAX);
    Kd = clampGain("Kd", newKd, KD_MIN, KD_MAX);
    myPID.SetTunings(Kp, Ki, Kd);
    Serial.print("[PID] Gains set: Kp="); Serial.print(Kp, 3);
    Serial.print(" Ki="); Serial.print(Ki, 3);
    Serial.print(" Kd="); Serial.println(Kd, 3);
}

float clampGain(const char *name, float value, float minValue, float maxValue) {
    float clamped = constrain(value, minValue, maxValue);
    if (clamped != value) {
        Serial.print("[PID] "); Serial.print(name); Serial.print(" = "); Serial.print(value, 3);
        Serial.print(" is outside "); Serial.print(minValue, 1); Serial.print("-");
        Serial.print(maxValue, 1); Serial.print(", clamped to "); Serial.println(clamped, 3);
    }
    return clamped;
}

// Stop the fan now and leave the PID idle at zero output and zero target.
void stopFan() {
    desiredAirSpeed = 0;
    currPWM = 0;
    setFanPwm(0);
    pidBumplessStart();
}

// The only place that writes the fan PWM. It remembers the applied value
// (used by the rate limit, the tune hand-over and the data line) and skips
// writes that would change nothing, because sTune calls it on every pass.
void setFanPwm(int pwm) {
    pwm = constrain(pwm, 0, 255);
    if (pwm == fanPwm) return;
    #if defined(ARDUINO_ARCH_ESP32)
        // The ESP32 core sets up the PWM channel on the first analogWrite()
        // using the frequency chosen beforehand, so set it once, first.
        if (fanPwm < 0 && FAN_PWM_FREQUENCY_HZ > 0) {
            analogWriteFrequency(FAN_PWM_PIN, FAN_PWM_FREQUENCY_HZ);
        }
    #endif
    analogWrite(FAN_PWM_PIN, pwm);
    fanPwm = pwm;
}

// Hand the fan to the PID without a bump. Switching QuickPID from manual to
// timer mode runs its Initialize(): the integral starts at the present
// output (currPWM) and the derivative at the present airspeed, so the first
// Compute() continues smoothly from where the fan is now.
void pidBumplessStart() {
    myPID.SetMode(QuickPID::Control::manual);
    myPID.SetOutputLimits(0, 255);   // drop the rate-limit window of the last pass
    myPID.SetMode(QuickPID::Control::timer);
}

// Auto-tunes only work with a clear airspeed signal around the target.
bool tuneTargetOk() {
    if (desiredAirSpeed >= MIN_TUNE_SETPOINT) return true;
    Serial.print("[ERROR] Set a target of at least "); Serial.print(MIN_TUNE_SETPOINT, 1);
    Serial.println(" m/s before tuning (e.g. 10, then tune or stune)");
    return false;
}

// End a running relay tune or sTune without applying any gains (user abort,
// timeout or tuner error): fan off, target 0, PID re-initialised.
void abortTuning(const char *reason) {
    if (controlMode == MODE_STUNE) sTuner->reset();
    controlMode = MODE_PID;
    stopFan();
    Serial.print("\n[TUNE] Tuning aborted (");
    Serial.print(reason);
    Serial.println(") - fan stopped, target 0 m/s, gains unchanged\n");
}

// A tune succeeded: apply the gains and hand the fan to the PID at its
// present PWM and the tune target, so the fan keeps running smoothly.
void finishTuning(float newKp, float newKi, float newKd) {
    if (controlMode == MODE_STUNE) sTuner->reset();   // note: rewrites currPWM
    controlMode = MODE_PID;
    applyGains(newKp, newKi, newKd);
    currPWM = fanPwm;                                  // PWM the fan runs at now
    pidBumplessStart();
    Serial.print("[TUNE] PID running at target "); Serial.print(desiredAirSpeed, 2);
    Serial.println(" m/s with the new gains");
    Serial.print("[TUNE] To reuse these gains later: tune ");
    Serial.print(Kp, 3); Serial.print(" "); Serial.print(Ki, 3); Serial.print(" "); Serial.println(Kd, 3);
    Serial.println("=====================================\n");
}

void printHelp() {
    Serial.println("COMMANDS (type a line and press Enter):");
    Serial.print  ("  <m/s>                Set target airspeed, 0 to "); Serial.print(MAX_AIRSPEED, 1);
    Serial.println(" (e.g. 10)");
    Serial.println("  0 or stop            Stop the fan now (also aborts a running tune)");
    Serial.println("  tune                 Relay auto-tune, automatic PWM range");
    Serial.println("  tune <low> <high>    Relay auto-tune, PWM range 1-255 (e.g. tune 40 100)");
    Serial.println("  tune <Kp> <Ki> <Kd>  Set PID gains (e.g. tune 20 8 12)");
    Serial.print  ("                       Ranges: Kp "); Serial.print(KP_MIN, 0); Serial.print("-");
    Serial.print(KP_MAX, 0); Serial.print(", Ki "); Serial.print(KI_MIN, 1); Serial.print("-");
    Serial.print(KI_MAX, 0); Serial.print(", Kd "); Serial.print(KD_MIN, 1); Serial.print("-");
    Serial.println(KD_MAX, 0);
    Serial.println("  stune                sTune step-response auto-tune");
    Serial.print  ("  recal [N]            Re-zero the pressure sensor (N = ");
    Serial.print(MIN_CALIB_SAMPLES); Serial.print("-"); Serial.print(MAX_CALIB_SAMPLES);
    Serial.println(" samples)");
    Serial.print  ("  avg <N>              Pressure moving-average length (");
    Serial.print(MIN_AVG_SAMPLES); Serial.print("-"); Serial.print(MAX_AVG_SAMPLES);
    Serial.println(")");
    Serial.println("  help                 Show this list");
    Serial.print  ("Auto-tunes need a target of at least "); Serial.print(MIN_TUNE_SETPOINT, 1);
    Serial.println(" m/s. During a tune only 0 or stop is accepted.");
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
    Serial.print("[INIT] BMP585 barometer (DFRobot SEN0665)...");
    
    // Try to find BMP585 on any bus — check default address 0x47 then alternate 0x46
    const char* bmpBusName = nullptr;
    TwoWire* bmpBus = getDeviceBus(BMP_I2C_ADDRESS, &bmpBusName);
    
    if (bmpBus == nullptr) {
        // Try alternate address
        bmpBus = getDeviceBus(0x46, &bmpBusName);
    }
    
    if (bmpBus == nullptr) {
        Serial.println(" NOT FOUND!");
        Serial.println("[ERROR] BMP585 not found at 0x47 or 0x46 on any bus");
        Serial.println("[ERROR] Check SEN0665 sensor connections");
        haltWithError("Barometer not found");
    }
    
    Serial.print(" (found on "); Serial.print(bmpBusName); Serial.print(")");
    
    uint8_t bmpAddr = BMP_I2C_ADDRESS;
    bmp58x = new DFRobot_BMP58X_I2C(bmpBus, bmpAddr);
    
    if (!bmp58x->begin()) {
        // Try alternate address
        delete bmp58x;
        bmpAddr = 0x46;
        bmp58x = new DFRobot_BMP58X_I2C(bmpBus, bmpAddr);
        if (!bmp58x->begin()) {
            Serial.println(" FAILED!");
            Serial.println("[ERROR] BMP585 found in scan but cannot initialize");
            Serial.print("[ERROR] Check connections on ");
            Serial.println(bmpBusName);
            haltWithError("Barometer found but not responding");
        }
        Serial.println(" OK (0x46)");
    } else {
        Serial.print(" OK (0x"); Serial.print(bmpAddr, HEX); Serial.println(")");
    }
    
    bmp58x->setMeasureMode(bmp58x->eNormal);
    
    float tempC = bmp58x->readTempC();
    float pressPa = bmp58x->readPressPa();
    if (pressPa > 50000.0) {
        Serial.print("        T:"); Serial.print(tempC, 1); Serial.print("°C ");
        Serial.print("P:"); Serial.print(pressPa / 100.0, 1); Serial.println("hPa");
        Serial.println("        Temperature compensation: ENABLED");
    }
    delay(100);
}

// ---------------------------------------------------------------------------
// sTune auto-tune (open-loop step test, inflection point method)
// ---------------------------------------------------------------------------

void startSTune() {
    if (!tuneTargetOk()) return;

    // Step size: rough PWM needed for the target, kept in a range that gives
    // a clear response without running the fan flat out.
    float outputStep = constrain(desiredAirSpeed * 8.0, 60.0, 200.0);

    // setpoint, input span, output span, output start, output step,
    // test time (s), settle time (s), samples (60 s / 300 = one per 200 ms)
    if (sTuner->startTuning(desiredAirSpeed, MAX_AIRSPEED, 255.0, 0.0, outputStep, 60, 5, 300)) {
        controlMode = MODE_STUNE;
    }
}

// sTune step test, called on every loop() pass. sTune sets currPWM through
// the pointer it was given; setFanPwm() only writes when the value changes.
void runSTune() {
    STunePIDTuner::TuningStatus status = sTuner->update();

    if (status == STunePIDTuner::COMPLETED) {
        float newKp, newKi, newKd;
        sTuner->getTunedGains(newKp, newKi, newKd);
        finishTuning(newKp, newKi, newKd);
        return;
    }
    if (status == STunePIDTuner::TIMEOUT || status == STunePIDTuner::ERROR) {
        abortTuning(status == STunePIDTuner::TIMEOUT ? "sTune timeout" : "sTune error");
        return;
    }

    setFanPwm((int)round(currPWM));
}

// Fatal error: keep the fan off and stop here for good. Blinking the status
// LED (if the board has one) shows the fault even without a serial monitor.
// The sketch never continues, because running the fan without valid sensor
// data would mean running it without control.
void haltWithError(const char *msg) {
    setFanPwm(0);
    Serial.print("[FATAL] ");
    Serial.println(msg);
    Serial.println("[FATAL] System halted with the fan off. Fix the fault and reset the board.");
    bool ledOn = false;
    while (true) {
        if (STATUS_LED_PIN >= 0) {
            ledOn = !ledOn;
            digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
        }
        delay(250);
    }
}
