/*
 * sTune_WindTunnel_Example.ino
 * 
 * Example demonstrating sTune integration for Wind Tunnel PID autotuning
 * This is a simplified example showing how to use the STunePIDTuner wrapper
 * 
 * Hardware Requirements:
 * - Arduino Giga R1 or Mega 2560
 * - MS4525DO differential pressure sensor
 * - BMP3XX barometric sensor
 * - PWM-controlled fan
 * 
 * Library Requirements (install via Arduino Library Manager):
 * - QuickPID
 * - sTune
 * - Adafruit BMP3XX
 * - Adafruit Unified Sensor
 * - Bolder Flight Systems MS4525DO
 * 
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <QuickPID.h>
#include "ms4525do.h"
#include <sTune.h>
#include "STunePIDTuner.h"

// Pin assignments
const int PWM_pin = 9;

// Sensor configuration
#define DPS_ADDRESS 0x28
#define BMP_I2C_ADDRESS 0x77
const float Specific_gas_constant = 287.058;

// PID gains - will be updated by autotuner
float Kp = 20.0, Ki = 8.0, Kd = 12.0;

// Process variables
float currentAirSpeed = 0.0;
float desiredAirSpeed = 0.0;
float currPWM = 0.0;

// Sensor objects
bfs::Ms4525do pres;
Adafruit_BMP3XX bmp;

// PID controller
QuickPID myPID(&currentAirSpeed, &currPWM, &desiredAirSpeed);

// sTune wrapper
STunePIDTuner *tuner = nullptr;

// Timing
const unsigned long updateInterval = 200; // 200ms
unsigned long lastUpdate = 0;

// State
bool tuning = false;

// Forward declarations
float readAirSpeed();
float readAirDensity();

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n=====================================");
    Serial.println("   sTune Wind Tunnel Example");
    Serial.println("   Demonstrating sTune Integration");
    Serial.println("=====================================\n");
    
    // Initialize I2C
    Serial.print("[INIT] I2C bus...");
    Wire.begin();
    Wire.setClock(400000);
    Serial.println(" OK");
    
    // Initialize pressure sensor
    Serial.print("[INIT] MS4525DO pressure sensor...");
    pres.Config(&Wire, DPS_ADDRESS, 1.0f, -1.0f);
    if (!pres.Begin()) {
        Serial.println(" FAILED!");
        while (1) { delay(1000); }
    }
    Serial.println(" OK");
    
    // Initialize barometer
    Serial.print("[INIT] BMP3XX barometer...");
    if (!bmp.begin_I2C(BMP_I2C_ADDRESS)) {
        Serial.println(" FAILED!");
        while (1) { delay(1000); }
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    Serial.println(" OK");
    
    // Initialize PWM
    Serial.print("[INIT] PWM output...");
    pinMode(PWM_pin, OUTPUT);
    analogWrite(PWM_pin, 0);
    Serial.println(" OK");
    
    // Configure PID
    Serial.print("[INIT] QuickPID controller...");
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetOutputLimits(0, 255);
    myPID.SetSampleTimeUs(updateInterval * 1000);
    myPID.SetMode(QuickPID::Control::automatic);
    myPID.SetProportionalMode(QuickPID::pMode::pOnMeas);
    myPID.SetDerivativeMode(QuickPID::dMode::dOnMeas);
    myPID.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
    Serial.println(" OK");
    
    // Create sTune wrapper
    tuner = new STunePIDTuner(&currentAirSpeed, &currPWM);
    tuner->setEmergencyStop(28.0); // Safety limit
    
    Serial.println("\n[READY] System initialized!");
    Serial.println("\nCOMMANDS:");
    Serial.println("  <number>  - Set target airspeed (e.g., '10')");
    Serial.println("  stune     - Start sTune autotuning");
    Serial.println("  0         - Stop fan");
    Serial.println("\nEXAMPLE USAGE:");
    Serial.println("  1. Type '10' to set target to 10 m/s");
    Serial.println("  2. Type 'stune' to start autotuning");
    Serial.println("  3. Wait for tuning to complete (~30-60 sec)");
    Serial.println("  4. Gains automatically applied to PID");
    Serial.println("=====================================\n");
    
    lastUpdate = millis();
}

void loop() {
    unsigned long now = millis();
    
    // Update at fixed interval
    if (now - lastUpdate >= updateInterval) {
        lastUpdate = now;
        
        // Read sensors
        currentAirSpeed = readAirSpeed();
        
        if (tuning) {
            // Run sTune update
            STunePIDTuner::TuningStatus status = tuner->update();
            
            if (status == STunePIDTuner::COMPLETED) {
                // Get tuned gains and apply to PID
                float newKp, newKi, newKd;
                if (tuner->getTunedGains(newKp, newKi, newKd)) {
                    Kp = newKp;
                    Ki = newKi;
                    Kd = newKd;
                    
                    myPID.SetTunings(Kp, Ki, Kd);
                    myPID.Reset();
                    
                    Serial.println("[EXAMPLE] Gains applied to PID controller");
                    Serial.println("[EXAMPLE] Resuming normal operation");
                }
                
                tuning = false;
                tuner->reset();
                
            } else if (status == STunePIDTuner::TIMEOUT || status == STunePIDTuner::ERROR) {
                Serial.println("[EXAMPLE] Tuning failed - keeping previous gains");
                tuning = false;
                tuner->reset();
                currPWM = 0;
                analogWrite(PWM_pin, 0);
            }
            
        } else {
            // Normal PID control
            checkSerialInput();
            myPID.Compute();
            analogWrite(PWM_pin, (int)round(currPWM));
        }
        
        // Print status
        Serial.print("V:"); Serial.print(currentAirSpeed, 2);
        Serial.print(" | T:"); Serial.print(desiredAirSpeed, 2);
        Serial.print(" | PWM:"); Serial.print((int)round(currPWM));
        if (tuning) {
            Serial.print(" | Status:"); Serial.print(tuner->getStatusString());
        }
        Serial.println();
    }
}

void checkSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.equalsIgnoreCase("stune")) {
            if (desiredAirSpeed < 2.0) {
                Serial.println("[EXAMPLE] ERROR: Set target airspeed first (min 2 m/s)");
                Serial.println("[EXAMPLE] Example: type '10' then 'stune'");
                return;
            }
            
            // Start sTune autotuning
            // Parameters: setpoint, inputSpan, outputSpan, outputStart, outputStep, testTime, settleTime, samples
            if (tuner->startTuning(desiredAirSpeed, 30.0, 255.0, 0.0, 100.0, 60, 5, 300)) {
                tuning = true;
                Serial.println("[EXAMPLE] sTune autotuning started");
            }
            
        } else {
            // Parse as airspeed setpoint
            float newSetpoint = input.toFloat();
            if (newSetpoint >= 0.0 && newSetpoint <= 28.0) {
                desiredAirSpeed = newSetpoint;
                myPID.Reset();
                Serial.print("[EXAMPLE] Target set to "); 
                Serial.print(desiredAirSpeed, 2); 
                Serial.println(" m/s");
            } else {
                Serial.println("[EXAMPLE] Invalid setpoint (range: 0-28 m/s)");
            }
        }
    }
}

float readAirSpeed() {
    // Simplified airspeed reading for example
    // In production, use proper oversampling and averaging
    float pressure = 0.0;
    if (pres.Read()) {
        pressure = pres.pres_pa();
        if (pressure < 0.0) pressure = 0.0;
    }
    
    float airDensity = readAirDensity();
    if (pressure < 0.0) pressure = 0.0;
    if (airDensity < 0.1) airDensity = 1.225;
    
    return sqrt((2.0 * pressure) / airDensity);
}

float readAirDensity() {
    // Simplified density reading
    static float lastDensity = 1.225;
    
    if (bmp.performReading()) {
        float temp_k = bmp.temperature + 273.15;
        float pressure_pa = bmp.pressure;
        lastDensity = pressure_pa / (Specific_gas_constant * temp_k);
    }
    
    return lastDensity;
}
