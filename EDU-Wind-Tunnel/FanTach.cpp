/*
 * FanTach.cpp
 *
 * Fan tachometer; see FanTach.h.
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include "FanTach.h"

// The ISR must be in IRAM on the ESP32 (it can run while flash is busy).
#if defined(ARDUINO_ARCH_ESP32)
  #define TACH_ISR_ATTR IRAM_ATTR
#else
  #define TACH_ISR_ATTR
#endif

static int tachPin = -1;
static int tachPulsesPerRev = 2;

// Written by the ISR
static volatile uint32_t isrPulses = 0;       // valid edges since the last update
static volatile uint32_t isrLastEdgeUs = 0;   // time of the last valid edge
static volatile bool isrSeen = false;         // any valid edge since start-up

// Used by fanTachUpdate() only
static bool detected = false;
static bool haveRefEdge = false;   // refEdgeUs is a recent pulse to measure from
static uint32_t refEdgeUs = 0;     // last edge of the previous update
static float rpm = 0.0;

static void TACH_ISR_ATTR onTachEdge() {
    uint32_t now = micros();
    if (isrSeen && now - isrLastEdgeUs < FAN_TACH_MIN_PULSE_US) return;   // noise
    isrLastEdgeUs = now;
    isrSeen = true;
    isrPulses = isrPulses + 1;   // not ++: deprecated on volatile in C++20
}

void fanTachBegin(int pin, int pulsesPerRev) {
    tachPin = pin;
    tachPulsesPerRev = (pulsesPerRev > 0) ? pulsesPerRev : 2;
    if (tachPin < 0) return;
    pinMode(tachPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(tachPin), onTachEdge, FALLING);
}

// RPM from all pulse intervals that ended in this update: from the last
// edge of the previous update to the last edge of this one. A 200 ms window
// holds only a few pulses at low speed, so counting edges per window alone
// would give coarse steps (150 RPM at 2 pulses/rev).
void fanTachUpdate() {
    if (tachPin < 0) return;
    // The ISR runs on the same core as loop() (ESP32: the core that called
    // attachInterrupt()), so masking interrupts here is enough.
    noInterrupts();
    uint32_t pulses = isrPulses;
    isrPulses = 0;
    uint32_t lastEdgeUs = isrLastEdgeUs;
    bool seen = isrSeen;
    interrupts();
    if (!seen) return;
    detected = true;

    if (pulses > 0) {
        uint32_t span = lastEdgeUs - refEdgeUs;
        if (haveRefEdge && span > 0 && span <= FAN_TACH_TIMEOUT_US * pulses) {
            rpm = pulses * 60.0e6 / ((float)tachPulsesPerRev * span);
        }
        // else: first pulses after a standstill; the next update has an interval
        refEdgeUs = lastEdgeUs;
        haveRefEdge = true;
    } else if (micros() - lastEdgeUs > FAN_TACH_TIMEOUT_US) {
        rpm = 0.0;
        haveRefEdge = false;
    }
}

bool fanTachDetected() {
    return detected;
}

float fanTachRpm() {
    return rpm;
}
