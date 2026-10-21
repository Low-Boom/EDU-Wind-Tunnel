/*
 * FanTach.h
 *
 * Fan speed (RPM) from the tachometer wire of a 3- or 4-wire fan.
 *
 * The tach output is open collector: it pulls the line low a fixed number
 * of times per revolution (2 for PC fans, Intel 4-wire specification). The
 * input uses the board's internal pull-up, so the line idles at the logic
 * voltage of the board. An interrupt counts the falling edges; once per
 * update fanTachUpdate() turns them into RPM.
 *
 * The tach wire is optional. Without it no pulses arrive, fanTachDetected()
 * stays false, and the sketch uses no tach function.
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef FAN_TACH_H
#define FAN_TACH_H

#include <Arduino.h>

// Edges closer together than this are noise (PWM crosstalk on the tach
// wire), not pulses: 2 ms is 15000 RPM at 2 pulses per revolution.
const uint32_t FAN_TACH_MIN_PULSE_US = 2000;
// No pulse for this long: the fan stands still (30 RPM at 2 pulses/rev).
const uint32_t FAN_TACH_TIMEOUT_US = 1000000;

/**
 * Start the tach input.
 * @param pin           Interrupt-capable input pin; -1 = no tach (all
 *                      other functions then do nothing)
 * @param pulsesPerRev  Tach pulses per fan revolution
 */
void fanTachBegin(int pin, int pulsesPerRev);

/** Turn the pulses since the last call into RPM. Call once per update. */
void fanTachUpdate();

/** True once tach pulses have arrived since start-up (the wire is connected). */
bool fanTachDetected();

/**
 * Fan speed from the last fanTachUpdate(), in RPM. 0 if no pulse came for
 * FAN_TACH_TIMEOUT_US (or none came yet).
 */
float fanTachRpm();

#endif // FAN_TACH_H
