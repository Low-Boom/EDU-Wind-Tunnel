/*
 * I2CScanner.h
 * 
 * Multi-bus I2C scanner utility for Arduino platforms
 * Automatically detects and scans all available I2C buses (Wire, Wire1, Wire2)
 * based on WIRE_INTERFACES_COUNT macro.
 * 
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include <Arduino.h>
#include <Wire.h>

/**
 * Scan all available I2C buses and report found devices
 * 
 * Automatically detects number of I2C buses using WIRE_INTERFACES_COUNT:
 * - Classic Uno/Nano: Scans Wire only
 * - Arduino Uno Rev4: Scans Wire1, then Wire
 * - RP2040/ESP32/SAMD: Scans Wire1, then Wire
 * - Arduino GIGA (H7): Scans Wire1, Wire2, then Wire
 * 
 * Includes early abort logic to prevent long delays when scanning empty buses:
 * - Uses short timeout (1ms) for each I2C transaction
 * - Aborts scan after 10 consecutive timeouts (bus has no pull-ups)
 * - Typical scan time: <1 second for empty bus, <3 seconds for full scan
 * 
 * On multi-bus boards, Wire1 is scanned first to find sensors quickly,
 * then Wire is scanned last (may abort early if empty).
 * 
 * Initializes all available I2C buses before scanning.
 * 
 * For each address 1-126, performs beginTransmission() and checks
 * endTransmission() == 0 to detect device presence.
 * 
 * Safe to call even if Wire.begin() already called with custom pins.
 * 
 * @param out Output stream for results (typically Serial)
 */
void scanAllI2CBuses(Stream &out);

#endif // I2C_SCANNER_H
