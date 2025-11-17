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
 * - Arduino Uno Rev4: Scans Wire1 only (Wire skipped due to hardware hang issue)
 * - RP2040/ESP32/SAMD: Scans Wire and Wire1
 * - Arduino GIGA (H7): Scans Wire, Wire1, and Wire2
 * 
 * Platform-specific handling:
 * - Uno Rev4: Wire bus scanning is disabled when Wire1 is available due to
 *   hardware limitation where empty Wire bus hangs indefinitely (no timeout)
 * - Other boards: All available buses are scanned normally
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
