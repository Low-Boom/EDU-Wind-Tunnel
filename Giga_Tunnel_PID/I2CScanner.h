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

// Maximum number of I2C devices that can be tracked
#define MAX_I2C_DEVICES 20

// Structure to store device location
struct I2CDeviceInfo {
    uint8_t address;
    TwoWire* bus;
    const char* busName;
};

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
 * Builds a registry of found devices and their bus locations that can be
 * queried using getDeviceBus().
 * 
 * Safe to call even if Wire.begin() already called with custom pins.
 * 
 * @param out Output stream for results (typically Serial)
 */
void scanAllI2CBuses(Stream &out);

/**
 * Find which I2C bus a device with the given address is on
 * 
 * Searches the device registry built by scanAllI2CBuses() to find
 * which bus the device was detected on.
 * 
 * @param address I2C address to search for (e.g., 0x28, 0x77)
 * @param busName Pointer to store the bus name (e.g., "Wire1")
 * @return Pointer to the TwoWire bus object, or nullptr if not found
 */
TwoWire* getDeviceBus(uint8_t address, const char** busName = nullptr);

/**
 * Get count of devices found during last scan
 * 
 * @return Number of I2C devices in the registry
 */
int getDeviceCount();

/**
 * Get device info by index
 * 
 * @param index Index in the device registry (0 to getDeviceCount()-1)
 * @return Pointer to device info, or nullptr if index out of range
 */
const I2CDeviceInfo* getDeviceInfo(int index);

#endif // I2C_SCANNER_H
