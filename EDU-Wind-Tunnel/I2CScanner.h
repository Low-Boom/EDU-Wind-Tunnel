/*
 * I2CScanner.h
 * 
 * Multi-bus I2C scanner: scans Wire (and Wire1 where the board has it)
 * and remembers which bus each device was found on.
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
 * Buses scanned (from WIRE_INTERFACES_COUNT):
 * - Uno R4 WiFi: Wire1 (Qwiic connector), then Wire
 * - Uno R4 Minima: Wire only
 * - ESP32-P4: Wire only (the core does not define WIRE_INTERFACES_COUNT;
 *   the sketch starts Wire with the pins from PlatformConfig.h)
 * 
 * Early abort keeps scans of empty buses short:
 * - 25 ms timeout per I2C transaction
 * - The scan of a bus stops after 10 consecutive timeouts (no pull-ups)
 * 
 * On the Uno R4 WiFi, Wire1 is initialised and scanned before Wire.
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
 * Probe one address directly on every bus the scanner uses: Wire first,
 * then Wire1 where the board has it. For devices the scan does not cover,
 * e.g. the LWLP5000 at the general-call address 0x00.
 *
 * @param address I2C address to probe
 * @param busName Pointer to store the bus name (e.g., "Wire1")
 * @return Pointer to the first bus on which the address ACKs, or nullptr
 */
TwoWire* probeDeviceBus(uint8_t address, const char** busName = nullptr);

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
