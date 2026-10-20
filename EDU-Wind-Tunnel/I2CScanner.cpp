/*
 * I2CScanner.cpp
 * 
 * Multi-bus I2C scanner: scans Wire (and Wire1 where the board has it)
 * and remembers which bus each device was found on.
 * 
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include "I2CScanner.h"

// Number of hardware I2C buses to scan. Uno R4 defines WIRE_INTERFACES_COUNT
// (Minima: 1, WiFi: 2). The ESP32 core does not, and the sketch only starts
// Wire there (Wire1 has no default pins), so fall back to one bus.
#ifndef WIRE_INTERFACES_COUNT
#define WIRE_INTERFACES_COUNT 1
#endif

// Per-transaction timeout used while scanning, so an empty bus or a bus
// without pull-ups cannot stall the scan for long.
const uint16_t SCAN_TIMEOUT_MS = 25;

// Global device registry
static I2CDeviceInfo deviceRegistry[MAX_I2C_DEVICES];
static int deviceCount = 0;

// Helper function to add a device to the registry
static void registerDevice(uint8_t address, TwoWire* bus, const char* busName) {
    if (deviceCount < MAX_I2C_DEVICES) {
        deviceRegistry[deviceCount].address = address;
        deviceRegistry[deviceCount].bus = bus;
        deviceRegistry[deviceCount].busName = busName;
        deviceCount++;
    }
}

// Helper function to scan a single I2C bus
static void scanSingleBus(TwoWire *wire, const char* busName, Stream &out) {
    out.print("Scanning I2C bus ");
    out.println(busName);
    
    // Short timeout so an empty bus aborts quickly. The API differs per core:
    // ESP32 takes milliseconds, the Uno R4 (Renesas) core microseconds.
    // endTransmission() returns 5 on timeout on both.
    #if defined(ARDUINO_ARCH_ESP32)
        wire->setTimeOut(SCAN_TIMEOUT_MS);
    #else
        wire->setWireTimeout(SCAN_TIMEOUT_MS * 1000UL);
    #endif
    
    int devicesFound = 0;
    int consecutiveTimeouts = 0;
    const int MAX_CONSECUTIVE_TIMEOUTS = 10; // Abort after 10 consecutive timeouts
    
    // Scan addresses 1-126 (0x01 to 0x7E)
    // Address 0 is reserved for general call
    // Addresses 127 are reserved
    for (uint8_t addr = 1; addr < 127; addr++) {
        wire->beginTransmission(addr);
        // Use sendStop=true for proper I2C bus release, prevents hanging
        uint8_t error = wire->endTransmission(true);
        
        if (error == 0) {
            // Device found - add to registry
            registerDevice(addr, wire, busName);
            
            out.print(" - Found device at 0x");
            if (addr < 16) {
                out.print("0");
            }
            out.println(addr, HEX);
            devicesFound++;
            consecutiveTimeouts = 0; // Reset timeout counter on success
        } else if (error == 5) {
            // Error 5 is timeout - increment counter
            consecutiveTimeouts++;
            if (consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
                out.print(" - Scan aborted at address 0x");
                if (addr < 16) {
                    out.print("0");
                }
                out.print(addr, HEX);
                out.println(" (too many timeouts - bus likely has no pull-ups)");
                break;
            }
        } else {
            // Other errors (NACK, etc.) - reset timeout counter
            consecutiveTimeouts = 0;
        }
        // Note: error codes: 0=success, 1=data too long, 2=NACK on address, 
        //                    3=NACK on data, 4=other error, 5=timeout
        
        // Small delay between scans to prevent bus lockup
        delayMicroseconds(100);
    }
    
    if (devicesFound == 0) {
        out.print(" - No I2C devices found on ");
        out.println(busName);
    } else {
        out.print(" - Total devices found on ");
        out.print(busName);
        out.print(": ");
        out.println(devicesFound);
    }
    
    out.println();
}

void scanAllI2CBuses(Stream &out) {
    // Clear the device registry before scanning
    deviceCount = 0;
    
    out.println("\n=====================================");
    out.println("[I2C] Multi-Bus Scanner");
    out.println("=====================================");
    
    out.print("Available I2C buses: ");
    out.println(WIRE_INTERFACES_COUNT);
    out.println();
    
    // Uno R4 WiFi: Wire1 is the Qwiic connector. Initialise and scan it
    // first, because an empty Wire bus can hang if Wire1 is not yet set up.
    #if WIRE_INTERFACES_COUNT > 1
        out.println("Initializing Wire1...");
        Wire1.begin();
        scanSingleBus(&Wire1, "Wire1", out);
    #endif
    
    // Scan Wire (the main bus) last; the early abort in scanSingleBus keeps
    // an empty bus from taking long.
    scanSingleBus(&Wire, "Wire", out);
    
    out.println("=====================================");
    out.println("[I2C] Scan complete");
    out.println("=====================================\n");
}

// Public helper functions to query the device registry

TwoWire* getDeviceBus(uint8_t address, const char** busName) {
    for (int i = 0; i < deviceCount; i++) {
        if (deviceRegistry[i].address == address) {
            if (busName != nullptr) {
                *busName = deviceRegistry[i].busName;
            }
            return deviceRegistry[i].bus;
        }
    }
    return nullptr; // Device not found
}

// beginTransmission() + endTransmission() == 0: a device ACKed the address.
static bool deviceAcks(TwoWire *wire, uint8_t address) {
    wire->beginTransmission(address);
    return wire->endTransmission(true) == 0;
}

TwoWire* probeDeviceBus(uint8_t address, const char** busName) {
    if (deviceAcks(&Wire, address)) {
        if (busName != nullptr) *busName = "Wire";
        return &Wire;
    }
    #if WIRE_INTERFACES_COUNT > 1
        if (deviceAcks(&Wire1, address)) {
            if (busName != nullptr) *busName = "Wire1";
            return &Wire1;
        }
    #endif
    return nullptr;
}

int getDeviceCount() {
    return deviceCount;
}

const I2CDeviceInfo* getDeviceInfo(int index) {
    if (index >= 0 && index < deviceCount) {
        return &deviceRegistry[index];
    }
    return nullptr;
}