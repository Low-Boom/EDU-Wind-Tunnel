/*
 * I2CScanner.cpp
 * 
 * Multi-bus I2C scanner utility for Arduino platforms
 * Automatically detects and scans all available I2C buses (Wire, Wire1, Wire2)
 * based on WIRE_INTERFACES_COUNT macro.
 * 
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include "I2CScanner.h"

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
    
    // Set a very short timeout to prevent long delays on empty buses
    // The timeout unit varies by platform, so we use a conservative value
    #if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_MBED)
        wire->setWireTimeout(1000, true); // 1ms timeout (or 1000us depending on platform)
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
    
    // Clear any timeout flags that may have been set during scanning
    #if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_MBED)
        wire->clearWireTimeoutFlag();
    #endif
    
    out.println();
}

void scanAllI2CBuses(Stream &out) {
    // Clear the device registry before scanning
    deviceCount = 0;
    
    out.println("\n=====================================");
    out.println("[I2C] Multi-Bus Scanner");
    out.println("=====================================");
    
    // Determine number of I2C buses available
    #if !defined(WIRE_INTERFACES_COUNT)
        // If not defined, assume single bus (classic Arduino boards)
        #define WIRE_INTERFACES_COUNT 1
        out.println("WIRE_INTERFACES_COUNT not defined, assuming 1 bus");
    #endif
    
    out.print("Available I2C buses: ");
    out.println(WIRE_INTERFACES_COUNT);
    out.println();
    
    // Initialize all I2C buses FIRST before scanning any of them
    // This prevents issues on boards like Uno Rev4 where an empty Wire bus
    // can hang during scanning if Wire1 hasn't been initialized yet
    #if WIRE_INTERFACES_COUNT > 1
        out.println("Initializing Wire1...");
        Wire1.begin();
    #endif
    
    #if WIRE_INTERFACES_COUNT > 2
        out.println("Initializing Wire2...");
        Wire2.begin();
    #endif
    
    // On multi-bus boards (like Uno Rev4), scan Wire1 FIRST
    // This works around issue where empty Wire bus can hang even with timeout
    #if WIRE_INTERFACES_COUNT > 1
        scanSingleBus(&Wire1, "Wire1", out);
    #endif
    
    // Scan Wire2 before Wire if available (Arduino GIGA H7)
    #if WIRE_INTERFACES_COUNT > 2
        scanSingleBus(&Wire2, "Wire2", out);
    #endif
    
    // Scan Wire bus (primary I2C bus)
    // On multi-bus boards, Wire is scanned last so Wire1/Wire2 are already found
    // Early abort logic in scanSingleBus prevents long delays on empty buses
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

int getDeviceCount() {
    return deviceCount;
}

const I2CDeviceInfo* getDeviceInfo(int index) {
    if (index >= 0 && index < deviceCount) {
        return &deviceRegistry[index];
    }
    return nullptr;
}