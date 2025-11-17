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

// Helper function to scan a single I2C bus
static void scanSingleBus(TwoWire *wire, const char* busName, Stream &out) {
    out.print("Scanning I2C bus ");
    out.println(busName);
    
    // Set timeout to prevent hanging when no devices are connected
    // Modern Arduino Wire libraries support this (GIGA R1, Uno Rev4, etc.)
    #if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_MBED)
        wire->setWireTimeout(250, true); // 25ms timeout, reset on each call
    #endif
    
    int devicesFound = 0;
    
    // Scan addresses 1-126 (0x01 to 0x7E)
    // Address 0 is reserved for general call
    // Addresses 127 are reserved
    for (uint8_t addr = 1; addr < 127; addr++) {
        wire->beginTransmission(addr);
        // Use sendStop=true for proper I2C bus release, prevents hanging
        uint8_t error = wire->endTransmission(true);
        
        if (error == 0) {
            // Device found
            out.print(" - Found device at 0x");
            if (addr < 16) {
                out.print("0");
            }
            out.println(addr, HEX);
            devicesFound++;
        }
        // Note: error == 4 means unknown error, other values are timeouts/NACK
        
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
    
    // Always scan Wire (primary bus)
    // Note: Wire should already be initialized by caller
    scanSingleBus(&Wire, "Wire", out);
    
    // Scan Wire1 if available (many modern boards)
    #if WIRE_INTERFACES_COUNT > 1
        // Initialize Wire1 before scanning to prevent hanging
        Wire1.begin();
        scanSingleBus(&Wire1, "Wire1", out);
    #endif
    
    // Scan Wire2 if available (Arduino GIGA H7)
    #if WIRE_INTERFACES_COUNT > 2
        // Initialize Wire2 before scanning to prevent hanging
        Wire2.begin();
        scanSingleBus(&Wire2, "Wire2", out);
    #endif
    
    out.println("=====================================");
    out.println("[I2C] Scan complete");
    out.println("=====================================\n");
}