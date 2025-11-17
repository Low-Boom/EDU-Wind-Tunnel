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
        wire->setWireTimeout(25000, true); // 25ms timeout, reset on each call
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
    
    // On multi-bus boards, skip Wire scanning to prevent hangs on boards like Uno Rev4
    // where an empty Wire bus with no pull-ups can hang indefinitely
    // Single-bus boards (classic Uno, Mega) will still scan Wire normally
    #if WIRE_INTERFACES_COUNT > 1
        out.println("Scanning I2C bus Wire (skipped - use Wire1 for sensors on multi-bus boards)");
        out.println(" - Note: Wire scanning disabled on multi-bus boards to prevent hangs");
        out.println(" - Connect sensors to Wire1 (Qwiic connector) instead");
        out.println();
    #else
        // Single-bus boards: scan Wire normally
        scanSingleBus(&Wire, "Wire", out);
    #endif
    
    out.println("=====================================");
    out.println("[I2C] Scan complete");
    out.println("=====================================\n");
}