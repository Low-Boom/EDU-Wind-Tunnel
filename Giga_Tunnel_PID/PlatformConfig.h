/*
 * PlatformConfig.h
 *
 * Platform-specific hardware configuration for the EDU Wind Tunnel Controller.
 * Provides sensible pin defaults and hardware settings for all supported boards.
 *
 * Edit DEFAULT_PWM_PIN / DEFAULT_TACH_PIN here if your wiring differs from the
 * defaults listed below.  All other sketch code stays unchanged.
 *
 * Supported platforms
 * -------------------
 *   Classic Arduino (AVR)    : Uno, Nano, Mega 2560
 *   Arduino MBED             : Uno R4, Giga R1
 *   ESP32-PICO (M5 Atom Stack) : M5 Atom Lite / M5 Atom Matrix
 *   ESP32-S3                 : Generic dev-boards
 *   ESP32-P4                 : Waveshare ESP32-P4 and other P4 dev-boards
 *   Generic ESP32            : Any other ESP32 board
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>

// ============================================================
// ESP32 family  (ESP32-PICO, ESP32-S3, ESP32-P4, generic ESP32)
// Requires ESP32 Arduino core 2.0 or later (3.x for ESP32-P4).
// ============================================================
#if defined(ARDUINO_ARCH_ESP32)

  // ---- M5 Atom Stack / M5 Atom Lite (ESP32-PICO-D4) ----
  // Sensors connect to the Grove / HY2.0 bottom connector.
  // Grove pins: SDA = GPIO 26, SCL = GPIO 32
  #if defined(ARDUINO_M5Stack_ATOM_LITE) || defined(ARDUINO_M5Stack_ATOM) || \
      defined(ARDUINO_M5Atom)             || defined(ARDUINO_M5ATOM_LITE)
    #define I2C_SDA_PIN       26   // Grove SDA
    #define I2C_SCL_PIN       32   // Grove SCL
    #define DEFAULT_PWM_PIN   19   // Available GPIO (e.g. HAT header)
    #define DEFAULT_TACH_PIN  23   // Available GPIO (e.g. HAT header)

  // ---- Generic ESP32-S3 dev-board ----
  // Adjust I2C and PWM pins to match your wiring.
  #elif defined(CONFIG_IDF_TARGET_ESP32S3)
    #define I2C_SDA_PIN        8
    #define I2C_SCL_PIN        9
    #define DEFAULT_PWM_PIN   10
    #define DEFAULT_TACH_PIN  11

  // ---- ESP32-P4 (e.g. Waveshare ESP32-P4-WiFi6-Touch-LCD-7B) ----
  // Uses GPIO ≤ 36 to avoid LDO power issues on high-numbered pins.
  // WiFi is provided by an on-board ESP32-C6; GPIOs 14-19 are reserved for
  // the SDIO host interface to that companion chip — do NOT reassign them.
  #elif defined(CONFIG_IDF_TARGET_ESP32P4)
    #define I2C_SDA_PIN        7
    #define I2C_SCL_PIN        8
    #define DEFAULT_PWM_PIN    6
    #define DEFAULT_TACH_PIN   5

  // ---- Generic ESP32 (ESP32-WROOM, ESP32-WROVER, etc.) ----
  #else
    #define I2C_SDA_PIN       21
    #define I2C_SCL_PIN       22
    #define DEFAULT_PWM_PIN    9
    #define DEFAULT_TACH_PIN   2
  #endif

  // LED_BUILTIN fallback: many ESP32 boards don't define this in all BSPs.
  // Standard ESP32 DevKit uses GPIO 2; M5 Atom has an RGB LED on GPIO 27
  // (not a simple digital output, so we fall back to a safe GPIO here).
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 2
  #endif

// ============================================================
// Classic Arduino / MBED (Uno, Mega, Giga R1, Uno R4, etc.)
// Wire.begin() with no arguments picks up the board-defined SDA/SCL.
// ============================================================
#else
  #define I2C_SDA_PIN       -1   // -1 = use board default (no args to Wire.begin)
  #define I2C_SCL_PIN       -1
  #define DEFAULT_PWM_PIN    9
  #define DEFAULT_TACH_PIN   2
#endif

#endif // PLATFORM_CONFIG_H
