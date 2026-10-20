/*
 * PlatformConfig.h
 *
 * Board-specific hardware settings for the EDU Wind Tunnel Controller.
 * Change the pins here if your wiring differs; the rest of the sketch
 * stays unchanged.
 *
 * Supported boards
 * ----------------
 *   ESP32-P4      (recommended; arduino-esp32 core 3.x, e.g. Waveshare ESP32-P4)
 *   Arduino Uno R4 Minima / Uno R4 WiFi
 * Any other board stops the build with an #error.
 *
 * Defines
 * -------
 *   BOARD_NAME            Shown in the start-up banner
 *   I2C_SDA_PIN/SCL_PIN   I2C pins for Wire (-1 = core default pins)
 *   FAN_PWM_PIN           PWM output to the fan
 *   FAN_PWM_FREQUENCY_HZ  Fan PWM frequency (0 = core default)
 *   STATUS_LED_PIN        LED that blinks on a fatal error (-1 = none)
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>

// ---- ESP32-P4 (e.g. Waveshare ESP32-P4-WiFi6-Touch-LCD-7B) ----
// Pins are kept at GPIO <= 36 to avoid LDO power issues on high-numbered
// pins. GPIO 14-19 carry the SDIO link to the on-board ESP32-C6 (WiFi):
// do NOT use them.
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)
  #define BOARD_NAME            "ESP32-P4"
  #define I2C_SDA_PIN           7       // same as the core's default SDA
  #define I2C_SCL_PIN           8       // same as the core's default SCL
  #define FAN_PWM_PIN           6
  // 25 kHz is the PWM frequency of 4-pin PC fans (Intel spec) and is above
  // human hearing; the core's 1 kHz default can make fans whine.
  #define FAN_PWM_FREQUENCY_HZ  25000

#elif defined(ARDUINO_ARCH_ESP32)
  #error "Unsupported ESP32 chip. EDU-Wind-Tunnel supports the ESP32-P4 and Arduino Uno R4 Minima / WiFi."

// ---- Arduino Uno R4 Minima / Uno R4 WiFi ----
// Wire uses the board's SDA/SCL header pins (A4/A5).
#elif defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
  #if defined(ARDUINO_UNOR4_WIFI)
    #define BOARD_NAME          "Arduino Uno R4 WiFi"
  #else
    #define BOARD_NAME          "Arduino Uno R4 Minima"
  #endif
  #define I2C_SDA_PIN           -1
  #define I2C_SCL_PIN           -1
  #define FAN_PWM_PIN           9
  #define FAN_PWM_FREQUENCY_HZ  0       // core default (490 Hz)

#else
  #error "Unsupported board. EDU-Wind-Tunnel supports the ESP32-P4 and Arduino Uno R4 Minima / WiFi."
#endif

// Status LED for fatal errors. The ESP32-P4 core defines no LED_BUILTIN.
#ifdef LED_BUILTIN
  #define STATUS_LED_PIN        LED_BUILTIN
#else
  #define STATUS_LED_PIN        -1
#endif

#endif // PLATFORM_CONFIG_H
