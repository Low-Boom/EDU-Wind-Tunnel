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
 *   I2C_SDA_PIN/SCL_PIN   I2C pins for Wire (-1 = the board's default pins)
 *   FAN_PWM_PIN           PWM output to the fan
 *   FAN_PWM_FREQUENCY_HZ  Fan PWM frequency (0 = core default)
 *   SERIAL_TX_CAN_BLOCK   1 if Serial.print() can block while the host does
 *                         not read (native USB serial); see serialHasRoom()
 *   STATUS_LED_PIN        LED that blinks on a fatal error (-1 = none)
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>

// ---- ESP32-P4 (e.g. Waveshare ESP32-P4-WiFi6-Touch-LCD-7B) ----
// The fan pin is a low GPIO (<= 36), away from the high GPIOs that some
// boards power from a separate (LDO) supply. A board's own defaults may still
// use high pins, e.g. I2C on the M5Stack Tab5 (53/54, see below).
// GPIO 14-19 carry the SDIO link to the on-board ESP32-C6 (WiFi): do NOT
// use them.
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)
  #define BOARD_NAME            "ESP32-P4"
  // -1 = the default I2C pins of the board selected in the IDE (SDA/SCL in
  // the core's pins_arduino.h): GPIO 7/8 on the ESP32P4 Dev Module and the
  // Waveshare boards, 53/54 (Grove Port A) on the M5Stack Tab5. Put GPIO
  // numbers here only if your sensors are wired to other pins.
  #define I2C_SDA_PIN           -1
  #define I2C_SCL_PIN           -1
  #define FAN_PWM_PIN           6
  // 25 kHz is the PWM frequency of 4-pin PC fans (Intel spec) and is above
  // human hearing; the core's 1 kHz default can make fans whine.
  #define FAN_PWM_FREQUENCY_HZ  25000
  // Serial is UART0 by default and always drains. With "USB CDC On Boot:
  // Enabled" it is the chip's own USB port, which waits for the host.
  #if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    #define SERIAL_TX_CAN_BLOCK 1
  #else
    #define SERIAL_TX_CAN_BLOCK 0
  #endif

#elif defined(ARDUINO_ARCH_ESP32)
  #error "Unsupported ESP32 chip. EDU-Wind-Tunnel supports the ESP32-P4 and Arduino Uno R4 Minima / WiFi."

// ---- Arduino Uno R4 Minima / Uno R4 WiFi ----
// The I2C pins are fixed by the board: Wire is the SDA/SCL header (A4/A5),
// Wire1 the Qwiic connector (WiFi only). The core has no Wire.begin(sda, scl).
#elif defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
  #if defined(ARDUINO_UNOR4_WIFI)
    #define BOARD_NAME          "Arduino Uno R4 WiFi"
    #define SERIAL_TX_CAN_BLOCK 0       // UART to the on-board USB bridge
  #else
    #define BOARD_NAME          "Arduino Uno R4 Minima"
    #define SERIAL_TX_CAN_BLOCK 1       // native USB serial
  #endif
  #define I2C_SDA_PIN           -1      // fixed, leave at -1
  #define I2C_SCL_PIN           -1      // fixed, leave at -1
  #if I2C_SDA_PIN != -1 || I2C_SCL_PIN != -1
    #error "Uno R4: the I2C pins are fixed by the board; leave I2C_SDA_PIN and I2C_SCL_PIN at -1."
  #endif
  #define FAN_PWM_PIN           9
  #define FAN_PWM_FREQUENCY_HZ  0       // core default (490 Hz)

#else
  #error "Unsupported board. EDU-Wind-Tunnel supports the ESP32-P4 and Arduino Uno R4 Minima / WiFi."
#endif

// Status LED for fatal errors: the board's LED_BUILTIN if its core variant
// defines one (Uno R4 and some ESP32-P4 boards), otherwise none.
#ifdef LED_BUILTIN
  #define STATUS_LED_PIN        LED_BUILTIN
#else
  #define STATUS_LED_PIN        -1
#endif

// A native USB serial port (SERIAL_TX_CAN_BLOCK) makes Serial.print() wait
// while the host keeps the port open but does not read (paused logger,
// terminal in selection mode). loop() would freeze there, and with it the
// stop command and the safety checks. So routine output (data lines, status
// lines) is only printed if it fits into the transmit buffer and is skipped
// otherwise; essential messages are printed anyway, but only after the fan
// is in a safe state. Always true where printing never blocks (UART).
inline bool serialHasRoom(int chars) {
#if SERIAL_TX_CAN_BLOCK
  return Serial.availableForWrite() >= chars;
#else
  (void)chars;
  return true;
#endif
}

#endif // PLATFORM_CONFIG_H
