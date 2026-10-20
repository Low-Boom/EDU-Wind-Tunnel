/*
 * SerialOut.h
 *
 * Serial output that cannot stop loop() for long. All text output of the
 * sketch goes through SerialOut (SerialOut.print(...)) instead of Serial;
 * input still comes from Serial.
 *
 * Why: on a native USB serial port (SERIAL_TX_CAN_BLOCK, see
 * PlatformConfig.h) Serial.write() waits while the host keeps the port open
 * but does not read (paused logger, terminal in selection mode, script that
 * only writes). On the Uno R4 Minima it waits for good; on the ESP32-P4 up
 * to about 2 s per write (USB Serial/JTAG port), and one message is many
 * writes. loop() would stop there with the fan running, and with it the PID,
 * the stop command and the safety checks.
 *
 * How: SerialOut writes only as much as fits into the transmit buffer and
 * waits for more room only while the host reads. After
 * SERIAL_STALL_TIMEOUT_MS without progress the rest of the text is lost,
 * and so is all later text until the host has read the buffer down to
 * SERIAL_RESUME_ROOM free bytes. The next text then starts with a
 * "[WARN] ... bytes of output lost" line.
 *
 * Routine lines (data lines, relay status) must not be cut off, so they
 * are printed only if they fit completely (serialHasRoom()) and are skipped
 * otherwise.
 *
 * Where printing never blocks (UART), SerialOut passes everything to Serial.
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#ifndef SERIAL_OUT_H
#define SERIAL_OUT_H

#include <Arduino.h>
#include "PlatformConfig.h"     // SERIAL_TX_CAN_BLOCK

// Longest wait for the host to read; once per stall, not per call
const unsigned long SERIAL_STALL_TIMEOUT_MS = 50;
// Free bytes that show the host reads again. The smallest transmit buffer
// is 256 bytes (Uno R4 Minima, ESP32-P4 USB Serial/JTAG).
const int SERIAL_RESUME_ROOM = 128;

class SerialOutput : public Print {
public:
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buffer, size_t size) override;
    int availableForWrite() override { return Serial.availableForWrite(); }
    using Print::write;

private:
#if SERIAL_TX_CAN_BLOCK
    bool reportLostOutput();
    unsigned long _lostBytes = 0;   // > 0: output was lost, wait for the host
#endif
};

extern SerialOutput SerialOut;

// True if a line of 'chars' characters (incl. CR LF) fits into the transmit
// buffer now, so it is printed in full. Always true where printing never
// blocks.
inline bool serialHasRoom(int chars) {
#if SERIAL_TX_CAN_BLOCK
    return Serial.availableForWrite() >= chars;
#else
    (void)chars;
    return true;
#endif
}

#endif // SERIAL_OUT_H
