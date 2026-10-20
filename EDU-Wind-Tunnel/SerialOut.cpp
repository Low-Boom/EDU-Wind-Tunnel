/*
 * SerialOut.cpp
 *
 * Bounded serial output; see SerialOut.h.
 *
 * Copyright (c) 2025 Low-Boom
 * Licensed under MIT License
 */

#include "SerialOut.h"
#include <stdio.h>  // snprintf

SerialOutput SerialOut;

#if SERIAL_TX_CAN_BLOCK

size_t SerialOutput::write(const uint8_t *buffer, size_t size) {
    // After a loss, continue only once the host reads again, and say first
    // how much was lost.
    if (_lostBytes > 0) {
        if (Serial.availableForWrite() < SERIAL_RESUME_ROOM || !reportLostOutput()) {
            _lostBytes += size;
            return 0;
        }
    }

    size_t done = 0;
    unsigned long lastProgress = millis();
    while (done < size) {
        int room = Serial.availableForWrite();
        if (room > 0) {
            // Never more than fits: then Serial.write() does not wait.
            size_t chunk = ((size_t)room < size - done) ? (size_t)room : size - done;
            size_t sent = Serial.write(buffer + done, chunk);
            if (sent == 0) break;   // port not open (no host): the text is lost
            done += sent;
            lastProgress = millis();
        } else if (millis() - lastProgress >= SERIAL_STALL_TIMEOUT_MS) {
            break;                  // the host does not read
        }
    }
    _lostBytes += size - done;
    return done;
}

// Print the loss note. False if the port did not take it (still no host).
bool SerialOutput::reportLostOutput() {
    char note[80];
    // Starts with CR LF: the lost text can end in the middle of a line.
    int length = snprintf(note, sizeof(note),
                          "\r\n[WARN] %lu bytes of output lost: the host did not read\r\n",
                          _lostBytes);
    if (length <= 0 || length >= (int)sizeof(note)) return false;
    if (Serial.write((const uint8_t *)note, length) != (size_t)length) return false;
    _lostBytes = 0;
    return true;
}

#else

size_t SerialOutput::write(const uint8_t *buffer, size_t size) {
    return Serial.write(buffer, size);
}

#endif
