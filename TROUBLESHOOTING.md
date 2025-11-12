# Troubleshooting

---

## 1. Serial / Upload issues

- Ensure correct board selected: Tools → Board → "Arduino Giga R1".
- Use a data-capable USB cable.
- Serial Monitor must be set to **115200** baud and newline mode for commands.

---

## 2. I2C / Sensor issues

Symptom: "Cannot communicate with pressure sensor" or "BMP3 not found"

Checklist:
- Power: verify 3.3V present at sensor VCC.
- Wiring: verify SDA ↔ SDA and SCL ↔ SCL, GND common.
- I2C addresses: MS4525DO is fixed at 0x28; BMP3XX may be 0x77 or 0x76. The code probes both.
- Use an I2C scanner sketch to list bus devices.
- If raw readings are zero or stuck, check tubing (no leaks, correct ports) and connections.
- Different manufacuters flip the dynamic and static ports on the airspeed differential pressure sensor. If you keep getting zero readings or no response from the MS4525DO, swap the pneumatic connections.

Recalibration:
- Run `recal` or `recal <N>` from serial to re-establish zero-offset (ensure no airflow).

---

## 3. Fan or Motor Not Responding

Symptom: Motor doesn't spin or behaves unexpectedly.

Checklist:
- Verify The AC Fan or 12V Fan has separate power and is plugged in.
- Ensure signal ground and Arduino GND are securely connected.
- If you continue to experience issues, test PWM output in isolation and refer to HARDWARE.md:
  - Upload a minimal sketch that calls `analogWrite(9, value)` and verify voltage changes with a meter/scope.

---

## 4. Auto-tune failures or poor gains

Symptom: Auto-tune times out, never oscillates, or yields very low gains.

Causes & fixes:
- PWM range inappropriate: choose manual range `tune <low> <high>` with a delta ~50–80.
- Averaging too high: reduce averaging before tuning `avg 3`.
- Target unreachable: set a lower target airspeed then tune.
- Motor/ESC not responding correctly: verify manual `analogWrite()` tests.

If output gains are too conservative after tuning, try narrower PWM delta and re-run.

---

## 5. Noisy readings / chattering

Symptom: Airspeed readouts jump; relay chattering during tuning.

Fixes:
- Increase averaging: `avg 25` to smooth readings at cost of lag.
- Increase `relayHysteresis` compile-time constant if chattering persists (requires re-upload).
- Add hardware damping: secure tubing, add foam mount for sensors.

---

## 6. Performance tuning tips

- Faster response: lower `pressureAverageSamples` and `PRESSURE_OVERSAMPLES`, increase Kp/Ki/Kd carefully.
- More stability: increase averaging, reduce Kp and Ki, raise Kd slightly.
- Avoid integral windup: keep Ki moderate; code has conditional anti-windup but large Ki can still cause overshoot.

---

## 7. Debugging steps & tools

- Use Serial Monitor to capture full startup logs and tuning diagnostics — they contain state transitions and measured cycle data.
- Use an I2C scanner to confirm sensor addresses.
- Use a multimeter or oscilloscope to verify PWM on pin 9 (expected ~0–3.3V duty cycle).
- Log CSV data (modify `Serial.print` lines) for offline analysis.

---

## 8. When to seek help

Open an issue including:
- Serial log output (copy/paste)
- Hardware used (board, sensors, motor)
- Commands issued and their timings
- What you observed vs expected

Link issues to this repository: https://github.com/Low-Boom/EDU-Wind-Tunnel/issues

---
