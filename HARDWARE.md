# Hardware Configuration Guide — v1.0

Concise wiring, component guidance, and safety notes for the Wind Tunnel Controller.

---

## Required components (minimum)

- Arduino Giga R1 WiFi (or compatible MBED-based board)
- MS4525DO differential pressure sensor (I2C address 0x28)
- BMP3XX barometric sensor (BMP388/BMP390) (I2C address 0x77 or 0x76)
- PWM-capable ESC and brushless motor sized for your tunnel
- Silicone tubing for pitot/static ports
- Common jumper wiring, 3.3V supply for sensors

Optional:
- Tachometer sensor (Hall or optical) → pin 2 (interrupt). Note: tachometer is read but not used in control.

---

## Pin mapping (defaults used by sketch)

- PWM output: Arduino digital pin 9 → ESC signal input
- Tachometer input: Arduino digital pin 2 (optional)
- I2C: SDA / SCL → MS4525DO and BMP3XX (shared bus)
- Sensors power: 3.3V and GND from Arduino
- Serial: USB / Serial Monitor at 115200 baud

---

## Wiring summary

1. I2C bus
   - MS4525DO VCC → 3.3V
   - MS4525DO GND → GND
   - MS4525DO SDA → SDA
   - MS4525DO SCL → SCL
   - BMP3XX VCC → 3.3V (or VIN if breakout supports 5V)
   - BMP3XX GND → GND
   - BMP3XX SDA → SDA
   - BMP3XX SCL → SCL

2. ESC / Motor
   - ESC signal wire → Arduino pin 9
   - ESC signal ground → Arduino GND (signal reference)
   - ESC power → separate motor battery / PSU (+ and -)
   - Motor phases → ESC motor outputs

3. Pitot tubing
   - MS4525DO high-pressure port ← pitot forward-facing port
   - MS4525DO low-pressure port ← static port

---

## Power recommendations & safety

- Arduino and sensors: USB or regulated 5–7V VIN. Sensors use 3.3V.
- Motor/ESC: Separate supply sized for motor current; include proper fusing.
- Always common the Arduino ground with ESC signal ground.
- Do not route I2C or sensor wires alongside high-current motor wires to reduce EMI.
- Use decoupling capacitors on sensor power rails (0.1µF + 10µF recommended).

---

## ESC notes

- Many hobby ESCs expect servo-style pulses; Arduino `analogWrite()` on pin 9 at ~490 Hz works with most ESCs, but some require 50Hz. If the ESC does not respond, check ESC documentation.
- Some ESCs require calibration of min/max PWM—do this with the ESC manual or a servo tester before running auto-tune.
- If motor spins in the wrong direction, swap any two motor phase wires.

---

## Recommended test procedure after wiring

1. Power Arduino only. Verify sensors are detected in Serial Monitor messages.
2. With motor PSU disconnected, verify `analogWrite()` behaviour on pin 9 with a multimeter or scope.
3. Connect motor PSU, verify ESC beeps and responds to static `analogWrite()` commands at low PWM before running auto-tune.
4. Start with small airspeed setpoint (2–5 m/s) and confirm safe behaviour.

---

## BOM (suggested)

- Arduino Giga R1 WiFi — 1
- MS4525DO differential pressure — 1
- BMP388/BMP390 breakout — 1
- BLDC motor (size per tunnel) — 1
- ESC (matching motor) — 1
- Silicone tubing, connectors, mounting hardware — assorted

---

## Short troubleshooting pointers

- If MS4525DO not found: check 3.3V, wiring, and run I2C scanner.
- If BMP3XX not found: check address (0x76/0x77) and wiring.
- If ESC not responding: verify common ground, ESC arm sequence, and PWM test.

For extended diagnostics see TROUBLESHOOTING.md.

---
