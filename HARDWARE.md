# Hardware Configuration Guide

## Hardware Modifications to Educational Wind Tunnel
A static pressure ring must be added at the inlet of the test section to measure airspeed.
An updatead version of the test section or an additional component between the inlet and test section are included for this purpose. Get them from Printables at LINK LINK LINK.

## Minimum Required Components

- 3D Printed Educational Wind Tunnel with Static Pressure Ring
- PWM Controlled Fan
- Arduino Giga R1 WiFi (or compatible MBED-based board)
- MS4525DO differential pressure sensor (I2C address 0x28)
- BMP3XX barometric sensor (BMP388/BMP390) (I2C address 0x77 or 0x76)
- PWM-capable motor or fan
- Silicone tubing for pitot/static ports
- Common jumper wiring, 3.3V supply for sensors

---

## Arduino Pin mapping (defaults used by sketch)

- PWM output: Arduino digital pin 9 → PWM signal input
- PWM Tachometer input: Arduino digital pin 2 (optional)
- I2C: SDA / SCL → BMP3XX and MS4525DO (shared bus via Qwiic Daisy Chain)
- Sensors power: 5V or 3.3V and GND from Arduino
- Serial: USB / Serial Monitor at 115200 baud

---

## Pneumatic Tubing
   - MS4525DO high-pressure port ← Exposed to atmosphere away from the tunnel inlet or outlet
   - MS4525DO low-pressure port ← Test section static pressure ring

---

## Power recommendations & safety
- Always common the Arduino ground with PWM signal ground.
- Do not route I2C or sensor wires alongside high-current motor wires to reduce EMI.
- Start with small airspeed setpoint (2–5 m/s) and confirm safe behaviour.
---

## Suggested Components
(ADD LINKS HERE)
- Arduino Giga R1 WiFi — 1
- MS4525DO differential pressure — 1
- Adafruit BMP388/BMP390 Qwiic Breakout — 1
- BLDC motor (size per tunnel) — 1
- ESC (matching motor) — 1
- Silicone tubing, connectors, mounting hardware — assorted

---
For extended diagnostics see TROUBLESHOOTING.md.

---
