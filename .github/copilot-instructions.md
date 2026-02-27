# Copilot Instructions for EDU-Wind-Tunnel

## Project Overview

This repository contains an Arduino-based PID controller for educational wind tunnel systems. The controller targets Arduino MBED-compatible boards (Giga R1, Mega 2560, Uno R4) and uses a Pitot-static airspeed sensor (MS4525DO) and barometric sensor (BMP3XX) to regulate tunnel airspeed via closed-loop PID control.

## Repository Structure

```
Giga_Tunnel_PID/          # Main Arduino sketch folder
  Giga_Tunnel_PID.ino     # Main sketch: PID loop, relay auto-tune, serial commands
  I2CScanner.h/.cpp       # Multi-bus I2C scanner utility
  STunePIDTuner.h/.cpp    # sTune-based advanced auto-tuner wrapper
3D Print Files/           # STL files for hardware components
img/                      # Wiring diagrams and hardware photos
HARDWARE.md               # Wiring, components, and power supply details
TUNING_GUIDE.md           # PID tuning techniques
README.md                 # Project overview and quick-start guide
```

## Language and Platform

- **Language**: C++ (Arduino sketch dialect)
- **Platform**: Arduino (MBED-compatible boards)
- **IDE**: Arduino IDE 2.0+
- **Build**: Use Arduino IDE or `arduino-cli` to compile and upload sketches
- **No automated test framework** is currently configured; validate changes by uploading to hardware and observing Serial Monitor output at 115200 baud

## Key Libraries

| Library | Purpose |
|---|---|
| `QuickPID` | PID controller |
| `Adafruit_BMP3XX` | Barometric pressure/temperature sensor |
| `ms4525do` (Bolder Flight Systems) | Differential pressure / airspeed sensor |
| `sTune` | Advanced inflection-point auto-tuner (optional) |
| `Wire` | I2C communication |

## Architecture and Conventions

- **Main loop** runs on a fixed `updateInterval` (default 200 ms); avoid blocking calls longer than this interval.
- **Sensor reads** use oversampling (`PRESSURE_OVERSAMPLES`) and a rolling average (`pressureBuffer`) to reduce noise.
- **PID output** is PWM-rate-limited by `MAX_PWM_CHANGE_PER_CYCLE` to smooth motor transitions; disable by setting this to 255 during auto-tune.
- **Air density** is corrected at runtime from BMP3XX temperature/pressure readings.
- **I2C auto-discovery** (`I2CScanner`) detects sensor buses at startup; do not hard-code bus assignments.
- **Serial commands** are single-line strings parsed in the main loop; keep the parser logic inside the sketch's `handleSerialInput()` function.
- **Board-conditional code** uses `#if defined(...)` guards (e.g., `ARDUINO_GIGA`, `ARDUINO_AVR_MEGA2560`); add new board support with matching guards.
- **Compile-time constants** (pin assignments, limits, tuning parameters) are defined near the top of `Giga_Tunnel_PID.ino`; runtime-configurable values use `int`/`float` variables that can be changed via serial commands.

## Code Style

- Follow existing Arduino/C++ style in the file being modified: `camelCase` for variables and functions, `UPPER_SNAKE_CASE` for `#define` macros and constants.
- Add Doxygen-style comments (`/** ... */`) for new public functions declared in header files; match the style in `I2CScanner.h`.
- Keep `#include` lists grouped: Arduino core → sensor libraries → local headers.
- Prefer `const` variables over `#define` for typed constants.

## Safety Rules

- **Never power the motor/fan from the Arduino** — always use a separate power supply.
- Serial command `0` (or disconnecting motor power) must always be available as an emergency stop; do not break this behavior.
- Serial setpoints are validated against `MAX_AIRSPEED` (default 30.0 m/s) before being applied; always enforce this bound when modifying setpoint handling.
- The zero-offset calibration routine (`pressureOffset`) runs at startup and on `recal`; any changes to calibration must not allow uncalibrated sensor data to drive the PID loop.

## Common Tasks

- **Add a new serial command**: Parse the new keyword in `handleSerialInput()` in `Giga_Tunnel_PID.ino`; update the Quick Command Reference table in `README.md`.
- **Add a new compile-time configuration option**: Declare it as a `const` near the top of `Giga_Tunnel_PID.ino`; document it in the "Compile-time" section of `README.md`.
- **Add board support**: Add a `#if defined(BOARD_MACRO)` guard around board-specific pin or bus definitions; document the new board in `HARDWARE.md` and the badge list in `README.md`.
- **Modify I2C scanning**: Edit `I2CScanner.cpp`; update the header comments in `I2CScanner.h` to reflect any API changes.
