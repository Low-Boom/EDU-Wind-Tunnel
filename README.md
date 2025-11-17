# Educational Wind Tunnel PID Controller
**Arduino-based PID controller for educational wind tunnel systems**

Developed by [Louis M. Edelman](https://www.researchgate.net/profile/Louis-Edelman) and Olivia Coulon

Debugging assitance provided by Antropic Claude Sonnet 4.5 and verified by humans.

Developed for use with the [Modular Wind Tunnel for STEM Education](https://www.printables.com/@JerrodH) by [Jerrod Hofferth](https://jerrod.hofferth.net/)

[![Arduino](https://img.shields.io/badge/Arduino-Giga%20R1-00979D?logo=arduino)](https://store-usa.arduino.cc/products/giga-r1-wifi?srsltid=AfmBOop3Aqtaf3BpQPH1OCf3aJT8Pt39YOuv7pEPDY4g7xi1-DEwiopE)
[![Arduino](https://img.shields.io/badge/Arduino-Mega%202560%20R3-00979D?logo=arduino)](https://store-usa.arduino.cc/products/arduino-mega-2560-rev3?srsltid=AfmBOopk3fHnK26RSMwzgGKXDylUJ8HzUNyVYfBNoiiwnkMh62EVld_U)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-1.1-brightgreen.svg)](https://github.com/Low-Boom/EDU-wind-tunnel/releases)
[![Status](https://img.shields.io/badge/status-stable-success.svg)](https://github.com/Low-Boom/EDU-wind-tunnel)

---


## Features

### Control & Tuning
- **Full PID Control** (Proportional, Integral, Derivative)
- **Relay Auto-Tuning** (Åström-Hägglund method)
- **sTune Auto-Tuning** (Inflection Point method - optional advanced tuning)
- **Manual PID Tuning** (direct gain adjustment)
- **PWM Rate Limiting** (smooth motor transitions)

### Sensor Processing
- **Pressure Oversampling** (noise reduction)
- **Configurable Airspeed Averaging** (default 20)
- **Ambient Temperature Compensation** (automatic air density correction)
- **Zero-Offset Calibration** (automatic at startup, manual recalibration at will)
- **Multi-Bus I2C Scanner** (automatic device discovery on all available I2C buses)

### Features in Development (Eventually) 
- Serial free untethered control via Modulino controls and a screen for mobility
- A [Controllino](https://www.controllino.com) PLC-based version for industrial applications and education
- A [Mathworks Simulink](https://www.mathworks.com/products/simulink.html) and [Julia Hub DYAD](https://juliahub.com/products/dyad) compatible controller
- Coursework materials for wind tunnel operation and design

---
## Detailed Documentation

- **[HARDWARE.md](HARDWARE.md)** - Detailed wiring, components, and power supply info
- **[TUNING_GUIDE.md](TUNING_GUIDE.md)** - Advanced PID tuning techniques
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues encountered during development and solutions
---

## Quick Start

### Prerequisites
- Arduino IDE 2.0+
- Any Arduino mbed compatible board (Tested on Arduino MEGA 2560 Rev 3 and Giga R1 WiFi board)
- Pitot-Static Airspeed Sensor based on a MS4525DO Differential pressure sensor (e.g. [Pixhawk PX4 Flight Controller Comaptible](https://a.co/d/9VPEeWh) )
- BMP3XX barometric sensor (E.g. [Adafruit BMP390](https://www.adafruit.com/product/4816?srsltid=AfmBOoptmxxHVYD1jurp-CC4qjkaLOoQzmZeBuTtz28D0mF_Nyu4XSE9) )
- PWM-controlled fan with external power source (e.g.[ AC Infinity CLOUDLINE A8 EC-PWM Motor](https://acinfinity.com/hydroponics-growers/cloudline-a8-quiet-inline-fan-with-speed-controller-8-inch/#product-reviews) or [Noctua 12V PC Fan](https://www.noctua.at/en/products/nf-a14x25-g2-pwm) as recommended by Jerrod H.)
- Depending on board and fan combination, a [Bi-Directional Logic Level Lifter](https://www.adafruit.com/product/757?srsltid=AfmBOopBWeXgpfL63fDoCPbWWtu07TIO7QVbofNmTDtQ4QkU7BxYeCWk) with at least two channels
---

## Hardware Setup

See [HARDWARE.md](HARDWARE.md) for detailed wiring diagrams and component specifications.

**Quick Connections:**
```
Arduino Pin 9   → PWM Control Signal
Arduino Pin 2   → PWM Tachometer Feedback Signal (optional)
I2C (SDA/SCL)   → MS4525DO + BMP3XX via Daisy Chained Qwiic Wiring
```

⚠️ Motor power supply must be separate from Arduino! This is 120V AC Power for the AC Infinity Fan or a 12V power supply for the 12V PC Fan (e.g. [Noctua NV-PS1](https://www.noctua.at/en/products/nv-ps1))

---

### Installation

1. **Install Arduino IDE**
   ```
   Download from: https://www.arduino.cc/en/software
   ```

2. **Install Board Support**
   - Open: Tools → Board → Boards Manager
   - Search: "Arduino Mbed OS Giga Boards" (Or your Mbed board of choice)
   - Install latest version

3. **Install Required Libraries**
   - Open: Tools → Manage Libraries
   - Install:
     - Adafruit BMP3XX
     - Adafruit Unified Sensor
     - QuickPID
     - Bolder Flight Systems MS4525DO
     - **sTune** (optional - for advanced PID autotuning)

4. **Upload Code**
   - Open `Giga_Tunnel_PID.ino`
   - Select Board: "Arduino Giga R1" (Or your Mbed board of choice)
   - Select Port: Your Arduino's COM port
   - Click Upload

5. **Open Serial Monitor**
   - Set baud rate: **115200**
   - Watch initialization messages to confirm successful hardware setup. 

6. **Allow for Initial Calibration"
   - Once the fan is under control, it will ramp down to 0.
   - A default wait time of 30 seconds is set to allow for the airflow in the tunnel to fully stop
   - 50 Samples are taken from the airspeed sensor to establish a zero offset.
   - This routine can be re-run via the `recal` serial command later

---

## I2C Device Scanner

The system includes an automatic I2C device scanner that runs at startup to help verify sensor connections. The scanner automatically detects and scans all available I2C buses on your Arduino board:

- **Classic Arduino (Uno, Nano, Mega)**: Scans Wire only
- **Arduino Uno Rev4**: Scans Wire and Wire1
- **Modern Boards (RP2040, ESP32, SAMD21/51)**: Scans Wire and Wire1
- **Arduino GIGA R1**: Scans Wire, Wire1, and Wire2

### Expected Output

On successful startup, you should see:
```
[I2C] Multi-Bus Scanner
=====================================
Available I2C buses: 2
Scanning I2C bus Wire
 - Found device at 0x28
 - Found device at 0x77
 - Total devices found on Wire: 2

Scanning I2C bus Wire1
 - No I2C devices found on Wire1

[I2C] Scan complete
=====================================
```

The MS4525DO pressure sensor should appear at address `0x28`, and the BMP3XX barometer at `0x76` or `0x77`. If devices are not found, check:
- I2C wiring (SDA/SCL connections)
- Device power supply
- Pull-up resistors (typically built into breakout boards)

---

## Quick Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `<number>` | Set target airspeed | `10` |
| `tune` | Auto-tune (auto PWM) | `tune` |
| `tune <L> <H>` | Auto-tune (manual PWM) | `tune 40 100` |
| `tune <Kp> <Ki> <Kd>` | Manual PID | `tune 20 8 12` |
| `stune` | sTune auto-tune (advanced) | `stune` |
| `recal` | Recalibrate sensor | `recal` |
| `recal <N>` | Recalibrate (N samples) | `recal 100` |
| `avg <N>` | Set averaging | `avg 10` |

## User-configurable variables (runtime & compile-time) 

### Runtime Commands (via serial)
- Direct setpoint: type a number (m/s) to set target.
- `avg <N>`  
  - Purpose: set pressure averaging samples (1–50). Default 20.  
  - Effect: lower → faster but noisier; higher → smoother but more lag.  
  - Typical: `avg 3` for fast tuning, `avg 15–30` for stable operation.

- `tune` / `tune <low> <high>` / `tune <Kp> <Ki> <Kd>`  
  - `tune` : auto-estimate PWM band and run relay tuning.  
  - `tune <low> <high>` : force PWM range (manual). Use when auto-estimate is poor.  
  - `tune <Kp> <Ki> <Kd>` : manually apply PID gains; controller resets automatically.

- `stune` (requires sTune library)
  - Advanced autotuning using inflection point method
  - More stable and predictable than relay tuning
  - Automatically determines step size and monitors system response
  - Default: "No Overshoot" tuning for stable operation
  - Usage: Set target airspeed (min 2 m/s), then type `stune`
  - Duration: ~30-60 seconds depending on system response
  - **Tuning method can be changed** at compile-time (see below)

### Compile-time (edit `Giga_Tunnel_PID.ino`)
Open the sketch and edit the configuration block near the top:

- `const sTune::TuningMethod STUNE_METHOD = sTune::NoOvershoot_PID;`
  - Selects the sTune tuning method (default: NoOvershoot_PID for stability)
  - Available options:
    - `sTune::ZN_PID` - Ziegler-Nichols (moderate overshoot, fast response)
    - `sTune::DampedOsc_PID` - Damped Oscillation (balanced)
    - `sTune::NoOvershoot_PID` - No Overshoot (conservative, stable) **[DEFAULT]**
    - `sTune::CohenCoon_PID` - Cohen-Coon (good for systems with delay)
    - `sTune::Mixed_PID` - Mixed (average of all methods)
  - For PI control instead of PID: use ZN_PI, DampedOsc_PI, NoOvershoot_PI, etc.

- `const int PRESSURE_OVERSAMPLES = 5;`  
  - Oversampling per measurement (3–8 typical).

- `int pressureAverageSamples = 20;`  
  - Averaging window for measurements (1–50). Effective lag ≈ updateInterval × pressureAverageSamples.

- `const unsigned long updateInterval = 200;`  
  - Main loop period in ms (default 200 ms). If changed, QuickPID sample time will be set from this.

- `int calibrationSamples = 50;`  
  - Samples used for zero-offset calibration at startup/recal.

- `const float MAX_AIRSPEED = 28.0;`  
  - Upper bound for setpoints; change if your tunnel differs.

- `const float MAX_PWM_CHANGE_PER_CYCLE = 25.0;`  
  - PWM slew limit per update cycle; set to 255 to disable.

- `float relayHysteresis = 0.5;`  
  - m/s hysteresis for relay auto-tune to prevent chattering.

- `const int PWM_pin = 9;` and `const int TACH_pin = 2;`  
  - Pin assignments; change if wiring differs.

- `const float MIN_TUNE_SETPOINT = 2.0;` and `const int REQUIRED_CYCLES = 3;`  
  - Minimum auto-tune setpoint and cycles needed for a valid tune.

Data output format
These can be quickly plotted using the Arduino IDE Serial Plotter or logged for later processing in a .csv format.
Example line printed periodically:
```
123s | V:10.23 | T:10.00 | PWM:127 | P:61.45 | T:24.5°C | Err:-0.23

elapsed time | airspeed (m/s) | target (m/s) | PWM (0–255) | differential pressure (Pa) | ambient temp (°C) | error (m/s)
```
---

## Serial Control from Android

Once the sketch is compiled and uploaded to the Arduino, it can be controlled via a USB connection to an android phone using the excellent [Serial USB Terminal App by Kair Morichi](https://www.kai-morich.de/android/).

1. Connect via a USB cable with whatever adapters or USB-C hubs are required for your particular phone and Arduino. A direct USB-C to USB-C cable will work for the Arduino Giga. 

2. Set the Baud Rate to 115200 and operate as you would from the Arduino IDE

3. It is recommended that you set macros in the app according to the Quick Command Reference table above.

---

## ⚠️ Safety Warnings

1. **Motor Power**: NEVER power motor from Arduino - use separate supply
2. **Mechanical**: Secure all components, keep clear of fan during operation
3. **Testing**: Start with low speeds (2-5 m/s), gradually increase
4. **Emergency**: Type `0` to stop or disconnect motor/fan from power

---

## 📄 License
Derived from the Modular Wind Tunnel for STEM Education by Jerrod H. under a Creative Commons4.0 License

This project uses open-source libraries:
- QuickPID (MIT License)
- Adafruit BMP3XX (BSD License)
- Bolder Flight Systems MS4525DO (MIT License)
- sTune (MIT License) - Optional advanced autotuning library by David Lloyd

See individual library licenses for details.

---

## Support
This code is provided with no warranty or expectation of support. If you post something in the issues or discussion sections of GitHub I or another community member may be able to resolve your question. 
This is an initial release so the documentation and best practices are still evolving. If you have contributions, please fork, tweak, pull, and merge at your discretion.

- **Issues**: [GitHub Issues](https://github.com/Low-Boom/EDU-wind-tunnel/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Low-Boom/EDU-wind-tunnel/discussions)

---

## Acknowledgments
- Jerrod H. for the Wind Tunnel Design
- QuickPID Library by Dlloydev
- sTune Library by Dlloydev
- Adafruit for sensor libraries
- Bolder Flight Systems for MS4525DO library
- Arduino community for MBED support
