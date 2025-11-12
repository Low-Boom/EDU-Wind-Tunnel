# Serial-Interface Educational Wind Tunnel Controller
**PID-based airspeed and PWM fan based controller for educational wind tunnel systems**
Developed by [Dr. Louis M. Edelman](https://www.researchgate.net/profile/Louis-Edelman) and Olivia Coulon

[![Arduino](https://img.shields.io/badge/Arduino-Giga%20R1-00979D?logo=arduino)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-1.0-brightgreen.svg)](https://github.com/Low-Boom/EDU-wind-tunnel/releases)
[![Status](https://img.shields.io/badge/status-stable-success.svg)](https://github.com/Low-Boom/EDU-wind-tunnel)

---
Developed for use with the Modular Wind Tunnel for STEM Education by Jerrod H. (https://www.printables.com/model/849713-modular-wind-tunnel-for-stem-education)


## Features

### Control & Tuning
- **Full PID Control** (Proportional, Integral, Derivative)
- **Relay Auto-Tuning** (Åström-Hägglund method)
- **Manual PID Tuning** (direct gain adjustment)
- **PWM Rate Limiting** (smooth motor transitions)

### Sensor Processing
- **5x Pressure Oversampling** (noise reduction)
- **Configurable Averaging** (1-50 samples, default 20)
- **Temperature Compensation** (automatic air density correction)
- **Zero-Offset Calibration** (automatic at startup, manual recalibration)

---
## Detailed Documentation

- **[HARDWARE.md](HARDWARE.md)** - Detailed wiring, components, and power supply info
- **[TUNING_GUIDE.md](TUNING_GUIDE.md)** - Advanced PID tuning techniques
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues encountered during development and solutions
---

## Quick Start

### Prerequisites
- Arduino IDE 2.0+
- Any Arduino mbed compatible board (Tested on Arduino Giga R1 WiFi board)
- Pitot-Static Airspeed Sensor based on a MS4525DO Differential pressure sensor (e.g. [Pixhawk PX4 Flight Controller Comaptible](https://a.co/d/9VPEeWh) )
- BMP3XX barometric sensor (E.g. [Adafruit BMP390](https://www.adafruit.com/product/4816?srsltid=AfmBOoptmxxHVYD1jurp-CC4qjkaLOoQzmZeBuTtz28D0mF_Nyu4XSE9) )
- PWM-controlled fan with external power source (e.g.[ AC Infinity CLOUDLINE A8 EC-PWM Motor](https://acinfinity.com/hydroponics-growers/cloudline-a8-quiet-inline-fan-with-speed-controller-8-inch/#product-reviews) or [Noctua 12V PC Fan](https://www.noctua.at/en/products/nf-a14x25-g2-pwm) as recommended by Jerrod H.)
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

## Quick Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `<number>` | Set target airspeed | `10` |
| `tune` | Auto-tune (auto PWM) | `tune` |
| `tune <L> <H>` | Auto-tune (manual PWM) | `tune 40 100` |
| `tune <Kp> <Ki> <Kd>` | Manual PID | `tune 20 8 12` |
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

### Compile-time (edit `Giga_Tunnel_PID.ino`)
Open the sketch and edit the configuration block near the top:

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

## ⚠️ Safety Warnings

1. **Motor Power**: NEVER power motor from Arduino - use separate supply
2. **Mechanical**: Secure all components, keep clear of fan during operation
3. **Testing**: Start with low speeds (2-5 m/s), gradually increase
4. **Emergency**: Type `0` to stop, disconnect power if unstable

---

## 📄 License
Derived from the Modular Wind Tunnel for STEM Education by Jerrod H. under a Creative Commons4.0 License

This project uses open-source libraries:
- QuickPID (MIT License)
- Adafruit BMP3XX (BSD License)
- Bolder Flight Systems MS4525DO (MIT License)

See individual library licenses for details.

---

## Support
This code is provided with no warranty or expectation of support. If you post something in the issues or discussion sections of GitHub I or another community member may be able to resolve your question. 
This is an initial release so the documentation and best practices are still evolving. If you have contributions, such as improved PID tuning or support for different hardware, please fork, tweak, pull, and merge at your discretion.

- **Issues**: [GitHub Issues](https://github.com/Low-Boom/EDU-wind-tunnel/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Low-Boom/EDU-wind-tunnel/discussions)
- **Author**: Low-Boom
- **Release Date**: 2025-11-09
- **Status**: Stable

---

## Acknowledgments
- Jerrod H. for the Wind Tunnel Design
- QuickPID Library by dlloydev
- Adafruit for sensor libraries
- Bolder Flight Systems for MS4525DO library
- Arduino community for MBED support
