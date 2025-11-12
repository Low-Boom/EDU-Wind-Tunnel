# Serial Controlled Educational Wind Tunnel Controller v1.0

**PID-based airspeed and PWM fan based controller for educational wind tunnel systems**

[![Arduino](https://img.shields.io/badge/Arduino-Giga%20R1-00979D?logo=arduino)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-1.0-brightgreen.svg)](https://github.com/Low-Boom/EDU-wind-tunnel/releases)
[![Status](https://img.shields.io/badge/status-stable-success.svg)](https://github.com/Low-Boom/EDU-wind-tunnel)

---
Developed for use with the Modular Wind Tunnel for STEM Education by Jerrod H. (https://www.printables.com/model/849713-modular-wind-tunnel-for-stem-education)


## 🌟 Features

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

### Advanced Features
- P-on-Measurement (reduced overshoot)
- D-on-Measurement (derivative kick elimination)
- Conditional Anti-Windup (prevents integral saturation)
- Real-time diagnostics and monitoring

---
## 📖 Detailed Documentation

- **[HARDWARE.md](HARDWARE.md)** - Detailed wiring, components, and power supply info
- - **[TUNING_GUIDE.md](TUNING_GUIDE.md)** - Advanced PID tuning techniques
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues encountered during development and solutions
---

## 📋 Quick Start

### Prerequisites
- Arduino IDE 2.0+
- Any Arduino mbed compatible board (Tested on Arduino Giga R1 WiFi board)
- Pitot-Static Airspeed Sensor based on a MS4525DO Differential pressure sensor (e.g. [Pixhawk PX4 Flight Controller Comaptible](https://a.co/d/9VPEeWh) )
- BMP3XX barometric sensor (E.g. [Adafruit BMP390](https://www.adafruit.com/product/4816?srsltid=AfmBOoptmxxHVYD1jurp-CC4qjkaLOoQzmZeBuTtz28D0mF_Nyu4XSE9) )
- PWM-controlled fan with external power source (e.g.[ AC Infinity CLOUDLINE A8 EC-PWM Motor](https://acinfinity.com/hydroponics-growers/cloudline-a8-quiet-inline-fan-with-speed-controller-8-inch/#product-reviews) or [Noctua 12V PC Fan](https://www.noctua.at/en/products/nf-a14x25-g2-pwm) as recommended by Jerrod H.)
---

## 🔧 Hardware Setup

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

## User-configurable variables (runtime & compile-time)

This section documents the most useful user-configurable parameters that affect sensor averaging, control responsiveness, and tuning. Some variables can be changed at runtime via the serial console; others require editing the sketch and re-uploading.

---
### Runtime-adjustable (via serial commands)
These can be changed while the system is running; they take effect immediately.

- Direct setpoint (number)
  - Type a number (e.g., `10`) to set the target airspeed (m/s).

- `avg <N>`
  - Purpose: Set pressure averaging window (number of averaged samples).
  - Command: `avg 1` .. `avg 50`
  - Default: 20
  - Effect: Lower values reduce lag and increase responsiveness (but increase noise). Higher values smooth readings and increase lag.
  - Recommended: 1–5 for fast tuning/tests, 15–30 for stable operation/bench testing.

- `tune` / `tune <low> <high>` / `tune <Kp> <Ki> <Kd>`
  - Purpose: Start auto relay tuning or set manual PID gains.
  - Examples:
    - `tune` — auto-estimate PWM range and run relay auto-tune.
    - `tune 40 100` — run relay auto-tune using PWM 40..100.
    - `tune 20 8 12` — directly set PID gains (Kp=20, Ki=8, Kd=12).
  - Notes: Manual PID immediately applies and resets the PID controller.

- `recal` / `recal <N>`
  - Purpose: Re-run pressure sensor zero-offset calibration.
  - ⚠️ Motor must bet fully off or sneor disconencted from the test section such that both sides of the sensor are exposed to atmosphere
  - Default samples: 50 (unless overridden with `recal N`)
  - Use when the tunnel has changed or after moving sensors.
  - 
---

### Compile-time constants (edit in the sketch)
These are top-of-file constants in `Giga_Tunnel_PID.ino`. After editing, re-upload the sketch.

Important section: near the top of `Giga_Tunnel_PID.ino` (search for the comment block "sTune Auto-tuning configuration" or the constants block).

Key variables (name → default → recommended range / notes):

- `const int PRESSURE_OVERSAMPLES = 5;`
  - Default: 5
  - Purpose: How many raw pressure readings are taken and averaged per measurement (oversampling).
  - Notes: Larger = less noise at sensor-read level; increases sampling time. Keep small (3–8).

- `int pressureAverageSamples = 20;`
  - Default: 20
  - Purpose: Number of recent measurements averaged for output smoothing.
  - Range: 1–50
  - Notes: Controls effective system lag: effective lag ≈ updateInterval × pressureAverageSamples.

- `const unsigned long updateInterval = 200;`
  - Default: 200 ms (5 Hz)
  - Purpose: Main loop sampling interval.
  - Notes: Decreasing to 100 ms increases responsiveness but requires more CPU and may need QuickPID sample time adjustment.

- `const float MAX_AIRSPEED = 28.0;`
  - Default: 28 m/s
  - Purpose: Upper bound for setpoint and emergency checks.
  - Edit if your tunnel's maximum differs.

- `int calibrationSamples = 50;`
  - Default: 50
  - Purpose: Number of samples used for zero-offset calibration.
  - Notes: Increase (e.g., 100) for more stable offset at the cost of longer calibration.

- `const float MAX_PWM_CHANGE_PER_CYCLE = 25.0;`
  - Default: 25
  - Purpose: Rate-limit PWM changes per control loop. Prevents sudden motor steps.
  - Set higher for faster actuation, set to 255 to disable rate-limiting (not recommended).

- `float relayHysteresis = 0.5;`
  - Default: 0.5 m/s
  - Purpose: Hysteresis band used by the relay auto-tuner to avoid chattering.
  - Increase to reduce chattering if sensors are noisy.

- `const int PWM_pin = 9;`
  - Purpose: PWM output pin (change if your wiring differs).

- `const float MIN_TUNE_SETPOINT = 2.0;`
  - Default: 2.0 m/s
  - Purpose: Minimum setpoint allowed for auto-tuning.

- `const int REQUIRED_CYCLES = 3;`
  - Default: 3
  - Purpose: Number of relay oscillation cycles required to accept auto-tune results.

- `const int PRESSURE_OVERSAMPLES` and `pressureAverageSamples` working note:
  - Effective measurement latency ≈ updateInterval × pressureAverageSamples.
  - For fast auto-tuning you may temporarily reduce `pressureAverageSamples` (for example, the code sets it to 3 during tuning). Use `avg 3` or change the constant and re-upload for persistent changes.

---

### How to edit these values in the code
1. Open `Giga_Tunnel_PID.ino` in Arduino IDE.
2. Near the top of the file you will find the constants and configuration block — edit the values prescribed above.
3. If you change `updateInterval`, also update:
   ```cpp
   myPID.SetSampleTimeUs(updateInterval * 1000);
   ```
   (The code sets this value automatically from `updateInterval`).

4. Save → Compile → Upload.

---

## 📊 Data Output

```
123s | V:10.23 | T:10.00 | PWM:127 | P:61.45 | T:24.5°C | Err:-0.23
```

**Fields:**
- **T**: Time (s)
- **V**: Current airspeed (m/s)
- **Target**: Setpoint (m/s)
- **PWM**: Motor output (0-255)
- **P**: Airspeed Sensor Differential Pressure (Pa)
- **Temp**: Ambient temperature (°C)
- **Err**: PID Control Rrror

These can be quickly plotted using the Arduino IDE Serial Plotter or logged for later processing in a .csv format.
---

## 🎯 Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `<number>` | Set target airspeed | `10` |
| `tune` | Auto-tune (auto PWM) | `tune` |
| `tune <L> <H>` | Auto-tune (manual PWM) | `tune 40 100` |
| `tune <Kp> <Ki> <Kd>` | Manual PID | `tune 20 8 12` |
| `recal` | Recalibrate sensor | `recal` |
| `recal <N>` | Recalibrate (N samples) | `recal 100` |
| `avg <N>` | Set averaging | `avg 10` |

---

## ⚠️ Safety Warnings

1. **Motor Power**: NEVER power motor from Arduino - use separate supply
2. **Mechanical**: Secure all components, keep clear of fan during operation
3. **Testing**: Start with low speeds (2-5 m/s), gradually increase
4. **Emergency**: Type `0` to stop, disconnect power if unstable

---

## 🔬 Theory of Operation

### PID Control Equation
```
PWM = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt
```

### Airspeed Calculation (Bernoulli)
```
V = √(2ΔP / ρ)
```
Where ρ is temperature-compensated air density.

### Relay Auto-Tuning
Uses controlled oscillation to determine:
- Ultimate gain (Ku)
- Ultimate period (Tu)
- Applies "Some Overshoot" tuning rules for fast response
- 
---

## 📄 License
Derived from the Modular Wind Tunnel for STEM Education by Jerrod H. under a Creative Commons4.0 License

This project uses open-source libraries:
- QuickPID (MIT License)
- Adafruit BMP3XX (BSD License)
- Bolder Flight Systems MS4525DO (MIT License)

See individual library licenses for details.

---

## 📧 Support
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
