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

## 📋 Quick Start

### Prerequisites
- Arduino IDE 2.0+
- Any Arduino mbed compatible board (Tested on Arduino Giga R1 WiFi board)
- Pitot-Static Airspeed Sensor based on a MS4525DO Differential pressure sensor (e.g. [Pixhawk PX4 Flight Controller Comaptible](https://a.co/d/9VPEeWh) )
- BMP3XX barometric sensor (E.g. [Adafruit BMP390](https://www.adafruit.com/product/4816?srsltid=AfmBOoptmxxHVYD1jurp-CC4qjkaLOoQzmZeBuTtz28D0mF_Nyu4XSE9) )
- PWM-controlled fan with external power source (e.g.[ AC Infinity CLOUDLINE A8 EC-PWM Motor](https://acinfinity.com/hydroponics-growers/cloudline-a8-quiet-inline-fan-with-speed-controller-8-inch/#product-reviews) or [Noctua 12V PC Fan](https://www.noctua.at/en/products/nf-a14x25-g2-pwm) as recommended by Jerrod H.)

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
   - Watch initialization messages

---

## 🔧 Hardware Setup

See [HARDWARE.md](HARDWARE.md) for detailed wiring diagrams and component specifications.

**Quick Connections:**
```
Arduino Pin 9   → PWM Control Signal
Arduino Pin 2   → PWM Tachometer Feedback Signal (optional)
I2C (SDA/SCL)   → MS4525DO + BMP3XX via Daisy Chained Qwiic Wiring
```

⚠️ **CRITICAL:** Motor power supply must be separate from Arduino! This is 120V AC Power for the AC Infinity Fan and a 12V power supply for a 12V PC Fan (e.g. [Noctua NV-PS1](https://www.noctua.at/en/products/nv-ps1))

---

## Serial Command Control Usage

### Setting Target Airspeed
```
##              # Set the target airpseed by inputting a number. Default acceptable range is 0 - 25 m/s
10              # e.g. Set target to 10 m/s
15.5            # e.g, Set target to 15.5 m/s
0               # e.g. Stop motor
```

### Auto-Tuning (Recommended)
```
tune LOW HIGH     # Auto-tune with by relaying between set PWM values targeting the current set airspeed.
tune 40 100       # e.g. bounce between PWM level 40 and PWM level 100.
```
Target airspeed must be set greater than 2 m/s prior to invokingthis command.
This might take some experimentation to find levels around your target airspeed.
Wait 30-60 seconds for completion.

### Manual PID Tuning
```
tune Kp Ki Kd    # Manually set PID controller paramters.
tune 20 8 12     # e.g. set Kp=20, Ki=8, Kd=12
```

### Recalibration
```
recal           # Recalibrate or zero the differential pressure sensor for airspeed 
```
⚠️ **CRITICAL:** Motor must bet turned off or the differential pressure sensor disconnected from the test secion such that both sides see atmospheric pressure.
---

## 📖 Documentation

- **[HARDWARE.md](HARDWARE.md)** - Detailed wiring, components, and power supply info
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions
- **[TUNING_GUIDE.md](TUNING_GUIDE.md)** - Advanced PID tuning techniques

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
