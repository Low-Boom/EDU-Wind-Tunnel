# Wind Tunnel Controller v1.0

**Comprehensive PID-based airspeed controller for educational wind tunnel systems**

[![Arduino](https://img.shields.io/badge/Arduino-Giga%20R1-00979D?logo=arduino)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-1.0-brightgreen.svg)](https://github.com/Low-Boom/EDU-wind-tunnel/releases)
[![Status](https://img.shields.io/badge/status-stable-success.svg)](https://github.com/Low-Boom/EDU-wind-tunnel)

---

## 🌟 Features

### Control & Tuning
- **Full PID Control** (Proportional, Integral, Derivative)
- **Relay Auto-Tuning** (Åström-Hägglund method with aggressive tuning rules)
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
- Arduino Giga R1 WiFi board
- MS4525DO differential pressure sensor
- BMP3XX barometric sensor (BMP388 or BMP390)
- PWM-controlled motor/ESC
- Arduino IDE 2.0+

### Installation

1. **Install Arduino IDE**
   ```
   Download from: https://www.arduino.cc/en/software
   ```

2. **Install Board Support**
   - Open: Tools → Board → Boards Manager
   - Search: "Arduino Mbed OS Giga Boards"
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
   - Select Board: "Arduino Giga R1"
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
Arduino Pin 9   → ESC PWM Signal
Arduino Pin 2   → Tachometer (optional)
I2C (SDA/SCL)   → MS4525DO + BMP3XX
```

⚠️ **CRITICAL:** Motor power supply must be separate from Arduino!

---

## 🚀 Basic Usage

### Setting Target Airspeed
```
10              # Set target to 10 m/s
15.5            # Set target to 15.5 m/s
0               # Stop motor
```

### Auto-Tuning (Recommended)
```
tune 40 100     # Auto-tune with PWM range 40-100
```
Wait 30-60 seconds for completion.

### Manual PID Tuning
```
tune 20 8 12    # Set Kp=20, Ki=8, Kd=12
```

### Recalibration
```
recal           # Recalibrate pressure sensor
```

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
- **P**: Pressure (Pa)
- **Temp**: Ambient temperature (°C)
- **Err**: Control error

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

---

## 📝 Version History

**v1.0** (2025-11-09) - **STABLE RELEASE**
- Production-ready release
- Relay auto-tuner with aggressive tuning rules
- Manual PID setting via `tune Kp Ki Kd`
- Comprehensive documentation
- Proven performance in educational settings

**v0.10.x** (Development versions)
- Relay auto-tuner implementation
- Åström-Hägglund method
- PWM range specification
- Debug and diagnostics

**v0.9.x** (Beta versions)
- Advanced sensor processing
- Temperature compensation
- Experimental auto-tuners

---

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

---

## 📄 License

This project uses open-source libraries:
- QuickPID (MIT License)
- Adafruit BMP3XX (BSD License)
- Bolder Flight Systems MS4525DO (MIT License)

See individual library licenses for details.

---

## 📧 Support

- **Issues**: [GitHub Issues](https://github.com/Low-Boom/EDU-wind-tunnel/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Low-Boom/EDU-wind-tunnel/discussions)
- **Author**: Low-Boom
- **Release Date**: 2025-11-09
- **Status**: Stable

---

## 🙏 Acknowledgments

- QuickPID Library by dlloydev
- Adafruit for sensor libraries
- Bolder Flight Systems for MS4525DO library
- Arduino community for MBED support

---

**Built for educational wind tunnel applications** 🌬️

**Version 1.0 - Production Ready**