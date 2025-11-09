# Changelog

All notable changes to the Wind Tunnel Controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] - 2025-11-09

### 🎉 Initial Stable Release

**Status:** Production Ready

### Added
- Full PID control implementation with QuickPID library
- Relay auto-tuning (Åström-Hägglund method)
- Manual PID tuning via serial commands (`tune Kp Ki Kd`)
- Automatic PWM range relay tuning (`tune low high`)
- MS4525DO differential pressure sensor support
- BMP3XX barometric sensor support (temperature compensation)
- 5x pressure sensor oversampling
- Configurable pressure averaging (1-50 samples)
- Automatic zero-offset calibration at startup
- Manual recalibration command (`recal`)
- P-on-Measurement mode (reduced setpoint kick)
- D-on-Measurement mode (no derivative kick)
- Conditional anti-windup
- PWM rate limiting for smooth motor transitions
- Real-time diagnostics and monitoring
- Comprehensive documentation (README, HARDWARE, TROUBLESHOOTING, TUNING_GUIDE)
- Serial command interface (115200 baud)
- Temperature-compensated air density calculation
- Emergency stop (type `0`)

### Features
- **Auto-Tuning**: Aggressive "Some Overshoot" tuning rules for fast response
- **Sensor Processing**: Oversampling + averaging + temperature compensation
- **Safety**: PWM rate limiting, emergency stop, separate motor power required
- **Flexibility**: Manual or automatic tuning, adjustable parameters
- **Educational**: Comprehensive documentation and examples

### Supported Hardware
- Arduino Giga R1 WiFi
- MS4525DO differential pressure sensor (I2C 0x28)
- BMP388/BMP390 barometric sensors (I2C 0x77/0x76)
- PWM-controlled BLDC motors with ESC
- Optional: Hall effect or optical tachometer

### Command Reference
```
<number>         - Set target airspeed (m/s)
tune             - Auto-tune with estimated PWM range
tune <L> <H>     - Auto-tune with manual PWM range
tune <Kp> <Ki> <Kd> - Set PID gains manually
recal            - Recalibrate pressure sensor
recal <N>        - Recalibrate with N samples
avg <N>          - Set averaging samples (1-50)
```

### Performance
- Update rate: 200ms (5 Hz)
- Sensor oversampling: 5x
- Default averaging: 20 samples
- PWM resolution: 8-bit (0-255)
- Auto-tune duration: 30-60 seconds
- Typical rise time: 3-6 seconds
- Typical settling time: 6-10 seconds

---

## [0.10.5] - 2025-11-09 (Pre-release)

### Added
- Manual PID setting via three-parameter tune command
- Enhanced PWM verification and debug output
- Improved parameter parsing (2 vs 3 arguments)

### Fixed
- PWM startup issues during relay tuning
- State machine initialization

---

## [0.10.4] - 2025-11-09 (Pre-release)

### Added
- Aggressive "Some Overshoot" tuning rules
- Alternative tuning calculation display
- Enhanced relay tuning diagnostics

### Changed
- Default tuning from conservative ZN to aggressive rules
- Improved PWM range estimation (8 PWM per m/s)

---

## [0.10.3] - 2025-11-09 (Pre-release)

### Fixed
- Relay tuning startup handling
- PWM initialization sequence
- Added explicit motor stop before tuning

### Added
- Detailed PWM debug output
- Startup verification messages

---

## [0.10.2] - 2025-11-09 (Pre-release)

### Fixed
- PID interference during relay tuning
- Added PID disable during auto-tune
- Improved PWM control handoff

### Added
- Better startup sequence
- Motor status verification

---

## [0.10.1] - 2025-11-09 (Pre-release)

### Added
- Manual PWM range specification
- Better PWM range estimation
- Error checking for PWM values

### Fixed
- PWM range validation

---

## [0.10.0] - 2025-11-08 (Pre-release)

### Added
- Relay auto-tuning implementation
- Åström-Hägglund oscillation detection
- Ziegler-Nichols gain calculation
- Cycle counting and validation
- Timeout handling

### Removed
- sTune library (incompatible with system)

---

## [0.9.7] - 2025-11-08 (Development)

### Added
- sTune auto-tuning attempt (debug mode)
- State tracking and diagnostics

### Note
- sTune found incompatible, abandoned

---

## [0.9.x] - 2025-11-08 (Development)

### Added
- Advanced sensor processing
- Temperature compensation
- Oversampling and averaging
- Configurable parameters

---

## Earlier Versions

Development versions with basic PID implementation, sensor integration, and manual tuning only.

---

## Upgrade Notes

### From 0.10.x to 1.0.0
- No breaking changes
- Update Arduino code to v1.0
- Review documentation updates
- Test auto-tuning with your system

### From 0.9.x to 1.0.0
- Complete rewrite of auto-tuning system
- Update all code files
- Recalibrate sensors
- Re-tune PID gains

---

## Future Plans

### Planned for v1.1.0
- [ ] Gain scheduling for multiple speed ranges
- [ ] Data logging to SD card
- [ ] Web interface for remote monitoring
- [ ] Multiple tuning profiles
- [ ] Enhanced safety features

### Planned for v1.2.0
- [ ] Feed-forward control
- [ ] Adaptive tuning
- [ ] Disturbance rejection improvements
- [ ] MATLAB/Python interface

### Planned for v2.0.0
- [ ] Multi-variable control
- [ ] Advanced filtering options
- [ ] Real-time performance analytics
- [ ] Model predictive control (MPC)

---

**Version 1.0.0 - Production Ready**  
**Release Date:** 2025-11-09  
**Author:** Low-Boom  
**Repository:** [EDU-wind-tunnel](https://github.com/Low-Boom/EDU-wind-tunnel)