# sTune Integration Testing Guide

This document provides instructions for testing the sTune PID autotuning integration.

## Prerequisites

Before testing, ensure you have:

1. **Arduino IDE 2.0+** installed
2. **Required libraries** installed via Arduino Library Manager:
   - Adafruit BMP3XX
   - Adafruit Unified Sensor
   - QuickPID
   - Bolder Flight Systems MS4525DO
   - **sTune** (required for this integration)

## Installation and Compilation Test

### Step 1: Install sTune Library

1. Open Arduino IDE
2. Go to **Tools → Manage Libraries**
3. Search for "sTune"
4. Install the latest version by David Lloyd (Dlloydev)

### Step 2: Verify Main Sketch Compiles

1. Open `Giga_Tunnel_PID/Giga_Tunnel_PID.ino`
2. Select your board:
   - **Tools → Board → Arduino Mbed OS Giga Boards → Arduino Giga R1**
   - Or: **Tools → Board → Arduino Mbed OS Boards → Arduino Mega 2560**
3. Click **Verify** (checkmark icon)
4. Compilation should complete successfully

**Expected output:**
```
Sketch uses XXXXX bytes (XX%) of program storage space.
Global variables use XXXXX bytes (XX%) of dynamic memory.
```

**Note:** If compilation fails with "sTune.h: No such file or directory", ensure the sTune library is installed.

### Step 3: Verify Example Sketch Compiles

1. Open `examples/sTune_WindTunnel_Example/sTune_WindTunnel_Example.ino`
2. Verify compilation succeeds

## Functional Testing (With Hardware)

### Test 1: Verify sTune Command is Recognized

1. Upload `Giga_Tunnel_PID.ino` to your board
2. Open Serial Monitor (115200 baud)
3. Wait for initialization to complete
4. Check that the help text shows:
   ```
   COMMANDS:
     ...
     stune                   - sTune autotuning (advanced)
   ```

### Test 2: Run sTune Autotuning

1. In Serial Monitor, set a target airspeed: `10`
2. Type: `stune`
3. Observe the tuning process:
   ```
   [sTune] AUTOTUNING STARTED
   [sTune] Target setpoint: 10.00 m/s
   [sTune] Method: Inflection Point (No Overshoot)
   ...
   [sTune] Starting settling phase...
   [sTune] Settling complete - starting test
   [sTune] Monitoring inflection point...
   ```
4. Wait for completion (30-60 seconds):
   ```
   [sTune] AUTOTUNING COMPLETE!
   [sTune] Tuned PID Gains:
           Kp = XX.XXXX
           Ki = XX.XXXX
           Kd = XX.XXXX
   [sTune] Process Characteristics:
           Process Gain: X.XXXX
           Dead Time: X.XX s
           Time Constant (Tau): X.XX s
   ```

### Test 3: Verify Both Tuning Methods Work

Test that both tuning methods are available and work independently:

**Relay tuning:**
1. Set target: `10`
2. Type: `tune`
3. Verify relay tuning runs

**sTune tuning:**
1. Set target: `10`
2. Type: `stune`
3. Verify sTune runs

Both should complete and apply gains to the PID controller.

## Validation Without Hardware

If you don't have the hardware, you can still validate the integration:

### Validation 1: Code Review

Review the following to ensure integration is correct:

1. **STunePIDTuner.h** - Interface is clean and documented
2. **STunePIDTuner.cpp** - Implementation handles errors properly
3. **Giga_Tunnel_PID.ino** - Integration doesn't break existing functionality

### Validation 2: Example Sketch

The example sketch `examples/sTune_WindTunnel_Example/sTune_WindTunnel_Example.ino` demonstrates:
- Proper sTune initialization
- Correct state machine handling
- Error handling for timeout/failure
- Gain application after tuning

### Validation 3: Documentation

Verify documentation is complete:
- [x] README.md mentions sTune
- [x] TUNING_GUIDE.md has sTune section
- [x] THIRD_PARTY_LICENSES.md includes sTune
- [x] Command reference includes "stune"

## Expected Behavior

### Normal Operation

When sTune is working correctly:

1. **Settling phase** (5 seconds): Output held at starting value
2. **Testing phase** (30-60 seconds): Step applied, inflection point detected
3. **Completion**: Gains calculated and applied
4. **Resume**: Normal PID control with new gains

### Error Handling

The system should handle errors gracefully:

- **Low setpoint**: Error message if target < 2 m/s
- **Timeout**: Aborts after 120 seconds with message
- **Emergency stop**: Stops if limit exceeded

## Troubleshooting

### Compilation Error: "sTune.h: No such file"

**Solution:** Install sTune library via Arduino Library Manager

### Compilation Error: "STunePIDTuner.h: No such file"

**Solution:** Ensure STunePIDTuner.h and STunePIDTuner.cpp are in the same folder as the .ino file

### Runtime: "stune" command not recognized

**Solution:** Verify main sketch has the sTune integration code (check for `#include <sTune.h>`)

### Tuning times out

**Possible causes:**
- System doesn't respond to PWM output
- Sensors not working
- Output step too small
- Test time estimate too short

**Solution:** Check hardware connections and sensor readings

## Success Criteria

✅ **Integration is successful if:**

1. Main sketch compiles without errors (with sTune library installed)
2. Example sketch compiles without errors
3. "stune" command appears in help text
4. Both "tune" and "stune" commands work
5. sTune completes and applies gains (when hardware is available)
6. Documentation is complete and accurate

## Notes

- sTune is **optional** - the system still works without it using relay tuning
- The wrapper provides a simplified interface to sTune
- Default configuration uses conservative "NoOvershoot" tuning
- Advanced users can modify tuning parameters in the wrapper code

## Reference

- Main sketch: `Giga_Tunnel_PID/Giga_Tunnel_PID.ino`
- Example: `examples/sTune_WindTunnel_Example/sTune_WindTunnel_Example.ino`
- Wrapper: `Giga_Tunnel_PID/STunePIDTuner.h/cpp`
- Documentation: `TUNING_GUIDE.md`
