# PID Autotuning Methods Comparison

This document compares the two autotuning methods available in the EDU Wind Tunnel controller.

## Overview

The system now supports two autotuning methods:

1. **Relay Tuning** (Åström-Hägglund method) - Original method
2. **sTune** (Inflection Point method) - New advanced method

Both methods automatically determine PID gains, but use different approaches.

## Quick Comparison

| Feature | Relay Tuning | sTune |
|---------|--------------|-------|
| **Command** | `tune` | `stune` |
| **Method** | Oscillation-based | Step response |
| **Duration** | 30-60 seconds | 30-60 seconds |
| **Tuning Style** | Aggressive | Conservative |
| **Expected Overshoot** | 10-20% | <5% |
| **Settling Time** | Fast | Moderate |
| **Stability** | Good | Excellent |
| **Process Info** | Limited | Detailed |
| **Best For** | Quick response | Stable operation |
| **User Control** | PWM range selection | Automatic |
| **Complexity** | Moderate | Simple |

## Detailed Comparison

### Relay Tuning (Åström-Hägglund)

**How it works:**
1. Switches PWM between HIGH and LOW values
2. System oscillates around setpoint
3. Measures oscillation period and amplitude
4. Calculates ultimate gain (Ku) and period (Tu)
5. Applies tuning rules to determine PID gains

**Advantages:**
- ✅ Fast, responsive control
- ✅ User can specify PWM range
- ✅ Good for dynamic environments
- ✅ Well-tested method
- ✅ Works well for competition/demo

**Disadvantages:**
- ❌ Some overshoot expected
- ❌ Requires multiple oscillation cycles
- ❌ Can be noisy
- ❌ May need manual PWM range adjustment
- ❌ Limited process information

**Usage:**
```
> 10              # Set target
> tune            # Auto PWM estimation
# or
> tune 40 100     # Manual PWM range
```

**Output:**
```
[RELAY] AUTO-TUNE COMPLETE!
        Measured Parameters:
          Period (Tu): 8.50 s
          Amplitude: 1.23 m/s
          Ultimate Gain (Ku): 0.8234
        Calculated PID Gains (AGGRESSIVE):
          Kp = 25.1234
          Ki = 10.4567
          Kd = 15.7890
```

### sTune (Inflection Point)

**How it works:**
1. Applies a step change to PWM output
2. Monitors system response curve
3. Detects inflection point using moving tangent
4. Determines process gain, dead time, time constant
5. Applies conservative tuning rules

**Advantages:**
- ✅ Minimal overshoot
- ✅ Very stable operation
- ✅ Detailed process characterization
- ✅ Automatic step sizing
- ✅ Better for education/research
- ✅ Consistent results

**Disadvantages:**
- ❌ Slower response than relay tuning
- ❌ Requires S-shaped step response
- ❌ Less aggressive tuning
- ❌ Needs sTune library installed

**Usage:**
```
> 10              # Set target
> stune           # Automatic tuning
```

**Output:**
```
[sTune] AUTOTUNING COMPLETE!
[sTune] Duration: 45 seconds
[sTune] Tuned PID Gains:
        Kp = 18.2456
        Ki = 7.8234
        Kd = 11.3421
[sTune] Process Characteristics:
        Process Gain: 0.8234
        Dead Time: 2.34 s
        Time Constant (Tau): 8.67 s
```

## When to Use Each Method

### Use Relay Tuning When:

1. **Fast response is critical**
   - Demonstrations or competitions
   - Quick setpoint changes needed
   - Some overshoot is acceptable

2. **You need control over the test**
   - Want to specify PWM range
   - System needs aggressive tuning
   - Familiar with relay method

3. **Educational context allows overshoot**
   - Teaching about control systems
   - Demonstrating PID behavior
   - Comparing tuning methods

### Use sTune When:

1. **Stability is paramount**
   - Research or data collection
   - Educational environments
   - Precision measurements needed

2. **You want process information**
   - System characterization needed
   - Understanding process dynamics
   - Documentation purposes

3. **Simplicity is preferred**
   - Don't want to guess PWM range
   - Want automatic configuration
   - Prefer conservative tuning

## Real-World Scenarios

### Scenario 1: Classroom Demonstration

**Goal:** Show students stable PID control

**Recommendation:** Use **sTune**
- Minimal overshoot looks professional
- Process info helps teaching
- Consistent, predictable results

**Command sequence:**
```
> recal           # Calibrate sensor
> 10              # Set target
> stune           # Run sTune
# Wait for completion
# System now operates stably
```

### Scenario 2: Wind Tunnel Competition

**Goal:** Fast response for various test conditions

**Recommendation:** Use **Relay Tuning**
- Quick, aggressive response
- Good for changing setpoints
- Can fine-tune PWM range

**Command sequence:**
```
> 15              # Set target
> tune 60 120     # Tune with specific range
# Or let it auto-estimate:
> tune            # Auto PWM
# System responds quickly
```

### Scenario 3: Research Measurements

**Goal:** Collect data with minimal disturbance

**Recommendation:** Use **sTune**
- Maximum stability
- Process characterization valuable
- Reproducible gains

**Command sequence:**
```
> 12              # Set measurement speed
> stune           # Conservative tuning
# Document gains and process parameters
# Stable operation for data collection
```

### Scenario 4: First-Time Setup

**Goal:** Get system working quickly

**Recommendation:** Try both, compare results

**Command sequence:**
```
# Try relay first
> 10
> tune
# Note the gains

# Then try sTune
> 10
> stune
# Note the gains

# Choose based on performance
```

## Tuning Rule Comparison

### Relay Tuning Rules

Based on Ziegler-Nichols modified for "Some Overshoot":
```
Kp = 0.7 * Ku
Ki = 1.75 * Ku / Tu
Kd = 0.15 * Ku * Tu
```

**Characteristics:**
- Aggressive integral action
- Moderate derivative
- Designed for ¼ decay ratio
- Expects 10-20% overshoot

### sTune Rules (NoOvershoot_PID)

Based on Chien-Hrones-Reswick (CHR) for setpoint tracking:
```
Based on process gain (K), dead time (θ), time constant (τ)
Kp = 0.6 * τ / (K * θ)
Ki = τ / θ
Kd = 0.5 * θ
```

**Characteristics:**
- Conservative proportional
- Moderate integral
- Damped derivative
- Designed for 0% overshoot

## Performance Expectations

### Relay Tuning Performance

**Typical behavior:**
- Rise time: Fast (2-3 time constants)
- Overshoot: 10-20%
- Settling time: 4-6 time constants
- Steady-state error: Minimal
- Disturbance rejection: Good

**Example response:**
```
Setpoint change 5→10 m/s:
t=0s:   5.0 m/s
t=2s:   9.2 m/s
t=4s:   10.5 m/s (overshoot)
t=6s:   10.2 m/s
t=8s:   10.0 m/s (settled)
```

### sTune Performance

**Typical behavior:**
- Rise time: Moderate (3-4 time constants)
- Overshoot: <5%
- Settling time: 5-7 time constants
- Steady-state error: Minimal
- Disturbance rejection: Excellent

**Example response:**
```
Setpoint change 5→10 m/s:
t=0s:   5.0 m/s
t=2s:   8.5 m/s
t=4s:   9.8 m/s
t=6s:   10.1 m/s (minimal overshoot)
t=8s:   10.0 m/s (settled)
```

## Maintenance and Retuning

### When to Retune

Retune the system when:
- Fan or motor replaced
- Significant airflow path changes
- Temperature/altitude changes significantly
- Performance degrades over time
- Switching between different fans

### Retuning Recommendations

**Quick retune:** Use relay tuning
**Thorough retune:** Use sTune, document results

**Best practice:**
1. Run sTune to characterize system
2. Document process parameters
3. Use relay tuning for quick adjustments
4. Re-run sTune if major changes made

## Conclusion

Both methods have their place:

**Relay Tuning:** Best for fast, responsive control when some overshoot is acceptable

**sTune:** Best for stable, predictable control when precision matters

**Try both** and choose based on your application needs. The system preserves both methods so you can switch between them as needed.

## References

- Original Relay Tuning: Lines 338-600 in Giga_Tunnel_PID.ino
- sTune Integration: STunePIDTuner.h/cpp
- Detailed tuning info: TUNING_GUIDE.md
- sTune documentation: https://github.com/Dlloydev/sTune
