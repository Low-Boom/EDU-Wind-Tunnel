# Advanced PID Tuning Guide

**Recommendations from Claude Sonnet 4.5 on PID tuning for this repository. The content is determined to be suitably accurate by the author, but take all AI generated content with skepticism and critical thinking.**
**Repository:** [EDU-wind-tunnel](https://github.com/Low-Boom/EDU-wind-tunnel)

---

## Table of Contents
- [PID Basics](#pid-basics)
- [Understanding Each Term](#understanding-each-term)
- [Tuning Strategies](#tuning-strategies)
- [Auto-Tune Deep Dive](#auto-tune-deep-dive)
- [Manual Tuning Process](#manual-tuning-process)
- [Performance Optimization](#performance-optimization)
- [Troubleshooting Performance](#troubleshooting-performance)
- [Reference Tables](#reference-tables)

---

(Previous content from earlier in the message...)

### Problem: Oscillation

**Symptoms:**
- Bounces around setpoint
- Never settles
- ±0.5-2 m/s variation

**Diagnosis:**
```
Sustained oscillation = Too much gain

Check:
1. Kp too high (most common)
2. Ki too high
3. Mechanical resonance
4. Sensor noise
```

**Solutions:**
```
Immediate:
tune 12 4 15  # Low Kp, low Ki, high Kd

Then test incrementally:
tune 15 4 15
tune 18 4 15
tune 20 5 15  # Find stability threshold
```

### Problem: Integral Windup

**Symptoms:**
- Large overshoot after long error
- Slow to recover from disturbances
- PWM saturates at 255

**Diagnosis:**
```
Integral term accumulates when PWM saturated
System can't respond, but integral keeps growing
When error reduces, huge overshoot results
```

**Solutions:**
1. **Code already has anti-windup** (conditional integration)
2. Reduce Ki if still happening
3. Reduce setpoint if consistently hitting PWM limits

```
Before: tune 20 15 12  (high Ki)
After:  tune 20 8 12   (moderate Ki)
```

### Problem: Noise Amplification

**Symptoms:**
- PWM output jumpy
- Motor "buzzes"
- Output changes rapidly even when stable

**Diagnosis:**
```
Kd amplifies measurement noise
Check derivative term contribution
```

**Solutions:**
```
1. Reduce Kd:
   tune 20 8 8   # Was Kd=12

2. Increase averaging:
   avg 25        # More smoothing

3. Check sensor mounting:
   Vibration → noise → derivative action
```

---

## Reference Tables

### Quick Reference: Gain Effects

| Gain | ↑ Increase Effect | ↓ Decrease Effect |
|------|-------------------|-------------------|
| **Kp** | Faster response | Slower response |
|  | More overshoot | Less overshoot |
|  | May oscillate | More stable |
|  | Smaller steady error | Larger steady error |
| **Ki** | Eliminates SS error | May have SS error |
|  | More overshoot | Less overshoot |
|  | Windup risk | Stable |
|  | Slower disturbance recovery | Faster recovery |
| **Kd** | Less overshoot | More overshoot |
|  | Dampens oscillation | May oscillate |
|  | Sensitive to noise | Less noise sensitive |
|  | Faster settling | Slower settling |

### Typical Gain Ranges by Application

| Application | Kp | Ki | Kd | Avg | Notes |
|-------------|----|----|----|----|-------|
| **Fast Response** | 30-40 | 10-15 | 15-20 | 3-5 | Competition/demo |
| **Balanced** | 20-25 | 6-10 | 10-15 | 15-20 | General use |
| **Stable** | 15-20 | 4-6 | 10-12 | 25-30 | Educational |
| **Precision** | 20-25 | 12-18 | 12-15 | 15-20 | Research/testing |

### Tuning Decision Tree

```
START
  ↓
Response too slow? ──YES──> Increase Kp by 30%
  ↓ NO
  ↓
Overshoots? ──YES──> Reduce Kp by 20%, Increase Kd by 30%
  ↓ NO
  ↓
Oscillates? ──YES──> Reduce Kp by 40%, Reduce Ki by 50%
  ↓ NO
  ↓
Steady-state error? ──YES──> Increase Ki by 30%
  ↓ NO
  ↓
✅ OPTIMAL TUNING
```

### Auto-Tune Success Checklist

Before auto-tuning:
- [ ] Sensor calibrated (`recal`)
- [ ] Target speed set (>2 m/s)
- [ ] Motor responds to PWM commands
- [ ] System can reach target speed
- [ ] No obstructions in tunnel

During auto-tune:
- [ ] Motor oscillating between HIGH and LOW
- [ ] Airspeed oscillating around target
- [ ] State changes visible in serial output
- [ ] 3 complete cycles detected

After auto-tune:
- [ ] Gains calculated successfully
- [ ] Gains within expected ranges
- [ ] Test with small setpoint change
- [ ] Fine-tune if needed

### Common Gain Combinations

| Scenario | Kp | Ki | Kd | Expected Behavior |
|----------|----|----|----|--------------------|
| **Default** | 31 | 12 | 10 | Moderate response, some overshoot |
| **Conservative** | 15 | 5 | 8 | Slow, stable, no overshoot |
| **Aggressive** | 35 | 15 | 18 | Fast, 10-20% overshoot |
| **Precision** | 22 | 15 | 13 | Slow approach, exact setpoint |
| **Noisy System** | 18 | 6 | 6 | Low derivative, more averaging |

---

## Advanced Topics

### Gain Scheduling

For systems needing different gains at different speeds:

```cpp
// Add to code after line with myPID.SetTunings()
void updateGainSchedule() {
    float speed = desiredAirSpeed;
    
    if (speed < 8.0) {
        // Low speed: gentle, precise
        Kp = 18.0;
        Ki = 6.0;
        Kd = 10.0;
    } else if (speed < 15.0) {
        // Medium speed: balanced
        Kp = 22.0;
        Ki = 8.0;
        Kd = 12.0;
    } else {
        // High speed: aggressive
        Kp = 28.0;
        Ki = 10.0;
        Kd = 15.0;
    }
    
    myPID.SetTunings(Kp, Ki, Kd);
}
```

Call in `loop()` when setpoint changes.

### Feed-Forward Control

For predictable disturbances:

```cpp
// Add feed-forward term based on target
float feedForward = desiredAirSpeed * 10.0;  // ~10 PWM per m/s
currPWM += feedForward;
```

Reduces reliance on feedback, faster response.

### Adaptive Tuning

Auto-adjust gains based on performance:

```cpp
// Pseudo-code concept
if (overshoot > 20%) {
    Kp *= 0.9;  // Reduce by 10%
}
if (settlingTime > 10.0) {
    Kp *= 1.1;  // Increase by 10%
}
```

Requires performance metrics tracking.

---

## Conclusion

**Remember:**
1. Start conservative, increase gradually
2. Auto-tune first, manual fine-tune second
3. Test thoroughly at different setpoints
4. Document your final gains
5. Safety first - start slow!

**Tuning is iterative:**
```
Tune → Test → Observe → Adjust → Repeat
```

**When in doubt:**
```
tune 20 8 12    # Good starting point for most systems
```

---

## sTune Advanced Autotuning (Optional)

### What is sTune?

sTune is an advanced open-loop PID autotuner that uses a novel inflection point test method. It's an alternative to the relay tuning method and offers several advantages:

**Benefits:**
- **Faster tuning**: Typically completes in ½τ (half the system time constant)
- **More stable**: Uses conservative tuning rules for minimal overshoot
- **Automatic step sizing**: Determines optimal test parameters automatically
- **Better process characterization**: Provides dead time, time constant, and process gain

**How it works:**
1. Applies a step change to the output (PWM)
2. Monitors the system response curve
3. Detects the inflection point using a moving tangent method
4. Calculates process characteristics (gain, dead time, tau)
5. Applies tuning rules to determine optimal PID gains

### Using sTune

**Prerequisites:**
- Install sTune library via Arduino Library Manager
- System must have S-shaped step response (first-order with delay)

**Basic usage:**
```
1. Set target airspeed: 10
2. Type: stune
3. Wait for tuning to complete (~30-60 seconds)
4. Gains automatically applied
```

**sTune vs Relay Tuning:**

| Feature | Relay Tuning | sTune |
|---------|--------------|-------|
| Method | Oscillation-based | Step response |
| Duration | ~30-60 seconds | ~30-60 seconds |
| Tuning style | Aggressive | Conservative |
| Overshoot | Some overshoot | Minimal/none |
| Process info | Limited | Detailed |
| Best for | Fast response | Stable operation |

**When to use sTune:**
- Educational environments where stability is critical
- Research applications requiring precise control
- Systems where overshoot must be minimized
- When you want detailed process characterization

**When to use relay tuning:**
- Fast, responsive control needed
- Competition or demonstration scenarios
- Quick tuning iterations desired
- Some overshoot is acceptable

### sTune Tuning Methods

sTune provides multiple tuning methods (configured in code):

1. **NoOvershoot_PID** (default): Conservative, stable response
2. **ZN_PID**: Ziegler-Nichols, moderate aggressiveness
3. **DampedOsc_PID**: Balanced response
4. **CohenCoon_PID**: Good for systems with delay
5. **Mixed_PID**: Average of all methods

The default configuration uses **NoOvershoot_PID** for maximum stability.

### Example sTune Output

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

### Troubleshooting sTune

**Problem: Tuning times out**
- Increase test time estimate in code
- Check motor/fan responds to PWM
- Verify sensors are working
- Ensure target is achievable

**Problem: Poor gains**
- System may not have S-curve response
- Try increasing output step size
- Use relay tuning instead
- Check for mechanical issues

**Problem: Error during tuning**
- Target too low (must be ≥2 m/s)
- Emergency stop triggered
- Sensor malfunction
- Review serial output for details

### Customizing sTune Parameters

Advanced users can modify parameters in `STunePIDTuner.cpp`:

```cpp
// Change tuning method (in constructor)
_tuner = new sTune(input, output, 
                   sTune::ZN_PID,        // Try different method
                   sTune::directIP,      // Or direct5T for full test
                   sTune::printSUMMARY);

// Adjust in startTuning() call
tuner->startTuning(
    setpoint,     // Target speed
    30.0,         // Input span (max airspeed)
    255.0,        // Output span (PWM range)
    0.0,          // Output start
    100.0,        // Output step (adjust for system)
    60,           // Test time estimate (seconds)
    5,            // Settle time (seconds)
    300           // Number of samples
);
```

### References

- [sTune GitHub Repository](https://github.com/Dlloydev/sTune)
- [sTune Wiki](https://github.com/Dlloydev/sTune/wiki)
- [Inflection Point Method](https://en.wikipedia.org/wiki/Inflection_point)

---

## Conclusion
