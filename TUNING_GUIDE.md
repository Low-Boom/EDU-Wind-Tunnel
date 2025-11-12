# Advanced PID Tuning Guide

**Recommendations from Claude Sonnet 4.5 on PID tuning for this repository. The content is determined to be suitably accurate by the author, but take all AI generated content with skepticism and critical thinking.**

**Version:** 1.0 (Stable)  
**Last Updated:** 2025-11-09  
**Author:** Low-Boom  
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
