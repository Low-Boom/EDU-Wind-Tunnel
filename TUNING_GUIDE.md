# PID Tuning Guide

NOTE: AI tools helped to write this guide. Verify all advice on your own tunnel.

This guide tells you how the airspeed controller works and how to tune it.
For the commands and the serial output, see [README.md](README.md).

---

## Before you start

- The default gains are `Kp=25.0, Ki=10.0, Kd=15.0`. They are only a starting point.
  They are validated in simulation only, not on a real tunnel.
  Tune them for your fan and verify the result on your tunnel.
- Fast fans can need lower gains than the defaults.
- The firmware does not store gains. After a reset, it uses the defaults again.
  - To use your gains again, type the `tune <Kp> <Ki> <Kd>` line that the firmware prints after an auto-tune.
  - To change the defaults, edit `DEFAULT_KP`, `DEFAULT_KI` and `DEFAULT_KD` in the Settings section of `EDU-Wind-Tunnel.ino`, then upload again.
- To stop the fan, type `0` or `stop`. This also stops a running auto-tune.
- Run `fanmap` first (see [Fan map](#fan-map-fanmap)). Both auto-tunes then take their PWM levels from your fan, not from an estimate.

Gain units (all gains use seconds):

| Gain | Unit | Limits |
|---|---|---|
| Kp | PWM per m/s | 1-100 |
| Ki | PWM per (m/s x s) | 0.1-50 |
| Kd | PWM x s per m/s | 0.1-50 |

The same limits apply to manual entry and to both auto-tunes.
The firmware clamps a value outside the limits and tells you, for example:

```
[PID] Kp = 150.000 is outside 1.0-100.0, clamped to 100.000
```

---

## How the controller works

### Update loop and timer mode

Every 200 ms (`UPDATE_INTERVAL_MS`) the firmware reads the sensors, runs the typed commands, runs the controller and prints one data line.

The controller is QuickPID 3.1.9 in timer mode.
The firmware calls `Compute()` exactly once per 200 ms update.
QuickPID scales Ki and Kd for this 200 ms sample time, so you give the gains in seconds-based units.

When the target airspeed is 0, the firmware does not run the controller. It holds the PWM value at 0.

When you set a new target, only the target changes. The controller continues from the present PWM value.
When you set new gains, the integral stays. The PWM value does not jump.

### P and D on measurement

The P and D terms act on the measured airspeed, not on the error.

- A step in the target airspeed does not make the PWM value jump.
- After a target change, the I term (Ki) moves the fan to the new target. The P term (Kp) resists fast changes of the airspeed.
- Against disturbances (for example, a model in the test section), Kp and Kd act as usual.

### Anti-windup by clamping

QuickPID keeps its running sum (the integral and the P-on-measurement part) inside the PWM range 0-255.
Thus the sum cannot grow without limit while the fan runs at full power.

QuickPID also has a conditional anti-windup mode (`iAwCondition`). The firmware does not use it.
In QuickPID 3.1.9 this mode acts on most updates in which the airspeed rises. Then it makes the loop much more aggressive upwards than downwards.

### PWM rate limit (PID mode only)

In PID mode, the PWM value changes by not more than 25 per update (`MAX_PWM_CHANGE_PER_CYCLE`).
The firmware applies this limit around `Compute()`, in two steps:

1. The running sum moves by not more than 25 from its last value. The controller does not store the excess, so there is no windup from the limit.
2. The fan PWM value moves by not more than 25 from its present value.

A change from 0 to 255 takes at least 11 updates (2.2 s).
The limit does not apply to a stop, to the relay levels or to the sTune steps.

### Averaging delay

The firmware calculates the airspeed from a moving average of the last N pressure readings, one reading per update.
Set N with `avg <N>` (1-50, default 5).

The average delays the airspeed signal. The mean delay is (N - 1) / 2 x 200 ms:

| `avg` | Window | Mean delay |
|---|---|---|
| 5 (default) | 1.0 s | 0.4 s |
| 20 | 4.0 s | 1.9 s |
| 50 | 10.0 s | 4.9 s |

This delay is inside the control loop. More delay makes the loop less stable.
In simulation, 20 samples made the default gains oscillate on 3 of 5 fan models. With 5 samples, 4 of 5 fan models were stable.

- A longer average gives a smoother reading. If you increase `avg`, reduce the gains or tune again.
- Run the auto-tunes with the `avg` value that you will use. The tuners measure the averaged airspeed.

---

## Manual tuning

1. Set a target airspeed, for example `10`.
2. Wait until the airspeed is steady. Read `V`, `PWM` and `Err` in the data lines.
3. Change the target by a small step, for example from `10` to `12`.
4. Look at the response: the time to reach the target, the overshoot and any oscillation.
5. Set the target back to `10` and look at the response again.
6. Change one gain with `tune <Kp> <Ki> <Kd>`, for example `tune 20 8 12`.
7. Do steps 3 to 6 again until the response is good.
8. Do the test at a low, a middle and a high target airspeed.
9. Write down the final gains.

Every 20th data line in PID mode shows the present gains and the averaging length, for example `Kp:25.0 Ki:10.0 Kd:15.0 Avg:5`.
The firmware confirms new gains, for example `[PID] Gains set: Kp=20.000 Ki=8.000 Kd=12.000`.

For the effect of each gain, see [Gain-effect reference](#gain-effect-reference).

---

## Fan map (`fanmap`)

The fan map shows how your fan responds to the PWM value: the airspeed and (with the tach wire) the fan RPM at fixed PWM values.
Both auto-tunes use it to find the PWM value for the target airspeed.
Without a map, they use a fixed estimate of 8 PWM per m/s (`FAN_PWM_PER_MPS_ESTIMATE`). This estimate can be wrong for your fan.

### Run

1. Make sure that the tunnel is clear and that the fan can run up to its high speeds.
2. Type `fanmap`.
3. Wait about 2.5 min. The fan runs at PWM 40, 65, 90, ... 240. It holds each value for 15 s, then measures for 2 s.
4. The firmware stops the fan, sets the target airspeed to 0, and shows the table.

`fanmap` ends early if the airspeed goes above the sensor limit. The table then has the values up to that point. The auto-tunes cannot use a higher target in any case.
Type `0` or `stop` to abort; the previous map stays.
The firmware does not store the map. After a reset, run `fanmap` again.

Example (simulated fan with a tach wire):

```
  PWM |   RPM | V (m/s) | Gain (m/s per PWM)
   40 |   470 |    4.71 | -
   65 |   764 |    7.63 | 0.117
   90 |  1059 |   10.56 | 0.117
  115 |  1352 |   13.50 | 0.118
[...]
  215 |  2528 |   25.25 | 0.117
[FANMAP] Done (stopped at the sensor limit) - fan stopped, target 0 m/s
[FANMAP] Highest airspeed: 25.25 m/s. tune and stune now take their PWM levels from this map.
```

Without the tach wire, the RPM column shows `-`.
The map is usable if it has at least two values and its highest airspeed is 2.0 m/s or more. If not, the firmware shows `[ERROR] Fan map not usable: ...` and keeps the previous map.

### How the auto-tunes use the map

The firmware finds the PWM value for the target by linear interpolation between the values of the map.
If the target is above the highest airspeed of the map, it uses the estimate.

- `tune`: the relay levels are this PWM value -20 and +20 (see [Start](#start)). The start text then shows `(from the fan map)`.
- `stune`: the step PWM is this PWM value (see [Step test](#step-test)). The start text shows `[sTune] Step PWM for the target: from the fan map`.

In simulation, a relay tune at 10 m/s without a map did not complete on two of five fan models: PWM 100 (the estimate 80 + 20) did not reach 10 m/s. With a map, it completed on all five. The map put the relay levels and the sTune step within 3 PWM of the correct value for each model.

### Gain column and manual tuning

`Gain` is the change of airspeed per PWM step between two rows: the gain of your fan.
The loop gain is Kp x fan gain. Thus a fan with a higher gain needs lower PID gains for the same response.
The default gains (Kp 25, Ki 10, Kd 15) were validated on a simulated fan with a gain of about 0.12 m/s per PWM.
For a start value on your fan, multiply all three default gains by 0.12 / (your fan gain near your target).
This is a rule of thumb only: it does not include the time constants of your fan. Then tune as [Manual tuning](#manual-tuning) shows, or run an auto-tune.

Example: your fan gain near 10 m/s is 0.24 m/s per PWM (two times 0.12). Start with `tune 12.5 5 7.5`.

The RPM column shows if the fan reaches its rated speed and at which PWM value it starts to turn. Use a relay low level above that PWM value.

---

## Relay auto-tune (`tune`)

The relay auto-tune (Astrom-Hagglund method) runs the fan in a controlled oscillation around the target.
From this oscillation it calculates the gains.

### Start

1. Set a target airspeed of at least 2.0 m/s, for example `10`.
2. Type `tune` (automatic PWM range) or `tune <low> <high>` (your own PWM range, for example `tune 40 100`).

Conditions:

- The target must be at least 2.0 m/s.
- Target + 0.5 m/s must not be more than the sensor limit in the present air.
- `<low>` and `<high>` must be whole numbers from 1 to 255, and `<low>` must be less than `<high>`.

Automatic PWM range: the firmware finds the PWM value for the target in the fan map (see [Fan map](#fan-map-fanmap)). Without a map, or for a target above the map, it estimates 8 PWM per m/s (`FAN_PWM_PER_MPS_ESTIMATE`):

```
center = PWM for the target from the fan map, or round(target x 8)
high   = center + 20, kept within 40-245
low    = center - 20, kept within 20-150
```

Estimate examples (no map): 2 m/s gives 20-40, 5 m/s gives 20-60, 10 m/s gives 60-100, 20 m/s gives 140-180, 25 m/s gives 150-220.

CAUTION: The estimate of 8 PWM per m/s can be wrong for your fan. Then the two levels do not bracket the target and the tune times out.
Run `fanmap` first. Or give the range by hand:

1. Set the target in PID mode and wait until the airspeed is steady.
2. Read the PWM value in the data lines.
3. Type `tune <PWM - 20> <PWM + 20>`. Use a wider range if the tune times out again.

### How it works

- The fan starts at the low PWM level.
- When the airspeed falls below target - 0.5 m/s, the fan goes to the high level.
- When the airspeed rises above target + 0.5 m/s, the fan goes back to the low level.
- The first switch from HIGH to LOW starts the measurement. Each later switch from HIGH to LOW ends one cycle.
- A cycle is valid only if its amplitude is more than 0.5 m/s and its period is more than 2 s. The firmware ignores other cycles.
- The tune ends after 3 valid cycles.

During the tune the data lines start with `[TUNE]` and show the number of valid cycles, for example:

```
[TUNE] 16s | Cycle:2 | V:10.55 | SP:10.00 | PWM:60 | P:66.97 | Temp:20.0 | Err:-0.55
```

### Gain formulas

```
Tu = mean period of the valid cycles (s)
a  = mean amplitude of the valid cycles (m/s)
d  = (high - low) / 2                          relay amplitude (PWM)
Ku = 4 d / (pi a)                              ultimate gain (PWM per m/s)

Pessen integral rule: Kp = 0.7 Ku, Ti = 0.4 Tu, Td = 0.15 Tu
Kp = 0.7 Ku
Ki = Kp / Ti = 1.75 Ku / Tu
Kd = Kp x Td = 0.105 Ku x Tu
```

Ku and Tu are estimates. The hysteresis band moves the oscillation away from the true ultimate point.
The firmware clamps the gains to the limits (Kp 1-100, Ki 0.1-50, Kd 0.1-50) and prints a `[PID] ... clamped` line for each changed gain.

### Hand-over

After a successful tune:

1. The fan moves to the mean relay PWM, (low + high) / 2, within the rate limit.
2. The firmware applies the new gains.
3. The PID controller continues at the same target.

Example from the simulation (target 10 m/s, automatic range 60-100):

```
=====================================
[RELAY] AUTO-TUNE COMPLETE
=====================================
        Period Tu (relay estimate): 5.80 s
        Amplitude: 1.06 m/s
        Ultimate gain Ku (relay estimate): 24.073 PWM per m/s
        Pessen integral rule: Kp=16.851 Ki=7.263 Kd=14.660
[PID] Gains set: Kp=16.851 Ki=7.263 Kd=14.660
[TUNE] PID running at target 10.00 m/s with the new gains
[TUNE] To reuse these gains later: tune 16.851 7.263 14.660
=====================================
```

### Aborts

Each abort stops the fan, sets the target to 0 and keeps the old gains.
The abort line is `[TUNE] Tuning aborted (<reason>) - fan stopped, target 0 m/s, gains unchanged`.

| Reason | Cause | Extra message |
|---|---|---|
| `stopped by user` | You typed `0` or `stop`. | - |
| `relay tune timeout` | Fewer than 3 valid cycles after 120 s. | `[RELAY] Not enough oscillation: <n> of 3 valid cycles` and advice |
| `no airflow` | Airspeed below 1.0 m/s for 15 s in total at the high level. | `[ERROR] No airflow measured at the high relay PWM - check the pitot tubes and the fan` |
| `no valid pressure data` | 5 invalid sensor readings in a row (1 s). | `[ERROR] Differential pressure sensor not responding` |
| `airspeed above MAX_AIRSPEED` | Airspeed above 28.0 m/s. | - |
| `the tuner gave invalid gains` | A gain is not a finite number. | `[ERROR] PID gains must be finite numbers - gains unchanged` |

While a tune runs, the firmware accepts only `0` or `stop`. Other input gets `[BUSY] Tuning in progress - type 0 or stop to abort`.

---

## sTune auto-tune (`stune`)

sTune (library sTune 2.4.0) makes an open-loop step test.
It finds the inflection point of the airspeed curve after a PWM step and calculates a process model from it.
The firmware uses the `Mixed_PID` rule: the average of the Ziegler-Nichols, damped-oscillation, no-overshoot and Cohen-Coon PID rules.

### Start

1. Set a target airspeed of at least 2.0 m/s, for example `10`.
2. Type `stune`.

### Step test

The firmware sets the step PWM from the target: the PWM value for the target from the fan map (see [Fan map](#fan-map-fanmap)), or the estimate of 8 PWM per m/s without a map:

```
step PWM  = PWM for the target from the fan map, or target x 8; kept within 60-200
start PWM = round(0.6 x step PWM)
```

Estimate examples (no map):

| Target | Step (start PWM -> step PWM) |
|---|---|
| 2 m/s | 36 -> 60 |
| 10 m/s | 48 -> 80 |
| 20 m/s | 96 -> 160 |
| 25 m/s or more | 120 -> 200 |

The test has two phases:

1. **SETTLING:** the fan holds the start PWM for 25 s. Then every run starts from a steady airspeed.
2. **TESTING:** the PWM steps to the step PWM. The test ends when sTune finds the inflection point, or after 15 s (75 samples, one per 200 ms).

In simulation at 10 m/s, a complete run took 27 s.
The steps do not use the PWM rate limit.
During the test, the data lines show the phase, for example:

```
[sTune] SETTLING | V:9.97 | SP:10.00 | PWM:48 | P:59.88 | Temp:20.0 | Err:0.03
[sTune] TESTING | V:5.65 | SP:10.00 | PWM:80 | P:19.23 | Temp:20.0 | Err:4.35
```

### Unit conversion

sTune gives Kp per unit of span, and 1/Ti and 1/Td. QuickPID needs gains in PWM and m/s.
The firmware converts them (input span 28.0 m/s, output span 255 PWM):

```
Kp = Kp(sTune) x 255 / 28.0      PWM per m/s
Ki = Kp x (1/Ti)                 = Kp / Ti
Kd = Kp / (1/Td)                 = Kp x Td   (0 for PI rules)
```

The report shows the process gain in m/s per PWM, the dead time and the time constant (Tau).
Example from the simulation:

```
=====================================
[sTune] AUTOTUNING COMPLETE
=====================================
[sTune] Duration: 27 seconds
[sTune] Process model from the step response:
        Process gain: 0.0953 m/s per PWM
        Dead time: 0.80 s
        Time constant (Tau): 4.42 s
[sTune] PID gains (Mixed PID):
        Kp = 29.6671
        Ki = 16.1840
        Kd = 11.2095
=====================================

[PID] Gains set: Kp=29.667 Ki=16.184 Kd=11.210
[TUNE] PID running at target 10.00 m/s with the new gains
[TUNE] To reuse these gains later: tune 29.667 16.184 11.210
=====================================
```

### Rejection

The firmware uses the result only if all these values are finite and:

- Kp > 0, Ki > 0 and Kd >= 0
- Tau > 0, dead time > 0 and process gain > 0

If not, the tune stops with the reason `sTune found no usable step response`. The report then shows `[sTune] NO USABLE STEP RESPONSE`.
This occurs when the airspeed does not rise clearly after the step.
Check the fan power, the PWM wiring and the pitot tubes, or set a higher target (larger step). Then try again.

### Emergency stop and other aborts

- **Emergency stop:** the firmware checks the airspeed during both phases. Above 28.0 m/s (`MAX_AIRSPEED`) it stops the fan with the reason `airspeed above MAX_AIRSPEED`, then prints `[sTune] ABORT: airspeed <V> m/s above the limit of 28.00 m/s`.
  The emergency stop of the sTune library is not used, because it latches and only holds the fan at the start PWM.
- **Timeout:** after 60 s ((25 s + 15 s) x 1.5) the tune stops with the reason `sTune timeout`. This occurs only if the update loop runs much too slowly. Look for `[WARN] Update pass took ...` messages and check the I2C wiring.
- **Sensor:** 5 invalid pressure readings in a row stop the tune (`no valid pressure data`).
- **Stop:** `0` or `stop` stops the tune (`stopped by user`). There is no sTune report.

NOTE: The no-airflow check does not run during sTune.

### Hand-over

After a successful tune, the firmware applies the gains (clamped to the limits).
The PID controller continues at the same target from the step PWM.

### Settings

The sTune settings are in the Settings section of `EDU-Wind-Tunnel.ino`. `startSTune()` gives them to the tuner.

| Name | Default | Meaning |
|---|---|---|
| `STUNE_METHOD` | `sTune::Mixed_PID` | Tuning rule. The code comment above it lists the other rules. |
| `STUNE_TEST_TIME_S` | 15 | Test time (s). Keep it close to the time constant of the fan. |
| `STUNE_SETTLE_TIME_S` | 25 | Time at the start PWM before the step (s). |
| `STUNE_START_FRACTION` | 0.6 | Start PWM as a fraction of the step PWM. |
| `STUNE_STEP_PWM_MIN`, `STUNE_STEP_PWM_MAX` | 60, 200 | Limits of the step PWM. |
| `FAN_PWM_PER_MPS_ESTIMATE` | 8.0 | PWM per m/s estimate. Both auto-tunes use it. |

NOTE: In simulation, a test time of 10 s underestimated slow fans and the tuned loop oscillated.

NOTE: The PI rules (for example `sTune::Mixed_PI`) give Kd = 0. The firmware then clamps Kd to 0.1 and prints `[PID] Kd = 0.000 is outside 0.1-50.0, clamped to 0.100`.

CAUTION: sTune results can be aggressive on fast fans or fans with a high gain. After each tune, test the loop with small target steps.

---

## Troubleshooting by symptom

| Symptom | Possible cause | Action |
|---|---|---|
| The airspeed oscillates around the target and does not settle. | The gains are too high for this fan, or the averaging delay is too long. | Reduce Kp and Ki by about 20 % (from the defaults: `tune 20 8 15`). If you increased `avg`, type `avg 5`. Then run `tune` or `stune` again. |
| The airspeed overshoots after a target change, then settles. | Ki is too high for the present Kp. | Reduce Ki (`tune 25 8 15`), or increase Kp a little. |
| The airspeed moves slowly to a new target, with no overshoot. | Ki is too low. With P on measurement, Ki moves the fan to a new target. | Increase Ki in small steps (`tune 25 12 15`). For large steps, the rate limit also adds time: 0 to 255 takes 2.2 s. |
| The PWM value jumps from line to line at a steady target. The fan sound changes. | Kd amplifies the sensor noise. | Reduce Kd (`tune 25 10 8`). Check the pitot tubes. Type `recal` to set the sensor zero again (this stops the fan). |
| The PWM value stays at 255 and the airspeed stays below the target. | The fan cannot give this airspeed. | Set a lower target. The running sum stays inside 0-255, so the loop recovers without a long overshoot. |
| The fan stops with `[ERROR] No airflow measured at full fan power - check the pitot tubes and the fan`. | The airspeed stayed below 1.0 m/s at PWM 250 or more for 15 s in total. | Make sure that the pitot tubes are connected to the correct ports. Check the fan power and the PWM wiring. |
| The loop overshoots or oscillates after an auto-tune. | The tuner result is too aggressive for this fan. | Reduce Kp and Ki by about 20 %. Or try the other auto-tune. |
| The relay tune stops with `relay tune timeout`. | The two PWM levels do not bracket the target. | Give the range by hand (see [Relay auto-tune](#relay-auto-tune-tune)). |
| sTune stops with `[sTune] NO USABLE STEP RESPONSE`. | The airspeed did not rise clearly after the step. | Check the fan and the pitot tubes. Set a higher target. Or use `tune`. |
| sTune stops with `sTune timeout`. | The update loop ran too slowly. | Look for `[WARN] Update pass took ...`. Check the I2C wiring (long cables, loose connectors). |

---

## Gain-effect reference

The firmware uses P and D on measurement. This changes the usual table for target changes.

| Gain | Increase | Too high | Too low |
|---|---|---|---|
| Kp | Faster correction of disturbances. More damping after a target change. | Oscillation. | Slow correction of disturbances. More overshoot after a target change. |
| Ki | Faster move to a new target. Removes the remaining error faster. | Overshoot, slow oscillation. | Slow move to a new target. The error stays longer. |
| Kd | More damping of fast airspeed changes. | PWM value jumps with the sensor noise. The loop can oscillate. | Less damping, more overshoot. |

---

## References

- [QuickPID](https://github.com/Dlloydev/QuickPID) (David Lloyd)
- [sTune](https://github.com/Dlloydev/sTune) and the [sTune wiki](https://github.com/Dlloydev/sTune/wiki) (David Lloyd)
