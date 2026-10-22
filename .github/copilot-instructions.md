# Copilot Instructions for EDU-Wind-Tunnel

## Project overview

Arduino firmware (version 1.4.1, `FIRMWARE_VERSION` in the `.ino`) that holds the airspeed of a small educational wind tunnel at a target value. A DFRobot SEN0343 (LWLP5000) differential pressure sensor measures the room air against a static pressure ring in the test section. A DFRobot SEN0665 (BMP585) barometric sensor gives the air density. QuickPID drives the fan with a PWM value (0-255). Two auto-tuners (relay and sTune) find PID gains. All control is through serial commands at 115200 baud.

Readers of the docs and the serial messages are students, teachers, and makers. Some are not native English speakers.

## Repository structure

```
EDU-Wind-Tunnel/                 # Firmware source: PlatformIO src_dir, also an Arduino IDE sketch folder
  EDU-Wind-Tunnel.ino            # Settings, setup()/loop(), sensors, PID, safety checks,
                                 #   relay auto-tune, serial commands, output
  PlatformConfig.h               # Board selection: pins, PWM frequency, SERIAL_TX_CAN_BLOCK; #error for other boards
  SerialOut.h/.cpp               # All text output: bounded wait on a native USB port, serialHasRoom()
  I2CScanner.h/.cpp              # Scans Wire (and Wire1 on the Uno R4 WiFi), device registry, direct probe
  STunePIDTuner.h/.cpp           # Wrapper around sTune 2.4.0 (step test, unit conversion, e-stop, report)
.github/workflows/compile-sketch.yml  # CI "Compile sketch" (PlatformIO and Arduino builds)
platformio.ini                   # PlatformIO (primary build): environments, pinned versions, serial monitor
3D Print Files/                  # Changed tunnel parts (static pressure taps)
img/                             # Wiring photos (Uno R4, AC Infinity)
README.md                        # Entry page: quick start, commands, output, configuration, troubleshooting
HARDWARE.md                      # Parts, pins, wiring, pitot tubes
TUNING_GUIDE.md                  # Controller, manual tuning, relay auto-tune, sTune
THIRD_PARTY_LICENSES.md          # Library and tunnel-design licenses
LICENSE                          # MIT (firmware and docs)
```

## Supported boards

PlatformIO is the primary build; the Arduino IDE is the alternative. Keep both working.

| Board | Core | PlatformIO environment | Arduino FQBN |
|---|---|---|---|
| ESP32-P4, chip before v3.0 | arduino-esp32 3.3.12 | `esp32p4` (default) | `esp32:esp32:esp32p4` |
| ESP32-P4, chip v3.0 or newer | arduino-esp32 3.3.12 | `esp32p4_r3` | `esp32:esp32:esp32p4:ChipVariant=postv3` |
| Arduino Uno R4 Minima | Arduino UNO R4 Boards (Renesas) | `uno_r4_minima` | `arduino:renesas_uno:minima` |
| Arduino Uno R4 WiFi | Arduino UNO R4 Boards (Renesas) | `uno_r4_wifi` | `arduino:renesas_uno:unor4wifi` |

`PlatformConfig.h` selects the board with `ARDUINO_ARCH_ESP32` + `CONFIG_IDF_TARGET_ESP32P4`, or `ARDUINO_UNOR4_MINIMA` / `ARDUINO_UNOR4_WIFI`. Any other ESP32 chip and any other board stop the build with `#error`. Giga R1, Mega 2560, Uno R3, Nano, M5 Atom, ESP32-S3 and generic ESP32 are removed. Do not add code paths for them.

Board facts used by the code:

- ESP32-P4: `FAN_PWM_PIN` 4, `FAN_PWM_FREQUENCY_HZ` 25000, I2C pins -1 (the selected board's default: SDA 7 / SCL 8 on the ESP32P4 Dev Module, 53 / 54 on the M5Tab5 board entry). Do not use GPIO 14-19 (SDIO link to the on-board ESP32-C6). `SERIAL_TX_CAN_BLOCK` is 1 only with USB CDC on boot (`-DARDUINO_USB_CDC_ON_BOOT=1` in PlatformIO `build_flags`, "USB CDC On Boot: Enabled" in the Arduino IDE).
- Uno R4: `FAN_PWM_PIN` 9, core default PWM frequency (`FAN_PWM_FREQUENCY_HZ` 0). The I2C pins are fixed (`Wire` = SDA/SCL header, `Wire1` = Qwiic on the WiFi). `I2C_SDA_PIN`/`I2C_SCL_PIN` must stay -1 (`#error` otherwise). `SERIAL_TX_CAN_BLOCK` is 1 on the Minima (native USB), 0 on the WiFi.
- `STATUS_LED_PIN` is `LED_BUILTIN` if the core variant defines it, else -1.

## Libraries

| Library | Git tag | Include | Notes |
|---|---|---|---|
| QuickPID (David Lloyd) | `3.1.9` | `<QuickPID.h>` | PID controller |
| sTune (David Lloyd) | `2.4.0` | `<sTune.h>` (via `STunePIDTuner.h`) | Required, not optional |
| DFRobot_LWLP | `V1.0.0` | `<DFRobot_LWLP.h>` | Differential pressure sensor |
| DFRobot_BMP58X | `v1.0.1` | `"DFRobot_BMP58X.h"` | Barometric sensor; depends on DFRobot_RTU |
| DFRobot_RTU | `V1.0.6` | (dependency) | |
| Wire | core | `<Wire.h>` | I2C |

The versions are pinned in `platformio.ini` (`lib_deps`) and in `.github/workflows/compile-sketch.yml`. `THIRD_PARTY_LICENSES.md` is the version list for users (the README links to it). If you change a version, change it in these three files.

## Architecture

`loop()` (in `EDU-Wind-Tunnel.ino`):

1. If `controlMode == MODE_STUNE`, call `runSTune()` on every pass (sTune keeps its own sample timing).
2. Once per `UPDATE_INTERVAL_MS` (200 ms):
   1. `readPressureAveraged()` and `calcAirspeed(pressure, readAirDensity())` -> `currentAirspeed`.
   2. `pollSerialCommands()` -> `endSerialLine()` -> `handleCommand()`.
   3. `checkPressureSensor()` (after the commands, so a new target is cancelled before it acts).
   4. Control, only if the mode did not change in this pass: `MODE_PID` -> `runPidControl()` + `checkAirflow(fanPwm >= NO_AIRFLOW_PWM, "full fan power")`; `MODE_RELAY_TUNE` -> `runRelayTune()`.
   5. `printDataLine()` and `warnIfSlowPass()`.

Main state: `targetAirspeed` (setpoint), `currentAirspeed` (input), `pidOutput` (PID output, also written by the tuners), `fanPwm` (PWM last written, -1 = never), `kp`/`ki`/`kd`, `controlMode` (`MODE_PID`, `MODE_RELAY_TUNE`, `MODE_STUNE`), `pressureAverageSamples`, `pressureOffset`, `invalidPressureReads`, `airDensity`, `airTemperatureC`, `noAirflowDrivenMs`, `myPID` (QuickPID), `sTuner` (`STunePIDTuner *`).

Key functions: `setFanPwm()`, `setFanPwmRateLimited()`, `stopFan()`, `stopAll(reason)`, `pidBumplessStart()`, `applyGains()`, `clampGain()`, `startRelayTune()`, `completeRelayTune()`, `startSTune()`, `abortTuning(reason)`, `finishTuning()`, `calibratePressureSensor()`, `setPressureAverageSamples()`, `setTargetAirspeed()`, `setManualGains()`, `maxAirspeed()`, `printHelp()`, `haltWithError()`. In the helpers: `scanAllI2CBuses()`, `getDeviceBus()`, `probeDeviceBus()`; `STunePIDTuner::startTuning()`, `update()`, `getTunedGains()`, `printReport()`, `reset()`.

Settings are `const` values in the "Settings" section at the top of the `.ino` (for example `MAX_AIRSPEED`, `UPDATE_INTERVAL_MS`, `DEFAULT_AVG_SAMPLES`, `DEFAULT_KP`/`DEFAULT_KI`/`DEFAULT_KD`, `MAX_PWM_CHANGE_PER_CYCLE`, `STUNE_METHOD`). Board settings are `#define`s in `PlatformConfig.h`. Runtime changes (`tune`, `avg`) are not stored; there is no EEPROM or Preferences code.

## Rules (do not break)

- **Stop is always available.** `handleCommand()` checks the stop before the word-count check and the `[BUSY]` check. A stop is the first word `stop` (any case) or one word with the value 0. `stopAll(reason)` stops the fan in every mode (a running tune is aborted with gains unchanged). Do not add code that delays `pollSerialCommands()` by more than one update. Only `calibratePressureSensor()` blocks on purpose, and it stops the fan first.
- **Fan pin LOW first.** `setup()` drives `FAN_PWM_PIN` LOW (`pinMode` + `digitalWrite`) before anything else, because a floating PWM pin can run some fans at full speed. Keep it the first statement.
- **One writer for the fan.** After that, `setFanPwm()` is the only function that writes the fan PWM (`analogWrite`). It clamps to 0-255 and skips unchanged values. On the ESP32, it attaches the LEDC channel with `analogWrite(pin, 0)` before `analogWriteFrequency()`, because arduino-esp32 3.0.0-3.3.0 ignore the frequency before attach. Use `setFanPwmRateLimited()` in PID mode and `stopFan()` to stop.
- **Print only through `SerialOut`.** On a native USB port `Serial.print()` blocks while the host does not read. `SerialOut` waits at most 50 ms, then drops text and later reports the lost bytes, so `loop()` keeps running. Never call `Serial.print()`/`Serial.write()` directly (input uses `Serial`). Routine output (data lines, relay status lines, sTune progress) must check `serialHasRoom()` and be skipped whole. Stop the fan before you print a stop or abort message.
- **No QuickPID `Reset()`.** It zeroes the integral and drops the output to about 0. Re-initialise with `pidBumplessStart()` (manual -> timer). Change gains only through `applyGains()` (finite check, clamp to `KP_MIN`..`KD_MAX`, `SetTunings()` keeps the integral).
- **QuickPID configuration stays as is:** timer mode, one `Compute()` per update, `SetSampleTimeUs(UPDATE_INTERVAL_MS * 1000)`, `pOnMeas`, `dOnMeas`, `iAwClamp` (not `iAwCondition`, which is flawed in 3.1.9), output limits fixed at 0-255. The PWM rate limit acts around `Compute()` (`GetOutputSum()`/`SetOutputSum()` plus `setFanPwmRateLimited()`), never through `SetOutputLimits()`.
- **Calibration stops the fan.** `calibratePressureSensor()` calls `stopFan()` first, waits `CALIBRATION_SETTLE_S`, and keeps the old offset if fewer than half of the samples are valid. Any airflow during calibration would become the zero, so never start the fan in or around it. Invalid readings never enter the moving average.
- **One control mode at a time.** While a tuner runs, every command except stop gets `[BUSY] Tuning in progress - type 0 or stop to abort`.
- **Safety checks stay active:** `checkPressureSensor()` (5 invalid readings, code `LWLP_INVALID_CODE` 0x3FFF), `checkAirflow()` (PID mode and relay HIGH level), relay over-speed abort and the sTune wrapper e-stop at `MAX_AIRSPEED`, target cap at `maxAirspeed()`, fatal halt with the fan off in `haltWithError()`.
- **Board support only in `PlatformConfig.h`.** Keep the `#error` for unsupported boards. Do not put board pins in the `.ino`.
- **Strict input parsing.** Parse user numbers only with `parseDecimal()` / `parseWholeNumber()`. Do not use `atof()`, `toFloat()` or `toInt()` directly on typed text.
- **sTune wrapper.** Each run builds a new sTune object in zero-filled storage (sTune 2.4.0 leaves members uninitialised and latches its e-stop). Call `Configure()` once per run: sTune leaks at least 16 bytes of heap per run (75 samples).

## Serial protocol

- One command per line, LF, CR or CR+LF. Until the first CR or LF since reset, a line also ends after `SERIAL_LINE_TIMEOUT_MS` (1000 ms) without a new character (`lineEndingSeen`).
- Max 63 characters (`SERIAL_LINE_SIZE` 64), max 4 words (`MAX_COMMAND_WORDS`). Keywords are case-insensitive (`wordIs()`).
- Commands: `<m/s>`, `0`/`stop`, `tune`, `tune <low> <high>`, `tune <Kp> <Ki> <Kd>`, `stune`, `recal [N]`, `avg <N>`, `help`.
- Data line: `<t>s | V:<m/s> | SP:<m/s> | PWM:<0-255> | P:<Pa> | Temp:<C> | Err:<m/s>`, with ` | Kp:.. Ki:.. Kd:.. Avg:N` every 20th line in PID mode and ` | Skipped:<n>` after skipped lines. Users log and parse this format: change it only on purpose and update the README.
- Message tags: `[INIT]`, `[I2C]`, `[CALIB]`, `[READY]`, `[SERIAL]`, `[CONFIG]`, `[PID]`, `[STOP]`, `[TUNE]`, `[RELAY]`, `[sTune]`, `[BUSY]`, `[WARN]`, `[ERROR]`, `[FATAL]`. Serial output is ASCII only.

## How to check a build

- **CI:** the workflow "Compile sketch" (`.github/workflows/compile-sketch.yml`) runs on every push and pull request (and on manual start). Job `platformio` runs `pio run` for the four environments of `platformio.ini`. Job `compile` runs `arduino/compile-sketches` for the four FQBNs above with the pinned library versions and reports warnings. A change is ready only when all eight jobs pass.
- **PlatformIO (local, primary):** `pio run -e esp32p4`, `pio run -e esp32p4_r3`, `pio run -e uno_r4_minima`, `pio run -e uno_r4_wifi`.
- **arduino-cli (local, for the Arduino IDE alternative):**

  ```sh
  URL=https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core update-index --additional-urls "$URL"
  arduino-cli core install esp32:esp32@3.3.12 --additional-urls "$URL"
  arduino-cli core install arduino:renesas_uno
  ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true arduino-cli lib install --git-url \
    https://github.com/Dlloydev/QuickPID.git#3.1.9 \
    https://github.com/Dlloydev/sTune.git#2.4.0 \
    https://github.com/DFRobot/DFRobot_LWLP.git#V1.0.0 \
    https://github.com/DFRobot/DFRobot_RTU.git#V1.0.6 \
    https://github.com/DFRobot/DFRobot_BMP58X.git#v1.0.1
  for fqbn in esp32:esp32:esp32p4 esp32:esp32:esp32p4:ChipVariant=postv3 \
              arduino:renesas_uno:minima arduino:renesas_uno:unor4wifi; do
    arduino-cli compile --fqbn "$fqbn" --warnings all EDU-Wind-Tunnel || break
  done
  ```

- There is no unit-test framework. On hardware, open the Serial Monitor at 115200 baud and check these items:
  - The boot reaches `[READY] System initialized`.
  - The airspeed reaches a target (for example `5`).
  - `0` and `stop` stop the fan within one update, also during `tune` and `stune`.

## Code style

- Follow the style of the file you change: `camelCase` for variables and functions, `UPPER_SNAKE_CASE` for constants and macros. Prefer `const` over `#define` for typed constants (the `#define`s in `PlatformConfig.h` are for the preprocessor checks).
- Comments explain why, in short plain sentences. Use Doxygen comments (`/** ... */`) for functions declared in headers, as in `I2CScanner.h` and `STunePIDTuner.h`.
- `#include` order: Arduino core, libraries, local headers.
- Serial messages and docs use simple English (ASD-STE100 style): short sentences, one term per concept ("target airspeed", "airspeed", "fan", "PWM value", "differential pressure sensor", "barometric sensor"). Docs must quote messages, commands and values exactly as the code prints them.

## Common tasks

- **New serial command:** add it in `handleCommand()`, after the stop and `[BUSY]` checks. Add a line to `printHelp()`. Update the command table in `README.md`.
- **New setting:** add a `const` in the Settings section of the `.ino`. If users should change it, add it to the configuration table in `README.md`.
- **Board change:** edit the board's branch in `PlatformConfig.h`, the CI matrix in `compile-sketch.yml`, the environments in `platformio.ini`, and `HARDWARE.md` / `README.md`.
- **I2C scanning:** edit `I2CScanner.cpp`; keep the header comments in `I2CScanner.h` true.

## Commits

- Subject line in the imperative mood, about 50-70 characters, no trailing period (for example "Use 5-sample pressure averaging by default").
- Body: what changed and why, wrapped at about 72 characters. Name the board and the evidence (build, log, simulation) for behaviour changes.
