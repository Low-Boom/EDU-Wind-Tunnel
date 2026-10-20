# EDU Wind Tunnel Controller

Firmware for a microcontroller board that keeps the airspeed in a small educational wind tunnel at a target value.

Developed by [Louis M. Edelman](https://www.researchgate.net/profile/Louis-Edelman) and Olivia Coulon.

Debugging assistance provided by Anthropic Claude Sonnet 4.5 and verified by humans. Claude Code (Anthropic) helped with the version 1.3 changes. For their test status, see [Limitations](#limitations).

Developed for use with the [Modular Wind Tunnel for STEM Education](https://www.printables.com/@JerrodH) by [Jerrod Hofferth](https://jerrod.hofferth.net/).

[![Version](https://img.shields.io/badge/version-1.3-brightgreen.svg)](https://github.com/Low-Boom/EDU-wind-tunnel/releases)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ESP32-P4](https://img.shields.io/badge/ESP32--P4-supported-E7352C?logo=espressif)](https://www.waveshare.com/product/arduino/boards-kits/esp32-p4/esp32-p4-wifi6-touch-lcd-7b.htm)
[![Uno R4](https://img.shields.io/badge/Arduino-Uno%20R4-00979D?logo=arduino)](https://store-usa.arduino.cc/pages/uno-r4)

---

## Features

- Closed-loop control of the airspeed with a PID controller (QuickPID), updated every 200 ms.
- Two auto-tune methods: relay auto-tune (`tune`) and sTune step-response auto-tune (`stune`).
- Manual PID gains with one serial command.
- Air density from the temperature and pressure of the barometric sensor.
- Zero calibration of the differential pressure sensor at start-up and on command.
- Automatic search for both sensors on the I2C buses of the board.
- Safety functions that stop the fan (see [Safety](#safety)).

## Supported hardware

| Microcontroller board | Arduino board package | Status |
|---|---|---|
| ESP32-P4 boards, for example Waveshare ESP32-P4-WiFi6-Touch-LCD-7B or M5Stack Tab5 | "esp32 by Espressif Systems" | Recommended |
| Arduino Uno R4 Minima, Arduino Uno R4 WiFi | "Arduino UNO R4 Boards" | Alternative |

For all other boards, `PlatformConfig.h` stops the build with an `#error`.

You also need:

- The DFRobot SEN0343 (LWLP5000) differential pressure sensor
- The DFRobot SEN0665 (BMP585) barometric sensor
- The static pressure ring parts and the pressure tubing
- A PWM fan with its own power supply
- A logic level shifter for some board and fan combinations

[HARDWARE.md](HARDWARE.md) gives the parts list, the pins, and the wiring for each board and fan.

## Safety

**WARNING:** Connect the fan to its own power supply. Do not power the fan from the microcontroller board.

**WARNING:** Attach the fan and all parts tightly. Keep fingers, hair, loose clothing, and tools away from the fan when it has power.

**WARNING:** Until the firmware starts (at power-up, during a reset or an upload), the board does not drive the fan PWM pin. Some fans then run at full speed. Disconnect the fan power before you upload. To prevent this, add a pull-down resistor (see [HARDWARE.md](HARDWARE.md#fan-pwm-line-at-power-up-and-during-uploads)).

**WARNING:** Start with a low target airspeed (2 to 5 m/s). Increase it in small steps.

**WARNING:** To stop the fan, type `0` or `stop` and send the line. This also stops a running auto-tune. Some exceptions apply (see [Limitations](#limitations)). If the serial connection does not work, disconnect the fan power supply.

The firmware has these safety functions:

| Function | What it does | Message |
|---|---|---|
| Stop | `0` or `stop` sets the PWM value to 0 and the target airspeed to 0 within one update (200 ms). | `[STOP] Fan stopped, target 0 m/s (stopped by user)` |
| Calibration | The start-up calibration and `recal` stop the fan first. | `Fan stopped. Keep the tunnel still:` |
| No airflow | The fan stops if the airspeed stays below 1.0 m/s for 15 s in total at full power. Full power is a PWM value of 250 or more. During a relay auto-tune, the high PWM level counts as full power. This check does not run during `stune`. | `[STOP] Fan stopped, target 0 m/s (no airflow)` |
| Pressure sensor fault | After 5 invalid readings in a row (1 s), the fan stops in all modes. | `[STOP] Fan stopped, target 0 m/s (no valid pressure data)` |
| Airspeed limit | The firmware changes a target above the sensor limit to the limit. The limit is 28.0 m/s, or less in dense (cold) air. | `[WARN] Capping at 28.00 m/s (sensor limit in the present air)` |
| Over-speed during an auto-tune | An airspeed above 28.0 m/s stops the auto-tune and the fan. | `[TUNE] Tuning aborted (airspeed above MAX_AIRSPEED) - fan stopped, target 0 m/s, gains unchanged` |
| Sensor fault at start-up | The firmware halts with the fan off. The board LED blinks if the board has an `LED_BUILTIN` (see [HARDWARE.md](HARDWARE.md#check-the-sensors-at-start-up)). | `[FATAL] System halted with the fan off. Fix the fault and reset the board.` |
| Blocked serial port | On a native USB port, the firmware waits at most 50 ms for a computer that does not read the port. It skips data lines, and other text is lost. Thus the stop command, the PID and the checks continue to work. | `Skipped:<n>` at the end of the next data line; `[WARN] <n> bytes of output lost: the host did not read` |
| Input check | The firmware accepts only plain decimal numbers. A typing error cannot become a fan speed. | `[ERROR] Unknown command or wrong number of values: ...` |

---

## Quick start (Arduino IDE)

1. Install [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. Open **Tools > Board > Boards Manager**. Install the board package:
   - ESP32-P4: **esp32 by Espressif Systems**, version 3.3.12 (the version that the CI build uses). If the package is not in the list, add `https://espressif.github.io/arduino-esp32/package_esp32_index.json` in **File > Preferences > Additional boards manager URLs**.
   - Uno R4: **Arduino UNO R4 Boards**.
3. Select the board in **Tools > Board**:
   - ESP32-P4: **ESP32P4 Dev Module**. For the M5Stack Tab5, also read [M5Stack Tab5 I2C pins](HARDWARE.md#m5stack-tab5-i2c-pins).
   - Uno R4: **Arduino UNO R4 Minima** or **Arduino UNO R4 WiFi**.
4. ESP32-P4 only: select **Tools > Chip Variant**: **Before v3.00** (default) or **v3.00 or newer**. The Chip Variant must match the chip revision of your ESP32-P4. The upload output of esptool shows the chip revision.
5. Open **Tools > Manage Libraries**. Install QuickPID, sTune, DFRobot_LWLP, DFRobot_BMP58X, and DFRobot_RTU in the versions that [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md#libraries) gives.

   NOTE: DFRobot_BMP58X needs DFRobot_RTU. If the Library Manager does not show a version, download that version (Git tag) from GitHub. Then use **Sketch > Include Library > Add .ZIP Library**.
6. Wire the sensors and the fan as [HARDWARE.md](HARDWARE.md) shows.
7. Open `EDU-Wind-Tunnel/EDU-Wind-Tunnel.ino`. Keep all files of the `EDU-Wind-Tunnel` folder together.
8. Disconnect the fan power.
9. Select the port in **Tools > Port**.
10. Click **Upload**.
11. Open **Tools > Serial Monitor**. Set the speed to **115200 baud**. All line-ending settings work.

    NOTE: With **No Line Ending**, a command runs 1.0 to 1.2 s after its last character. This is also true for `0` and `stop`. Do not pause for 1 s or more in a command. For the fastest stop, select **New Line**.

    NOTE: After the board receives the first line ending (CR or LF), the 1 s timeout is off until the next reset. If you then change to **No Line Ending**, reset the board.
12. Reset the board. Keep the tunnel still: no airflow, both pitot ports open to the room. The start-up does these steps:
    - The firmware waits 2 s for the Serial Monitor. Then it shows the version and the board.
    - It starts the I2C bus and searches for the sensors. The scan shows only the barometric sensor (`0x47` or `0x46`). The differential pressure sensor (address `0x00`) is not in the scan. This is normal.
    - It calibrates the differential pressure sensor with the fan stopped: a 15 s wait, then 50 samples. This takes about 23 s.
    - It shows `[READY] System initialized`, the command list, and then one data line every 200 ms.

    NOTE: Do not type commands before `[READY]`. The firmware ignores the input of the first 2 s. It runs later input only after `[READY]`.

    Example (simulated ESP32-P4; your values are different, `[...]` marks removed lines):

    ```
    =====================================
       EDU Wind Tunnel PID Controller v1.3
       Board: ESP32-P4
    [...]
    [INIT] I2C bus... OK (SDA 7, SCL 8, 100 kHz)
    [...]
     - Found device at 0x47
     - Total devices found on Wire: 1
    [...]
    [INIT] LWLP5000 differential pressure sensor (DFRobot SEN0343)... OK (on Wire)
            Averaging: 5 samples
    [INIT] BMP585 barometer (DFRobot SEN0665)... OK (0x47 on Wire)
            Temp: 20.0 C, air density: 1.204 kg/m3
    [...]
    [CALIB] Pressure Sensor Calibration
    [...]
            Taking 50 samples: ..................................................
    [...]
    [READY] System initialized
    [...]
    0s | V:0.90 | SP:0.00 | PWM:0 | P:0.48 | Temp:20.0 | Err:-0.90
    ```
13. After `[READY]`, connect the fan power.
14. Type `5` to set a target airspeed of 5 m/s. Make sure that `V` moves to `SP`.
15. Type `0` to stop the fan.
16. Tune the controller on your tunnel. Follow [TUNING_GUIDE.md](TUNING_GUIDE.md).

## PlatformIO (alternative)

1. Install [VS Code](https://code.visualstudio.com/) and the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode).
2. Wire the sensors and the fan as [HARDWARE.md](HARDWARE.md) shows.
3. Open the repository folder. PlatformIO reads `platformio.ini` and installs the five libraries (same versions, from their Git tags).
4. Select the environment for your board:

   | Environment | Board |
   |---|---|
   | `esp32p4` (default) | ESP32-P4, chip revision before v3.0 |
   | `esp32p4_r3` | ESP32-P4, chip revision v3.0 or newer |
   | `uno_r4_minima` | Arduino Uno R4 Minima |
   | `uno_r4_wifi` | Arduino Uno R4 WiFi |

5. Disconnect the fan power.
6. Upload: `pio run -e esp32p4 -t upload`. Use the name of your environment.
7. Open the serial monitor: `pio device monitor`. `platformio.ini` sets the speed (115200 baud).
8. Continue with steps 12 to 16 of the Arduino IDE quick start.

---

## Commands

Type one command on a line. Then send the line (press Enter).

| Command | What it does | Limits and notes |
|---|---|---|
| `<m/s>`, for example `10` | Sets the target airspeed. The controller continues from the present PWM value. | 0 to the sensor limit (28.0 m/s, or less in dense air; `help` shows the present limit). Reply: `[SERIAL] New target: 10.00 m/s` |
| `0` or `stop` | Stops the fan now: PWM value 0, target airspeed 0. Stops a running auto-tune; the gains do not change. | Works in all modes. `0` must be the only word. Any line that starts with the word `stop` works, for example `stop now`. |
| `tune` | Relay auto-tune. The firmware selects the two PWM levels. | Set a target airspeed of at least 2.0 m/s first. The target plus 0.5 m/s must not be more than the sensor limit. |
| `tune <low> <high>` | Relay auto-tune with your PWM levels, for example `tune 40 100`. | Same as `tune`. Whole numbers 1-255. `<low>` must be less than `<high>`. |
| `tune <Kp> <Ki> <Kd>` | Sets the PID gains, for example `tune 20 8 12`. | Kp more than 0, Ki and Kd 0 or more. Values outside Kp 1-100, Ki 0.1-50, Kd 0.1-50 change to the nearest limit. |
| `stune` | sTune step-response auto-tune. | Set a target airspeed of at least 2.0 m/s first. |
| `recal` or `recal <N>` | Stops the fan, waits 15 s, and takes `<N>` samples for a new zero offset. The target airspeed stays 0. | `<N>` = 5-100, default 50. The firmware reads no commands during `recal`. |
| `avg <N>` | Sets the length of the moving average of the pressure. | 1-50, default 5. A longer average adds delay to the control loop. |
| `help` | Shows the command list. | |

Rules:

- Keywords are not case-sensitive (`Stop`, `TUNE`). Spaces or tabs separate the words.
- Type numbers as plain decimals: `12`, `-3`, `7.5`. The firmware does not accept `1e3`, `10abc`, or `1,5`.
- A line can have 63 characters or fewer. The firmware ignores a longer line and shows `[ERROR] Line too long (max 63 characters) - ignored`.
- During an auto-tune, the firmware accepts only `0` or `stop`. Other commands (also `help`) get `[BUSY] Tuning in progress - type 0 or stop to abort`.
- The firmware reads commands once per update (200 ms).
- The firmware does not store the gains or the averaging length. After a reset, it uses the defaults again (see [Configuration](#configuration)). After an auto-tune, the firmware shows a command that sets the same gains again, for example `[TUNE] To reuse these gains later: tune 16.851 7.263 14.660`.

[TUNING_GUIDE.md](TUNING_GUIDE.md) tells how to use `tune`, `stune`, and manual gains, and what the auto-tunes show.

## Serial output

The firmware shows one data line per update (every 200 ms):

```
<t>s | V:<m/s> | SP:<m/s> | PWM:<0-255> | P:<Pa> | Temp:<C> | Err:<m/s>
```

Example (simulation, target airspeed 10 m/s):

```
59s | V:10.01 | SP:10.00 | PWM:85 | P:60.28 | Temp:20.0 | Err:-0.01
59s | V:10.01 | SP:10.00 | PWM:85 | P:60.34 | Temp:20.0 | Err:-0.01
60s | V:10.02 | SP:10.00 | PWM:84 | P:60.41 | Temp:20.0 | Err:-0.02 | Kp:25.0 Ki:10.0 Kd:15.0 Avg:5
60s | V:10.02 | SP:10.00 | PWM:85 | P:60.44 | Temp:20.0 | Err:-0.02
```

| Field | Meaning | Unit |
|---|---|---|
| `<t>s` | Time since `[READY]`, in whole seconds (includes the time of `recal`) | s |
| `V:` | Airspeed, from the averaged pressure and the air density. Never negative. | m/s |
| `SP:` | Target airspeed | m/s |
| `PWM:` | PWM value sent to the fan | 0-255 |
| `P:` | Averaged differential pressure minus the calibration offset. Small negative values with no airflow are normal. | Pa |
| `Temp:` | Air temperature from the barometric sensor | deg C |
| `Err:` | `SP` minus `V` | m/s |

Additional parts at the end of a data line:

- ` | Kp:<Kp> Ki:<Ki> Kd:<Kd> Avg:<N>`: the present gains and averaging length. Every 20th line in PID mode (no auto-tune running), thus every 4 s.
- ` | Skipped:<n>`: the number of data lines that the firmware did not show, because the computer did not read the serial port. This occurs only on a native USB port: the Uno R4 Minima, and the ESP32-P4 with **Tools > USB CDC On Boot: Enabled**.

On these ports, the firmware waits at most 50 ms for the computer to read. Other text that does not fit into the transmit buffer in this time is lost. When the computer reads again, the next text starts with `[WARN] <n> bytes of output lost: the host did not read`.

During an auto-tune, the data lines start differently:

```
[TUNE] 16s | Cycle:2 | V:10.55 | SP:10.00 | PWM:60 | P:66.97 | Temp:20.0 | Err:-0.55
[sTune] SETTLING | V:9.97 | SP:10.00 | PWM:48 | P:59.88 | Temp:20.0 | Err:0.03
```

`[TUNE] <t>s` is the time since the start of the relay auto-tune. `Cycle:` is the number of valid cycles. `[sTune]` shows the phase: `SETTLING` or `TESTING`.

Most other lines start with a tag, for example `[SERIAL]`, `[STOP]`, `[WARN]`, `[ERROR]`, or `[FATAL]`. Each line ends with CR LF.

## Configuration

To change a default, edit the constant, then upload again. The `.ino` constants are in the "Settings" section at the top of the file.

| Name | File | Default | Meaning |
|---|---|---|---|
| `MAX_AIRSPEED` | `EDU-Wind-Tunnel.ino` | 28.0 m/s | Highest target airspeed. Over-speed limit of both auto-tunes. |
| `DEFAULT_AVG_SAMPLES` | `EDU-Wind-Tunnel.ino` | 5 | Length of the moving average at start-up (1 s window, 0.4 s mean delay). |
| `DEFAULT_CALIB_SAMPLES` | `EDU-Wind-Tunnel.ino` | 50 | Samples for the start-up calibration and for `recal`. |
| `CALIBRATION_SETTLE_S` | `EDU-Wind-Tunnel.ino` | 15 s | Wait before the calibration samples. |
| `DEFAULT_KP`, `DEFAULT_KI`, `DEFAULT_KD` | `EDU-Wind-Tunnel.ino` | 25.0, 10.0, 15.0 | PID gains at start-up. |
| `KP_MIN`, `KP_MAX`, `KI_MIN`, `KI_MAX`, `KD_MIN`, `KD_MAX` | `EDU-Wind-Tunnel.ino` | 1.0-100.0, 0.1-50.0, 0.1-50.0 | Gain limits for `tune <Kp> <Ki> <Kd>` and both auto-tunes. |
| `MAX_PWM_CHANGE_PER_CYCLE` | `EDU-Wind-Tunnel.ino` | 25 | Largest change of the PWM value per update in PID mode. |
| `MIN_TUNE_SETPOINT` | `EDU-Wind-Tunnel.ino` | 2.0 m/s | Lowest target airspeed for `tune` and `stune`. |
| `FAN_PWM_PIN` | `PlatformConfig.h` | 6 (ESP32-P4), 9 (Uno R4) | Fan PWM output pin. |
| `I2C_SDA_PIN`, `I2C_SCL_PIN` | `PlatformConfig.h` | -1 | -1 = the default I2C pins of the selected board. Change them only on the ESP32-P4. On the Uno R4 they must stay -1. |
| `FAN_PWM_FREQUENCY_HZ` | `PlatformConfig.h` | 25000 (ESP32-P4), 0 (Uno R4) | Fan PWM frequency in Hz. 0 = core default (490 Hz on the Uno R4). |

[TUNING_GUIDE.md](TUNING_GUIDE.md#settings) gives the auto-tune settings (for example `STUNE_METHOD`). [HARDWARE.md](HARDWARE.md#pin-assignments) describes the pins.

## Troubleshooting

| Symptom | Possible cause | Action |
|---|---|---|
| `[FATAL] Differential pressure sensor not responding` at start-up. The fan stays off. | The differential pressure sensor is not connected correctly. | Check the SEN0343 wiring and power. Reset the board. |
| `[FATAL] Barometer not found` or `[FATAL] Barometer not responding` at start-up | The barometric sensor is not connected correctly. | Check the SEN0665 wiring and power. Reset the board. |
| `[FATAL] I2C bus could not be started - check I2C_SDA_PIN/I2C_SCL_PIN in PlatformConfig.h` (ESP32-P4) | Incorrect I2C pins | Set the correct I2C pins in `PlatformConfig.h`, or set them to -1. |
| ` - Scan aborted at address 0x... (too many timeouts - bus likely has no pull-ups)` | No device or no pull-up resistors on the bus | Check the I2C wiring of that bus. |
| `V` stays 0 and `P` is negative when the fan runs | The pitot tubes are connected to the wrong ports. | Interchange the two pitot tubes on the differential pressure sensor. |
| `[STOP] Fan stopped, target 0 m/s (no airflow)` | A pitot tube is disconnected, or the fan has no power. | Check the pitot tubes, the fan power, and the PWM wiring. |
| `[STOP] Fan stopped, target 0 m/s (no valid pressure data)` | The differential pressure sensor stopped responding. | Check the SEN0343 wiring. Then type `recal`. |
| `V` shows a small value (for example `V:0.90`) with the fan stopped | Pressure noise near 0 Pa | This is normal. Type `recal` to zero the sensor again. |
| The airspeed reading is noisy. | Short moving average | Type a larger value, for example `avg 10`. A longer average slows the control loop and can make it oscillate. Then reduce the gains or tune again (see [TUNING_GUIDE.md](TUNING_GUIDE.md#averaging-delay)). |
| The airspeed oscillates around the target | The gains are too high for your fan. | Run `tune` or `stune`, or set lower gains. See [TUNING_GUIDE.md](TUNING_GUIDE.md#troubleshooting-by-symptom). |
| `[TUNE] Tuning aborted (relay tune timeout) - ...` | The two PWM levels do not give airspeeds above and below the target. | Give the PWM levels, for example `tune 40 100`. See [TUNING_GUIDE.md](TUNING_GUIDE.md#relay-auto-tune-tune). |
| Commands have no effect | Incorrect speed, or a change to **No Line Ending** after a line ending | Set 115200 baud. Reset the board. |

## Limitations

- The version 1.3 changes have tests in simulation only (simulated fan and sensors), not on hardware.
- The default gains (Kp 25, Ki 10, Kd 15) are a starting point. They are validated only in simulation, where they were stable on 4 of 5 fan models. Tune the controller on your tunnel and make sure that it is stable.
- The stop command (`0` or `stop`) does not work in these conditions:
  - The line has more than 63 characters (also a line that starts with `stop`).
  - `recal` runs (about 23 s with 50 samples). The fan is already stopped.
  - The firmware halted after a `[FATAL]` message. The fan is off.
- Each `stune` run loses at least 16 bytes of heap memory (a known problem of the sTune 2.4.0 library). Many runs fit into memory. A reset frees the memory.

## Serial control from an Android phone

You can control the firmware from an Android phone with the [Serial USB Terminal app by Kai Morich](https://www.kai-morich.de/android/).

1. Connect the phone to the board with a USB cable. Use the adapters or USB-C hubs that your phone and board need.
2. Set the speed to 115200 baud. All line-ending settings work.
3. Make macros for the commands that you use frequently (see [Commands](#commands)).

## Documentation map

| File | Content |
|---|---|
| [HARDWARE.md](HARDWARE.md) | Tunnel changes, parts list, pins, wiring, pitot tubes |
| [TUNING_GUIDE.md](TUNING_GUIDE.md) | How the controller works, manual tuning, relay auto-tune, sTune |
| [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) | Library versions and licenses, tunnel design license |
| [platformio.ini](platformio.ini) | PlatformIO environments and library versions |
| [.github/workflows/compile-sketch.yml](.github/workflows/compile-sketch.yml) | CI workflow "Compile sketch". It compiles the firmware for the ESP32-P4 (both chip variants), the Uno R4 Minima and the Uno R4 WiFi on each push and pull request. |
| `3D Print Files/` | Changed tunnel parts with static pressure taps |

## Planned

These items are not part of the firmware yet:

- Control without a computer, with Modulino controls and a screen
- A [Controllino](https://www.controllino.com) PLC version for industrial applications and education
- A controller that is compatible with [MathWorks Simulink](https://www.mathworks.com/products/simulink.html) and [JuliaHub Dyad](https://juliahub.com/products/dyad)
- An integrated lift and drag force balance
- Course materials for wind tunnel operation and design

## License and acknowledgments

The firmware and the documentation use the MIT License (see [LICENSE](LICENSE)). [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) gives the licenses of the libraries and of the tunnel design. The tunnel design license also applies to the files in `3D Print Files/`.

Thanks to:

- Jerrod Hofferth for the wind tunnel design
- David Lloyd (Dlloydev) for the QuickPID and sTune libraries
- DFRobot for the sensor libraries
- The Arduino and Espressif communities for board and library support

## Support

This code comes with no warranty and no promise of support. Ask your questions in GitHub Issues or Discussions. The authors or other users can help when they have time. To contribute, fork the repository and open a pull request.

- Issues: [GitHub Issues](https://github.com/Low-Boom/EDU-wind-tunnel/issues)
- Discussions: [GitHub Discussions](https://github.com/Low-Boom/EDU-wind-tunnel/discussions)
