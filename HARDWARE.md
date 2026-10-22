# Hardware Guide

This guide tells you which parts to get and how to connect them.
For the software setup, see [README.md](README.md).

WARNING: Disconnect the fan power before you change the wiring.

---

## Tunnel modification

To measure the airspeed, the tunnel needs a static pressure ring at the inlet of the test section.
Add this ring to the [Modular Wind Tunnel for STEM Education](https://www.printables.com/@JerrodH) by Jerrod Hofferth.

You have two options:

- Print the modified test section with the static pressure ring.
- Print a short duct section with static pressure taps. Install it between the inlet and the test section.

Get the parts from [Printables](https://www.printables.com/model/1480487-airspeed-static-pressure-ring-modification-for-edu).
The same parts are in the folder `3D Print Files/` of this repository (`.3MF` and SolidWorks `.SLDPRT` files):

- `Test Section with Static Pressure Ring and Fillet Windows`
- `5-6 - Small Duct Section with Static Pressure Taps`
- `Static Pressure Ring Manifold`

---

## Bill of materials

### Microcontroller board

Use one of these boards:

- **ESP32-P4 board (recommended).** Examples:
  - [Waveshare ESP32-P4-WiFi6-Touch-LCD-7B](https://www.waveshare.com/product/arduino/boards-kits/esp32-p4/esp32-p4-wifi6-touch-lcd-7b.htm)
  - [M5Stack Tab5](https://shop.m5stack.com/products/m5stack-tab5-iot-development-kit-esp32-p4)
- **[Arduino Uno R4](https://store-usa.arduino.cc/pages/uno-r4) Minima or Uno R4 WiFi.** Only the Uno R4 WiFi has a Qwiic connector.

### Sensors

- DFRobot SEN0343 (LWLP5000) differential pressure sensor, ±500 Pa, I2C address `0x00` (fixed).
- DFRobot SEN0665 (BMP585) barometric sensor, I2C address `0x47` (`0x46` if you change the address jumper).

### Fan

Select the fan for your tunnel size:

| Tunnel | Fan | Fan power |
|---|---|---|
| Standard 140 mm tunnel | [Noctua NF-A14x25 G2 PWM](https://www.noctua.at/en/products/nf-a14x25-g2-pwm) (12 V, 4-pin PWM) | Noctua [NV-PS1](https://www.noctua.at/en/products/nv-ps1) or [NV-SPH1](https://www.noctua.at/en/products/nv-sph1), or a similar 12 V supply |
| Larger 8 in (200 mm) tunnel | [AC Infinity CLOUDLINE A8](https://acinfinity.com/hydroponics-growers/cloudline-a8-quiet-inline-fan-with-speed-controller-8-inch/), the version with the standard PWM speed controller | Mains, through the fan's own power cable |

CAUTION: Do not get the "smart" version of the AC Infinity fan. You need the version with the standard PWM speed controller.
The older [AC Infinity CLOUDLINE S8 Pro](https://a.co/d/iekl0lr) is an equivalent option.

### Level shifter board

- [Adafruit 4-channel I2C-safe Bi-directional Logic Level Converter - BSS138](https://www.adafruit.com/product/757) (product 757).
  Some fan and board combinations need it. See [Level shifter](#level-shifter).

### Pressure tubing and parts

- 3D-printed static pressure manifold: 4 x 1/16 in (0.063 in) tubes to 1 x 1/8 in (0.125 in) tube.
  An alternative is a [Scanivalve pneumatic manifold](https://scanivalve.com/products/pneumatic-connectors-and-tubing/pneumatic-manifolds/).
- 1/16 in (0.063 in) OD steel tubing for the static pressure taps, for example:
  - [McMaster 304 Stainless Steel Tubing, 0.065 in OD, 0.009 in wall](https://www.mcmaster.com/5560K73-5560K731/)
  - [Scanivalve 063 Stainless Steel Tubulations](https://scanivalve.com/products/pneumatic-connectors-and-tubing/stainless-steel-tubulations/)
  - [304 Stainless Steel Capillary Tube, OD 1.5 mm (Amazon)](https://a.co/d/c9E1Kos)
- 1/16 in (0.063 in) ID soft tubing (silicone, polyurethane or Tygon), for example:
  - [Scanivalve 063 Plastic Tubing](https://scanivalve.com/products/pneumatic-connectors-and-tubing/plastic-tubing/)
  - [1/16 in ID x 1/8 in OD silicone tubing (Amazon)](https://a.co/d/4P2acrw)
- 1/8 in (0.125 in) ID soft tubing (silicone, polyurethane or Tygon), for example:
  - [Scanivalve 125 Plastic Tubing](https://scanivalve.com/products/pneumatic-connectors-and-tubing/plastic-tubing/)
  - [1/8 in ID x 3/16 in OD silicone tubing (Amazon)](https://a.co/d/5y4ox0C)

### Cables

- 4-pin STEMMA QT / Qwiic I2C cables. Get at least three QT-to-QT cables and one QT-to-female-Dupont cable. Spare cables are useful.
  - [Adafruit Qwiic cables](https://www.adafruit.com/product/4399)
  - [I2C Qwiic cable kit (Amazon)](https://a.co/d/anmSIUS)
- Check the connectors on your sensor boards. Select cables that fit them.
- A 4-pin PWM fan cable, for example the [Noctua NA-SEC1](https://www.noctua.at/en/products/na-sec1).

---

## Pin assignments

The pins are in `EDU-Wind-Tunnel/PlatformConfig.h`.
Change them there only if your wiring is different.

### ESP32-P4

| Function | Pin | Notes |
|---|---|---|
| Fan PWM | GPIO 4 | `FAN_PWM_PIN`. 25 kHz, 3.3 V logic. |
| I2C SDA | GPIO 7 (default) | `I2C_SDA_PIN` = -1 uses the default pin of the board that you select in the IDE. **ESP32P4 Dev Module**: GPIO 7 (also for the Waveshare boards). |
| I2C SCL | GPIO 8 (default) | `I2C_SCL_PIN` = -1. **ESP32P4 Dev Module**: GPIO 8 (also for the Waveshare boards). |
| Fan tachometer | Not used | You can leave the tach wire unconnected. |
| Serial | USB | 115200 baud |

CAUTION: Do not use GPIO 14-19. On boards with an ESP32-C6 WiFi chip (for example the Waveshare boards), these pins carry the SDIO link to that chip.

NOTE: If GPIO 4 is not available on your board, change `FAN_PWM_PIN`. Use a GPIO of 36 or lower. Some boards supply the higher GPIOs from a separate (LDO) supply.

The firmware uses only the `Wire` I2C bus on the ESP32-P4.
At start-up it prints the pins that it uses, for example `[INIT] I2C bus... OK (SDA 7, SCL 8, 100 kHz)`.

There are no wiring images for the ESP32-P4. Use the Uno R4 images below with these changes:

- Connect the fan PWM to GPIO 4, not D9.
- Connect the sensors to the I2C pins in the table above.
- Use a level shifter for all fans. See [Level shifter](#level-shifter).

#### M5Stack Tab5 I2C pins

On the M5Stack Tab5, the I2C port (Grove Port A) uses GPIO 53 (SDA) and GPIO 54 (SCL).
The PlatformIO ESP32-P4 environments and the **ESP32P4 Dev Module** selection use GPIO 7 and 8 instead.
Use one of these two methods:

- Keep your PlatformIO environment (or the **ESP32P4 Dev Module** selection in the Arduino IDE). In `PlatformConfig.h`, set `I2C_SDA_PIN` to 53 and `I2C_SCL_PIN` to 54. This is the method for PlatformIO.
- Or, in the Arduino IDE, select the board **M5Tab5**. Keep `I2C_SDA_PIN` and `I2C_SCL_PIN` at -1. This board selection uses GPIO 53 and 54 by default. Also set its **Chip Variant**.

NOTE: The CI build does not compile for the **M5Tab5** board selection.

With correct pins, the start-up message is `[INIT] I2C bus... OK (SDA 53, SCL 54, 100 kHz)`.

### Arduino Uno R4 Minima and Uno R4 WiFi

| Function | Pin | Notes |
|---|---|---|
| Fan PWM | D9 | `FAN_PWM_PIN`. Core default frequency 490 Hz, 5 V logic. |
| I2C SDA / SCL (bus `Wire`) | SDA and SCL header pins (same signals as A4 / A5) | Fixed by the board. Leave `I2C_SDA_PIN` and `I2C_SCL_PIN` at -1. The start-up message shows them as `SDA 18, SCL 19`. |
| Qwiic connector (bus `Wire1`) | Qwiic | Uno R4 WiFi only. 3.3 V. |
| Fan tachometer | Not used | The images show it on D2. You do not need this wire. |
| Serial | USB | 115200 baud |

NOTE: The Uno R4 uses the core default PWM frequency of 490 Hz. The Intel specification for 4-pin fans uses 25 kHz.
At 490 Hz some PC fans can make a ticking noise at low speed. The authors used the Uno R4 with the wiring shown in this guide.

---

## Fan wiring

### Level shifter

The fan expects a PWM signal at its own logic level: 5 V for a PC fan, 10 V for the AC Infinity fan.
A lower signal level can stop the fan from reaching full speed.
Use the level shifter as this table shows:

| Board (logic level) | Fan | Level shifter | LV side | HV side |
|---|---|---|---|---|
| ESP32-P4 (3.3 V) | 12 V PC fan | Yes | 3.3 V of the board | 5 V |
| ESP32-P4 (3.3 V) | AC Infinity | Yes | 3.3 V of the board | +10 V from the fan |
| Uno R4 (5 V) | 12 V PC fan | No. Connect D9 directly to the fan PWM wire. | - | - |
| Uno R4 (5 V) | AC Infinity | Yes | 5 V of the board | +10 V from the fan |

To connect the level shifter:

1. Connect LV to the logic voltage of the board (3.3 V or 5 V).
2. Connect HV to the PWM voltage of the fan (5 V or +10 V).
3. Connect the GND pins of both sides to GND.
4. Connect the fan PWM pin of the board to one low-voltage channel (A1-A4).
5. Connect the PWM wire of the fan to the matching high-voltage channel (B1-B4).

For more information on fan PWM, read the [Noctua PWM specifications white paper](https://cdn.noctua.at/media/Noctua_PWM_specifications_white_paper.pdf).

### Fan PWM line at power-up and during uploads

Before `setup()` runs, the board does not drive the fan PWM pin: the line floats.
This occurs at power-up, during a reset and during an upload.
Most 4-pin fans read a floating PWM input as full speed.
With the level shifter, its 10 kOhm pull-up resistor on the LV side also pulls the line high.
Thus the fan can run at full speed until `setup()` starts.
The first step of `setup()` sets the pin LOW.

WARNING: The fan can start at full speed during an upload or a reset. Keep clear of the fan, or disconnect the fan power before you upload.

To keep the fan off, connect a pull-down resistor between the fan PWM pin of the board and GND.
With a level shifter, connect it on the LV side.

- Estimated value (not tested): about 1 kOhm to 2.2 kOhm, against the 10 kOhm pull-up of the level shifter. Use the lower value with 5 V logic.
- A lower value pulls harder, but takes more current from the pin when it is HIGH (about 3.3 mA with 1 kOhm at 3.3 V).
- Without a level shifter (Uno R4 with a PC fan), the pull-up inside the fan sets the necessary value. This guide has no estimate for this case. Refer to the fan datasheet.
- After you install the resistor, reset the board with the fan power on. Make sure that the fan does not start.

### Fan power

- Use a separate power supply for the fan. Do not power the fan from the microcontroller board.
- Connect the GND of the fan supply to the GND of the board. The PWM signal needs this common ground.
- The Uno R4 images power the board from the fan supply through VIN.
  Do this only if the board documentation allows that voltage on VIN.
  If not, power the board from USB and connect only GND to the fan supply.

### 12 V PC fan (140 mm tunnel)

PC fans use the 5 V PWM standard (see the Noctua white paper above).
Connect the fan PWM wire as the [Level shifter](#level-shifter) table shows:

- Uno R4: connect D9 directly to the fan PWM wire.
- ESP32-P4: connect GPIO 4 through the level shifter.

The image shows an Uno R4 WiFi with a 12 V PC fan. It also shows older sensors; see [I2C sensor wiring](#i2c-sensor-wiring).

<img height="600px" src="./img/Uno to 12V Fan Config.jpg" alt="Arduino Uno R4 and 12 V PC fan wiring" />

### AC Infinity inline fan (8 in tunnel)

The AC Infinity fans use a 10 V PWM signal. Thus all boards need the level shifter (HV = +10 V from the fan).

Replace the AC Infinity UIS connector (similar to USB-C) with a standard 4-pin fan connector, for example the [Noctua NA-SEC1](https://www.noctua.at/en/products/na-sec1).
Connect a matching female 4-pin connector to the level shifter.
You can make these from one 4-pin fan extension cable with a crimp tool.

The image shows an Uno R4 WiFi with an AC Infinity fan and the level shifter. It also shows older sensors; see [I2C sensor wiring](#i2c-sensor-wiring).

<img height="600px" src="./img/Uno to AC Infinity.jpg" alt="Arduino Uno R4 and AC Infinity fan wiring" />

#### AC Infinity terminal block

WARNING: The terminal block also has the mains connections (AC HOT, AC NEU, AC GND). Disconnect the fan from the mains before you open the hatch.

1. Remove the four self-tapping screws of the hatch on the side of the fan.
2. Find the terminal block.
3. Connect the PWM wire of your 4-pin cable to the `PWM` terminal.
4. Connect the GND wire to the `GND` terminal.
5. Connect the power wire to the `+10V` terminal. This supplies the HV side of the level shifter.
6. You can leave the `TAC` terminal (tachometer) unconnected.

NOTE: Some newer AC Infinity models do not have this terminal block. The wire colors appear to be the same. You can add your own terminal block.

<img height="600px" src="./img/AC Infinity Wiring.jpg" alt="AC Infinity terminal block wiring" />

---

## I2C sensor wiring

Both sensors use I2C at 100 kHz (the core default).
Supply both sensor boards with 3.3 V, the same voltage as the Qwiic connector.

NOTE: This guide does not verify the supply range of the sensor boards. Check it on the DFRobot product page of each sensor.

The firmware finds both sensors on any I2C bus that it uses:

| Board | I2C buses |
|---|---|
| ESP32-P4 | `Wire` (pins in the [ESP32-P4](#esp32-p4) table) |
| Uno R4 Minima | `Wire` (SDA / SCL header) |
| Uno R4 WiFi | `Wire` (SDA / SCL header) and `Wire1` (Qwiic connector) |

You can connect both sensors to one bus (chained). On the Uno R4 WiFi, you can also put each sensor on a different bus (split).

- The differential pressure sensor has the fixed address `0x00`. The firmware checks this address directly, first on `Wire`, then on `Wire1`.
- The bus scanner checks the addresses `0x01` to `0x7E` only. Thus it never lists the differential pressure sensor. This is normal.
- The firmware finds the barometric sensor in the scan results at `0x47`, then at `0x46`, on any bus.

### Uno R4 WiFi examples

The two images show the Uno R4 WiFi with the Qwiic connector (`Wire1`):

- **Chained:** both sensors are on the Qwiic bus. The second sensor connects to the first sensor board.
- **Split:** the barometric sensor is on the Qwiic connector (`Wire1`). The differential pressure sensor is on the SDA / SCL header (`Wire`).

The fan images above show a third option: both sensors chained on the SDA / SCL header (`Wire`).
The Uno R4 Minima has no Qwiic connector. Use the SDA / SCL header.

CAUTION: The images show older sensors that the firmware does not support: an MS4525DO differential pressure sensor (`0x28`) and a BMP390 barometer (`0x77`).
Use the same bus connections for the DFRobot SEN0343 and SEN0665. The addresses in the images do not apply.

CAUTION: Some images supply the older sensors with +5 V. The Qwiic connector supplies 3.3 V.
Before you connect a sensor board to 5 V, make sure that its DFRobot product page permits 5 V.

<img height="600px" src="./img/Uno with Qwiic Chained.jpg" alt="Arduino Uno R4 WiFi with the sensors chained on the Qwiic bus" /> <img height="600px" src="./img/Uno with Qwiic Split.jpg" alt="Arduino Uno R4 WiFi with the sensors split across the Qwiic and header buses" />

### Check the sensors at start-up

Open the serial monitor at 115200 baud. With correct wiring, the start-up output on an ESP32-P4 contains these lines:

```
Scanning I2C bus Wire
 - Found device at 0x47
 - Total devices found on Wire: 1
...
[INIT] LWLP5000 differential pressure sensor (DFRobot SEN0343)... OK (on Wire)
[INIT] BMP585 barometer (DFRobot SEN0665)... OK (0x47 on Wire)
```

`Total devices found on Wire: 1` is correct with both sensors connected, because the scan does not show `0x00`.
On the Uno R4 WiFi, the scanner also shows a `Wire1` block, and the `OK` lines name the bus where each sensor is.

If a sensor is missing, the firmware stops with the fan off. Examples:

```
[ERROR] No device at 0x00 on any I2C bus; check the SEN0343 wiring
[FATAL] Differential pressure sensor not responding
```

```
[ERROR] No device at 0x47 or 0x46 on any I2C bus; check the SEN0665 wiring
[FATAL] Barometer not found
```

After a fatal error, the board LED blinks (on for 250 ms, off for 250 ms).
This occurs only if the board has an `LED_BUILTIN`, for example D13 on the Uno R4.
The **ESP32P4 Dev Module** and **M5Tab5** selections have no `LED_BUILTIN`, so no LED blinks.
Correct the wiring, then reset the board.

---

## Pitot plumbing

Connect the differential pressure sensor (DFRobot SEN0343) with two soft tubes. The firmware messages call them the pitot tubes.

- **High-pressure port:** open to the room air. Keep it away from the tunnel inlet and outlet.
- **Low-pressure port:** to the static pressure ring of the test section, through the manifold.

NOTE: The port orientation can be different between manufacturers and batches.
If the airspeed stays at 0 m/s while the fan runs, interchange the two pitot tubes.
With interchanged tubes, the `P:` value in the data lines goes negative when the fan runs.

At each start and after `recal`, the firmware stops the fan and sets the sensor zero.
Keep the air in the tunnel still during this time.

If a pitot tube comes off, the controller drives the fan to full power. The no-airflow check then stops the fan (see [Safety](README.md#safety)).
