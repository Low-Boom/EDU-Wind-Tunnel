# Hardware Configuration Guide

## Hardware Modifications to Educational Wind Tunnel
A static pressure ring must be added at the inlet of the test section to measure airspeed.
An updatead version of the test section or an additional component between the inlet and test section are included for this purpose. Get them from Printables at LINK LINK LINK.

## Required Components

- Arduino or Arduino Compatible MBED microcontroller
     - [Arduino Uno Rev. 4](https://store-usa.arduino.cc/pages/uno-r4?srsltid=AfmBOorOs0ix30CZM4vLKSA9m6F7qrPhlblCfdp3-wQMstfBhTvr8o13)
     - [Arduino Mega 2560 Rev. 3](https://store-usa.arduino.cc/products/arduino-mega-2560-rev3?srsltid=AfmBOooJeuE40gzrgW6OqlN0fd6tiL2gktAr5MX0-S2eOb7eE9hD2HyQ)
     - [Arduino Giga R1/R1 Wifi](https://store-usa.arduino.cc/products/giga-r1-wifi?queryID=a518cc78eda98ff33faf6b976f3d470a)
          - Note, the display drivers are very bugged so don't buy a Giga Display Shield and expect it to work.
          - Future versions will likely depracate direct support for the Giga and its idiocycracies.
- [3D Printed Educational Wind Tunnel with Static Pressure Ring](https://www.printables.com/model/1480487-airspeed-static-pressure-ring-modification-for-edu)
- [Static Pressure Manifold](https://www.printables.com/model/1480487-airspeed-static-pressure-ring-modification-for-edu)
     - 3D Printed Manifold for 4x 0.063in (1/16in) to 1x 0..125in (1/8in) tubing
     - [Scanivalve Pneumatic Manifolds](https://scanivalve.com/products/pneumatic-connectors-and-tubing/pneumatic-manifolds/)
- PWM Controlled Fan or Blower
- Arduino Giga R1 WiFi (or compatible MBED-based board)
     - Note that drivers for the Arduino Giga Display Shield are broken, do not bother buying one.
- MS4525DO differential pressure sensor (I2C address 0x28)
- BMP3XX barometric sensor (BMP388/BMP390) (I2C address 0x77 or 0x76)
- PWM-capable motor or fan
     - For Standard 140 mm Wind Tunnel: [Noctua NF-A12x25 G2 PWM](https://www.noctua.at/en/products/nf-a12x25-g2-pwm) with a Noctua [NV-PS1](https://www.noctua.at/en/products/nv-ps1) or [NV-SPH1](https://www.noctua.at/en/products/nv-sph1) Power Supply or similar with the 
     - For Larger 8in/200mm Fann Driven Tunnel: [AC Infinity CLOUDLINE A8, Quiet Inline Fan with Speed Controller](https://acinfinity.com/hydroponics-growers/cloudline-a8-quiet-inline-fan-with-speed-controller-8-inch/)
          - Note DO NOT get the "smart" version. The standard PWM speed controller version is needed here.
          - The now depracated but still available [AC Infinity CLOUDLINE S8 Pro](https://a.co/d/iekl0lr) is an equivalent option
- Four Pin PWM Fan Cable such as the [Noctua NA-SEC1](https://www.noctua.at/en/products/na-sec1)
- 1/16 in (0.063in) OD steel tubing for static pressure taps
     - [McMaster 304 Stainless Steel Tubing Miniature, 0.065" OD, 0.009" Wall Thickness](https://www.mcmaster.com/5560K73-5560K731/)
     - [063 Scanivalve Stainless Steel Tubulations](https://scanivalve.com/products/pneumatic-connectors-and-tubing/stainless-steel-tubulations/)
     - [Amazon 304 Stainless Steel Capillary Tube, OD 1.5mm](https://a.co/d/c9E1Kos)
- 1/16in (0.063in) ID soft line silicone or polyurethane/Tygon tubing
     - [Scanivalve 063 Plastic Tubing](https://scanivalve.com/products/pneumatic-connectors-and-tubing/plastic-tubing/)
     - [Amazon 1/16" ID Silicone Tubing, Food Grade 1/16" ID x 1/8" OD 10 Feet Length Pure Silicone Hoses](https://a.co/d/4P2acrw)
- 1/8in (0.125in) ID soft line silicone or polyurethane/Tygon tubing
     - [Scanivalve 125 Plastic Tubing](https://scanivalve.com/products/pneumatic-connectors-and-tubing/plastic-tubing/)
     - [Amazon 1/8" ID Silicone Tubing, Food Grade 1/8" ID x 3/16" OD 10 Feet Length Pure Silicone](https://a.co/d/5y4ox0C)
- Assorted STEMMA QT/Qwiic 4-pin I2C Cabling. At least three (3) QT to QT and one QT to Female Dupont Sockets are recommended. Extras are always a good idea
     - [Adafruit Qwiic Cabling](https://www.adafruit.com/product/4399)
     - [Amazon I2C Qwiic Cable Kit Assortment](https://a.co/d/anmSIUS)
- One STEMA QT/Qwiic to JST-GH Adapter for the airspeed sensor
     - You can make one yourself by splicing the airspeed sensors included cable with a Qwiic
     - [Aamazon GH Connector to Dupont 2.54 Adapter Kit](https://a.co/d/dRP7Vl2)

---

## Arduino Pin Mapping (defaults used by sketch)

- Serial: USB / Serial Monitor at 115200 baud
- PWM output: Arduino digital pin 9 → PWM signal input
- PWM Tachometer input: Arduino digital pin 2 (optional)
- I2C: SDA / SCL → BMP3XX and MS4525DO
     - Automatic I2C bus detection (e.g. Wire, Wire1, Wire2).
     - Sensors can be used on the same or different buses
	- BMP3XX Barometer supports both 5V and 3.3V power.
	- MS4525DO Airspeed Sensor ONLY SUPPORTS 5V.
 	- See I2C section below for more detail on wiring configuration alternatives

## If Using the AC Infinity PWM Fans (Skip if Using 12V PC Fan)

AC Infinity uses the an industrial 10V PWM standard while the Arduino Mega 2560 and Uno R4 use 5V PWM logic and the Giga R1 3.3V PWM logic (don't ask my why). Thus you will need to interpose a bi-directional level logic shifter to ensure the AC Infinity blower runs at full performance.

The [Adafruit 4-channel Bi-Directional Logic Level Converter - BS138](https://www.adafruit.com/product/757) is recommended for this. For more information on PWM wiring standards, read the [PWM White Paper by Noctua](https://cdn.noctua.at/media/Noctua_PWM_specifications_white_paper.pdf).

The wiring configration for control logic and sensors for the Arduino Mega 2560, Giga R1, and Uno R4 with an AC Infinity fan are diagramed below.

<img height="600px" src="./img/Mega to AC Infinity Config.jpg" alt="Arduino Mega 2560 and AC Infinity Fan Wiring Configration" /> <img height="600px" src="./img/Giga to AC Infinity Config.jpg" alt="Arduino Giga R1 and AC Infinity Fan Wiring Configration" /><img height="600px" src="./img/Uno to AC Infinity.jpg" alt="Arduino Uno R4 and AC Infinity Fan Wiring Configration" />

Additionally. You should wire a standard 4-pin fan connector, such as the [Noctua NA-SEC1](https://www.noctua.at/en/products/na-sec1), in place of the AC Infinity UIS USB-C(ish) connector. Connect a corresponding female 4-pin connector to your logic level board. These can be created quickly using a single 4-pin fan extension cabling and a little crimping.

Unscrew the four self tapping screws that secure hatch on the side of the AC Infinity fan to reveal the wiring terminal block. Wire it as noted in the image below. TAC is the fan tachometer feedback, PWM is the control signal.
     
 <img height="600px" src="./img/AC Infinity Wiring.jpg" alt="AC Infinity Terminal Block Wiring" />

## If Using a 12V PC Fan (Skip if Using AC Infinity Fan)

PC fans uses the 5V PWM standard as outlined in the [PWM White Paper by Noctua](https://cdn.noctua.at/media/Noctua_PWM_specifications_white_paper.pdf). While the Arduino Mega 2560 and Uno R4 use 5V logic level standard, the Arduino Giga R1 uses a 3.3V logic level. Thus you will need to interpose a bi-directional level logic shifter to ensure full performance.

The [Adafruit 4-channel Bi-Directional Logic Level Converter - BS138](https://www.adafruit.com/product/757) is recommended for this.

The wiring configration for control logic and sensors for the Arduino Mega 2560, Giga R1, and Uno R4 with an externally powered 12V PC Fan are diagramed below.

<img height="600px" src="./img/Mega to 12V Fan Config.jpg" alt="Arduino Mega 2560 and 12V Fan Wiring Configration" /> <img height="600px" src="./img/Giga to 12V Fan Config.jpg" alt="Arduino Giga R1 and 12V Fan Wiring Configration" /> <img height="600px" src="./img/Uno to 12V Fan Config.jpg" alt="Arduino Uno R4 and 12V Fan Wiring Configration" />

## I2C Wiring and Power Permutations



 
---

## Airspeed Sensor Plumbing
   - MS4525DO high-pressure port ← Exposed to atmosphere away from the tunnel inlet or outlet
   - MS4525DO low-pressure port ← Test section static pressure ring through the manifold
   - Note that the orientation of the sensor can change between manufactureres and batches.
        - For mine, the top port was the low-pressure static ring port. Swap if you are getting only 0 m/s speed readings.

---
For extended diagnostics see TROUBLESHOOTING.md.

---
