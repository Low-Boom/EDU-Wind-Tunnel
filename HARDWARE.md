# Hardware Configuration Guide

## Hardware Modifications to Educational Wind Tunnel
A static pressure ring must be added at the inlet of the test section to measure airspeed.
An updatead version of the test section or an additional component between the inlet and test section are included for this purpose. Get them from Printables at LINK LINK LINK.

## Components

- 3D Printed Educational Wind Tunnel with Static Pressure Ring
- Static Pressure Manifold
     - 3D Printed Manifold for 4x 0.063in (1/16in) to 1x 0..125in (1/8in) tubing
     - [Scanivalve Pneumatic Manifolds](https://scanivalve.com/products/pneumatic-connectors-and-tubing/pneumatic-manifolds/)
     - 
- PWM Controlled Fan
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

## Arduino Pin mapping (defaults used by sketch)

- PWM output: Arduino digital pin 9 → PWM signal input
- PWM Tachometer input: Arduino digital pin 2 (optional)
- I2C: SDA / SCL → BMP3XX and MS4525DO (shared bus via Qwiic Daisy Chain)
- Sensors power: 5V or 3.3V and GND from Arduino
- Serial: USB / Serial Monitor at 115200 baud

<img width="500px" src="./img/Serial Controlled Arduino Wiring.jpg" alt="Arduino Wiring Diagram" />

## If Using the AC Infinity PWM Fans (Skip if Using 12V PC Fan)

AC Infinity uses the a 10V PWM standard while the Arduino boards use a 3.3V




---

## Airspeed Sensor Plumbing
   - MS4525DO high-pressure port ← Exposed to atmosphere away from the tunnel inlet or outlet
   - MS4525DO low-pressure port ← Test section static pressure ring through the manifold
   - Note that the orientation of the sensor can change between manufactureres and batches.
        - For mine, the top port was the low-pressure static ring port. Swap if you are getting only 0 m/s speed readings.

---
For extended diagnostics see TROUBLESHOOTING.md.

---
