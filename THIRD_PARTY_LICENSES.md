# Third-Party Licenses

This page lists the third-party libraries and designs that this project uses, and their licenses.

## Libraries

The firmware needs these five Arduino libraries.
They are not part of this repository. [README.md](README.md) tells how to install them.
Use the versions in this table. `platformio.ini` and the CI workflow use the same versions (Git tags).
Each library ships its own license file.

| Library | Version | Author | License | Copyright line | Repository |
|---|---|---|---|---|---|
| QuickPID | 3.1.9 | David Lloyd | MIT | Copyright 2023 David Lloyd | https://github.com/Dlloydev/QuickPID |
| sTune | 2.4.0 | David Lloyd | MIT | Copyright (c) 2022 Dlloydev | https://github.com/Dlloydev/sTune |
| DFRobot_LWLP | 1.0.0 | DFRobot | MIT | Copyright 2010 DFRobot Co.Ltd | https://github.com/DFRobot/DFRobot_LWLP |
| DFRobot_BMP58X | 1.0.1 | DFRobot | MIT | Copyright (c) 2025 DFRobot | https://github.com/DFRobot/DFRobot_BMP58X |
| DFRobot_RTU | 1.0.6 | DFRobot | MIT | Copyright 2010 DFRobot Co.Ltd | https://github.com/DFRobot/DFRobot_RTU |

What each library does in this project:

- **QuickPID:** the PID controller for the fan.
- **sTune:** the step-response auto-tune (`stune` command). The firmware needs it.
- **DFRobot_LWLP:** the driver for the DFRobot SEN0343 (LWLP5000) differential pressure sensor.
- **DFRobot_BMP58X:** the driver for the DFRobot SEN0665 (BMP585) barometric sensor.
- **DFRobot_RTU:** a dependency of DFRobot_BMP58X. The Library Manager can install it together with DFRobot_BMP58X.

### MIT License text

All five libraries use the MIT License text below. Each library has its own copyright line from the table above.
The license files of QuickPID, DFRobot_LWLP and DFRobot_RTU contain this text without the title "MIT License".

```
MIT License

Copyright <see the copyright line of each library in the table above>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Modular Wind Tunnel design

- **Design:** Modular Wind Tunnel for STEM Education
- **Designer:** Jerrod Hofferth
- **License:** Creative Commons 4.0. See the design's Printables page for the exact license variant.
- **Website:** https://www.printables.com/@JerrodH

The files in the folder `3D Print Files/` are modifications of this design (static pressure ring, duct section with pressure taps, manifold).
They follow the license of the original design.
The modification is also on [Printables](https://www.printables.com/model/1480487-airspeed-static-pressure-ring-modification-for-edu).

## This project

The MIT License in [LICENSE](LICENSE) (Copyright (c) 2025 Low-Boom) covers the firmware in `EDU-Wind-Tunnel/` and the documentation of this repository.
It does not cover:

- the files in `3D Print Files/` (see [Modular Wind Tunnel design](#modular-wind-tunnel-design)),
- the third-party libraries (see [Libraries](#libraries)).
