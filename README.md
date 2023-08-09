# PicoFabric MicroPython SDK :lemon: # 
[![Install](https://img.shields.io/badge/VSCode-Extension-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-ide)
[![datasheet (pdf)](https://img.shields.io/badge/Data%20Sheet-PDF-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-hardware/doc/datasheet.pdf)
[![sch (pdf)](https://img.shields.io/badge/SCH-PDF-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-hardware/doc/sch.pdf)
[![Store](https://img.shields.io/badge/Store-PicoLemon-f3cd5a?longCache=true&style=flat-rounded)](http://picolemon.com/board/PICOFABRIC)
[![Examples](https://img.shields.io/badge/Code-Examples-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-examples)
[![Discord](https://img.shields.io/badge/@-Discord-f3cd5a?longCache=true&style=flat-rounded)](https://discord.gg/Be3yFCzyrp)

![HyperRam board overview](doc/images/micropythonsdk.png)

### Overview :hammer:
Provides an interface to the PicoFabric FPGA over SPI. Allows programming bitstream and accessing board features.

### Whats included :musical_note:
- [x] FPGA Bit Stream uploading.
- [x] DAC initialisation.

### Getting started :mag:
- Upload a bit stream file to the Pico flash storage using Thonny or Pico-W-go file sync.

- Upload the bit stream to the FPGA device using the following.
```
import libfabric
libfabric.fpga_program_device( "bitstream.bit" ):
```

### Support :zap:
- Drop by the [discord](https://discord.gg/Be3yFCzyrp)
- Email help@picolemon.com

### License :penguin:
 
The MIT License (MIT)

Copyright (c) 2023 picoLemon

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
