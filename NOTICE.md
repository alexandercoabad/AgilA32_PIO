# Third-party notices

AgilA32 is released under the Apache License 2.0 (see `LICENSE`). This file lists material that is not original to this
project, and the names used with it.

## Independent implementation, trademarks

The PIO block in `src/pio*.v` is an independent implementation of the RP2040 PIO instruction set, written from the
public RP2040 datasheet. It is not affiliated with, endorsed by or sponsored by Raspberry Pi Ltd. "Raspberry Pi",
"RP2040" and "Pico" are trademarks of Raspberry Pi Ltd; they are used here only to say what the block is compatible
with. No Raspberry Pi logos or datasheet text or figures are included.

## PIO programs derived from the Raspberry Pi Pico SDK and pico-examples

Raspberry Pi's Pico SDK and pico-examples are licensed under the BSD 3-Clause License. The following files in `pio/`
are copies of, or derived from, programs in those projects:

| File | Origin | Change here |
|---|---|---|
| `pio/uart_tx.pio` | Pico SDK `uart_tx` | none |
| `pio/uart_rx.pio` | Pico SDK / pico-examples `uart_rx_mini` | none |
| `pio/spi_master.pio` | Pico SDK `spi_cpha0` | none |
| `pio/spi_cpha1.pio` | Pico SDK `spi_cpha1` | none |
| `pio/i2c.pio` | pico-examples `pio/i2c/i2c.pio` | side-set polarity swapped |
| `pio/spi_cpol1_cpha0.pio`, `pio/spi_cpol1_cpha1.pio` | Pico SDK SPI programs | side-set values inverted (CPOL = 1 without a pad override) |
| `pio/i2c_mm.pio` | structure of the pico-examples I2C program | bit-level arbitration added |
| `pio/ws2812.pio` | same three-phase structure as the pico-examples WS2812 program | delays re-chosen |

The other files in `pio/` (JTAG, SWD, 1-Wire, CAN, PS/2, USB low-speed, VGA, I2C slave, the gate-level test programs and
`ws2812_repeat`) are original to this project.

The BSD 3-Clause License that applies to the derived programs above:

```
Copyright (c) 2020 Raspberry Pi (Trading) Ltd.

SPDX-License-Identifier: BSD-3-Clause

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Other tools

`tools/pioasm.py` is a new assembler that accepts the same source syntax as Raspberry Pi's `pioasm`; it contains no code
from that tool. The RISC-V C flow in `c/` uses an RV32I GCC toolchain installed separately; none of its files are included.
