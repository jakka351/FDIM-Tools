# FG Falcon FDIM (8R29-14D017-CB) — firmware tooling & emulator

Reverse-engineering toolkit and a from-scratch emulator for the Ford FG Falcon
**Front Display Interface Module** ("High Display") application firmware shipped in
Ford PHF files (`...-14D017-CB-applicationFNOS.phf`).

```
phf/phf2bin.py            PHF (binary Intel-HEX) -> raw flash image, verifies Ford FILE CHECKSUM
analysis/survey.py        first-pass survey: vector table, strings, immediates, SFR footprint
analysis/xref.py          linear sweep + address resolution -> peripheral cross-reference (pickle)
analysis/adis.py          annotated disassembler (Ghidra V850 SLEIGH via pypcode + SFR names)
analysis/decode_tables.py PNET image directory, fonts, pages/fields, vector map, CAN config -> JSON
analysis/sj3_regs.json    V850ES register names (parsed from the Renesas manual)
emu/                      V850ES emulator (C) + Python frame compositor
docs/FDIM-analysis.md     full write-up of what was found and how the emulator models it
```

## Quick start

```bash
pip install pypcode pillow          # pypcode = Ghidra's V850 decoder, used by the analysis tools
python3 phf/phf2bin.py file.phf fdim.bin
cd emu && make
./fdim-emu ../fdim.bin -ms 8000 -can ../examples/ign_on.can -snap 1000 -out out -quiet -log out/run.log
python3 render.py ../fdim.bin out/final.ppm.json -scale 2
```

`fdim.bin` is a 384 KB flash image with the application at 0x4400-0x5DFFF (the bootloader
0x0000-0x43FF is not in the PHF; the emulator synthesises its vector table).

## Emulator options

| option | meaning |
|---|---|
| `-ms N` | emulated run time in ms (default 10000) |
| `-can FILE` | CAN input: `ms id d0..d7` lines, or `candump -L` logs, or plain candump output |
| `-can-offset MS` | emulated time at which the first candump frame is injected (default 500) |
| `-rom FILE` | display image ROM dump (up to 4 MB RGB565 + 2bpp glyphs) — enables real pixels |
| `-adc CH VAL` / `-adcall VAL` | 10-bit ADC readings (default 700; the display power-up needs >= 615 on the supply monitor channel) |
| `-snap MS` | write `frame_NNNN.ppm` + `.json` object dumps every MS |
| `-watch PC` | log register state whenever PC hits an address |
| `-trace-at PC -trace-insns N` | instruction trace from an address |
| `-gdcv` | log every graphics-controller bus transaction |
| `-log FILE` | log file (default stderr) ; `-quiet` suppresses SFR/unknown-access logging |

Outputs: `final.ppm(.json)`, `objects.txt` (object table dump), and the CAN TX log in the log file.

## Driving the display

The module turns its display on when it receives ignition status RUN/START (value 4 or 8)
in CAN ID **0x406** on MS-CAN (125 kbit/s). A minimal stimulus file:

```
# ms  id  data
300 406 04 04 04 04 04 04 04 04
400 406 04 04 04 04 04 04 04 08
...
```

Replaying a real MS-CAN capture (`candump -L can0 > log`) from an FG with ignition on
reproduces the real screens (minus the image ROM pixels unless `-rom` is given).
