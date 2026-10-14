# FG Falcon audio-system firmware tooling & emulator (FDIM colour / FDIM mono / AIM)

Reverse-engineering toolkit and a from-scratch V850ES emulator for the Ford FG Falcon audio modules shipped as Ford PHF files:

* `...-14D017-CB-applicationFNOS.phf` — colour "High Display" FDIM (external sprite graphics controller) — `docs/FDIM-analysis.md`
* `...-14D017-AC_FDIM-application.phf` — monochrome "Low Display" FDIM (240×80 S1D15721 LCD driven directly, all bitmaps/fonts/logos
  in flash) — `docs/FDIM-AC-mono-analysis.md`, extracted graphics in `gfx-lo/`
* `...-14F029-AB_AIM-application.phf` — AIM / Front Entertainment Module (iPod iAP interface over UART, audio switch over SPI,
  ISO-TP text channels to the FDIM) — `docs/AIM-analysis.md`, iPod logo images in `gfx-aim/`

The emulator runs any of them alone, or several together on a **virtual MS-CAN bus** (`-module a.bin … -module b.bin …`); with the
built-in iPod simulator (`-ipod`) the real AIM firmware identifies the "iPod", uploads the Ford oval to it, and sends the track text
that the real FDIM firmware then draws (`docs/aim-fdim-ipod.png`).

```
phf/phf2bin.py            PHF (binary Intel-HEX) -> raw flash image, verifies Ford FILE CHECKSUM
analysis/survey.py        first-pass survey: vector table, strings, immediates, SFR footprint
analysis/xref.py          linear sweep + address resolution -> peripheral cross-reference (pickle)
analysis/adis.py          annotated disassembler (Ghidra V850 SLEIGH via pypcode + SFR names)
analysis/decode_tables.py PNET image directory, fonts, pages/fields, vector map, CAN config -> JSON   (colour unit)
analysis/decode_lo_tables.py  bitmaps (187), fonts (4), vector table, CAN RX buffer map, strings -> PNG/JSON (mono unit)
analysis/decode_aim_tables.py vector table, CAN RX/TX tables, iAP response list, iPod logo images -> PNG/JSON (AIM)
analysis/sj3_regs.json    V850ES register names (parsed from the Renesas manual)
emu/                      V850ES emulator (C), auto-detects the unit, multi-module virtual bus, iPod simulator (ipodsim.c);
                          render.py (colour) / render_lo.py (mono) frame converters
docs/FDIM-analysis.md     colour unit write-up;  docs/FDIM-AC-mono-analysis.md  mono unit write-up;  docs/AIM-analysis.md  AIM write-up
gfx-lo/                   every bitmap and font of the mono unit as PNG + JSON (Ford oval = img_00dcc4_235x66.png, FPV = img_00d4e0_235x66.png)
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

### Mono unit quick start

```bash
python3 phf/phf2bin.py 572e16f4-8R29-14D017-AC_FDIM-application.phf lo.bin      # repairs the corrupted record type byte
python3 analysis/decode_lo_tables.py lo.bin gfx-lo                               # bitmaps, fonts, tables
cd emu && make
./fdim-emu ../lo.bin -ms 4000 -can ../examples/lo_aux.can -adcall 430 -eeprom ee.bin -snap 250 -out out -quiet -log out/run.log
python3 render_lo.py out/final.pgm -scale 3 -crop                                # -> out/final.png  ("AUX1 / Auxiliary" page)
./fdim-emu ../lo.bin -ms 46000 -can ../examples/lo_logo.can -adcall 430 -eeprom ../examples/lo_eeprom_ford.bin -snap 250 -out out2 -quiet -log out2/run.log
python3 render_lo.py out2/frame_0166.pgm -scale 3 -crop                          # Ford oval splash (use lo_eeprom_fpv.bin for the FPV logo)
cd ../examples && ./lo_sources.sh ../emu/fdim-emu 8R29-14D017-AC-app.bin         # AUX1/AUX2/CD MP3/CD/AM/PHONE pages -> lo_sources.png
```
The emulator picks the board from the image size (256 KB = mono FDIM or AIM, told apart by the application header). Mono-specific
options: `-adcall 430` (supply voltage reading; the default 700 reads as over-voltage on this unit), `-eeprom FILE` (persistent
settings image; a missing/blank file is a first boot), `-eev` (log EEPROM traffic), `-width N -xoff N -flip` (render window,
default 240 px from column 16).

### AIM + FDIM on the virtual bus (iPod demo)

```bash
cd examples && ./bus_ipod.sh            # ~1 min: boots both firmwares, pairs the AIM, docks the simulated iPod, renders the FDIM
```
Options are grouped per module after each `-module file`; global options (`-ms -can -snap -quiet …`) go anywhere. Every frame a
module transmits is delivered to the others ~0.25 ms later and logged to `bus.log` with its sender; `-can` stimulus goes to all
modules. Module options: `-eeprom -adc -adcall -pin IDX VAL -uart FILE -ipod -rom -log -out -name -watch -call -wmem -rwatch -eev`.

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
| `-call PC` | log calls to an address with r6-r9 and the first 8 stack words |
| `-wmem ADDR` | log every change of a 32-bit RAM word (checked once per ms) |
| `-rwatch ADDR` | log data reads of a 256-byte window with the reading PC |
| `-fhist FILE` | write a histogram of flash data reads (64-byte blocks) at the end |
| `-eeprom FILE` / `-eev` | 256 KB units: SPI EEPROM image (read at start, written back when changed) / log EEPROM commands |
| `-pin IDX VAL` | preset an input port (hex index of the Pn register low byte, e.g. `0d` = P6H) |
| `-uart FILE` | AIM: inject bytes into UARTA1 (`ms hexbytes…` per line) |
| `-ipod` | AIM: attach the iPod simulator to UARTA1 (iAP Lingo 0/2/4, 4-track library); logs `IPOD RX/TX` lines |
| `-trace-at PC -trace-insns N` | instruction trace from an address |
| `-gdcv` | log every graphics-controller bus transaction |
| `-log FILE` | log file (default stderr) ; `-quiet` suppresses SFR/unknown-access logging |

Outputs: colour unit `final.ppm(.json)` + `objects.txt`; mono unit `final.pgm` (240×84, 4 grey levels) + `final_lcdram.bin`
(raw S1D15721 RAM); frames `frame_NNNN.*` with `-snap`; the CAN TX log and LCD/EEPROM transaction log go to the log file.

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

The mono unit additionally wants the audio unit's messages: 0x2F2 byte 6 bits 0-2 = source (1 AUX1 … 5 radio, 6 phone) and
0x311 byte 7 bit 0 = audio unit OK (otherwise it shows "Audio Code Error"). See `examples/lo_*.can` and the mono write-up for
the ignition-off / sleep / wake-up sequence that triggers the Ford or FPV start-up logo.
