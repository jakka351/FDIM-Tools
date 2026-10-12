# FG Falcon FDIM firmware (8R29-14D017-CB) — analysis & emulator notes

Source file: `c07cc811-8R29-14D017-CBFDIM-applicationFNOS.phf` (Ford IDS flash file,
"FORD FNOS-ORION APPL, adapted for Bootloader", released 16/06/2008 by Sumitomo Wiring
Systems, module "High Display", diagnostic ID 0x7A6).

## 1. PHF container

* Plain-text header (`KEY>value` pairs) terminated by `$`.
* Body = binary Intel-HEX records: `0x3A len addr16(BE) type data[len] checksum`, types
  00 (data, 32 bytes each), 04 (extended linear address), 01 (EOF). Checksum is the usual
  Intel-HEX two's complement.
* `FILE CHECKSUM>0x584B` = 16-bit sum of all data bytes (verified by `phf2bin.py`).
* Payload covers **0x04400–0x5DFFF** (367,616 bytes). The flash boot loader (FBL) at
  0x0000–0x43FF is not part of the application PHF (`FLASH ERASE SECTORS>0x00`).

## 2. Target hardware

| item | finding | evidence |
|---|---|---|
| CPU | NEC/Renesas **V850ES** core (V850E1 ISA) | instruction set, SFR map at 0xFFFFF000, PSW/ECR usage |
| family | ES/x2–x3 generation automotive part, 20 MHz | PLL ×4 from 5 MHz: CAN timing (BRP=9, 16 Tq → 125 kbit/s) and 1 ms tick (TMP2 /32, CCR0=624) both give fXX = 20 MHz |
| flash/RAM | 384 KB flash, RAM 0x03FF9000–0x03FFEFFF (app uses 0x03FFA000–0x03FFEFFF, top 32 bytes reset-persistent) | startup RAM test & clear loops |
| exact part | **not settled**: register footprint = V850ES/Fx3 style (P12 @0xFFFFF418, TAB1/TAB2 @0x610/0x620, INTF1/INTF6), vector map ≠ SJ3 ≠ Fx3 | read the chip marking on the PCB |
| RTOS | NEC **RX850-family µITRON** ("RX703000 Copyright (C) NEC Electronics 1997,2005") | string at 0x3E2D1, kernel at 0x3D3A0–0x3E2D1, ISR prologue at 0x33526 |
| CAN | aFCAN channel 1 at 0x03FEC600 (BPC=0x8FFB), **MS-CAN 125 kbit/s**, 1 TX buffer + 29 RX buffers + 2 masked ranges | CAN init at 0x4E60, buffer config dump from emulator |
| watchdog | WDTM2 written 0x1F (stopped); WDTE never serviced | startup 0x13A10 |
| DMA | unused (DCHC0–3 cleared only) | |
| display | external **graphics controller with sprite/object engine**, driven over a bit-banged 8-bit parallel bus on GPIO | see §4 |
| TFT driver | 17 × 16-bit SPI register writes (regs 0x20–0x30) on CSIB0, chip-select P4.0, paced by interrupt | 0x19800 |
| backlight | PWM on TMP1 (TO1), 150 Hz, duty from ambient-light lookup (761-entry table at 0x3FCC0) | 0x19A9E, 0x166CA |
| other I/O | 3-wire bit-banged device on P9.2/9.3/9.4 (16 regs, opcodes `(reg<<4)|3` write, `(reg<<4)|0xC` read, LSB first), P6H.6/P6H.7 display power, P3.1 blanking, P8.1 toggle, PCT5 = GDC reset, PCM4/PCM5 GDC control | |

### Interrupt vectors actually used (from the firmware's own dispatch slots)

Each 16-byte vector slot in the app table (0x4400+v) holds `jr shared_entry`, the C handler
pointer at +4 and the priority byte at +15. The kernel's shared entry reads ECR and jumps
through the slot pointer.

| vector | IC reg | source (firmware-derived) | handler |
|---|---|---|---|
| 0x090 | 0xFFFFF112 | INTP0 = graphics controller VSYNC/IRQ | 0x18228 → callback 0x175F8 |
| 0x0D0/0E0/0F0, 0x450/460/470 | | external pin interrupts (P9-related) | 0x149C8 |
| 0x1D0 | 0xFFFFF13A | TMP2 CC0 — **RTOS 1 ms tick** | kernel 0x3DB5C |
| 0x230 | 0xFFFFF146 | CSIB0 transfer complete | 0x19884 |
| 0x290 / 0x2A0 | | table-driven, unused at runtime | 0x15EA0 / 0x162A8 |
| 0x2B0 | 0xFFFFF156 | A/D conversion complete | 0x13700 |
| 0x310 | 0xFFFFF162 | watch timer interval (~31 ms) | 0x18700 |
| 0x3D0 / 0x3E0 / 0x3F0 / 0x400 | 0xFFFFF17A–180 | CAN1 ERR / WAKEUP / RX / TX | 0x4D9C / 0x5CDC / 0x4C70 / 0x4C18 |

## 3. Memory layout of the application

| range | content |
|---|---|
| 0x04400–0x049FF | vector table (90 slots × 16 bytes, handler pointer + priority inside each slot) |
| 0x04A00–0x04BFF | kernel configuration (system stack ptr at 0x4B04, task/semaphore counts 0x4BC7–0x4BD1) |
| 0x04C00–0x3DFFF | code (~230 KB); CAN driver 0x4C00–0x5FFF; app tasks; display stack 0x16000–0x1C800; diagnostics 0x37000–0x3CFFF; kernel 0x3D3A0–0x3E2D1 |
| 0x3E2D1 | "RX703000 Copyright (C) NEC Electronics 1997,2005" |
| 0x3F35E | CAN config (C1IE=0x2700, BRP=9, BTR=0x3508) |
| 0x3FCC0 | ambient light → brightness lookup (761 bytes) |
| 0x3FFDC | empty "PNET" directory (default) |
| 0x40228 | TFT-driver SPI register address list (0x20..0x30) |
| 0x4028C–0x4088B | 4 font tables × 96 glyphs × {u16 image id, u16 advance} |
| 0x4088C | font pointer table, count at 0x4089E |
| 0x40FFC–0x4549x | page → line → field tables (106 pages, 12-byte lines/fields) |
| 0x45AE4 | page table (106 × {u16 enabled, u16 lines, u32 line_table}) |
| 0x46554 | **"PNET" image directory**, 2123 × 12-byte sprite descriptors |
| 0x4D9CC–0x4DC00 | diagnostic/engineering screen strings ("Version Information", "Switch Information", button names…) |
| 0x4E7CC–0x4E9xx | kernel/app init tables (init function list at 0x4E840) |
| 0x4EB20, 0x4EEC2 | "8R29-14D017-CB" part number strings (DID data) |

## 4. Display pipeline

### 4.1 Bus (GPIO, 8080-style)

| signal | MCU pin |
|---|---|
| D0–D7 | P12 (0xFFFFF418), direction PM12 (0xFFFFF438) |
| A0–A2 | PCD0–PCD2 (0xFFFFF00E) |
| /CS | PCS0 (0xFFFFF008 bit 0) |
| /WR | PCS2 (bit 2, pulsed low) |
| /RD | PCS3 (bit 3, pulsed low) |
| READY | PCT2 input (0xFFFFF00A), polled up to 10× before a read |
| RESET | PCT5 (low pulse ≈ 80 µs) |
| misc | PCM4 set after reset; PCM5 toggled during the bus test |

Functions: `0x17FB0 write_port(a, d)`, `0x17F44 read_port(a)`, `0x17740 write_reg(n, v)`,
`0x176E8 read_reg(n)`, `0x17C50 block_transfer(request)`.

### 4.2 Controller ports (A2:A0)

| port | function |
|---|---|
| 0 | object-attribute memory data window (4 KB), address regs 5/6, auto-increment when reg4.bit1 |
| 1 | region-1 data window (1 KB; CLUT-sized), address regs 7/8, auto-inc when reg4.bit0 |
| 2 | status (bit 6 = busy); writing 0x80 precedes image-memory reads |
| 3 | image memory data window (32 MB space), address regs 9–12 (big-endian), auto-inc when reg4.bit2 |
| 4 | register index; bit 7 = burst/auto-increment |
| 5 | register data (48 registers 0x00–0x2F) |
| 6 | read/cleared once during init (status/IRQ ack?) |

Init sequence observed in the emulator: reset → read all 48 regs → clear OAM → regs 0x11–0x14=0
→ timing regs 0x17–0x2F (0F A8 9C 01 02 02 63 01 01 02 62 00 0F 00 5D 00 0F 00 72 01 00 02 62 82 0F)
→ bus test: read 2 bytes at image address 0x01FFFFFC, expects 0x5555 (PCM5=0) / 0xAAAA (PCM5=1)
→ port6 read/clear, regs 1–3 = 0, reg 0 = 0x20 → regs 0x0D–0x10 = 00 00 07 EC → background colour
regs 0x15/0x16 = **0x001F (RGB565 blue)** → VSYNC IRQ on INTP0 (≈60 Hz) → object uploads;
reg 0x0D alternates 0x87/0x80 every 100 ms = double-buffer swap (objects 0–169 vs 170–339).
A 3-tick watchdog (refresh timer 33 ms) with no VSYNC → controller reset and re-init.

### 4.3 Object descriptor (12 bytes, OAM, 340 objects)

```
b0 : [7:4] flags (0xA)   [3:0] y[11:8]      b1 : y[7:0]        (12-bit signed)
b2 : [7:4] flags         [3:0] x[11:8]      b3 : x[7:0]
b4 : bit6 = visible, [5:0] = width/8 - 1    b5 : height - 1
b6..b9 : image address in controller memory (big-endian)
b10,b11: attributes (0x40 0x40 everywhere)
```

Full-colour images are RGB565, 2 bytes/pixel, stored back-to-back (the PNET directory's
addresses are exactly cumulative w×h×2). Glyph images use the same descriptor but the data
size tracks glyph *width* — 32/64 bytes for 16-px fonts, 48/96/144 bytes for the 24-px
font — i.e. **2 bits per pixel** (outline/anti-alias font format), heights 16 or 24 px.
Image ids ≥ 20000 are font glyphs (id−20000 = directory index); ids ≥ 30000 select a
day/night pair (+1 when night mode). Total image ROM ≈ 3.06 MB → **the pictures are in the
display module's own flash and are not in this PHF**. The firmware can write that memory
byte-wise through port 3 (used by a diagnostic routine at 0x3C19C), so a second IDS file
carrying the images probably exists.

### 4.4 Rendering model in the firmware

Pages (106) → lines → fields (12-byte records: x0, y, type/attrs, x1 …). Text is drawn by
`0x1B910(row, col, string, style)` which looks up per-glyph advance widths, allocates one
object per character (`0x1C4D4`), and positions it. The screen appears to be ~480 × 234
(field coordinates reach x≈500, y≈210 + 24-px font). Backgrounds are composed from 240×61
tiles (per-object size limit of the controller).

## 5. CAN behaviour (MS-CAN, 125 kbit/s)

Transmitted by the FDIM (seen in the emulator with no stimulus):

| ID | period | notes |
|---|---|---|
| 0x55C | 2 s | network management `5C 01` → `5C 02` → `5C 04` |
| 0x2FC | 100 ms | byte 0 bit 0 set once any valid traffic is received; byte 7 = 0x46 |
| 0x2E2, 0x313, 0x315 | 250 ms | 0x315: `00 00 00 07 FF 00 00 00` |
| 0x307 | 500 ms | byte 3 0x64 → 0xE4 after traffic |

Received (receive buffers): 0x7A6 (diag request), 0x6FC, 0x409, **0x406 (ignition
status: value 4 = RUN, 8 = START → display on; absence > ~1.2 s → display off)**, 0x403,
0x360, 0x353, 0x330, 0x317, 0x311, 0x30F, 0x30D, 0x30B, 0x309, 0x304, 0x301, 0x2F9,
0x2F5, 0x2F4, 0x2F2, 0x2EF, 0x2EC, 0x2E6, 0x2E4, 0x2E1, 0x2D4, 0x2D2, 0x2CA, 0x128;
masked ranges 0x500–0x57F and 0x2C0–0x2FF (network management / diagnostics path).

System state machine (0x18A68 et al.): states 1→2→4→8→0x10→0x20 (display active);
transition events `(old<<16)|new` are broadcast to every module (0x100020 = display on,
0x200010 = display off).

## 6. Emulator architecture (`emu/`)

* `v850.c` — decoder + interpreter for the integer V850E1 ISA (all formats, prepare/dispose,
  switch, callt, bit ops, saturating ops, ldsr/stsr, trap/reti). Decoder verified against
  Ghidra's V850 SLEIGH over the whole code region.
* `sys.c` — memory map (flash, 24 KB RAM + 0xFFFFxxxx mirror, SFRs, CAN PPA), INTC with
  priorities/ISPR and `reti` hook, TMP0–3/TMQ0–2 timers (interval + PWM), watch timer,
  CSIB0, 10-bit ADC, aFCAN (RX/TX history lists, set/clear semantics, masks), ports incl.
  the GPIO bus decode and the P9 3-wire device, synthetic bootloader vector table.
* `gdc.c` — graphics controller: ports, registers, OAM, region-1, 32 MB image space (loads a
  ROM dump if given), bus-test pattern, VSYNC IRQ, PPM + JSON frame dumps.
* `render.py` — composites frames: real pixels when a ROM is present, otherwise text from the
  font tables via a substitute font and labelled boxes for pictures.

Known gaps: exact MCU part number; GDC register semantics beyond those exercised; the
3-wire P9 device identity (returns zeros, firmware tolerates it); CSIB1/UART peers (never
used at runtime); the display's image ROM (needs a dump or the second IDS file).
