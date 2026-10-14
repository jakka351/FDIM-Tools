# FG Falcon FDIM "Low Display" (8R29-14D017-AC) — monochrome unit analysis

Companion to `FDIM-analysis.md` (the colour "High Display" `-CB` unit). Same MCU family, same tooling,
very different display path: this unit drives a **240×80 4‑grey‑level LCD directly** from the micro
through an Epson **S1D15721** controller, and every bitmap, font and splash logo lives in the application
flash — which is why the Ford oval and the FPV logo could be pulled straight out of the PHF.

Everything below was derived from `572e16f4-8R29-14D017-AC_FDIM-application.phf` with the tools in this
tree and confirmed by running the firmware in the emulator (`emu/`), which boots it, talks to the modelled
LCD/EEPROM/CAN and renders the real screens (Ford/FPV splash, AM/CD/AUX/PHONE pages, "Audio Code Error",
"Power Saver Active").

---

## 1. Container

| | |
|---|---|
| format | Ford PHF: text `KEY>value` header terminated by `$`, then binary Intel‑HEX records (`3A len addr16 type data cks`), types 00/01/04 |
| image | 261 890 bytes, application at **0x4500–0x3FA73** plus 2 bytes at 0x3FF00 (256 KB part; bootloader 0x0000–0x44FF is not in the file) |
| header checksum | `FILE CHECKSUM = 0x2ACC` (16‑bit sum of all data bytes) |

The file is **slightly corrupted** (4 bytes):

* header: one `A` → `` ` ``;
* record @0xD69C (32 bytes for 0xF9E0): type byte `0x66` instead of `0x00`. The record checksum validates with type 00, so
  `phf2bin.py` now recovers it ("corrupted data record" message) — this restores 32 bytes of a string/pointer table;
* records @0x0FB4 (→0x5260) and @0x1C9E (→0x5D40): one data byte each is low by 0x29 and 0x3D respectively. 0x29+0x3D = 0x66 is
  exactly the gap between the header checksum (0x2ACC) and the sum of the recovered image (0x2A66), so nothing else is damaged.
  The affected bytes sit in a small pointer table and in the "CD" soft‑key bitmap; neither stops the firmware from running.

## 2. MCU and memory map

* NEC/Renesas V850ES core, same SFR layout as the colour unit (ports 0–9/DL/CM/CS/CT/CD, aFCAN at 0x03FEC000/0x03FEC600,
  TMP0‑3/TMQ0, TAB, WT, ADC, INTC at 0xFFFFF110…), i.e. a **V850ES/Fx3 (FG3 lineage)**. This unit has **256 KB flash / 12 KB RAM**
  (RAM 0x03FFC000–0x03FFEFFF, cleared at reset) → µPD70F3372 class; the colour unit is the 384 KB/24 KB sibling.
* 20 MHz CPU clock (PLL ×4 of the 5 MHz crystal, `LOCKR` polled), 32.768 kHz sub‑clock used by the watch timer and for sleep.
* `gp = 0x0400400C` (all globals are `gp‑0x….` offsets, RAM = gp‑0x800C…), `sp = 0x03FFEDAC`, `ep = 0x03FFC000`,
  `CTBP = 0x18200` (callt table).
* Bootloader interface: `0x700` = pointer to the FBL reprogramming entry (called from the diagnostic handler), `0x704–0x706`
  FBL version bytes. The emulator synthesises both.

### 2.1 Vector table — compact 4‑byte slots

Unlike the colour unit (16‑byte slots mirrored at 0x4400), this application has a **compact table at 0x4500**: 92 slots of
4 bytes, each a `jr` or a `reti`. Slot *k* corresponds to hardware vector `0x10·(k+1)` (slot 0 is the application entry):

| slot | vector | handler | source |
|---|---|---|---|
| 0x4500 | reset/entry | 0x183E0 | sets gp/sp/ep, clears RAM, CTBP, init |
| 0x4544 | 0x120 | 0x19318 | timer @0x540 (TMQ0/TAB0) CC0 — **1 ms tick** |
| 0x4558 | 0x170 | 0x194AA | TMP0 CC0 |
| 0x45A8 | 0x2B0 | 0x19C4A | INTAD (channels 0,1,4,5) |
| 0x45AC–0x45B8 | 0x2C0/2D0/2E0/2F0 | 0x1AE88/0x1ADA4/0x1AF54/0x1B110 | **CAN0** ERR / WUP / REC / TRX |
| 0x45BC | 0x300 | 0x33B00 | INTKR — key return, buttons on P50–P55 (`KRM = 0x3F`) |
| 0x45C4 | 0x320 | 0x19D94 | INTWT overflow (0.25 s; counts seconds) |
| 0x45D8 | 0x370 | 0x19236 | timer @0x610 CC0 — **5 ms** (CCR0 0xC30, ÷32) |

There is **no RTOS** in this unit: a super‑loop (`0x18ABE…`) plus these ISRs.

## 3. Peripherals

### 3.1 CAN0 (MS‑CAN, 125 kbit/s: BRP 9, BTR 0x3508)
* TX from mailbox 0: IDs **0x55C, 0x2E2, 0x2FC, 0x307, 0x313, 0x315** (periodic status / button reports).
* RX mailboxes 1–29 (exact‑ID): 7A6, 406, 353, 311, 30F, 30D, 30B, 309, 304, 301, 2FE, 2F9, 2F5, 2F4, 2F2, 2EF, 2EC, 2E8, 2E6,
  2E4, 2E1, 2DE, 2DB, 2D8, 2D4, 2CE, 2CC, 2CA, 2C0; mailboxes 30/31 are mask buffers (mask = everything) that catch the
  remaining IDs (diagnostic functional requests etc.) and go through a separate handler (0x1AD2A). Each received frame is copied into a RAM buffer selected by `0x4A48[mb+7]` → slot → `0x4AB8[slot]`
  (see `gfx-lo/tables.json`), e.g. **0x406 → gp‑0x5F70**, 0x2F2 → gp‑0x5DB4, 0x311 → gp‑0x5DDC, 0x2E1 → gp‑0x5F60, 0x353 → gp‑0x5F80.
* Message bits decoded so far (from the state machine, verified in the emulator):

| ID | field | meaning |
|---|---|---|
| 0x406 | byte1 bits0‑4 | ignition status: 1 = OFF (→ display off/sleep), 4 = RUN; 2 and 8 are handled like RUN (ACC/START by Ford convention) |
| 0x406 | byte0 bits0‑1 = 1 | "key/ignition‑off event": runs the ignition‑off handler (0x33D2A): saves settings to EEPROM, arms the start‑up logo |
| 0x2F2 | byte6 bits0‑2 | audio source: 0 off, 1 AUX1, 2 AUX2, 3 CD‑MP3, 4 CD, 5 radio (AM/FM pages), 6 PHONE, 7 blank |
| 0x311 | byte7 bit0 | audio unit present / security OK — clear ⇒ **"Audio Code Error"** page |
| 0x2E1, 0x2D4, 0x353, 0x30B, 0x2F4 | — | consulted by the display state machine (not decoded) |

* Diagnostics: 0x7A6 physical request, UDS‑style handler at 0x1F80A (routine control with routine IDs 0x7140, 0x7124, 0x3153,
  0x8000/1/4, 0x8134/5, 0x992D, 0xC11C, 0x1928; NRC 0x22/0x31 responses), display self‑test routines use the checker‑board bitmaps.

### 3.2 Supply monitoring
The ADC result (channel 0, raw 10‑bit value kept in gp‑0x5F3A) feeds the **system state machine** (gp‑0x7F58, 0x319CA):
< 0x120 (288) ⇒ under‑voltage, > 0x228 (552) ⇒ over‑voltage (display off), 0x146/0xD4 hysteresis thresholds.
The emulator defaults to 700 (fine for the colour unit) — use **`-adcall 430`** (≈12 V) for this unit or it sits in over‑voltage.

### 3.3 EEPROM (settings)
A 25xx‑type SPI EEPROM bit‑banged on **P4**: bit2 = SCK, bit1 = MOSI, bit0 = MISO; **/CS = P9H bit5**; PCM bit0 is raised around
writes (write‑protect). 8‑bit addresses, 8‑byte pages, commands WREN/WRDI/RDSR/READ/WRITE (0x3517E write byte, 0x35140 read byte —
note the 9‑sample read trick that compensates for SCK being left high after the address). Records seen:

| address | content |
|---|---|
| 0x00–0x05 | header; `0x05 = 0xA5` validity marker, `0x04 = 0x09` |
| 0x28–0x3C | audio/vehicle settings |
| **0x63** | display settings byte (gp‑0x5FAB): **bit3 = "Ford Logo" on** (toggled by the Display menu, 0x3278A), bit2 = the neighbouring "Screen Off" menu item, bits0/1 = runtime flags cleared at ignition‑off |
| **0x64** | vehicle configuration (gp‑0x5FBC): **bit3 = FPV** (shows the FPV logo instead of the Ford oval), bit4 = iPod/AUX1 variant |
| 0x65, 0x66 | further configuration |

A blank EEPROM (0xFF) makes the firmware write defaults on first boot (`[3]=FF [4]=09 [5]=A5 [65]=FB`); the emulator keeps the image in
a file (`-eeprom file`) so settings persist between runs.

### 3.4 Other I/O
* **P0 bit2** = display power / backlight enable (cleared in every "off" state). TMP2/TMP3 are configured without interrupts —
  almost certainly the PWM outputs for backlight brightness / LCD contrast. P9H bit5 = EEPROM /CS; the other P9H bits are outputs
  that have not been identified (illumination / status lines).
* **P5 bits 0–5** = front‑panel buttons via the key‑return interrupt (ISR 0x33B00 samples P5 into gp‑0x56BC).
* Sleep: when ignition is off and the audio unit is off, the display is switched off (`0xAE`), the CAN controller is put into
  power‑save, the CPU switches to the **32 kHz sub‑clock and stops the main oscillator (`PCC = 0x48`)** and waits for the
  CAN wake‑up interrupt (INTC0WUP). The emulator models the CLS status bit and the wake‑up on bus activity.

## 4. The LCD: Epson S1D15721 on port DL

8080‑style parallel bus, all bit‑banged:

| signal | pin | | signal | pin |
|---|---|---|---|---|
| D0–D7 | PDL0–7 (PMDL low byte 0x00 out / 0xFF in) | | A0 (1 = data/parameter) | PDLH bit2 |
| /RD | PDLH bit0 | | /CS | PDLH bit3 |
| /WR | PDLH bit1 | | /RES | PDLH bit4 |

Primitives: 0x3C98E write command, 0x3C934 write data/parameter, 0x3C8CC write data block, 0x3C864 write command block,
0x3C7C4 read data, 0x3C814 read status, 0x3C9E8 reset pulse. Three copies of the init sequence exist (0x184E0, 0x186F8/0x18924 with
reset, and 0x3E4D4 for re‑init); decoded against the S1D15721 manual (`docs/s1d15721.txt` has the command table):

```
66 00   display mode = 4-grey (2 bits/pixel), dummy period lit     A1      column (SEG) direction reversed
39 53   grey-scale pattern (contrast of levels 1/2)                 C5      COM scan reversed  -> panel mounted rotated 180°
B1 00   page 0                 8A 00  start line 0                   E7 1B   MLS drive select
A6 / A4 normal / all-on off    84     auto-increment along columns   6D 13 00  81 display lines (80 + icon line), block 0
2B 07   V3 range               A2 00  LCD bias                       81 4C   electronic volume (contrast; also set from EEPROM)
4E 00   temperature gradient   5F 09  oscillator frequency           AB      oscillator on
25 07   power control          13 00  column 0                       AF      display on
```
Afterwards: `29` temperature sensor on, `81 xx` contrast updates, `A8/A9` power‑save off/on, `AE` display off on every "off" state,
and a RAM read‑back test (`1D 55 … 1C` + dummy read) at 0x37DB8 to detect the glass.

Display RAM model: 21 pages × 256 columns; in 4‑grey mode a byte holds 4 vertically adjacent pixels (bits 1:0 = top line) so pages 0–19
are the 80 picture lines and page 20 the icon line. The firmware keeps a full **frame buffer at 0x03FFC928** (gp‑0x76E4, 21×256
bytes) and flushes rectangular windows with `B1 page / 13 col / 1D data…` (0x3ED66). The visible glass is **columns 16–255 = 240 px** (inferred: the
firmware draws its 235‑px‑wide frames at x = 0x15/0x16 and the full‑RAM clears start at column 21). The emulator renders the logical RAM unrotated (the two
"reversed" commands cancel the mechanical rotation) and crops to those 240 columns.

Pixel values: 0 = off (dark), 3 = fully lit. The firmware mostly uses 0 and 3 (the greys are used for the dimmed/"inactive" soft keys).

## 5. Graphics engine and resources

A small windowed GDI lives at 0x3CAF4–0x3EC08:

* `0x3CAF4 DrawImage(desc, x, y)` → `0x3D926 DrawImageClipped(x, y, w, h, flags, bpp, stride, data, colour)` → row blitter
  `0x3EAC6` → 1‑bpp row renderer `0x3E9D6` writing 2‑bit pixels at `fb[(y/4)*256 + x]`, bits `(y&3)*2`.
* `0x3CBAA SelectFont(table)`, `0x3CC38 DrawText(str, x, y)` (proportional, per‑glyph advance), `0x3CC10 SetTextColour`.

**Image descriptor** (16 bytes): `{u16 width, u16 height, u16 stride, u16 bpp(=1), u32 data, u32 aux, ...}`; bitmaps are row‑major,
1 bit/pixel, MSB first, `stride = ceil(width/8)`. `analysis/decode_lo_tables.py` scans the flash for descriptors and extracts
**187 images** (`gfx-lo/`): full‑screen frames and splash pages (235×66), soft‑key labels (40×15: CD+, TXT, RDM, RPT, EXIT, presets 1–6,
CTR/DRV/FRT/RR, MP3, iPod, TRN…), menu title bars (215×12: MAIN MENU, AUDIO, DISPLAY, LIGHTING, LOCKING, ALARM, SOUND EQUALISER…),
sliders, check boxes, the self‑test pages and the checker‑boards.

| descriptor | data | size | content |
|---|---|---|---|
| **0x0DCC4** | **0x0D508** | 235×66 | **Ford oval** (start‑up logo) |
| **0x0D4E0** | **0x0CD24** | 235×66 | **FPV logo** (start‑up logo on FPV‑configured cars, EEPROM 0x64 bit3) |
| 0x0A120 | 0x09964 | 235×66 | "Power Saver Active — Screen now shutting down" |
| 0x0BDA4 | 0x0B5E8 | 235×66 | rounded frame used by the audio pages |
| 0x06A58 / 0x0707C / 0x074F4 | | | "Insert 1 CD Now", "Ejecting Disc", "Wait" CD pages |
| 0x12B40 / 0x13324 | | | "Starting Self Test Mode" / "Self Test Mode complete" |

**Fonts** — four glyph tables, 8‑byte entries `{u8 width, u8 advance, u16 stride, u32 data}` from character 0x20:

| table | height | glyphs | use |
|---|---|---|---|
| 0x14B50 | 10 px | 69 (caps only) | small labels |
| 0x153C8 | 12 px bold | 96 | menus |
| 0x16568 | 20 px | 96 | main text ("Audio Code Error", "Auxiliary", "Track 0", "Empty") |
| 0x17660 | 12 px light | 95 | secondary text |

The logo page is drawn by `0x185EC`: in display state 7 it draws descriptor 0xD4E0 (FPV) when config bit3 is set, else 0xDCC4 (Ford),
at (0x15, 0) and starts a 2.5 s timer; state 0xD draws the Power‑Saver page.

## 6. Display power / page state machine

Two cooperating state variables: the **system state** gp‑0x7F58 (0 init → 4 → 1 → 2 → 3 "running", 5 = over‑voltage) driven by the
supply voltage, and the **display state** gp‑0x7F54 computed every 215 ms by 0x34358 and acted on by 0x33F30:

| state | meaning |
|---|---|
| 1, 2 | display off (`AE`), P0.2 low |
| 3 | sleep: CAN power‑save, sub‑clock, wait for CAN wake‑up |
| 5 | ignition off but audio unit on → audio page stays on (0x2F2 source ≠ 0); after a timeout → 0xD |
| **7** | **start‑up logo** (Ford/FPV), ~2.5 s, then 9 |
| 8 | ignition RUN, audio off → blank frame |
| **9** | ignition RUN, audio page (AUX/CD/radio/…) or "Audio Code Error" |
| 0xD | "Power Saver Active" |

The logo (7) is only entered on ignition RUN/ACC when the unit previously saw an ignition‑off event (0x406 byte0 bits0‑1 = 1 → EEPROM
display byte bit3 "Ford Logo" set and saved), the audio handshake is OK (0x311 byte7 bit0) and the unit has been off ≥ 30 s (seconds
counter kept by INTWT while in phase 2) or has just been reset (phase 3). This is why a plain power‑up with ignition already on goes
straight to the audio page, exactly as the real car does.

## 7. Emulator (`emu/`, one binary for both units)

`fdim-emu` auto‑detects the board from the image size (384 KB → colour/GDC/CAN1, 256 KB → mono/S1D15721/CAN0) and synthesises the
matching bootloader vector table. New for this unit: `lcd.c` (S1D15721 model + renderer), the SPI EEPROM, the sub‑clock sleep/CLS status,
CAN wake‑up, INTWT overflow, timer‑0x610 vectors, 16‑bit port‑DL writes, and the diagnostics options `-wmem ADDR` (log memory word
changes), `-call PC` (log calls with register + stack args), `-rwatch ADDR`, `-fhist FILE` (flash read histogram).

```
make
./fdim-emu ../examples/8R29-14D017-AC-app.bin -ms 8000 -can ../examples/lo_aux.can -adcall 430 -eeprom ee.bin -snap 250 -out out -quiet -log out/run.log
python3 render_lo.py out/final.pgm -scale 3 -crop            # or: -sheet gallery.png with several frames
```
Mono‑unit options: `-adcall 430` (supply), `-eeprom FILE` (persistent settings; blank file = first boot), `-eev` (log EEPROM traffic),
`-gdcv` (log every LCD command/burst), `-width/-xoff/-flip` (render window). Frames are 240×84 PGM (80 lines + icon line).

Stimulus files in `examples/`:

| file | what it shows |
|---|---|
| `lo_ign_only.can` | ignition RUN, no audio unit → "Audio Code Error" |
| `lo_aux.can` | ignition RUN + 0x2F2 source 1 + 0x311 alive → "AUX1 / Auxiliary" page |
| `lo_sources.sh` | sweeps 0x2F2 byte6 1..7 → AUX1, AUX2, CD MP3, CD, AM radio with presets, PHONE, blank |
| `lo_logo.can` | RUN 3 s → OFF (0x406 `01 01`, audio off) → silence 34 s (unit sleeps) → RUN: **Ford logo** (or FPV with `lo_eeprom_fpv.bin`) then AUX page |

What the emulator reproduces: boot, LCD init, EEPROM initialisation, CAN TX schedule, voltage/ignition logic, sleep + wake‑up, the
splash logos and the audio pages. What it cannot show without a real MS‑CAN capture: radio station text/frequency, clock, climate and
menu contents (those come from the other RX IDs). The front‑panel buttons (INTKR) are not modelled yet.

## 8. Open points

* Exact part marking (70F3372 vs a pin‑compatible sibling) — read it off the PCB.
* The remaining two corrupted PHF bytes (0x5260 table, 0x5D40 bitmap region) cannot be repaired from the file alone; a second copy of
  the PHF would settle them.
* Decoding of the other audio‑unit messages (0x2E1, 0x2D4, 0x30B, 0x2F4…) for the radio/CD text pages — straightforward with a candump
  from a running car (`-can` accepts `candump -L` logs directly).
