# FG Falcon Instrument Cluster (GS IPC) firmware — analysis

Full 256 KB flash dump of the FG Falcon **Instrument Panel Cluster** (the gauge cluster:
speedo/tacho stepper gauges, fuel + temp, telltale lamps, and the central **message
centre** LCD / trip computer). Same NEC/Renesas **V850ES** family as the FDIM and AIM, so
it drops into the same emulator. This file (`GS_IPC.bin`) is a raw dump including the boot
block, not a Ford PHF container.

## 1. Image layout

| Region | Contents |
|--------|----------|
| 0x00000–0x003FFF | **boot block / flash boot loader (FBL)**: reset vector @0x0 (`jr 0x6C0`), hardware vector table @0x0 (16-byte slots, each `jr` into the compact table), clock/PLL/RAM bring-up, the self-programming routines. Erased 0x3A8C–0x3FFF. |
| 0x04000–0x041AB | **compact vector table**: 4-byte slots, slot *k* ↔ hardware vector `0x10·k` (k≤6) / `0x10·(k+1)` (k≥7); vector 0x70 has no slot. Slot 0 = app entry 0x41AC. Unused vectors → spurious handler 0x3D7FE. |
| 0x041AC–~0x3F41F | **application**: C code, the µITRON-style kernel, data tables, fonts, the message-centre string tables. |
| 0x3FFF0 | programmed marker `AA AA 55 55`. |

22 live interrupt handlers; everything else points at the spurious-interrupt trap.

## 2. MCU & platform

- **V850ES, large SJ3/FJ3-class variant** — distinct from the 256 KB Fx3 parts in the
  FDIM/AIM: **60 KB RAM at 0x03FF0000–0x03FFEFFF**, interrupt-controller and timer banks
  of the SJ3 family (TMP0–TMP6 + an interval-timer bank at 0xFFFFF600).
- `gp = 0x03FF9F70`, stack from flash 0x3CC84, interrupt stack 0x03FF2714. Startup is
  position-independent (the usual V850 `jarl`/`lp` trick).
- **System clock 16 MHz** (the CAN bit-timings only resolve correctly at fCAN = 16 MHz).
- **RTOS**: a small preemptive kernel. Every ISR saves the full register file +
  EIPC/EIPSW/CTPC/CTPSW, checks a nesting flag at 0x03FF2372, switches to the interrupt
  stack, and dispatches through per-vector stubs (0x3EE30…0x3F2B0, 0x40 apart). Kernel
  services are reached through a **callt table at 0x3C8A4** (≈ 40 entries).

### Interrupts (real handlers)
| Vector | Source | Notes |
|--------|--------|-------|
| 0x150 | interval timer @0xF604 | **1 ms system tick** (cmp = 3999 @ 16 MHz/4) |
| 0x1A0 | TMP1 CC | main-loop scheduler timer (the super-loop polls IC 0xF134) |
| 0x390 | **INTAD** | ADC conversion complete (analogue gauge senders / inputs) |
| 0x3A0/3B0/3C0/3D0 | **CAN0** ERR/WUP/REC/TRX | MS-CAN, priority 5 |
| 0x470 | **INTIIC0** | I²C transfer complete |
| 0x510/520/530/540 | **CAN1** ERR/WUP/REC/TRX | HS-CAN, priority 4 |
| 0x40/0x60/0x90/0xA0/0xE0/0xF0/0x100/0x110/0x140/0x160 | TRAP, port & timer IRQs | enabled via IMR mask words |

### Peripherals
| Block | Use |
|-------|-----|
| **CAN0** @0x03FEC000 | **MS-CAN, 125 kbit/s** (BRP 15, BTR 0x1203 → 4+3+1 TQ). Body/comfort bus — shared with the FDIM/AIM. |
| **CAN1** @0x03FEC600 | **HS-CAN, 500 kbit/s** (BRP 1, BTR 0x120B → 12+3+1 TQ). Powertrain bus — engine/trans data for the gauges. |
| **IIC0** (I²C) @0xFFFFFD80 | serial **EEPROM** (24C16-class, device addr 0x50–0x57, 8-bit word address + block bits). Non-volatile settings/trip/service store. |
| **ADC** (ADA0) @0xFFFFF200 | analogue inputs — sender/supply monitoring. |
| **Timers** TP0/TP3/TP4/TP7/TP8 | PWM — gauge-needle drive, backlight/illumination dimming, buzzer. |
| **Ports 3/9/10/11/12/13** | the message-centre LCD bus (below), stepper-driver control, telltale outputs, switch inputs. |

## 3. CAN map

Both channels use aFCAN with per-ID receive buffers + a masked range for network management.

**Received (exact-ID buffers):**
- **MS-CAN (CAN0):** `0x720` (diag request), `0x6F8`, **`0x406` ignition**, `0x403`, `0x360`,
  `0x313`, `0x30D`, `0x309`, `0x307`, `0x2F9`, **`0x2F2` audio source**, `0x2E6`; masked range
  **`0x500–0x57F`** (network management).
- **HS-CAN (CAN1):** `0x6F6`, `0x640`, `0x44D`, `0x427`, `0x425`, `0x353`, `0x340`, `0x230`,
  `0x210`, `0x207`, `0x200` (powertrain frames carrying rpm / speed / temperatures / MIL).
- A per-ID **handler table at 0x14D28** (22 entries → 0x2CE14…0x2D392) runs one decode
  routine per received identifier.

**Transmitted (observed in the emulator, per channel):**
- **MS-CAN:** `0x128` (~100 ms), `0x330` (~60 ms), **`0x511` network-management** (NM state
  `11/5C 01→02→04`, like the FDIM's `0x55C`), `0x340`, `0x353`, `0x640`, `0x6F6`,
  `0x728` (diag response).
- **HS-CAN:** `0x128`, `0x307`, `0x350`, `0x403`, `0x437`, `0x453`, … (cluster status onto
  the powertrain bus).

The cluster is a bus **gateway**: it receives powertrain data on HS-CAN and re-broadcasts
cluster/vehicle state onto MS-CAN for the body modules (and vice-versa).

## 4. Message-centre LCD

The central dot-matrix display is driven **by bit-banging an 8080-style parallel bus on
GPIO** (not a dedicated controller port):

- **P9 = D0–D7**, **P10.4 = /RD, P10.5 = /WR, P10.6 = A0, P10.7 = /CS, P3.2 = /RES**.
- Controller is an **Epson S1D15-series** (same command set as the FDIM's S1D15721):
  init sequence `E3 A8 EA A6 A4 C4 8A A1 66 6D 36 E5 5F AB 55 A2 2B 81 25 EE E7 4E 85 39 AE`
  then `AF` on. **ADC reverse (`A1`) + COM reverse** → the glass is mounted **rotated 270° and
  mirrored** relative to the controller.
- RAM is written **180 columns × 17 pages**, 4 display lines/page = **180 × ~68 px**, in
  **4 grey levels** stored as two bit-planes per page (low nibble = plane 0, high nibble =
  plane 1). `B1` page / `13` column / `1D` data bursts of 17 bytes per column.

The emulator models this exactly: feeding the real firmware an ignition-on bus with no
engine data makes it draw **"Error"** in the message centre (the trip computer's response to
absent powertrain frames) — rendered pixel-for-pixel from the firmware's own font.

## 5. Message-centre vocabulary (strings @0x10400+)

Null-separated, in ~5 overlapping context blocks (main menu / settings / trip / warning
list). Menus: *Set Overspeed, Set Dimming, Set DisToDest, Reset All Trip, Performance,
Warnings, Settings, Sub Content, Set Illum Colour, Set Illum Colour (Red/Green…), Rest
Timer, Lane Change, Display Off, Rear Belt*. Trip computer: *Speed, Overspeed, Range, Fuel
Used, Avg/Inst Econ, DisToDest, Avg Speed, Time, Trip Time, Dig. Speed*. Audio sync:
*Audio, MP3, CD, AM, FM, Phone*. Telltale/warning text: *NO WARNINGS, HEADLAMPS ON, CHARGE
FAIL, LOW OIL, ENGINE TEMP, TRANS FAIL, Wash Fluid, AIRBAG FAIL, Service, ABS FAULT, BRAKE
FAIL, TRC Off, TCS FAIL, Door, ENGINE FAULT, DSC Off, DSC FAULT, ETC FAULT, FUEL CUT, RSS
FAULT, RSS INACTIVE, TRANS TEMP, EBD FAULT, BULB FAIL, COMMS FAIL, FUEL FAIL, AUTOHL FAIL,
BRAKE FLUID, HDC FAULT, AIRBAG LAMP, IMMOBILISED*. Plus *"Seek Ford dealer"*, *"See owners
manual to…"*, and the rest-timer intervals *2hr/3hr/4hr*.

This vocabulary is the obvious hook for an emulated/virtual cluster and for TES diagnostics
(every warning string maps to a telltale state the cluster derives from CAN).

## 6. Non-volatile store

The 24C16-class I²C EEPROM holds the cluster's persistent settings (illumination colour and
level, overspeed set-point, trip registers, units, the message-centre configuration, and the
service/odometer data with its integrity check). At start-up the firmware reads it (observed:
read of block 1, word 0xFA) and validates it before trusting it. Modelling the EEPROM is what
lets the cluster boot past its NV check in the emulator; its contents behave like any other
checksummed record store.

## 7. Emulation

`fdim-emu` now auto-detects this as **board 2** (a full 0x40000 dump whose own vector table
begins with `jr`) and configures: the 60 KB RAM window at 0x03FF0000, the two CAN channels
(CAN0 = MS @0x3A0.., CAN1 = HS @0x510..), the SJ3 interval-timer bank, the INTAD vector, an
**I²C master + 24C16 EEPROM** model, and the **bit-banged S1D15xxx LCD** on P9/P10/P3.

Run it on the virtual MS-CAN bus alongside the mono FDIM:
```
fdim-emu -module GS_IPC.bin \
         -module 8R29-14D017-AC-app.bin -adcall 430 -eeprom lo_eeprom_ford.bin \
         -ms 12000 -can bus.can -snap 500 -width 180 -xoff 0
```
Then render the message centre:
```
python3 render_ipc.py dual/IPC/final_lcdram.bin ipc_messagecentre.png
```
Boot sequence observed: clock/PLL up → ports & interrupt masks → I²C EEPROM read → both CAN
channels into operate mode (MS NM `0x511`, periodic `0x128`/`0x330`) → message-centre LCD
init and first draw. The mono-FDIM regression frame is byte-identical, so the two-channel/
RAM-window/timer refactor didn't disturb the existing boards.

## 8. Open points (not required for the above)
- Exact gauge-stepper drive mapping (which TPn channel → speedo/tacho/fuel/temp) and the
  telltale-lamp output bits — these need a port-level trace against a live cluster.
- The HS-CAN signal decode (rpm/speed/coolant scaling inside 0x200/0x207/0x340…) needs a
  real powertrain candump to calibrate; firmware gives the IDs and the handler routines, not
  the physical units.
- Precise EEPROM record offsets (settings vs trip vs service) — mappable by watching the I²C
  reads/writes per menu action in the emulator.
- Exact MCU part marking.
