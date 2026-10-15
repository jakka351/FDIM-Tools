# FG ICC fascia → Raspberry Pi Pico → RPi‑FDIM

How the factory FDIM read the Interior Command Centre fascia, what it put on the bus for every
button, and the Pico board + firmware (Arduino **and** CircuitPython) that does the same job for
`rpi/fdim_pi.py`. Everything here comes out of the mono FDIM firmware (8R29‑14D017‑AC) and was
checked by running that firmware in `emu/fdim-mirror` and watching what it transmits; the bit
assignments also agree with the FG DBC and the car‑proven `ICC_CAN_Interface.ino` in the
FG‑Falcon repo.

## 1. What the FDIM actually reads (corrected)

The FDIM is the only module inside the ICC assembly with a CAN connection, so **it scans every
fascia switch** — audio, climate and body keys included — and the ACM, HIM and BEM only ever see
the resulting bits in **0x2FC** and **0x307**. The earlier "six soft keys only" description in
this repo was wrong. The real inventory (all 33 keys of the workshop‑manual list, plus the knob
and the cabin temperature sensor):

| Input | FDIM pins | Keys |
|---|---|---|
| **Key matrix 5 × 8** | rows **PCS0–PCS4** (driven one at a time), columns **PCT0–PCT7** (read, LOW = pressed), scanned every 5 ms, majority‑of‑3 debounce | 27 keys — all the climate keys, DSC, the audio keys, OK, BACK/HOME and the six piano keys |
| **Direct lines** | **P50–P55**, active‑low, key‑return interrupt (these wake the module with the ignition off) | HAZARD, MENU, EJECT, DOOR LOCK, DOOR UNLOCK, DOME LAMP |
| **Rotary encoder** | **P612 / P613** (P6H bits 4/5), quadrature | volume / menu knob |
| **Cabin temperature sensor** | **ANI0** (ADC ch0) through a 251‑step table at 0x536C | NTC on the fascia PCB → 0x313 byte 0 |
| (not buttons) | ANI1 = supply voltage, ANI4 = second NTC‑type input (LCD compensation), ANI5 = 12‑step illumination input | — |

### 1.1 The matrix (row = PCS line, column = PCT line)

| row\col | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| **0** | CLIMATE_OFF | BLOWER_DOWN | MODE | DSC | LOAD | – | MFB4 | MFB3 |
| **1** | RECIRC | BLOWER_UP | FRONT_DEMIST | SCAN | OK | – | MFB5 | MFB2 |
| **2** | AC | – | TEMP_DOWN | CD_AUX | AUDIO_ONOFF | – | MFB6 | MFB1 |
| **3** | AUTO | – | TEMP_UP | FM_AM | SEEK_DOWN | – | – | – |
| **4** | – | – | REAR_DEMIST | BACK_HOME | SEEK_UP | – | – | – |

MFB1–MFB6 are the six "piano keys" under the screen (presets 1–6, soft keys in menus). The
firmware's own switch‑information table names them exactly like this (`MFB1`…`MFB6`,
`AudioON/OFF`, `BACK/HOME`, `SCAN/AS`, `FRDEMIST`, `D‑TEMP UP`…); the names here are those with
the spaces removed.

### 1.2 The direct lines

| P5 bit | key | wake | CAN |
|---|---|---|---|
| 0 | HAZARD | yes | 0x307 byte2 bit0 |
| 1 | MENU | yes | 0x2FC byte0 bit0 (**pulsed** for one frame per press; hold 3 s = screen off) |
| 2 | EJECT | yes | 0x2FC byte1 bit7 |
| 3 | DOOR LOCK | yes | 0x307 byte3 bit6 |
| 4 | DOOR UNLOCK | yes | 0x307 byte3 bit2 |
| 5 | DOME LAMP | yes | 0x307 byte3 bit5 |

### 1.3 What goes on the bus — the full map

**0x2FC** (every 100 ms, plus immediately on change). Idle payload on the car:
`00 00 01 00 pp 00 02 04` (byte2 bit0 is a constant 1, `pp` = display page id, bytes 5–7 status
fields).

| byte | bit | key |
|---|---|---|
| 0 | 0 | MENU (pulse) |
| 0 | 2 | SEEK_UP |
| 0 | 3 | SEEK_DOWN |
| 0 | 4 | AUDIO_ONOFF |
| 0 | 5 | FM_AM |
| 0 | 6 | SCAN / AS (the firmware moves this to bit 1 when a settings flag is set) |
| 0 | 7 | CD_AUX |
| 1 | 0–5 | MFB1 … MFB6 (presets / piano keys) |
| 1 | 6 | LOAD |
| 1 | 7 | EJECT |
| 2 | 5 | OK |
| 3 | 6–7 / 0–5 | encoder: `01` = right (volume up), `10` = left; low 6 bits = detents — `0x41` / `0x81` for one detent, present in exactly one frame |
| 4 | – | display page id (status, not a key) |

**0x307** (every 500 ms, plus immediately on change). Idle: `00 00 00 80 00 00 00 00` (byte3 bit7
= module awake).

| byte | bit | key | DBC name |
|---|---|---|---|
| 0 | 1 | FRONT_DEMIST | Screen_Switch |
| 0 | 5 | REAR_DEMIST | – |
| 0 | 6 | RECIRC | Recirc_Switch |
| 0 | 7 | AC | AC_Switch |
| 1 | 2 | BLOWER_DOWN | Fan_Spd_Down_Switch |
| 1 | 3 | BLOWER_UP | Fan_Spd_Up_Switch |
| 1 | 4 | CLIMATE_OFF | Climate_Ctrl_Off_Switch |
| 1 | 5 | AUTO | Auto_ACC_On_Switch |
| 1 | 7 | MODE (vents) | Air_Dist_Up_Switch |
| 2 | 0 | HAZARD | – |
| 2 | 6 | TEMP_DOWN | Driver_Temp_Down_Switch |
| 2 | 7 | TEMP_UP | Driver_Temp_Up_Switch |
| 3 | 2 | DOOR UNLOCK | – |
| 3 | 4 | DSC | Traction_VDC_Switch |
| 3 | 5 | DOME LAMP | – |
| 3 | 6 | DOOR LOCK | – |

The DBC's extra bits (byte0 bits 0/2/3/4 individual vent modes, byte1 bits 0/1 passenger temp,
byte1 bit6 air‑dist down, byte3 bit3 diff lock) have **no key on the mono fascia** — the firmware
never sets them. Keys with no bus effect: **BACK/HOME** (display only: back one level, hold 1.5 s
for home).

**0x313** (250 ms): byte 0 = cabin temperature code, **code ÷ 6 = °C** (0x80 → 21.3 °C; the HIM
uses it for automatic climate control). The FDIM derives it from the fascia thermistor through
the table in `docs/cts_table.csv` (ADC count ↔ code); the Pico computes it from its own NTC
reading.

`rpi/fdim_pi.py` carries all of this as `ICC_KEYS` / `ICC_INPUTS` and `FasciaState`, and emits
exactly these frames; `emu/fdim_mirror.c` has the same table (`KEY NAME 1|0`) for the
firmware‑mirror mode.

### 1.4 How it was verified

```
emu/fdim-mirror examples/8R29-14D017-AC-app.bin -canfile examples/ipc_bus.can -fast -frames 450 \
    -fb /dev/null -ctl keys.ctl -txlog tx.log
```
with a control script pressing keys by name (`KEY SEEK_UP 1`, `KEY HAZARD 1`, `ENC +1`, …) gives,
in `tx.log`, `2FC 04 …` for SEEK_UP, `307 80 …` for AC, `307 00 00 01 80` for HAZARD,
`2FC 01 …` for one frame on MENU, `2FC … 41 …` / `… 81 …` for the encoder, `2FC 00 01 …` for
MFB1 and `307 00 00 00 C0` for LOCK — the real firmware, the real bits.

## 2. The Pico board

### 2.1 Wiring (GP numbers; the same in both firmwares)

```
MATRIX ROWS   (FDIM PCS0..4)   GP2  GP3  GP4  GP5  GP6         driven LOW one at a time, else hi-Z
MATRIX COLS   (FDIM PCT0..7)   GP7  GP8  GP9  GP10 GP11 GP12 GP13 GP14   inputs, pull-up, LOW = pressed
DIRECT KEYS   (FDIM P50..P55)  GP15 HAZARD  GP16 MENU  GP17 EJECT  GP18 LOCK  GP19 UNLOCK  GP20 DOME_LAMP
ENCODER       (FDIM P612/613)  GP21 = A   GP22 = B   common -> GND
CTS NTC       (FDIM ANI0)      3V3 --[10 kΩ]--+-- GP26/A0 ;  NTC from that node to GND
ILLUMINATION                   GP27 PWM -> NPN/MOSFET low-side driver -> fascia LED supply
USB                            Pico micro-USB -> Pi (power + serial + HID)
```

The fascia switches are dry contacts: a matrix key shorts its row line to its column line, a
direct key shorts its line to ground. The FDIM biased them from its own rail through pull‑ups; the
Pico does the same from 3V3, so there is no 5 V anywhere and nothing to level‑shift. Optional
per‑line 1 kΩ series resistors + 100 nF to ground are cheap EMI insurance. If you keep the FDIM's
row/column order when you wire the fascia connector, the key names come out right by themselves;
the FDIM board is the pinout reference — buzz each fascia‑connector pin to the V850ES/FJ3's
PCS0–4 / PCT0–7 / P50–55 / P612–613 / ANI0 pins (144‑pin part; the Renesas pin table gives the
numbers).

Button illumination: the fascia LEDs were driven from the FDIM. GP27 PWM + a transistor
reproduces that; the Pi sets the level with `DIM n` (it can follow the dash dimming it sees on
the bus).

### 2.2 Firmware — Arduino IDE (`pico/fg_icc_buttons/fg_icc_buttons.ino`)

Board package: **Raspberry Pi Pico/RP2040/RP2350 by Earle Philhower** (Boards Manager URL
`https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`).
Tools → Board → Raspberry Pi Pico (or Pico W / Pico 2). Either USB stack works (default "Pico
SDK" gives serial + the HID keyboard; "Adafruit TinyUSB" also builds). Hold BOOTSEL the first
time, Upload. No libraries to install. The sketch compiles clean for Pico and Pico 2.

What it does: 5 ms matrix scan with the FDIM's majority‑of‑3 debounce, six direct inputs, the
FDIM's own encoder algorithm (A edge with B steady = right, one count per detent — so the
counts per turn match the factory module), NTC → temperature → `CTS` code every second, PWM
illumination, and an optional HID keyboard for `fdim_pi.py`'s bench shortcuts (`USE_HID`).

### 2.3 Firmware — CircuitPython (`pico/code.py` + `pico/boot.py`)

Flash the CircuitPython UF2, copy `boot.py` and `code.py` to `CIRCUITPY`, copy the
`adafruit_hid` folder from the library bundle into `CIRCUITPY/lib`. `boot.py` enables the second
USB serial port (the event channel) and names the device "TesterPresent FG‑ICC‑Buttons" so the
Pi finds it under `/dev/serial/by-id/`. The matrix is scanned by `keypad.KeyMatrix` (in C,
debounced), direct keys by `keypad.Keys`. `python3 pico/code.py` on a PC runs the logic self‑test.

### 2.4 Serial protocol (115200 8N1, one line per message)

| direction | line | meaning |
|---|---|---|
| Pico → Pi | `HELLO fg-icc-buttons 2 keys=33` | at start |
| | `KEY <NAME> DOWN` / `UP` | any key; unnamed matrix positions come as `R3C5` |
| | `ENC +1` / `ENC -1` | encoder detent right / left |
| | `CTS <code>` | cabin temperature code (code/6 °C), on change, ≤ 1/s |
| | `MAT r c DOWN|UP`, `DIRECT n DOWN|UP` | mapping mode only: raw positions |
| Pi → Pico | `MAP 1|0` | raw mapping output on/off |
| | `DIM 0‑255` | illumination PWM |
| | `HID 1|0` | keyboard output on/off |
| | `INFO`, `PING` | key table / `PONG` |

Older `code.py` lines (`BTN n` / `REL n`) are still accepted by `fdim_pi.py` as the direct keys.

## 3. Pi side (`rpi/fdim_pi.py`)

```bash
# skin mode: Pico events -> local UI + the factory frames on can0
python3 fdim_pi.py --interface can0 --unit mono --pico auto
# identical (firmware mirror): every key goes into the real firmware's pins; it does the bus itself
python3 fdim_pi.py --identical --mirror ../emu/fdim-mirror --app ../examples/8R29-14D017-AC-app.bin \
                   --interface can0 --pico auto [--txlog /tmp/fdim-tx.log]
```

Skin mode: `FasciaState` holds the pressed keys, pulses MENU, packs encoder detents into 0x2FC
byte 3 and the CTS code into 0x313, and the TX thread sends the periodic frames **plus an
immediate frame on every change**, like the firmware. Touch soft keys and the bench keyboard go
through the same model (a tap holds the key 80 ms), so they produce the same bits as the real
button.

Identical mode: `KEY NAME 1|0` → the firmware's P5 line or matrix position, `ENC ±1` → the
P612/P613 quadrature sequence, `CTS n` → ADC ch0 through the firmware's own table.

## 4. Bench test sequence

1. `python3 pico/code.py` → `PICO FIRMWARE SELF-TEST PASSED`; `python3 rpi/fdim_pi.py --selftest` → all OK.
2. Flash; `screen /dev/ttyACM1 115200`; press every fascia key: each prints `KEY <NAME> DOWN/UP`.
   Anything printing `KEY R?C? …` is a position the FDIM did not use → check the wiring order
   (`MAP 1` shows raw row/column).
3. Turn the knob: `ENC +1` per detent clockwise. Warm the fascia: `CTS` rises.
4. `python3 fdim_pi.py --sim --unit mono --windowed --pico auto` — the skin reacts, the log shows
   the 0x2FC/0x307 payloads per key.
5. In the car with `candump can0`: each key toggles exactly the bit in §1.3; the ACM/HIM/BEM react.

## 5. BOM

| Part | Qty | Notes |
|---|---|---|
| Raspberry Pi Pico / Pico W / Pico 2 | 1 | |
| 10 kΩ resistor | 1 | CTS bias |
| 1 kΩ series + 100 nF per switch line | optional | EMI / contact protection |
| NPN or logic MOSFET + base/gate resistor | 1 | fascia illumination |
| Mating connector / pigtail for the ICC fascia loom | 1 | keeps it reversible |
| Micro‑USB lead to the Pi | 1 | |

## 6. Caveats

- The row/column **order** on the fascia connector is not published; the FDIM board is the
  reference (§2.1). Mapping mode makes any order workable in minutes.
- SCAN/AS is sent on 0x2FC byte0 bit6 normally and bit1 when a settings flag is set in the
  FDIM's EEPROM — `fdim_pi.py` sends bit 6.
- The CTS thermistor's R25/β are not in the firmware (only the FDIM's conditioned ADC table, see
  `docs/cts_table.csv`); measure yours at room temperature and set `CTS_R25` / `CTS_BETA`, or
  trim `CTS_OFFSET` against the climate display.
- The colour unit's fascia has more keys (passenger temperature, individual vent modes); its
  firmware has the same signal layer, but its matrix order was not derived here.
