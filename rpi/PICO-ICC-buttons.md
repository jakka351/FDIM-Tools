# FG ICC button panel → Raspberry Pi Pico → RPi‑FDIM

What it takes to drive the factory ICC fascia buttons into `rpi/fdim_pi.py` through a Pico,
and the implementation that ships in `pico/` (CircuitPython `code.py` + `boot.py`).

## 1. What the buttons are, electrically (from the firmware)

The ICC fascia has three groups of switches, and only one of them ever went to the display module:

| Group | Wired to | How the Pi gets them |
|-------|----------|----------------------|
| **Six soft keys around the screen** (the ones the LCD labels: presets 1–6, CD+/TXT/RDM/RPT/EXIT…) | **the FDIM's own MCU** — mono unit reads them on **P50–P55** via the key‑return interrupt (active‑low, internal pull‑ups); colour unit reads its set on P9 external‑interrupt lines **plus a resistor ladder on the ADC** | **the Pico** (this project) |
| Audio hard keys (power/volume knob, source, seek) | the ACU (head unit) | already on MS‑CAN → `can0` |
| Climate keys | the HIM (climate module) | already on MS‑CAN → `can0` |

So once the FDIM is pulled, its six (mono) or ~10 (colour) switch lines are **dry contacts to
ground with nobody biasing them**. That's the key simplification: the Pico supplies the pull‑ups
from its own 3V3, so there is **no 5 V anywhere and no level shifting** — the panel's switches
don't care what voltage the pull‑up is. The colour unit's ladder is ratiometric, so biasing it
from 3V3 instead of 5 V keeps the same fractions.

Panel backlighting is normally fed from the dash dimmer circuit (BCM), not the display module —
check with the FDIM unplugged; if the key LEDs die, they were on the FDIM's rail and need a
12 V→LED supply (or the Pico's PWM) added.

## 2. Why a Pico (and not the Pi's own GPIO)

The Pi's header is 3.3 V‑only, has no hardware debounce or key‑scan engine, and sits on a
powered‑up Linux box you don't want a noisy fascia loom hanging off. The RP2040 gives you:
hardware‑debounced key scanning (`keypad` module), three 12‑bit ADC inputs for ladders, a
PIO quadrature decoder for the volume knob (`rotaryio`), and native **USB HID + USB serial at
the same time** — so the Pi sees a keyboard plus an event stream, with zero driver work.
AU$6–10 of hardware.

## 3. Wiring

```
ICC fascia switch ──┬──[1 kΩ]──┬── Pico GPn   (internal pull-up on; add 10 kΩ to 3V3 for extra margin)
                    │          └──[100 nF]── GND     (EMI / contact-bounce filter, optional)
                    └── common ── GND (Pico GND = chassis ground reference)

Resistor ladder:    3V3 ──[top R, e.g. 10 kΩ]──┬── Pico A0/A1/A2 (GP26/27/28)
                                               └── ladder chain ── GND  (buttons short taps)
Volume encoder:     A → GP14, B → GP15, common → GND   (rotaryio, divisor 4 = one event per detent)
Straps:             GP22 → GND = mirror keymap (open = skin keymap)   GP21 → GND = mapping mode
USB:                Pico micro‑USB → Pi USB (power + HID + serial)
```
Default pin plan in `code.py`: GP2–GP7 = SOFT1–SOFT6 (the FDIM's P50–P55), GP8–GP12 = optional
extra hard keys (SOURCE/MENU/POWER/SEEK_UP/SEEK_DOWN), A0–A2 = up to three ladders, GP14/15 =
encoder, GP21/22 = straps. Edit `DIRECT_KEYS` / `LADDERS` / `ENCODER` to match your harness.

**Finding the harness pinout (mapping mode).** You don't need the Ford pinout: ground GP21,
plug in, open the Pico's data port (`screen /dev/ttyACM1 115200`), and press each fascia key.
The Pico prints `MAP GP4 SOFT3 LOW` for direct inputs and `LADDER0 raw=0.41 idle` for ladder
levels. Label the wire, edit the config, done. The firmware's own pin map tells you which lines
to expect: six direct lines on the mono panel; on the colour panel expect a few direct lines
plus one or two ladders whose idle level is ~1.0 (biased high) and whose pressed levels you
copy straight into `LADDERS`.

## 4. Firmware (`pico/code.py`, CircuitPython)

- **Debounce**: the `keypad.Keys` scanner, 20 ms interval, event queue — no bounce reaches the Pi.
- **Ladders**: 10 ms sampling, median‑of‑3, ±0.03 hysteresis, press/release edges per level.
- **Encoder**: `rotaryio.IncrementalEncoder`, one VOL_UP/VOL_DOWN tap per detent.
- **Auto‑repeat**: held VOL/SEEK/NAV keys repeat after 450 ms at 10 Hz (fdim_pi acts on key‑down).
- **Two keymaps**, selected by `MODE` or the GP22 strap:
  - **skin** (`fdim_pi.py` normal mode): SOFT1–6 → `1`–`6`, SOURCE → `s`, MENU → `m`, POWER →
    `p`, SEEK → ↑/↓, VOL → →/←, NAV_OK → Enter, NAV_BACK → Esc. Matches `HidButtons.emap`
    and the pygame `KEY_ACTIONS` exactly.
  - **mirror** (`fdim_pi.py --identical`): SOFT1–6 → `1`–`6`, which `run_identical` maps to
    **BTN 0–5 → the real firmware's P50–P55**, so the factory code emits its own button reports.
- **Serial event stream** (second USB‑CDC port, enabled in `boot.py`): `BTN n`/`REL n`,
  `KEY NAME DOWN|UP|REPEAT`, `ENC ±1`, plus `MAP`/`LADDER` diagnostics. This path doesn't need
  keyboard focus, so it's the one to use in the car.
- **Self‑test**: `python3 pico/code.py` on a PC runs the same logic against mock hardware.

Install: flash the CircuitPython UF2 (hold BOOTSEL, drop the file), copy `boot.py` and
`code.py` to the `CIRCUITPY` drive, and copy the `adafruit_hid` folder from the Adafruit
CircuitPython library bundle into `CIRCUITPY/lib`. Power‑cycle. `lsusb` on the Pi shows
"TesterPresent FG‑ICC‑Buttons"; `/dev/serial/by-id/*FG-ICC-Buttons*if02` is the event port.

## 5. Pi side (`rpi/fdim_pi.py`)

Nothing to configure for the keyboard path — it's a keyboard. For the serial path:
```bash
# skin mode: Pico events drive the UI + CAN button reports
python3 fdim_pi.py --interface can0 --unit mono --pico auto
# identical (firmware mirror): soft keys go straight into the real firmware's P5 pins
python3 fdim_pi.py --identical --mirror ../emu/fdim-mirror --app 8R29-14D017-AC-app.bin --interface can0 --pico auto
```
`--pico auto` finds the port by USB name. `PICO_KEY_ACTIONS` maps the Pico's logical names to
actions; SOFT1–6 become PRESET1–6, NAV_UP/DOWN double as SEEK. The parser and the mapping are
covered by `fdim_pi.py --selftest`, and the serial transport was verified end‑to‑end over a pty.

## 6. BOM

| Part | Qty | Notes |
|------|-----|-------|
| Raspberry Pi Pico (or Pico 2 / W) | 1 | any RP2040/RP2350 board with USB |
| 1 kΩ series resistors | per switch line | contact protection |
| 10 kΩ pull‑ups to 3V3 | per direct line (optional) | belt‑and‑braces over the internal pull‑ups |
| 100 nF to GND | per line (optional) | EMI / bounce filter |
| 10 kΩ top resistor | per ladder | biases the ladder from 3V3 |
| Connector to the factory ICC loom | 1 | reuse the FDIM's mating connector / pigtail so it stays reversible |
| Micro‑USB lead to the Pi | 1 | |

## 7. Bench test sequence

1. `python3 pico/code.py` on the PC → "PICO FIRMWARE SELF‑TEST PASSED".
2. Flash; strap GP21 low; press each fascia key; fill in `DIRECT_KEYS`/`LADDERS` from the MAP lines.
3. Unstrap; `evtest` on the Pi shows KEY_1…KEY_6 / KEY_S etc. on key‑down.
4. `python3 fdim_pi.py --sim --unit mono --windowed --pico auto` — the skin reacts to the real panel.
5. `--identical … --pico auto` — the factory firmware reacts (watch its button‑report frames on `can0`).

## 8. Caveats / what's not here

- The exact FG harness pin numbers aren't published; mapping mode is the substitute. The
  firmware fixes the *count and type* of lines (six active‑low on the mono panel; direct + ladder
  on the colour panel), not the connector positions.
- The colour panel's ladder resistor values need one pass in mapping mode to populate `LADDERS`.
- If the volume knob turns out to be on the ACU rather than the fascia harness, leave `ENCODER`
  as `None` — volume then arrives on `can0` like the other ACU keys.
- Fascia LED backlighting: verify it survives FDIM removal (see §1).
