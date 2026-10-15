# FG Falcon ICC → Raspberry Pi FDIM Replacement — Full Build Spec

Replace the factory Front Display Interface Module (the centre‑console media/climate
display — **not** the instrument cluster) with a Raspberry Pi + official touchscreen that
reproduces the factory look, UI and on‑bus behaviour. Covers both the **mono** (AC_FDIM,
8R29‑14D017‑AC) and **colour** (8R29‑14D017‑CB) units.

Everything in this spec is derived from the two firmware teardowns in this repo
(`docs/FDIM-AC-mono-analysis.md`, `docs/FDIM-analysis.md`) plus the emulator models. Items
that firmware alone can't tell you (radio text, climate, clock message maps) are flagged in
§9 as "needs a car capture".

---

## 0. Design intent

The factory FDIM is, on the wire, just a node on **MS‑CAN** that:

1. **listens** to the body/infotainment bus (ignition, audio source, audio‑unit‑present,
   iPod/phone state, radio text, climate, clock, illumination…) and draws the matching page,
2. **talks** — sends its own network‑management heartbeat and periodic status so the ACU
   (head unit) and NM master see it present and awake, and forwards its local button presses,
3. reads a **handful of fascia buttons** wired straight to its MCU.

So the Pi replacement has three jobs: **render** the factory pages from CAN, **impersonate**
the FDIM's transmit set so the rest of the car notices nothing, and **capture the buttons**
that used to land on the FDIM's pins. Because you're putting the Pi on `can0`, job 1 and 2
are pure software and job 3 is a small, well‑bounded bit of electronics.

MS‑CAN is the body/comfort/infotainment bus — audio, climate, ICC. This retrofit lives
entirely on it. Nothing here touches HS‑CAN (powertrain) and it shouldn't.

---

## 1. System architecture

```
          ┌────────────────────────── Raspberry Pi 4 / CM4 ──────────────────────────┐
          │                                                                            │
 MS-CAN ──┤  CAN controller ── can0 (SocketCAN, 125 kbit/s)                            │
 (car)    │     │                                                                      │
          │     ├── RX: render pages from bus state (ignition, source, text, climate…) │
          │     └── TX: FDIM impersonation (NM heartbeat + periodic status + buttons)   │
          │                                                                            │
          │  Renderer ──► DSI ──► Official 7" touchscreen (800×480) + capacitive touch  │
          │     ▲                                                                       │
          │     │  touch = primary input                                                │
          │  USB/I²C                                                                    │
          │     │                                                                       │
          └─────┼───────────────────────────────────────────────────────────────────┘
                │
        ┌───────┴────────┐
        │ Front-panel MCU │  RP2040 / TCA8418 — reads factory fascia switches at native
        │  (button buffer)│  voltage, debounces, sends clean events to the Pi, drives
        └───────┬────────┘  volume encoder. (Only needed if you keep the hard buttons.)
                │
        Factory ICC switch harness (active-low keys + volume encoder)

   Power:  12V ── load-dump clamp ── 5V/≥3A buck ── UPS/supercap ── Pi
                                       ▲
                          ignition / CAN-traffic sense → boot & clean-shutdown
```

Two clean input paths, and you can mix them:

- **Touch** on the new panel is the primary UI (the factory never had it; it's a free upgrade).
- **Factory hard buttons** (if you keep the bezel's physical keys) go through a front‑panel
  MCU, never raw onto the Pi — see §3.
- **Steering‑wheel + some fascia switches** are *already* broadcast on MS‑CAN by the ACU/BCM,
  so for those the Pi just listens — **no GPIO at all**.

---

## 2. CAN interface (`can0`)

### 2.1 Physical layer — verify before you buy a transceiver
Both FDIMs configure the aFCAN identically: **125 kbit/s, BRP = 9, BTR = 0x3508**
(mono on controller CAN0 @ 0x03FEC000, colour on CAN1 @ 0x03FEC600 — same bus, different
on‑chip instance). So one `can0` bitrate (`125000`) fits both.

**Gotcha #1 — transceiver class.** A stock MCP2515 CAN HAT ships with a high‑speed
transceiver (TJA1050/MCP2551‑class, 120 Ω split termination). Confirm on the bench whether
your FG MS‑CAN harness is high‑speed at 125 k or **low‑speed fault‑tolerant** (ISO 11898‑3,
TJA1054/TJA1055/TH8056, RTH/RTL termination). A class mismatch will not communicate at all.
You've got the gear (Barra Scan / TP‑CANBridge) to scope it — do that first and pick the
matching transceiver. If LSFT, use an LSFT CAN board or an MCP2515 board with the transceiver
swapped for a TH8056/TJA1055.

**Gotcha #2 — termination.** If the factory FDIM is a terminating node, pulling it may drop
the bus for every other module. After removal, check MS‑CAN still has correct end‑to‑end
termination and add it at the Pi node if the FDIM was carrying it.

**Hardware options for `can0`:**
- Waveshare / Innomaker **MCP2515** HAT (SPI) with the correct transceiver — simplest.
- **CM4 + native CAN** (the SoC has no CAN; still needs an MCP2515 or an external FD controller) —
  cleaner for an embedded build.
- A dedicated MS‑CAN board you already spin for TP‑CANBridge — ideal, you control the PHY.

SocketCAN bring‑up:
```
# /boot/config.txt (MCP2515 @ 16 MHz example — match your crystal)
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
# bring up
sudo ip link set can0 type can bitrate 125000 restart-ms 100
sudo ip link set can0 up
```

### 2.2 What the Pi RECEIVES and renders from
The FDIM draws everything from these IDs. Subscribe and drive the state machine (§6):

| ID | Field | Meaning |
|----|-------|---------|
| **0x406** | byte1 bits0‑4 (mono) / value 4,8 (colour) | **ignition**: RUN = display on; absence > ~1.2 s → display off. byte0 bits0‑1 = 1 is the "ignition‑off event" (save + arm splash) |
| **0x2F2** | byte6 bits0‑2 | **audio source**: 0 off, 1 AUX1/iPod, 2 AUX2, 3 CD‑MP3, 4 CD, 5 radio, 6 phone, 7 blank |
| **0x311** | byte7 bit0 | audio unit present / security OK — clear ⇒ **"Audio Code Error"** page |
| **0x2D4** | byte6 bits1‑3 | iPod link state (from the AIM) → drives the iPod page |
| **0x317** | — | audio security/pairing key traffic |
| colour also RXes | | 0x409, 0x403, 0x360, 0x353, 0x330, 0x30F, 0x30D, 0x30B, 0x309, 0x304, 0x301, 0x2F9, 0x2F5, 0x2F4, 0x2EF, 0x2EC, 0x2E6, 0x2E4, 0x2E1, 0x2D2, 0x2CA, 0x128 |
| masked ranges | | 0x500–0x57F (network management), 0x2C0–0x2FF (NM / diag path) |

The content‑bearing IDs (radio station/frequency text, clock, climate set‑points, phone
caller) are in that RX list but their **byte layouts are not recoverable from firmware** —
they come from the ACU/BCM/climate head. See §9.

### 2.3 What the Pi must TRANSMIT (FDIM impersonation)
If the Pi stays silent, the NM master and ACU will log the FDIM as missing and may mute audio
or throw a "no ICC" fault. Replay the FDIM's transmit set at the factory cadence:

| ID | Period | Payload / purpose |
|----|--------|-------------------|
| **0x55C** | 2 s | network management: `5C 01` → `5C 02` → `5C 04` state progression (keeps the FDIM "alive" in NM) |
| **0x2FC** | 100 ms | byte0 bit0 set once valid traffic seen; byte7 = 0x46 |
| **0x2E2** | 250 ms | status |
| **0x313 / 0x315** | 250 ms | 0x315 = `00 00 00 07 FF 00 00 00` |
| **0x307** | 500 ms | byte3 0x64 → 0xE4 after traffic |

Mono TX set is the same family (0x55C, 0x2E2, 0x2FC, 0x307, 0x313, 0x315). **Button reports**
also go out here — when the user presses a soft‑key (touch or physical), emit the same frame
the FDIM would have, so the ACU changes source/volume exactly as before. Capture the exact
button‑report bytes from a running car (§9) since those are command frames the ACU acts on.

**Wake/sleep.** The FDIM sleeps with the bus and wakes on CAN activity (INTC0WUP). The Pi
can't do that natively — handle it in the power design (§7): boot on ignition/CAN‑traffic,
clean‑shutdown when the bus goes quiet.

---

## 3. Button / input interface — the right way onto the Pi

**Direct answer: don't hang the factory switches on the Pi's GPIO raw.** Use a dedicated
front‑panel microcontroller as a buffer. Reasons:

- Pi GPIO is **3.3 V only and not 5 V tolerant**; the factory switch matrix pulls to the old
  MCU's rail. One wrong pull‑up kills a GPIO (or the Pi).
- The Pi has **no hardware debounce, no key‑scan engine, no interrupt‑per‑pin** budget you'd
  want for a car. Bounce + EMI on an unshielded fascia loom = ghost presses.
- An MCU front‑end also reads the **volume encoder** and does the matrix scan for free, and
  gives you clean USB‑HID or I²C events.

### 3.1 What the firmware tells us the buttons actually are
- **Mono FDIM:** 6 fascia buttons on **P50–P55**, active‑low, read via the **key‑return
  interrupt (INTKR, KRM = 0x3F)** — ISR at 0x33B00 samples P5 into `gp‑0x56BC`. Effectively
  6 direct active‑low keys (press pulls one line low, fires the interrupt). Simple.
- **Colour FDIM:** fascia switches on **P9 external‑interrupt lines** (vectors 0x0D0/0E0/0F0,
  0x450/460/470) plus an **ADC** path (vector 0x2B0) — i.e. some direct digital keys and
  likely a resistor‑ladder group. The "Switch Information" engineering screen
  (strings at 0x4D9CC) confirms the unit reads each switch directly and can display its state.

Exact pin count / which key is which line: **confirm with a multimeter on your harness** —
firmware gives the method and count, the loom gives the pinout.

### 3.2 Recommended front‑panel board
- **MCU:** RP2040 (Pico) or any 5 V‑tolerant MCU you already use (ESP32‑C3/S3 fits your stack).
- **Matrix/keys:** scan the active‑low keys directly; for a resistor‑ladder group use the MCU's
  ADC and threshold it (same trick the colour FDIM uses).
- **Or a purpose IC:** **TCA8418** I²C keypad scanner (up to 8×10, hardware debounce + IRQ) if
  you want zero firmware — wire the matrix to it, read events over I²C with an interrupt line.
- **Volume:** quadrature encoder → MCU counts steps → relative volume events.
- **To the Pi:** present as **USB‑HID** (keyboard/gamepad — zero driver work, events in any
  language) *or* **I²C + a GPIO interrupt** if you want it on the HAT bus. USB‑HID is the
  lowest‑friction choice.
- **Level/protection:** series resistors + TVS on every switch line into the MCU; the MCU, not
  the Pi, faces the loom.

### 3.3 Event flow
```
factory switch → front-panel MCU (debounce) → USB-HID event → Pi app
   → (a) update UI page   AND   (b) TX the matching CAN button-report frame on can0
```
So a physical "SOURCE" press still tells the ACU to change source over CAN, *and* the new UI
follows — identical to factory. Touch presses take the same path minus the HID step.

### 3.4 Buttons you may not need to wire at all
Steering‑wheel controls and some fascia switches are broadcast on MS‑CAN by the ACU/BCM. Sniff
the bus first (§9); anything already on CAN needs **no GPIO** — the Pi just reacts to the frame.
Only the keys unique to the FDIM's own pins need the front‑panel board.

---

## 4. Display spec — MONO unit

### 4.1 Factory panel
- **240 × 80 pixels, 4 grey levels** (2 bpp), Epson **S1D15721** controller, panel mounted
  rotated 180° (firmware sets ADC/COM reverse to cancel it). Plus a 1‑line icon row (page 20).
- Frame buffer 21 pages × 256 cols at 0x03FFC928; visible glass = **columns 16–255 = 240 px**.
- Pixel values 0 = off (dark), 3 = fully lit; greys used for dimmed/inactive soft‑keys.
- Backlight/contrast on PWM outputs (P‑port) + electronic volume (contrast) command `81 xx`,
  also stored in EEPROM.

### 4.2 Replacement panel
- **Official Raspberry Pi 7" DSI touchscreen (800 × 480)** is the default — one cable, one
  known‑good driver, capacitive touch included. For a closer‑to‑factory *bar* shape, a
  800×480 or 1280×400 DSI/HDMI "stretched" LCD cut/masked to the factory aperture also works.
- Render the 240×80 logical screen **upscaled integer ×N** (e.g. ×3 = 720×240 centred) to keep
  the dot‑matrix crispness, or re‑skin at native panel res for a cleaner OEM+ look. Keep the
  factory green/amber monochrome palette so it reads as stock.

### 4.3 Assets you already have
`gfx-lo/` has **187 extracted bitmaps + 4 fonts** pulled straight from the firmware:
- full‑screen frames & splash pages (235×66), the rounded audio‑page frame (0x0BDA4),
- soft‑key labels 40×15 (CD+, TXT, RDM, RPT, EXIT, presets 1–6…), sliders, checkboxes,
  self‑test pages,
- fonts: 10 px, 12 px bold, 20 px, 12 px light (glyph tables decoded),
- splash logos: **Ford oval 0xDCC4, FPV 0xD4E0** (shown on ign‑off → sleep → wake).

Upscale these (nearest‑neighbour for pixel‑true, or trace to vector for a sharp re‑skin).

---

## 5. Display spec — COLOUR unit

### 5.1 Factory panel
- Approx **480 × 234**, **RGB565** (2 bytes/px). Firmware composites backgrounds from
  **240×61 tiles** and places text/sprite objects (field coords reach x≈500, y≈210 with a
  24‑px font).
- Graphics via an on‑board **GDC** with a 32 MB image space + OAM/sprite engine; TFT driven by
  17×16‑bit SPI register writes on CSIB0 (regs 0x20–0x30), VSYNC on INTP0 ≈ 60 Hz.
- **Backlight:** PWM on TMP1 (TO1), 150 Hz, duty from a **761‑entry ambient‑light lookup**
  (0x3FCC0) — i.e. auto‑dimming. Reproduce this on the Pi (ambient sensor or the car's
  illumination/dimmer CAN signal → PWM/DSI brightness).
- **RTOS:** NEC RX850 µITRON — the colour UI is a proper task/page system (106 pages).

### 5.2 Replacement panel
- **Official Pi 7" DSI 800×480** again is the clean default; render the factory 480×234 UI
  letterboxed or scaled, or redraw the pages at 800×480 for a sharper OEM+ result.
- Match the factory colour, font and layout so the re‑skin reads as stock.

### 5.3 Assets you already have
- **"PNET" sprite directory: 2123 × 12‑byte descriptors** (0x46554) — the full colour sprite
  set. Note this CB image ships with an *empty* PNET by default (0x3FFDC); the populated
  artwork lives in the companion resource flash the emulator loads. Extract RGB565 sprites with
  the existing `gdc.c`/`render.py` path.
- **Page → line → field tables** (106 pages, 0x40FFC–0x4549x; page table 0x45AE4) — this *is*
  the UI layout: every page, every text/sprite field and position. Port these straight across
  as your layout definition instead of re‑designing the screens.
- **4 fonts × 96 glyphs**, 2 bpp anti‑aliased, heights 16/24 px (0x4028C).

---

## 6. UI / behaviour parity

Reproduce the factory page state machine so the re‑skin behaves identically:

- **Colour** states `1 → 2 → 4 → 8 → 0x10 → 0x20` (display active) driven by ignition/source;
  `0x100020` = display‑on, `0x200010` = display‑off transition events on the internal bus.
- **Mono** display‑state table: RUN + audio page (state 9); ignition‑off with audio still on
  (state 5) → timeout → Power‑Saver (state 0xD); splash logo (state 7) drawn on
  ign‑off → ≥30 s sleep → wake (Ford 0xDCC4, or FPV 0xD4E0 if the EEPROM FPV bit is set).
- Source pages: AUX1/AUX2, CD, CD‑MP3, AM/FM radio, PHONE, iPod — selected by 0x2F2 byte6.
- "Audio Code Error" when 0x311 byte7 bit0 is clear.
- Startup splash + the 2.5 s logo timer.

### Software stack — the big head start
**You already have the renderer.** The emulator composites these exact mono and colour screens
from CAN state + the extracted assets (`emu/render_lo.py`, `emu/render.py`, `gdc.c`, `lcd.c`,
the page/field tables and `gfx-lo/`/PNET sets). The Pi app is a descendant of that:

```
can0 frames ─► (same state machine) ─► (same asset set) ─► framebuffer/DSI
```

Recommended implementations, easiest → slickest:
1. **Reuse the emulator's compositors** fed live from `python-can` on `can0`, blitting to the
   Linux framebuffer or an SDL window. Fastest path to "it works in the car".
2. **LVGL** (C) or **Qt/QML** app for a smooth, touch‑first OEM+ UI driving DSI directly — best
   long‑term, GPU‑accelerated, clean animations.
3. **Chromium kiosk** rendering an HTML/Canvas re‑skin fed by a small `can0→websocket` bridge —
   quickest to *style*, heaviest at runtime. Good for prototyping the look.

Pick (1) to validate on the bus, migrate the look into (2) for the shipped unit.

---

## 6A. Pixel-identical mode — the firmware mirror

"Looks like the factory" and "**is** the factory" are two different builds. The skins
in §§4–5 are a faithful *re-skin*. To be genuinely pixel-identical there is only one
correct method: **let the factory code draw the screen.** `emu/fdim_mirror.c` runs the
real FDIM firmware in real time, bridges MS-CAN, and streams the LCD framebuffer; the Pi
just paints it. Every pixel — fonts, kerning, frames, the little ⓘ glyph, the splash —
is exactly what the factory module would draw, because it is the factory module's own
code doing the drawing.

```
 can0 <==(SocketCAN)==> [ real FDIM firmware in emulator ] ==(FBM1 stream)==> fdim_pi --identical
                                      ^
                         touch / buttons injected on P5 (INTKR)
```

The firmware also does **all** the bus work itself — network-management heartbeat,
periodic status, and the real button-report frames from its own pins — so this path is
not just pixel-identical, it's **bus-identical**: nothing on MS-CAN can tell the Pi from
a factory FDIM, and there are no byte maps to capture for the TX side.

**Verified:** the mirror runs at ≈ **1.0× real-time** (19.2 s wall for 20 s simulated on
a desktop core; a Pi 5 is comfortable, a Pi 4 may need the HALT fast-path), streams the
LCD framebuffer, and reproduces the real pages exactly — confirmed end to end
(`fdim_pi --identical` painting the live "AUX1 / ⓘ Auxiliary" page from a candump feed).

```bash
# bench (replay a candump), windowed:
python3 fdim_pi.py --identical --mirror ../emu/fdim-mirror \
    --app ../examples/8R29-14D017-AC-app.bin --canfile ../examples/lo_aux.can --windowed
# in the car, on the DSI panel:
python3 fdim_pi.py --identical --mirror ../emu/fdim-mirror \
    --app 8R29-14D017-AC-app.bin --interface can0 --flip
```

`fdim-mirror` options: `-canif can0` (live) or `-canfile f` (bench), `-fb PATH` (default
stdout), `-fbhz N`, `-ctl PATH` (`BTN n`/`REL n`/`CAN id b..`), `-flip -width -xoff -adc`.

### Mono vs colour — the honest split
- **Mono: pixel-identical is available now.** Every bitmap, font and the splash logos
  live inside the app image and are extracted (`gfx-lo/`). The mirror reproduces the
  real 240×80 screen exactly. Done.
- **Colour: blocked on one chip dump.** The colour app image carries only the **PNET
  directory** (2123 sprite *descriptors* — sizes and addresses); the actual **pixels**
  (backgrounds, sprites, the anti-aliased fonts) live in a **separate resource flash**
  on the colour ICC board that was never dumped. Without it, *nothing* — not the mirror,
  not a re-skin, not me — can reproduce the exact colour pixels, because the artwork
  simply isn't in the firmware I have. The emulator confirms this: with no ROM it draws
  placeholder boxes where the sprites would go.

  **To unlock colour-identical, dump the colour FDIM's resource flash** (the second,
  larger memory on the CB board — a serial/parallel flash next to the GDC) and hand it
  over as `-rom`. The directory + the `gdc.c` compositor + `render.py` already know how
  to place every sprite; they just need the pixels. Failing a dump, high-resolution
  straight-on photos of each real screen let me trace the chrome, but a flash dump is the
  clean, exact answer. Until then the colour skin in §5 is the best available and is a
  close OEM+ look, not a pixel clone.

### Which to ship
Run **`--identical` (mirror)** when the goal is "indistinguishable from factory" — it's
the correct answer and needs no CAN reverse-engineering for the chrome or the TX side.
Run the **skins (§§4–5)** when you want a modernised/touch-first look, lower CPU, or
features the factory never had (bigger menus, a gauge page). Both live in one `fdim_pi.py`.

## 7. Power, boot & packaging

Automotive power is where Pi retrofits die. Spec it properly:

- **Input:** 12 V (13.8 V running, cranking dips to ~6–9 V, load‑dump spikes). Use an
  automotive‑rated buck (e.g. a 12→5 V ≥3 A module with input TVS/clamp) — not a bench USB brick.
- **Ignition / wake sense:** boot the Pi on switched‑12 V *or* on MS‑CAN traffic (the FDIM woke
  on CAN activity). A simple ign‑sense GPIO (opto/divider) is the robust trigger.
- **Clean shutdown — mandatory:** a **UPS/supercap HAT** so when ignition drops (or the bus
  sleeps) the Pi gets a shutdown signal and ~10–30 s to journal and halt. Without this, yanking
  power mid‑write corrupts the SD card — the #1 failure mode of car Pis. Use an SSD/eMMC (CM4)
  or an overlay‑FS read‑only root as belt‑and‑braces.
- **Thermal:** closed console, Aussie summer — heatsink + a small fan or a CM4 on an alloy
  carrier. Budget for 70 °C ambient behind the dash.
- **Fit:** the official 7" panel is bigger than the factory aperture — plan the bezel/adapter
  (3D‑printed or laser‑cut) early. Bond the DSI panel behind a custom smoked lens for the OEM
  flush look, or mask it to the factory window shape. Reuse the factory ICC connector via a
  harness adapter so the install is reversible (plug back in the original FDIM any time).

---

## 8. Bill of materials (reference)

| Block | Part | Notes |
|-------|------|-------|
| Compute | Pi 4 (2–4 GB) or **CM4 + carrier** | CM4 for embedded/eMMC, no SD to corrupt |
| Display | **Official Pi 7" DSI touchscreen** (800×480) | or a bar‑shaped 800×480/1280×400 DSI panel |
| CAN | MCP2515 board **with MS‑CAN‑correct transceiver** (verify HS vs LSFT) | or your own TP‑CANBridge PHY |
| Buttons | **RP2040/ESP32 front‑panel board** or TCA8418 I²C scanner | + quadrature encoder for volume |
| Power | automotive 12→5 V ≥3 A buck + **UPS/supercap HAT** + ign‑sense | clean shutdown is non‑negotiable |
| Harness | factory ICC connector adapter | keeps it reversible |
| Enclosure | printed bezel + lens | fit 7" into the factory aperture |

---

## 9. Build phases & what needs a real‑car capture

**Phase 1 — bench (no car).** Pi + CAN board on the **emulator's virtual MS‑CAN** or a
`candump` replay. Prove: `can0` up at 125 k, RX decode of 0x406/0x2F2/0x311/0x2D4, the render
stack drawing the right page, and the TX impersonation (0x55C NM + periodics) looking right.
Everything in §§2–6 is exercisable here today with the assets in this repo.

**Phase 2 — in‑car sniff.** Log MS‑CAN from your running FG (`candump can0 -l`). The firmware
**cannot** give you these — capture and map them:
- radio **station text / frequency**, clock,
- **climate** set‑points / fan / mode,
- the exact **button‑report byte layouts** the ACU acts on (so your TX frames are correct),
- illumination/dimmer signal for auto‑brightness,
- menu contents that come from other modules.

Map those IDs/bytes, feed them into the renderer, and the re‑skin reaches full parity.

**Phase 3 — fab & fit.** Transceiver confirmed, front‑panel board built, power + UPS proven,
bezel printed, harness adapter made. Install is reversible via the factory connector.

**Phase 4 — polish.** Migrate the look from the emulator compositor into LVGL/Qt, add the
touch‑native niceties the factory never had (bigger menus, your own pages, the perf/gauge and
media ideas from the mod list), keep the OEM skin as the default face.

---

## 10. Open items / honest unknowns

- MS‑CAN physical layer (HS vs LSFT) and whether the FDIM terminates the bus — **verify on the
  bench**, both change the hardware.
- Exact fascia button pinout and which line = which key — multimeter the loom.
- The content‑bearing CAN maps (radio/climate/clock/button‑report bytes) — Phase‑2 capture.
- Colour unit's populated PNET artwork lives in the companion resource flash, not the CB app
  image — extract from the resource ROM the emulator loads.
- Exact MCU part markings and a few GDC register semantics (not needed for this build).
```
