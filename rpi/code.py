# code.py — FG Falcon ICC button panel -> Raspberry Pi Pico -> fdim_pi.py
#
# CircuitPython firmware for an RP2040 (Pico / Pico W / Pico 2). The Pico sits between the
# factory ICC fascia switches and the Pi running fdim_pi.py, and presents the buttons as:
#
#   1. a USB HID keyboard   -> fdim_pi.py's HidButtons / pygame keymaps work with NO changes
#                              (skin mode: s/m/p/arrows/1..6 ; mirror mode: 1..6 -> BTN 0..5)
#   2. a USB serial channel -> "BTN n" / "REL n" / "KEY NAME DOWN|UP" / "ENC +1" lines
#                              (fdim_pi.py --pico /dev/ttyACM1 ; also the mapping tool)
#
# Why a Pico and not raw Pi GPIO: the Pi's pins are 3.3 V-only with no debounce or scan
# engine; this board faces the loom, supplies the switch bias, debounces in hardware
# (keypad module), reads resistor ladders on its ADC, decodes the volume encoder on PIO,
# and hands the Pi clean events. See docs/PICO-ICC-buttons.md for wiring and setup.
#
# Inputs supported (all optional except the six soft keys):
#   * DIRECT keys   : dry contacts to GND, one GPIO each (the mono FDIM read six of these on
#                     P50..P55 - the soft keys around the display). Internal pull-ups.
#   * LADDER groups : several buttons on one resistor ladder (the colour FDIM used these),
#                     read on A0..A2 as a fraction of 3V3 with hysteresis.
#   * ENCODER       : the volume knob (quadrature A/B) via rotaryio (PIO).
#   * MODE strap    : GP22 to GND = mirror keymap, open = skin keymap (or set MODE below).
#   * MAP strap     : GP21 to GND = mapping mode: every input change is printed on the
#                     serial channel so you can label the harness pin by pin.
#
# Install: flash CircuitPython UF2, copy boot.py + code.py to CIRCUITPY, copy the
# adafruit_hid library folder into CIRCUITPY/lib. That's it.

import time

try:
    import board, digitalio, keypad, analogio, usb_hid, usb_cdc
    from adafruit_hid.keyboard import Keyboard
    from adafruit_hid.keycode import Keycode
    try:
        import rotaryio
    except ImportError:
        rotaryio = None
    SIM = False
except ImportError:
    SIM = True          # running under CPython: use the mocks at the bottom for a self-test

# ------------------------------------------------------------------ CONFIG ----------------
MODE = "auto"           # "skin" | "mirror" | "auto" (auto = read the GP22 strap)
DEBOUNCE_S = 0.02       # keypad hardware debounce interval
REPEAT_DELAY_S = 0.45   # auto-repeat for held VOL/SEEK keys
REPEAT_RATE_S = 0.10
LADDER_HYST = 0.03      # +/- fraction of 3V3 around each level
LADDER_PERIOD_S = 0.01

# Direct active-low keys: (pin name, logical name). SOFT1..SOFT6 = the FDIM's P50..P55.
# Edit the pin names to match your harness (use mapping mode to find them).
DIRECT_KEYS = [
    ("GP2", "SOFT1"), ("GP3", "SOFT2"), ("GP4", "SOFT3"),
    ("GP5", "SOFT4"), ("GP6", "SOFT5"), ("GP7", "SOFT6"),
    # optional extra hard keys if your panel wires them to the display module:
    ("GP8", "SOURCE"), ("GP9", "MENU"), ("GP10", "POWER"),
    ("GP11", "SEEK_UP"), ("GP12", "SEEK_DOWN"),
]
# Resistor ladders: pin + list of (fraction_of_3V3_when_pressed, name). Empty = none.
# Fill from mapping mode ("LADDER0 raw=0.41") - the idle (nothing pressed) level is ~1.0
# when the ladder is biased through a top resistor to 3V3, or ~0 if biased to GND.
LADDERS = [
    # {"pin": "A0", "idle": 1.0, "levels": [(0.00, "NAV_OK"), (0.21, "NAV_UP"), (0.38, "NAV_DOWN"), (0.52, "NAV_BACK")]},
]
ENCODER = ("GP14", "GP15")   # volume knob A/B, or None
ENCODER_DIVISOR = 4          # detents per 4 quadrature steps on most knobs
MODE_STRAP = "GP22"
MAP_STRAP = "GP21"

# logical name -> HID keycode name, per mode. fdim_pi.py: skin mode acts on
#   s/m/p/up/down/left/right/1..6 ; mirror mode (--identical) maps 1..6 -> BTN 0..5
SKIN_MAP = {
    "SOFT1": "ONE", "SOFT2": "TWO", "SOFT3": "THREE", "SOFT4": "FOUR", "SOFT5": "FIVE", "SOFT6": "SIX",
    "SOURCE": "S", "MENU": "M", "POWER": "P",
    "SEEK_UP": "UP_ARROW", "SEEK_DOWN": "DOWN_ARROW",
    "VOL_UP": "RIGHT_ARROW", "VOL_DOWN": "LEFT_ARROW",
    "NAV_OK": "RETURN", "NAV_UP": "UP_ARROW", "NAV_DOWN": "DOWN_ARROW", "NAV_BACK": "ESCAPE",
}
MIRROR_MAP = {
    "SOFT1": "ONE", "SOFT2": "TWO", "SOFT3": "THREE", "SOFT4": "FOUR", "SOFT5": "FIVE", "SOFT6": "SIX",
    "VOL_UP": "RIGHT_ARROW", "VOL_DOWN": "LEFT_ARROW",
}
SOFT_INDEX = {"SOFT1": 0, "SOFT2": 1, "SOFT3": 2, "SOFT4": 3, "SOFT5": 4, "SOFT6": 5}   # BTN n = P5 bit n
REPEAT_KEYS = {"VOL_UP", "VOL_DOWN", "SEEK_UP", "SEEK_DOWN", "NAV_UP", "NAV_DOWN"}

# ------------------------------------------------------------------ hardware glue ---------
class Panel:
    """Owns the inputs and the two outputs (HID keyboard + serial)."""

    def __init__(self, hw):
        self.hw = hw
        self.mode = self._resolve_mode()
        self.keymap = MIRROR_MAP if self.mode == "mirror" else SKIN_MAP
        self.mapping = hw.strap_low(MAP_STRAP)
        self.keys = hw.make_keys([p for p, _ in DIRECT_KEYS])
        self.names = [n for _, n in DIRECT_KEYS]
        self.ladders = [hw.make_ladder(l["pin"]) for l in LADDERS]
        self.ladder_state = [None] * len(LADDERS)
        self.ladder_t = 0.0
        self.enc = hw.make_encoder(*ENCODER) if ENCODER else None
        self.enc_pos = 0
        self.held = {}      # name -> next repeat time
        self.kbd = hw.keyboard()
        self.ser = hw.serial()
        self.say("HELLO fg-icc-buttons mode=%s mapping=%d" % (self.mode, 1 if self.mapping else 0))

    def _resolve_mode(self):
        if MODE in ("skin", "mirror"):
            return MODE
        return "mirror" if self.hw.strap_low(MODE_STRAP) else "skin"

    # --- outputs -------------------------------------------------------------------
    def say(self, line):
        if self.ser:
            try:
                self.ser.write((line + "\n").encode())
            except Exception:
                pass

    def tap(self, keyname):
        code = self.hw.keycode(keyname)
        if code is None or self.kbd is None:
            return
        try:
            self.kbd.press(code); self.kbd.release(code)
        except Exception:
            pass

    def emit(self, name, pressed, now):
        """A logical button changed state."""
        if name in SOFT_INDEX:
            self.say("%s %d" % ("BTN" if pressed else "REL", SOFT_INDEX[name]))
        self.say("KEY %s %s" % (name, "DOWN" if pressed else "UP"))
        keyname = self.keymap.get(name)
        if pressed:
            if keyname:
                self.tap(keyname)
            if name in REPEAT_KEYS:
                self.held[name] = now + REPEAT_DELAY_S
        else:
            self.held.pop(name, None)

    # --- inputs --------------------------------------------------------------------
    def poll(self, now):
        # direct keys (hardware-debounced event queue)
        ev = self.keys.events.get()
        while ev:
            name = self.names[ev.key_number]
            if self.mapping:
                self.say("MAP %s %s %s" % (DIRECT_KEYS[ev.key_number][0], name, "LOW" if ev.pressed else "HIGH"))
            self.emit(name, ev.pressed, now)
            ev = self.keys.events.get()
        # resistor ladders
        if self.ladders and now - self.ladder_t >= LADDER_PERIOD_S:
            self.ladder_t = now
            for i, lad in enumerate(self.ladders):
                frac = lad.read()
                cfg = LADDERS[i]
                hit = None
                for level, name in cfg["levels"]:
                    if abs(frac - level) <= LADDER_HYST:
                        hit = name; break
                if self.mapping and hit != self.ladder_state[i]:
                    self.say("LADDER%d raw=%.2f %s" % (i, frac, hit or "idle"))
                if hit != self.ladder_state[i]:
                    if self.ladder_state[i]:
                        self.emit(self.ladder_state[i], False, now)
                    if hit:
                        self.emit(hit, True, now)
                    self.ladder_state[i] = hit
        # encoder
        if self.enc is not None:
            p = self.enc.position
            while p > self.enc_pos:
                self.enc_pos += 1; self.say("ENC +1"); self.tap(self.keymap.get("VOL_UP"))
            while p < self.enc_pos:
                self.enc_pos -= 1; self.say("ENC -1"); self.tap(self.keymap.get("VOL_DOWN"))
        # auto-repeat
        for name, t in list(self.held.items()):
            if now >= t:
                self.tap(self.keymap.get(name))
                self.say("KEY %s REPEAT" % name)
                self.held[name] = now + REPEAT_RATE_S


class RealHW:
    """CircuitPython bindings."""
    def pin(self, name):
        return getattr(board, name)
    def strap_low(self, name):
        d = digitalio.DigitalInOut(self.pin(name)); d.switch_to_input(pull=digitalio.Pull.UP)
        v = not d.value; d.deinit(); return v
    def make_keys(self, pins):
        return keypad.Keys([self.pin(p) for p in pins], value_when_pressed=False, pull=True, interval=DEBOUNCE_S)
    def make_ladder(self, pin):
        a = analogio.AnalogIn(self.pin(pin))
        class L:
            def read(_):
                s = sorted(a.value for _ in range(3))      # median of 3
                return s[1] / 65535.0
        return L()
    def make_encoder(self, a, b):
        if rotaryio is None:
            return None
        return rotaryio.IncrementalEncoder(self.pin(a), self.pin(b), divisor=ENCODER_DIVISOR)
    def keyboard(self):
        try:
            return Keyboard(usb_hid.devices)
        except Exception:
            return None
    def serial(self):
        return usb_cdc.data if usb_cdc.data else None
    def keycode(self, name):
        return getattr(Keycode, name, None) if name else None


def main(hw, iterations=None):
    panel = Panel(hw)
    n = 0
    while iterations is None or n < iterations:
        panel.poll(time.monotonic())
        time.sleep(0.002)
        n += 1
    return panel


# ------------------------------------------------------------------ CPython self-test ----
if SIM:
    class _Ev:
        def __init__(self, k, p): self.key_number, self.pressed = k, p
    class _Events:
        def __init__(self, script): self.q = list(script)
        def get(self): return self.q.pop(0) if self.q else None
    class _Keys:
        def __init__(self, script): self.events = _Events(script)
    class _Lad:
        def __init__(self, vals): self.vals = list(vals)
        def read(self): return self.vals.pop(0) if len(self.vals) > 1 else self.vals[0]
    class _Enc:
        def __init__(self): self.position = 0
    class _Kbd:
        def __init__(self): self.taps = []
        def press(self, c): self.taps.append(c)
        def release(self, c): pass
    class _Ser:
        def __init__(self): self.lines = []
        def write(self, b): self.lines.append(b.decode().strip())

    class SimHW:
        def __init__(self, key_script, ladder_vals, mode_low=False, map_low=False):
            self.key_script, self.ladder_vals, self.mode_low, self.map_low = key_script, ladder_vals, mode_low, map_low
            self.kbd, self.ser, self.enc = _Kbd(), _Ser(), _Enc()
        def strap_low(self, name): return self.mode_low if name == MODE_STRAP else self.map_low
        def make_keys(self, pins): return _Keys(self.key_script)
        def make_ladder(self, pin): return _Lad(self.ladder_vals)
        def make_encoder(self, a, b): return self.enc
        def keyboard(self): return self.kbd
        def serial(self): return self.ser
        def keycode(self, name): return name

    def selftest():
        global LADDERS
        # skin mode: SOFT1 tap, SOURCE tap, VOL_UP held (repeat), encoder +2/-1, ladder NAV_UP
        LADDERS = [{"pin": "A0", "idle": 1.0, "levels": [(0.21, "NAV_UP"), (0.38, "NAV_DOWN")]}]
        hw = SimHW(key_script=[_Ev(0, True), _Ev(0, False), _Ev(6, True), _Ev(6, False), _Ev(9, True)],
                   ladder_vals=[1.0, 1.0, 0.22, 0.22, 0.22, 1.0])
        p = Panel(hw)
        t = 100.0
        for i in range(12):
            p.poll(t); t += 0.1
            if i == 2: hw.enc.position = 2
            if i == 4: hw.enc.position = 1
        taps = hw.kbd.taps
        assert "ONE" in taps and "S" in taps and "UP_ARROW" in taps, taps
        assert taps.count("RIGHT_ARROW") == 2, taps               # encoder +2 detents
        assert taps.count("UP_ARROW") >= 3, taps                  # SEEK_UP held -> auto-repeat
        assert "LEFT_ARROW" in taps, taps                         # encoder -1
        assert any(l == "BTN 0" for l in hw.ser.lines) and any(l == "REL 0" for l in hw.ser.lines), hw.ser.lines
        assert any(l == "KEY SOURCE DOWN" for l in hw.ser.lines), hw.ser.lines
        assert any(l.startswith("KEY NAV_UP DOWN") for l in hw.ser.lines) and any(l.startswith("KEY NAV_UP UP") for l in hw.ser.lines), hw.ser.lines
        print("  skin mode: taps=%s" % taps)
        # mirror mode: SOFT3 -> THREE + "BTN 2"; SOURCE has no HID key but still a serial KEY line
        hw2 = SimHW(key_script=[_Ev(2, True), _Ev(2, False), _Ev(6, True)], ladder_vals=[1.0], mode_low=True, map_low=True)
        p2 = Panel(hw2)
        for i in range(3): p2.poll(200.0 + 0.1 * i)
        assert p2.mode == "mirror" and "THREE" in hw2.kbd.taps and "S" not in hw2.kbd.taps, hw2.kbd.taps
        assert "BTN 2" in hw2.ser.lines and "REL 2" in hw2.ser.lines, hw2.ser.lines
        assert any(l.startswith("MAP GP4 SOFT3 LOW") for l in hw2.ser.lines), hw2.ser.lines   # mapping mode output
        print("  mirror+mapping mode: taps=%s lines=%s" % (hw2.kbd.taps, hw2.ser.lines[:4]))
        print("PICO FIRMWARE SELF-TEST PASSED")

    if __name__ == "__main__":
        selftest()
else:
    main(RealHW())
