# code.py — FG Falcon ICC fascia buttons -> Raspberry Pi Pico -> fdim_pi.py   (CircuitPython version)
#
# Same job, protocol and pin plan as the Arduino sketch in pico/fg_icc_buttons/ — pick whichever toolchain you
# prefer. The Pico sits between the factory ICC front fascia and the Pi running rpi/fdim_pi.py and reads the
# fascia exactly the way the FDIM did (from the 8R29-14D017-AC firmware teardown):
#
#   * 5 x 8 KEY MATRIX  — rows = FDIM PCS0..4, columns = FDIM PCT0..7 (27 keys used), 5 ms scan, debounced
#   * 6 DIRECT KEYS     — FDIM P50..P55: HAZARD MENU EJECT LOCK UNLOCK DOME_LAMP (switch to ground)
#   * ROTARY ENCODER    — volume / menu knob (FDIM P612/P613), one event per detent, FDIM's own algorithm
#   * CABIN TEMP SENSOR — the NTC on the fascia PCB (FDIM ANI0) -> "CTS code" (code/6 = degC) for 0x313
#   * ILLUMINATION      — PWM for the fascia LEDs, set by the Pi ("DIM n")
#
# and presents them as a USB serial event stream (second CDC port enabled in boot.py) plus an optional HID keyboard.
#
# Serial protocol (115200, one line per message):
#   -> Pi : HELLO fg-icc-buttons 2 keys=33 | KEY <NAME> DOWN|UP | ENC +1|-1 | CTS <code> |
#           MAT r c DOWN|UP / DIRECT n DOWN|UP (mapping mode) | PONG / OK / INFO ...
#   <- Pi : MAP 1|0 | DIM 0-255 | HID 1|0 | INFO | PING
#
# Install: flash the CircuitPython UF2 (hold BOOTSEL), copy boot.py + code.py to CIRCUITPY, copy the
# adafruit_hid folder from the library bundle into CIRCUITPY/lib. `python3 code.py` on a PC runs a self-test.

import time
import math

try:
    import board, digitalio, keypad, analogio, pwmio, usb_hid, usb_cdc
    from adafruit_hid.keyboard import Keyboard
    from adafruit_hid.keycode import Keycode
    SIM = False
except ImportError:
    SIM = True          # running under CPython: mocks at the bottom, self-test

# ------------------------------------------------------------------ CONFIG ----------------
USE_HID = True            # also act as a USB keyboard (skin-mode shortcuts); serial events are always sent
SCAN_S = 0.005            # matrix / direct sample interval (FDIM: 5 ms)
CTS_PERIOD_S = 1.0        # cabin temperature report interval
ENC_PERIOD_S = 0.0005     # encoder poll

ROW_PINS = ("GP2", "GP3", "GP4", "GP5", "GP6")                               # FDIM PCS0..PCS4
COL_PINS = ("GP7", "GP8", "GP9", "GP10", "GP11", "GP12", "GP13", "GP14")     # FDIM PCT0..PCT7
DIRECT_PINS = ("GP15", "GP16", "GP17", "GP18", "GP19", "GP20")               # FDIM P50..P55
ENC_PINS = ("GP21", "GP22")                                                  # A (P612), B (P613)
CTS_PIN = "A0"                                                               # GP26, FDIM ANI0
ILLUM_PIN = "GP27"                                                           # PWM to the LED driver

# CTS thermistor model: 3V3 --[R_BIAS]--+-- A0 ; NTC from the node to GND
CTS_R_BIAS, CTS_R25, CTS_BETA, CTS_OFFSET = 10000.0, 10000.0, 3950.0, 0.0

# Key names in the FDIM's own scan order (index = row*8 + col). "" = unused position.
MATRIX_NAMES = (
    "CLIMATE_OFF", "BLOWER_DOWN", "MODE",         "DSC",       "LOAD",        "", "MFB4", "MFB3",   # row 0
    "RECIRC",      "BLOWER_UP",   "FRONT_DEMIST", "SCAN",      "OK",          "", "MFB5", "MFB2",   # row 1
    "AC",          "",            "TEMP_DOWN",    "CD_AUX",    "AUDIO_ONOFF", "", "MFB6", "MFB1",   # row 2
    "AUTO",        "",            "TEMP_UP",      "FM_AM",     "SEEK_DOWN",   "", "",     "",       # row 3
    "",            "",            "REAR_DEMIST",  "BACK_HOME", "SEEK_UP",     "", "",     "",       # row 4
)
DIRECT_NAMES = ("HAZARD", "MENU", "EJECT", "LOCK", "UNLOCK", "DOME_LAMP")

# HID keymap for fdim_pi.py's skin mode (climate/body keys are serial-only).
HID_MAP = {
    "MFB1": "ONE", "MFB2": "TWO", "MFB3": "THREE", "MFB4": "FOUR", "MFB5": "FIVE", "MFB6": "SIX",
    "SEEK_UP": "UP_ARROW", "SEEK_DOWN": "DOWN_ARROW", "OK": "RETURN", "BACK_HOME": "ESCAPE",
    "MENU": "M", "AUDIO_ONOFF": "P", "CD_AUX": "S", "FM_AM": "R", "SCAN": "A", "LOAD": "L", "EJECT": "E",
}
ENC_HID = ("RIGHT_ARROW", "LEFT_ARROW")


# ------------------------------------------------------------------ LOGIC -----------------
class Panel:
    """All the behaviour, hardware-agnostic so it can be unit-tested on a PC."""

    def __init__(self, hw):
        self.hw = hw
        self.out = hw.serial()
        self.kbd = hw.keyboard() if USE_HID else None
        self.matrix = hw.make_matrix(ROW_PINS, COL_PINS)       # keypad.KeyMatrix: scan + debounce in C
        self.direct = hw.make_keys(DIRECT_PINS)                # keypad.Keys
        self.enc_a, self.enc_b = hw.make_inputs(ENC_PINS)
        self.cts = hw.make_adc(CTS_PIN)
        self.illum = hw.make_pwm(ILLUM_PIN)
        self.mapping = False
        self.hid_on = USE_HID and self.kbd is not None
        self.ea = self.enc_a.value is False
        self.eb = self.enc_b.value is False
        self.last_cts = -1
        self.t_cts = 0.0
        self.t_enc = 0.0
        self.cmd = ""
        self.held = set()

    # --- output -------------------------------------------------------------------------
    def say(self, line):
        try:
            self.out.write((line + "\n").encode())
        except Exception:
            pass

    def emit(self, name, down):
        self.say("KEY %s %s" % (name, "DOWN" if down else "UP"))
        if down:
            self.held.add(name)
        else:
            self.held.discard(name)
        if self.hid_on and name in HID_MAP:
            kc = self.hw.keycode(HID_MAP[name])
            try:
                if down:
                    self.kbd.press(kc)
                else:
                    self.kbd.release(kc)
            except Exception:
                pass

    # --- inputs -------------------------------------------------------------------------
    def poll_keys(self):
        ev = self.matrix.events.get()
        while ev:
            r, c = divmod(ev.key_number, 8)
            if self.mapping:
                self.say("MAT %d %d %s" % (r, c, "DOWN" if ev.pressed else "UP"))
            name = MATRIX_NAMES[ev.key_number] or ("R%dC%d" % (r, c))
            self.emit(name, ev.pressed)
            ev = self.matrix.events.get()
        ev = self.direct.events.get()
        while ev:
            if self.mapping:
                self.say("DIRECT %d %s" % (ev.key_number, "DOWN" if ev.pressed else "UP"))
            self.emit(DIRECT_NAMES[ev.key_number], ev.pressed)
            ev = self.direct.events.get()

    def poll_encoder(self):
        # FDIM algorithm: an A edge while B is steady (and A != B after it) = RIGHT, a B edge while A is steady = LEFT.
        a = self.enc_a.value is False
        b = self.enc_b.value is False
        d = 0
        if a != self.ea and b == self.eb and a != b:
            d = 1
        elif b != self.eb and a == self.ea and a != b:
            d = -1
        self.ea, self.eb = a, b
        if d:
            self.say("ENC %+d" % d)
            if self.hid_on:
                kc = self.hw.keycode(ENC_HID[0] if d > 0 else ENC_HID[1])
                try:
                    self.kbd.press(kc)
                    self.kbd.release(kc)
                except Exception:
                    pass
        return d

    def cts_code(self):
        acc = 0
        for _ in range(16):
            acc += self.cts.value
        v = (acc / 16.0) / 65535.0
        if v <= 0.001 or v >= 0.999:
            return -1
        r = CTS_R_BIAS * v / (1.0 - v)
        inv_t = 1.0 / 298.15 + math.log(r / CTS_R25) / CTS_BETA
        tc = 1.0 / inv_t - 273.15 + CTS_OFFSET
        return max(0, min(250, int(round(tc * 6.0))))

    def poll_cts(self, now):
        if now - self.t_cts < CTS_PERIOD_S:
            return
        self.t_cts = now
        code = self.cts_code()
        if code >= 0 and code != self.last_cts:
            self.last_cts = code
            self.say("CTS %d" % code)

    # --- commands from the Pi -------------------------------------------------------------
    def handle(self, line):
        p = line.strip().split()
        if not p:
            return
        try:
            if p[0] == "PING":
                self.say("PONG")
            elif p[0] == "MAP" and len(p) > 1:
                self.mapping = p[1] != "0"
                self.say("OK")
            elif p[0] == "DIM" and len(p) > 1:
                v = max(0, min(255, int(p[1])))
                self.illum.duty_cycle = v * 257
                self.say("OK")
            elif p[0] == "HID" and len(p) > 1:
                self.hid_on = USE_HID and self.kbd is not None and p[1] != "0"
                self.say("OK")
            elif p[0] == "INFO":
                for r in range(5):
                    self.say("INFO row %d: %s" % (r, " ".join(MATRIX_NAMES[r * 8 + c] or "-" for c in range(8))))
                self.say("INFO direct: " + " ".join(DIRECT_NAMES))
                self.say("INFO hid=%d mapping=%d" % (1 if self.hid_on else 0, 1 if self.mapping else 0))
            else:
                self.say("ERR unknown command")
        except ValueError:
            self.say("ERR bad argument")

    def poll_commands(self):
        try:
            n = self.out.in_waiting
        except Exception:
            n = 0
        if n:
            data = self.out.read(n)
            for ch in data.decode(errors="replace"):
                if ch in "\r\n":
                    if self.cmd:
                        self.handle(self.cmd)
                    self.cmd = ""
                elif len(self.cmd) < 64:
                    self.cmd += ch

    def tick(self, now):
        self.poll_commands()
        self.poll_keys()
        if now - self.t_enc >= ENC_PERIOD_S:
            self.t_enc = now
            self.poll_encoder()
        self.poll_cts(now)


# ------------------------------------------------------------------ HARDWARE --------------
class RealHW:
    def pin(self, name):
        return getattr(board, name)

    def make_matrix(self, rows, cols):
        # columns_to_anodes=False: a row is driven low when selected, columns are pulled up, LOW = pressed
        # (that is how the FDIM's row driver + pull-ups work).
        return keypad.KeyMatrix(row_pins=[self.pin(p) for p in rows], column_pins=[self.pin(p) for p in cols],
                                columns_to_anodes=False, interval=SCAN_S, max_events=64)

    def make_keys(self, pins):
        return keypad.Keys([self.pin(p) for p in pins], value_when_pressed=False, pull=True,
                           interval=SCAN_S, max_events=32)

    def make_inputs(self, pins):
        outs = []
        for p in pins:
            d = digitalio.DigitalInOut(self.pin(p))
            d.direction = digitalio.Direction.INPUT
            d.pull = digitalio.Pull.UP
            outs.append(d)
        return outs

    def make_adc(self, name):
        return analogio.AnalogIn(self.pin(name))

    def make_pwm(self, name):
        return pwmio.PWMOut(self.pin(name), frequency=1000, duty_cycle=0)

    def keyboard(self):
        try:
            return Keyboard(usb_hid.devices)
        except Exception:
            return None

    def serial(self):
        return usb_cdc.data if usb_cdc.data else usb_cdc.console

    def keycode(self, name):
        return getattr(Keycode, name)


def main(hw, iterations=None):
    panel = Panel(hw)
    panel.say("HELLO fg-icc-buttons 2 keys=33")
    n = 0
    while iterations is None or n < iterations:
        panel.tick(time.monotonic())
        time.sleep(0.0005)
        n += 1
    return panel


# ------------------------------------------------------------------ SELF-TEST (CPython) ---
if SIM:
    class _Ev:
        def __init__(self, k, p):
            self.key_number, self.pressed = k, p

    class _Q:
        def __init__(self):
            self.q = []

        def get(self):
            return self.q.pop(0) if self.q else None

    class _Scanner:
        def __init__(self):
            self.events = _Q()

    class _In:
        def __init__(self):
            self.value = True        # pulled up = released

    class _Adc:
        value = 65535 // 2           # mid-scale -> NTC == R_BIAS -> 25 degC -> code 150

    class _Pwm:
        duty_cycle = 0

    class _Serial:
        def __init__(self):
            self.lines, self.inbox = [], b""

        def write(self, b):
            self.lines.append(b.decode().strip())

        @property
        def in_waiting(self):
            return len(self.inbox)

        def read(self, n):
            d, self.inbox = self.inbox[:n], self.inbox[n:]
            return d

    class _Kbd:
        def __init__(self):
            self.pressed = []

        def press(self, k):
            self.pressed.append(("down", k))

        def release(self, k):
            self.pressed.append(("up", k))

    class MockHW(RealHW):
        def __init__(self):
            self.ser, self.kb, self.ins = _Serial(), _Kbd(), {}

        def make_matrix(self, rows, cols):
            self.mat = _Scanner(); return self.mat

        def make_keys(self, pins):
            self.dir = _Scanner(); return self.dir

        def make_inputs(self, pins):
            self.ins = [_In() for _ in pins]; return self.ins

        def make_adc(self, name):
            return _Adc()

        def make_pwm(self, name):
            self.pwm = _Pwm(); return self.pwm

        def keyboard(self):
            return self.kb

        def serial(self):
            return self.ser

        def keycode(self, name):
            return name

    def selftest():
        hw = MockHW()
        p = Panel(hw)
        p.say("HELLO")
        # matrix: SEEK_UP is row 4 col 4 -> index 36 ; MFB1 = 23 ; unused position 5 -> R0C5
        hw.mat.events.q += [_Ev(36, True), _Ev(36, False), _Ev(23, True), _Ev(5, True)]
        hw.dir.events.q += [_Ev(0, True), _Ev(1, True)]
        p.tick(0.0)
        assert "KEY SEEK_UP DOWN" in hw.ser.lines and "KEY SEEK_UP UP" in hw.ser.lines, hw.ser.lines
        assert "KEY MFB1 DOWN" in hw.ser.lines and "KEY R0C5 DOWN" in hw.ser.lines
        assert "KEY HAZARD DOWN" in hw.ser.lines and "KEY MENU DOWN" in hw.ser.lines
        assert ("down", "UP_ARROW") in hw.kb.pressed and ("down", "ONE") in hw.kb.pressed and ("down", "M") in hw.kb.pressed
        assert not any(k[1] == "HAZARD" for k in hw.kb.pressed)        # body keys are serial-only
        # encoder, FDIM algorithm: A edge with B steady = +1, then B edge = no count ; B edge first = -1
        a, b = hw.ins
        a.value = False; assert p.poll_encoder() == 1
        b.value = False; assert p.poll_encoder() == 0
        a.value = True;  assert p.poll_encoder() == 1
        b.value = True;  assert p.poll_encoder() == 0
        b.value = False; assert p.poll_encoder() == -1
        a.value = False; assert p.poll_encoder() == 0
        assert hw.ser.lines.count("ENC +1") == 2 and hw.ser.lines.count("ENC -1") == 1
        # cabin temperature: mid-scale = 25 degC -> code 150 (0x313 byte 0)
        assert p.cts_code() == 150, p.cts_code()
        p.poll_cts(5.0)
        assert "CTS 150" in hw.ser.lines
        # commands
        hw.ser.inbox = b"PING\nMAP 1\nDIM 128\nHID 0\nBOGUS\n"
        p.poll_commands()
        assert "PONG" in hw.ser.lines and hw.pwm.duty_cycle == 128 * 257 and p.mapping and not p.hid_on
        assert "ERR unknown command" in hw.ser.lines
        hw.mat.events.q += [_Ev(0, True)]
        p.tick(1.0)
        assert "MAT 0 0 DOWN" in hw.ser.lines and "KEY CLIMATE_OFF DOWN" in hw.ser.lines
        # the table has exactly the 27 matrix keys + 6 direct keys the firmware scans
        assert sum(1 for n in MATRIX_NAMES if n) == 27 and len(DIRECT_NAMES) == 6
        print("PICO FIRMWARE SELF-TEST PASSED (%d events)" % len(hw.ser.lines))
        return 0

    if __name__ == "__main__":
        raise SystemExit(selftest())
else:
    main(RealHW())
