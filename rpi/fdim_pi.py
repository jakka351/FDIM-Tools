#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fdim_pi.py  —  FG Falcon ICC → Raspberry Pi FDIM replacement, all-in-one.

Replaces the factory Front Display Interface Module (centre-console media/climate
display) with a Pi + touchscreen that reproduces the factory look, UI and on-bus
behaviour, for BOTH the mono (8R29-14D017-AC) and colour (8R29-14D017-CB) units.

It does three jobs, exactly like the real FDIM:
  1. RENDER   - draws the factory pages from MS-CAN state (ignition, source,
                audio-present, iPod link, ...).
  2. IMPERSONATE - transmits the FDIM's network-management heartbeat + periodic
                status so the ACU / NM master see the module present and awake,
                and emits button-report frames when keys are pressed.
  3. INPUT    - takes touch (new), factory hard buttons (via a front-panel MCU as
                USB-HID or GPIO), and keyboard (bench).

Design goals: one file, runs on the Pi against real `can0`, and ALSO runs on a
bench/dev box with no hardware (`--sim`) or against SocketCAN `vcan0`. Heavy libs
(python-can, pygame, evdev) are imported lazily so the core logic + `--selftest`
run with nothing installed.

--------------------------------------------------------------------------------
QUICK START
--------------------------------------------------------------------------------
  # logic self-test, no hardware, no deps:
  python3 fdim_pi.py --selftest

  # bench UI with a synthetic bus, in a window:
  python3 fdim_pi.py --sim --unit colour --windowed
  python3 fdim_pi.py --sim --unit mono   --windowed

  # against a virtual SocketCAN bus you feed with candump/canplayer:
  sudo modprobe vcan; sudo ip link add dev vcan0 type vcan; sudo ip link set vcan0 up
  python3 fdim_pi.py --interface vcan0 --unit colour --windowed

  # in the car, on the DSI panel, full behaviour:
  python3 fdim_pi.py --interface can0 --unit colour --assets ../gfx-lo --allow-shutdown

Dependencies on the Pi:
  sudo apt install python3-pygame
  pip3 install python-can evdev       # evdev optional (HID buttons)
SocketCAN bring-up (MCP2515 example — match your crystal/transceiver, see the spec):
  sudo ip link set can0 type can bitrate 125000 restart-ms 100 && sudo ip link set can0 up

--------------------------------------------------------------------------------
IMPORTANT — what is firmware-accurate vs what needs a car capture
--------------------------------------------------------------------------------
Accurate (from the firmware teardowns in this repo):
  * bus = MS-CAN 125 kbit/s (BRP 9, BTR 0x3508).
  * RX decode of ignition 0x406, source 0x2F2, audio-present 0x311, iPod 0x2D4.
  * TX set + cadences: 0x55C NM (2 s), 0x2FC (100 ms), 0x2E2/0x313/0x315 (250 ms),
    0x307 (500 ms).
  * page/state machine, source map, splash-on-wake, "Audio Code Error".
Needs a Phase-2 capture from YOUR running car (search for "CAPTURE" below):
  * exact button-report byte layouts the ACU acts on,
  * radio station text / frequency, clock, climate set-points, illumination.
The mechanism for all of those is implemented; only the byte maps are placeholders.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Callable, Optional

LOG = logging.getLogger("fdim")

# ==============================================================================
#  CONFIG
# ==============================================================================

UNIT_MONO = "mono"
UNIT_COLOUR = "colour"


@dataclass
class Config:
    unit: str = UNIT_COLOUR            # "mono" | "colour"
    interface: str = "can0"            # SocketCAN interface, or "sim"
    bitrate: int = 125000              # MS-CAN
    panel_w: int = 800                 # official 7" DSI is 800x480
    panel_h: int = 480
    windowed: bool = False             # window vs framebuffer/KMSDRM
    assets_dir: str = ""               # path to gfx-lo (mono bitmaps/logos)
    aim_assets_dir: str = ""           # path to gfx-aim (iPod logos)
    tx_enabled: bool = True            # FDIM impersonation on the bus
    hid_device: str = ""               # /dev/input/eventN for front-panel MCU
    allow_shutdown: bool = False       # let PowerManager halt the Pi
    sleep_seconds: float = 30.0        # ign-off -> sleep (arms splash on wake)
    linger_seconds: float = 5.0        # audio page lingers after ign-off (state 5)
    splash_seconds: float = 2.5        # logo dwell on wake
    bus_timeout: float = 1.2           # no 0x406 for this long -> display off
    fps: int = 30
    fpv_logo: bool = False             # show FPV splash instead of Ford oval
    sim: bool = False
    verbose: bool = False
    # --- identical (firmware-mirror) mode: display the real firmware's framebuffer ---
    identical: bool = False            # run the real firmware via fdim-mirror, pixel-exact
    mirror_bin: str = ""               # path to the compiled fdim-mirror host
    app_bin: str = ""                  # path to the firmware image (…-AC-app.bin)
    canfile: str = ""                  # candump file for bench replay (else live --interface)
    adc: int = 430                     # supply-sense ADC override (keeps the panel on)
    flip: bool = False                 # panel 180° rotation (pass through to the mirror)
    max_frames: int = 0                # testing: stop after N mirror frames (0 = run)

    def resolve_assets(self):
        """Best-effort locate the extracted asset dirs relative to this file."""
        here = os.path.dirname(os.path.abspath(__file__))
        if not self.assets_dir:
            for c in ("../gfx-lo", "gfx-lo", "../docs/gfx-lo"):
                p = os.path.normpath(os.path.join(here, c))
                if os.path.isdir(p):
                    self.assets_dir = p
                    break
        if not self.aim_assets_dir:
            for c in ("../gfx-aim", "gfx-aim"):
                p = os.path.normpath(os.path.join(here, c))
                if os.path.isdir(p):
                    self.aim_assets_dir = p
                    break


# ==============================================================================
#  BUS STATE  (decoded signals, thread-safe)
# ==============================================================================

class Source(IntEnum):
    OFF = 0
    AUX1 = 1          # AUX1 / iPod
    AUX2 = 2
    CD_MP3 = 3
    CD = 4
    RADIO = 5
    PHONE = 6
    BLANK = 7


SOURCE_LABEL = {
    Source.OFF: "",
    Source.AUX1: "AUX 1",
    Source.AUX2: "AUX 2",
    Source.CD_MP3: "CD MP3",
    Source.CD: "CD",
    Source.RADIO: "RADIO",
    Source.PHONE: "PHONE",
    Source.BLANK: "",
}


@dataclass
class NowPlaying:
    title: str = ""
    artist: str = ""
    album: str = ""
    index: int = 0
    total: int = 0
    elapsed: int = 0          # seconds
    duration: int = 0


class BusState:
    """Everything the renderer needs, updated from CAN under a lock."""

    def __init__(self):
        self._lock = threading.Lock()
        self.ignition_on = False
        self.ign_off_event = False
        self.source = Source.OFF
        self.audio_present = True          # 0x311 b7.0 ; False -> "Audio Code Error"
        self.ipod_state = 0                # 0x2D4 b6 bits1-3 ; >0 -> iPod linked
        self.last_0x406 = 0.0
        self.last_rx = 0.0
        # content-bearing, from a CAPTURE (placeholders until mapped):
        self.radio_text = ""               # CAPTURE: station name / RDS
        self.radio_freq = ""               # CAPTURE: e.g. "101.7 FM"
        self.clock = ""                    # CAPTURE: "14:35"
        self.climate = ""                  # CAPTURE: "22.0C  AUTO"
        self.illumination = 1.0            # CAPTURE: 0..1 dash dimmer
        self.now = NowPlaying()
        self.frame_count = 0

    # --- mutators (called from the RX thread) ---
    def note_rx(self, t: float):
        with self._lock:
            self.last_rx = t
            self.frame_count += 1

    def set_ignition(self, on: bool, off_event: bool, t: float):
        with self._lock:
            self.ignition_on = on
            self.ign_off_event = off_event
            self.last_0x406 = t

    def set_source(self, s: Source):
        with self._lock:
            self.source = s

    def set_audio_present(self, p: bool):
        with self._lock:
            self.audio_present = p

    def set_ipod(self, st: int):
        with self._lock:
            self.ipod_state = st

    def set_now(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self.now, k, v)

    # --- snapshot (called from the render thread) ---
    def snapshot(self) -> dict:
        with self._lock:
            return dict(
                ignition_on=self.ignition_on,
                ign_off_event=self.ign_off_event,
                source=self.source,
                audio_present=self.audio_present,
                ipod_state=self.ipod_state,
                last_0x406=self.last_0x406,
                last_rx=self.last_rx,
                radio_text=self.radio_text,
                radio_freq=self.radio_freq,
                clock=self.clock,
                climate=self.climate,
                illumination=self.illumination,
                now=NowPlaying(**vars(self.now)),
                frame_count=self.frame_count,
            )


# ==============================================================================
#  CAN FRAME DECODE  (firmware-accurate)
# ==============================================================================

def decode_frame(bus: BusState, arb_id: int, data: bytes, t: float):
    """Map received MS-CAN frames onto BusState. IDs/fields per the teardowns."""
    bus.note_rx(t)

    if arb_id == 0x406 and len(data) >= 2:
        # ignition: mono byte1 bits0-4 (1 OFF, 4 RUN, 2/8 RUN-like);
        #           colour value 4 RUN, 8 START. byte0 bits0-1==1 -> ign-off event.
        raw = data[1] & 0x1F
        on = raw in (2, 4, 8)
        off_event = (data[0] & 0x03) == 1
        bus.set_ignition(on, off_event, t)

    elif arb_id == 0x2F2 and len(data) >= 7:
        # audio source selector
        try:
            bus.set_source(Source(data[6] & 0x07))
        except ValueError:
            bus.set_source(Source.BLANK)

    elif arb_id == 0x311 and len(data) >= 8:
        # audio unit present / security OK (clear -> "Audio Code Error")
        bus.set_audio_present(bool(data[7] & 0x01))

    elif arb_id == 0x2D4 and len(data) >= 7:
        # iPod link state from the AIM (byte6 bits1-3)
        bus.set_ipod((data[6] >> 1) & 0x07)

    # -------------------------------------------------------------------------
    # CAPTURE: add your Phase-2 decoders here once mapped from a car candump.
    #   elif arb_id == 0x3??:   bus.radio_text = data[...].decode(...)
    #   elif arb_id == 0x3??:   bus.radio_freq = ...
    #   elif arb_id == 0x1??:   bus.clock = ...
    #   elif arb_id == 0x3??:   bus.climate = ...
    # The AIM already sends now-playing text via ISO-TP on 0x2E8/0x2E4/... ;
    # reuse the AIM teardown's reassembly if you want track metadata here.
    # -------------------------------------------------------------------------


# ==============================================================================
#  FDIM STATE MACHINE  (which page to show)
# ==============================================================================

class Page(Enum):
    OFF = "off"
    SPLASH = "splash"              # Ford / FPV oval on wake
    POWERSAVE = "powersave"
    AUDIO_CODE_ERROR = "audio_code_error"
    AUX = "aux"
    IPOD = "ipod"
    CD = "cd"
    CD_MP3 = "cd_mp3"
    RADIO = "radio"
    PHONE = "phone"
    HOME = "home"                  # source off / default (clock+climate strip)


class FDIMStateMachine:
    """
    Reproduces the factory display-state behaviour:
      * display off when ignition absent > bus_timeout,
      * on ign-off then a long sleep, the next wake shows the splash logo,
      * "Audio Code Error" when the audio unit isn't present/authorised,
      * otherwise the page follows the audio source (0x2F2), with AUX1 becoming
        the iPod page when the AIM reports a link (0x2D4).
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.page = Page.OFF
        self._was_on = False
        self._off_since: Optional[float] = None
        self._slept = False
        self._splash_until = 0.0

    def tick(self, snap: dict, now_t: float) -> Page:
        ign = snap["ignition_on"]
        fresh = (now_t - snap["last_0x406"]) < self.cfg.bus_timeout if snap["last_0x406"] else False
        powered = ign and fresh

        # track off->sleep to arm the wake splash
        if not powered:
            if self._was_on:
                self._off_since = now_t      # just lost power
                self._was_on = False
            off_for = (now_t - self._off_since) if self._off_since else 0.0
            if off_for >= self.cfg.sleep_seconds:
                self._slept = True           # truly asleep -> next wake shows splash
            # page: audio lingers briefly (state 5), then power-saver, then off.
            # linger_seconds must be < sleep_seconds for both windows to appear.
            if self._off_since is None:
                self.page = Page.OFF            # cold start, never powered
            elif off_for < self.cfg.linger_seconds \
                    and snap["source"] != Source.OFF and snap["audio_present"]:
                self.page = self._source_page(snap)
            elif off_for < self.cfg.sleep_seconds:
                self.page = Page.POWERSAVE
            else:
                self.page = Page.OFF
            return self.page

        # powered
        if not self._was_on:
            # just woke
            if self._slept:
                self._splash_until = now_t + self.cfg.splash_seconds
            self._slept = False
            self._off_since = None
        self._was_on = True

        if now_t < self._splash_until:
            self.page = Page.SPLASH
        elif not snap["audio_present"]:
            self.page = Page.AUDIO_CODE_ERROR
        else:
            self.page = self._source_page(snap)
        return self.page

    def _source_page(self, snap: dict) -> Page:
        s = snap["source"]
        if s == Source.AUX1:
            return Page.IPOD if snap["ipod_state"] > 0 else Page.AUX
        if s == Source.AUX2:
            return Page.AUX
        if s == Source.CD:
            return Page.CD
        if s == Source.CD_MP3:
            return Page.CD_MP3
        if s == Source.RADIO:
            return Page.RADIO
        if s == Source.PHONE:
            return Page.PHONE
        return Page.HOME


# ==============================================================================
#  CAN INTERFACE  (RX thread + TX impersonation scheduler)
# ==============================================================================

# FDIM transmit set — cadence in seconds -> (arb_id, builder()).
# Builders return 8 data bytes. NM 0x55C steps 5C01->5C02->5C04 then holds.
class _NM:
    seq = [bytes([0x5C, 0x01, 0, 0, 0, 0, 0, 0]),
           bytes([0x5C, 0x02, 0, 0, 0, 0, 0, 0]),
           bytes([0x5C, 0x04, 0, 0, 0, 0, 0, 0])]
    i = 0

    @classmethod
    def next(cls):
        b = cls.seq[min(cls.i, len(cls.seq) - 1)]
        cls.i += 1
        return b


def _tx_table(seen_traffic: Callable[[], bool]):
    def f2fc():
        b = bytearray(8)
        b[0] = 0x01 if seen_traffic() else 0x00     # byte0 bit0 once valid traffic seen
        b[7] = 0x46
        return bytes(b)

    def f307():
        b = bytearray(8)
        b[3] = 0xE4 if seen_traffic() else 0x64
        return bytes(b)

    return [
        # period(s)  id      builder
        (2.000, 0x55C, _NM.next),
        (0.100, 0x2FC, f2fc),
        (0.250, 0x2E2, lambda: bytes(8)),
        (0.250, 0x313, lambda: bytes(8)),
        (0.250, 0x315, lambda: bytes([0x00, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x00])),
        (0.500, 0x307, f307),
    ]


# Button-report frames the ACU acts on.
# CAPTURE: these arbitration IDs / bytes are PLACEHOLDERS. Sniff your car while
# pressing each fascia key, then fill the real frame here. The send mechanism is
# correct; only the bytes are unknown from firmware. The local UI still responds.
BUTTON_REPORTS = {
    "SOURCE":   (0x2E2, bytes([0x01, 0, 0, 0, 0, 0, 0, 0])),
    "SEEK_UP":  (0x2E2, bytes([0x02, 0, 0, 0, 0, 0, 0, 0])),
    "SEEK_DOWN":(0x2E2, bytes([0x03, 0, 0, 0, 0, 0, 0, 0])),
    "VOL_UP":   (0x2E2, bytes([0x04, 0, 0, 0, 0, 0, 0, 0])),
    "VOL_DOWN": (0x2E2, bytes([0x05, 0, 0, 0, 0, 0, 0, 0])),
    "MENU":     (0x2E2, bytes([0x06, 0, 0, 0, 0, 0, 0, 0])),
    "POWER":    (0x2E2, bytes([0x07, 0, 0, 0, 0, 0, 0, 0])),
    "PRESET1":  (0x2E2, bytes([0x10, 0, 0, 0, 0, 0, 0, 0])),
    "PRESET2":  (0x2E2, bytes([0x11, 0, 0, 0, 0, 0, 0, 0])),
}


class CanInterface:
    """Wraps python-can SocketCAN. RX -> BusState; background TX scheduler."""

    def __init__(self, cfg: Config, bus: BusState):
        self.cfg = cfg
        self.bus = bus
        self._can = None
        self._stop = threading.Event()
        self._rx_t: Optional[threading.Thread] = None
        self._tx_t: Optional[threading.Thread] = None
        self._tx_table = _tx_table(lambda: self.bus.frame_count > 0)

    def open(self):
        import can  # lazy
        self._can = can.interface.Bus(channel=self.cfg.interface,
                                      interface="socketcan",
                                      receive_own_messages=False)
        LOG.info("CAN open on %s @ %d", self.cfg.interface, self.cfg.bitrate)

    def start(self):
        self._rx_t = threading.Thread(target=self._rx_loop, name="can-rx", daemon=True)
        self._rx_t.start()
        if self.cfg.tx_enabled:
            self._tx_t = threading.Thread(target=self._tx_loop, name="can-tx", daemon=True)
            self._tx_t.start()

    def _rx_loop(self):
        import can  # lazy
        while not self._stop.is_set():
            try:
                msg = self._can.recv(timeout=0.5)
            except Exception as e:
                LOG.warning("CAN recv error: %s", e)
                time.sleep(0.2)
                continue
            if msg is None:
                continue
            decode_frame(self.bus, msg.arbitration_id, bytes(msg.data), time.monotonic())

    def _tx_loop(self):
        import can  # lazy
        due = [time.monotonic() + p for (p, _i, _b) in self._tx_table]
        while not self._stop.is_set():
            now = time.monotonic()
            for k, (period, arb_id, builder) in enumerate(self._tx_table):
                if now >= due[k]:
                    due[k] = now + period
                    try:
                        self._send(arb_id, builder())
                    except Exception as e:
                        LOG.warning("CAN tx error: %s", e)
            time.sleep(0.01)

    def _send(self, arb_id: int, data: bytes):
        import can  # lazy
        if self._can is None:
            return
        self._can.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def send_button(self, action: str):
        rep = BUTTON_REPORTS.get(action)
        if rep and self.cfg.tx_enabled:
            try:
                self._send(*rep)
                LOG.info("TX button %s -> 0x%03X %s", action, rep[0], rep[1].hex())
            except Exception as e:
                LOG.warning("button tx failed: %s", e)

    def stop(self):
        self._stop.set()
        if self._can is not None:
            try:
                self._can.shutdown()
            except Exception:
                pass


class SimBus(CanInterface):
    """
    No hardware. Synthesises a plausible MS-CAN: ignition RUN, source cycling
    through iPod/CD/Radio, now-playing text ticking. Lets the whole UI + state
    machine run on a dev box. TX is logged, not sent.
    """

    _TRACKS = [
        ("Turbo Lag Blues", "Tester Present", "Barra Sessions"),
        ("Boost Leak Boogie", "Tester Present", "Barra Sessions"),
        ("Kalgoorlie Highway", "Bromigo", "Single"),
        ("ZF Six Speed Shuffle", "Tester Present", "Barra Sessions"),
    ]

    def open(self):
        LOG.info("SIM bus: no hardware; synthesising MS-CAN state")

    def start(self):
        self._tx_t = threading.Thread(target=self._sim_loop, name="sim-bus", daemon=True)
        self._tx_t.start()

    def _sim_loop(self):
        t0 = time.monotonic()
        src_cycle = [Source.AUX1, Source.CD, Source.RADIO, Source.PHONE]
        while not self._stop.is_set():
            now = time.monotonic()
            el = now - t0
            # ignition on after 2 s (so you can see the wake splash if it slept)
            decode_frame(self.bus, 0x406, bytes([0x00, 0x04, 0, 0, 0, 0, 0, 0]), now)
            # audio present
            decode_frame(self.bus, 0x311, bytes([0, 0, 0, 0, 0, 0, 0, 0x01]), now)
            # source changes every 12 s
            src = src_cycle[int(el // 12) % len(src_cycle)]
            decode_frame(self.bus, 0x2F2, bytes([0, 0, 0, 0, 0, 0, int(src), 0]), now)
            # iPod linked when AUX1
            decode_frame(self.bus, 0x2D4,
                         bytes([0, 0, 0, 0, 0, 0, (0x06 if src == Source.AUX1 else 0x00), 0]), now)
            # now-playing + clock + radio
            ti, ar, al = self._TRACKS[int(el // 3) % len(self._TRACKS)]
            self.bus.set_now(title=ti, artist=ar, album=al,
                             index=1 + int(el // 3) % len(self._TRACKS),
                             total=len(self._TRACKS),
                             elapsed=int(el) % 240, duration=240)
            with self.bus._lock:
                self.bus.radio_text = "TRIPLE M"
                self.bus.radio_freq = "101.7 FM"
                self.bus.clock = time.strftime("%H:%M")
                self.bus.climate = "22.0°  AUTO"
            time.sleep(0.1)

    def send_button(self, action: str):
        LOG.info("SIM button %s (no bus send)", action)


# ==============================================================================
#  BUTTON INPUT  (front-panel MCU via HID, + keyboard on bench)
# ==============================================================================

# Map evdev/keyboard key codes to logical actions. The front-panel MCU presents
# the factory switches as an HID keyboard; adjust to its keymap.
KEY_ACTIONS = {
    # keyboard (pygame key names) -> action, for bench use
    "s": "SOURCE", "m": "MENU", "p": "POWER",
    "up": "SEEK_UP", "down": "SEEK_DOWN",
    "right": "VOL_UP", "left": "VOL_DOWN",
    "1": "PRESET1", "2": "PRESET2",
}


class HidButtons:
    """Optional: read the front-panel MCU as a USB-HID device via evdev."""

    def __init__(self, cfg: Config, on_action: Callable[[str], None]):
        self.cfg = cfg
        self.on_action = on_action
        self._stop = threading.Event()
        self._t: Optional[threading.Thread] = None

    def start(self):
        if not self.cfg.hid_device:
            return
        self._t = threading.Thread(target=self._loop, name="hid", daemon=True)
        self._t.start()

    def _loop(self):
        try:
            import evdev
        except Exception:
            LOG.warning("evdev not installed; HID buttons disabled")
            return
        try:
            dev = evdev.InputDevice(self.cfg.hid_device)
        except Exception as e:
            LOG.warning("cannot open HID %s: %s", self.cfg.hid_device, e)
            return
        LOG.info("HID buttons: %s (%s)", self.cfg.hid_device, dev.name)
        # map evdev KEY_* -> action; extend to match your MCU
        emap = {
            "KEY_S": "SOURCE", "KEY_M": "MENU", "KEY_P": "POWER",
            "KEY_UP": "SEEK_UP", "KEY_DOWN": "SEEK_DOWN",
            "KEY_RIGHT": "VOL_UP", "KEY_LEFT": "VOL_DOWN",
            "KEY_1": "PRESET1", "KEY_2": "PRESET2",
            "KEY_VOLUMEUP": "VOL_UP", "KEY_VOLUMEDOWN": "VOL_DOWN",
        }
        import evdev
        for ev in dev.read_loop():
            if self._stop.is_set():
                break
            if ev.type == evdev.ecodes.EV_KEY and ev.value == 1:  # key down
                name = evdev.ecodes.KEY[ev.code]
                act = emap.get(name if isinstance(name, str) else name[0])
                if act:
                    self.on_action(act)

    def stop(self):
        self._stop.set()


# ==============================================================================
#  RENDERING
# ==============================================================================

# Soft-key layout: logical rects are filled per-skin; touch maps presses here.
@dataclass
class SoftKey:
    label: str
    action: str
    rect: tuple = (0, 0, 0, 0)   # x,y,w,h in panel coords (filled by skin.layout)


class Renderer:
    """Base pygame renderer. Subclasses implement the two factory skins."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.pg = None
        self.screen = None
        self.clock = None
        self.fonts = {}
        self.assets = {}
        self.softkeys: list[SoftKey] = []

    def init(self):
        import pygame  # lazy
        self.pg = pygame
        if not self.cfg.windowed:
            # Pi DSI/KMS: try modern driver, fall back
            os.environ.setdefault("SDL_VIDEODRIVER", "kmsdrm")
        pygame.display.init()
        pygame.font.init()
        flags = 0 if self.cfg.windowed else pygame.FULLSCREEN
        try:
            self.screen = pygame.display.set_mode((self.cfg.panel_w, self.cfg.panel_h), flags)
        except Exception:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            self.screen = pygame.display.set_mode((self.cfg.panel_w, self.cfg.panel_h))
        pygame.display.set_caption("FG FDIM (%s)" % self.cfg.unit)
        pygame.mouse.set_visible(self.cfg.windowed)
        self.clock = pygame.time.Clock()
        self._load_fonts()
        self._load_assets()
        self.layout()

    def _font(self, size, bold=False, mono=False):
        key = (size, bold, mono)
        if key not in self.fonts:
            name = "dejavusansmono" if mono else "dejavusans"
            try:
                self.fonts[key] = self.pg.font.SysFont(name, size, bold=bold)
            except Exception:
                self.fonts[key] = self.pg.font.Font(None, size)
        return self.fonts[key]

    def _load_fonts(self):
        pass

    def _load_assets(self):
        """Load the extracted firmware bitmaps if present (optional)."""
        d = self.cfg.assets_dir
        if not d or not os.path.isdir(d):
            return
        want = {
            "ford": "img_00dcc4_235x66.png",
            "fpv": "img_00d4e0_235x66.png",
            "frame": "img_006a58_235x66.png",
        }
        for key, fn in want.items():
            p = os.path.join(d, fn)
            if os.path.isfile(p):
                try:
                    self.assets[key] = self.pg.image.load(p).convert_alpha()
                except Exception as e:
                    LOG.debug("asset %s load failed: %s", fn, e)
        # iPod logo (colour skin)
        if self.cfg.aim_assets_dir:
            p = os.path.join(self.cfg.aim_assets_dir, "ipod_logo_rgb565_120x44.png")
            if os.path.isfile(p):
                try:
                    self.assets["ipod"] = self.pg.image.load(p).convert_alpha()
                except Exception:
                    pass

    def layout(self):
        raise NotImplementedError

    def draw(self, page: Page, snap: dict):
        raise NotImplementedError

    def softkey_at(self, pos) -> Optional[str]:
        x, y = pos
        for sk in self.softkeys:
            rx, ry, rw, rh = sk.rect
            if rx <= x <= rx + rw and ry <= y <= ry + rh:
                return sk.action
        return None

    def flip(self):
        self.pg.display.flip()
        self.clock.tick(self.cfg.fps)

    def quit(self):
        try:
            self.pg.quit()
        except Exception:
            pass


# ---- MONO skin: 240x80 4-grey look, green dot-matrix, integer-upscaled -------

class MonoSkin(Renderer):
    BG = (6, 16, 6)
    DIM = (34, 96, 40)
    LIT = (140, 255, 106)
    LOGICAL = (240, 80)

    def layout(self):
        self.scale = min(self.cfg.panel_w // self.LOGICAL[0],
                         (self.cfg.panel_h - 40) // self.LOGICAL[1]) or 1
        self.lw = self.LOGICAL[0] * self.scale
        self.lh = self.LOGICAL[1] * self.scale
        self.ox = (self.cfg.panel_w - self.lw) // 2
        self.oy = (self.cfg.panel_h - self.lh) // 2 - 10
        # 6 soft-keys along the bottom (the factory fascia count)
        self.softkeys = []
        labels = [("SRC", "SOURCE"), ("SEEK-", "SEEK_DOWN"), ("SEEK+", "SEEK_UP"),
                  ("MENU", "MENU"), ("VOL-", "VOL_DOWN"), ("VOL+", "VOL_UP")]
        n = len(labels)
        bw = self.cfg.panel_w // n
        by = self.cfg.panel_h - 36
        for i, (lab, act) in enumerate(labels):
            self.softkeys.append(SoftKey(lab, act, (i * bw, by, bw - 4, 32)))

    def _tint(self, surf, colour):
        s = surf.copy()
        s.fill(colour, special_flags=self.pg.BLEND_RGBA_MULT)
        return s

    def _text(self, s, size, colour, x, y, center=False, mono=True, bold=False):
        f = self._font(size, bold=bold, mono=mono)
        img = f.render(s, True, colour)
        r = img.get_rect()
        if center:
            r.center = (x, y)
        else:
            r.topleft = (x, y)
        self.screen.blit(img, r)
        return r

    def draw(self, page: Page, snap: dict):
        self.screen.fill((0, 0, 0))
        # LCD field
        lcd = self.pg.Rect(self.ox, self.oy, self.lw, self.lh)
        self.pg.draw.rect(self.screen, self.BG, lcd)
        self.pg.draw.rect(self.screen, self.DIM, lcd, 1)
        cx = self.ox + self.lw // 2

        if page in (Page.OFF, Page.POWERSAVE):
            if page == Page.POWERSAVE:
                self._text("POWER SAVER", 20, self.DIM, cx, self.oy + self.lh // 2, center=True)
            self._draw_softkeys(dim=True)
            return

        if page == Page.SPLASH:
            logo = self.assets.get("fpv" if self.cfg.fpv_logo else "ford")
            if logo:
                g = self._tint(logo, self.LIT)
                g = self.pg.transform.scale(g, (int(logo.get_width() * self.scale),
                                                int(logo.get_height() * self.scale)))
                self.screen.blit(g, g.get_rect(center=lcd.center))
            else:
                self._text("FORD" if not self.cfg.fpv_logo else "FPV", 36,
                           self.LIT, cx, lcd.centery, center=True, bold=True)
            return

        if page == Page.AUDIO_CODE_ERROR:
            self._text("AUDIO CODE", 22, self.LIT, cx, self.oy + 26, center=True, bold=True)
            self._text("ERROR", 22, self.LIT, cx, self.oy + 52, center=True, bold=True)
            self._draw_softkeys()
            return

        # source header
        src = SOURCE_LABEL.get(snap["source"], "")
        hdr = {Page.IPOD: "iPod", Page.RADIO: snap["radio_freq"] or "RADIO",
               Page.CD: "CD", Page.CD_MP3: "CD MP3", Page.PHONE: "PHONE",
               Page.AUX: src or "AUX", Page.HOME: snap["clock"] or ""}.get(page, src)
        self._text(hdr, 18, self.LIT, self.ox + 6, self.oy + 4, bold=True)
        self._text(snap["clock"], 14, self.DIM, self.ox + self.lw - 6, self.oy + 6, center=False)

        y = self.oy + 30
        if page == Page.IPOD:
            n = snap["now"]
            self._text(n.title[:28], 18, self.LIT, self.ox + 6, y)
            self._text(n.artist[:28], 14, self.DIM, self.ox + 6, y + 22)
            self._text("%s  %d/%d" % (self._mmss(n.elapsed), n.index, n.total),
                       12, self.DIM, self.ox + 6, y + 40)
        elif page == Page.RADIO:
            self._text(snap["radio_text"][:20] or "---", 20, self.LIT, self.ox + 6, y, bold=True)
            self._text(snap["radio_freq"], 14, self.DIM, self.ox + 6, y + 24)
        elif page in (Page.CD, Page.CD_MP3):
            n = snap["now"]
            self._text("Track %d" % max(1, n.index), 20, self.LIT, self.ox + 6, y, bold=True)
            self._text(self._mmss(n.elapsed), 14, self.DIM, self.ox + 6, y + 24)
        elif page == Page.PHONE:
            self._text("PHONE", 20, self.LIT, self.ox + 6, y, bold=True)
        else:
            self._text(snap["climate"], 16, self.DIM, self.ox + 6, y)

        self._draw_softkeys()

    def _draw_softkeys(self, dim=False):
        col = self.DIM if dim else self.LIT
        for sk in self.softkeys:
            x, y, w, h = sk.rect
            r = self.pg.Rect(x, y, w, h)
            self.pg.draw.rect(self.screen, (10, 24, 10), r)
            self.pg.draw.rect(self.screen, self.DIM, r, 1)
            self._text(sk.label, 14, col, x + w // 2, y + h // 2, center=True)

    @staticmethod
    def _mmss(s):
        return "%d:%02d" % (s // 60, s % 60)


# ---- COLOUR skin: OEM+ 480x234-style UI scaled to the panel --------------------

class ColourSkin(Renderer):
    BG = (8, 10, 16)
    BAR = (18, 26, 44)
    ACCENT = (40, 120, 220)
    TEXT = (235, 240, 250)
    SUB = (150, 165, 190)

    def layout(self):
        self.softkeys = []
        labels = [("SRC", "SOURCE"), ("◀◀", "SEEK_DOWN"), ("▶▶", "SEEK_UP"),
                  ("MENU", "MENU"), ("–", "VOL_DOWN"), ("+", "VOL_UP")]
        n = len(labels)
        bw = self.cfg.panel_w // n
        by = self.cfg.panel_h - 64
        for i, (lab, act) in enumerate(labels):
            self.softkeys.append(SoftKey(lab, act, (i * bw + 6, by, bw - 12, 56)))

    def _text(self, s, size, colour, x, y, center=False, bold=False):
        f = self._font(size, bold=bold)
        img = f.render(s, True, colour)
        r = img.get_rect()
        if center:
            r.center = (x, y)
        else:
            r.topleft = (x, y)
        self.screen.blit(img, r)
        return r

    def _bright(self, snap):
        # auto-dim from the (captured) illumination signal
        return 0.35 + 0.65 * max(0.0, min(1.0, snap.get("illumination", 1.0)))

    def draw(self, page: Page, snap: dict):
        self.screen.fill(self.BG)
        W = self.cfg.panel_w

        if page in (Page.OFF, Page.POWERSAVE):
            if page == Page.POWERSAVE:
                self._text("Power Saver", 28, self.SUB, W // 2, self.cfg.panel_h // 2, center=True)
            return

        if page == Page.SPLASH:
            logo = self.assets.get("fpv" if self.cfg.fpv_logo else "ford")
            if logo:
                big = self.pg.transform.rotozoom(logo, 0, 2.0)
                self.screen.blit(big, big.get_rect(center=(W // 2, self.cfg.panel_h // 2)))
            else:
                self._text("FPV" if self.cfg.fpv_logo else "FORD", 72, self.TEXT,
                           W // 2, self.cfg.panel_h // 2, center=True, bold=True)
            return

        # top status bar
        self.pg.draw.rect(self.screen, self.BAR, (0, 0, W, 48))
        self._text(SOURCE_LABEL.get(snap["source"], "") or "HOME", 24, self.TEXT, 16, 10, bold=True)
        self._text(snap["clock"], 22, self.TEXT, W - 90, 12)
        self._text(snap["climate"], 18, self.SUB, W // 2 - 60, 14)

        if page == Page.AUDIO_CODE_ERROR:
            self._text("Audio Code Error", 40, (240, 90, 90), W // 2, 170, center=True, bold=True)
            self._draw_softkeys()
            return

        cx = W // 2
        if page == Page.IPOD:
            ipod = self.assets.get("ipod")
            if ipod:
                self.screen.blit(ipod, (16, 64))
            n = snap["now"]
            self._text(n.title[:32], 40, self.TEXT, 150, 72, bold=True)
            self._text(n.artist[:36], 26, self.SUB, 150, 120)
            self._text(n.album[:36], 22, self.SUB, 150, 152)
            self._progress(n.elapsed, n.duration, 150, 196, W - 180)
            self._text("%s / %s" % (self._mmss(n.elapsed), self._mmss(n.duration)),
                       18, self.SUB, 150, 206)
            self._text("%d of %d" % (n.index, max(1, n.total)), 18, self.SUB, W - 150, 206)
        elif page == Page.RADIO:
            self._text(snap["radio_text"] or "---", 52, self.TEXT, cx, 120, center=True, bold=True)
            self._text(snap["radio_freq"], 30, self.ACCENT, cx, 170, center=True)
        elif page in (Page.CD, Page.CD_MP3):
            n = snap["now"]
            self._text("CD" if page == Page.CD else "CD MP3", 30, self.SUB, cx, 80, center=True)
            self._text(n.title[:28] or ("Track %d" % max(1, n.index)), 40, self.TEXT, cx, 130,
                       center=True, bold=True)
            self._progress(n.elapsed, n.duration, 150, 190, W - 300)
        elif page == Page.PHONE:
            self._text("Phone", 40, self.TEXT, cx, 130, center=True, bold=True)
        else:  # HOME
            self._text(snap["clock"] or "", 64, self.TEXT, cx, 120, center=True, bold=True)
            self._text(snap["climate"] or "", 26, self.SUB, cx, 180, center=True)

        self._draw_softkeys()

    def _progress(self, el, dur, x, y, w):
        self.pg.draw.rect(self.screen, self.BAR, (x, y, w, 6), border_radius=3)
        if dur > 0:
            fw = int(w * max(0.0, min(1.0, el / dur)))
            self.pg.draw.rect(self.screen, self.ACCENT, (x, y, fw, 6), border_radius=3)

    def _draw_softkeys(self):
        for sk in self.softkeys:
            x, y, w, h = sk.rect
            r = self.pg.Rect(x, y, w, h)
            self.pg.draw.rect(self.screen, self.BAR, r, border_radius=8)
            self.pg.draw.rect(self.screen, self.ACCENT, r, 1, border_radius=8)
            self._text(sk.label, 22, self.TEXT, x + w // 2, y + h // 2, center=True)

    @staticmethod
    def _mmss(s):
        return "%d:%02d" % (s // 60, s % 60)


# ==============================================================================
#  POWER MANAGER  (ignition / bus-activity watchdog -> clean shutdown)
# ==============================================================================

class PowerManager:
    """
    The real FDIM slept with the bus and woke on CAN activity. A Pi must instead
    boot on ignition/traffic and SHUT DOWN CLEANLY when the bus goes quiet, or the
    SD card eventually corrupts. Pair with a UPS/supercap HAT (see the spec).
    """

    def __init__(self, cfg: Config, bus: BusState, on_shutdown: Callable[[], None]):
        self.cfg = cfg
        self.bus = bus
        self.on_shutdown = on_shutdown
        self._stop = threading.Event()
        self._t: Optional[threading.Thread] = None
        self._quiet_since: Optional[float] = None

    def start(self):
        self._t = threading.Thread(target=self._loop, name="power", daemon=True)
        self._t.start()

    def _loop(self):
        grace = 20.0  # seconds of dead bus before we halt
        while not self._stop.is_set():
            snap = self.bus.snapshot()
            now = time.monotonic()
            quiet = (now - snap["last_rx"]) > self.cfg.bus_timeout if snap["last_rx"] else False
            if quiet and not snap["ignition_on"]:
                self._quiet_since = self._quiet_since or now
                if (now - self._quiet_since) > grace:
                    LOG.warning("bus quiet %.0fs + ignition off -> shutdown", grace)
                    self.on_shutdown()
                    if self.cfg.allow_shutdown:
                        os.system("sudo shutdown -h now")
                    return
            else:
                self._quiet_since = None
            time.sleep(1.0)

    def stop(self):
        self._stop.set()


# ==============================================================================
#  APP
# ==============================================================================

class App:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.bus = BusState()
        self.sm = FDIMStateMachine(cfg)
        self.can: CanInterface = SimBus(cfg, self.bus) if (cfg.sim or cfg.interface == "sim") \
            else CanInterface(cfg, self.bus)
        self.renderer: Renderer = MonoSkin(cfg) if cfg.unit == UNIT_MONO else ColourSkin(cfg)
        self.hid = HidButtons(cfg, self._on_action)
        self.power = PowerManager(cfg, self.bus, self._on_shutdown)
        self._running = True
        # local source override so buttons respond even before the ACU map is captured
        self._src_cycle = [Source.AUX1, Source.CD, Source.RADIO, Source.PHONE]

    def _on_action(self, action: str):
        LOG.info("ACTION %s", action)
        self.can.send_button(action)   # tell the car (CAPTURE: real bytes)
        # local UI response (demo/bench; the ACU normally drives source via 0x2F2)
        if action == "SOURCE":
            cur = self.bus.source
            try:
                i = self._src_cycle.index(cur)
            except ValueError:
                i = -1
            self.bus.set_source(self._src_cycle[(i + 1) % len(self._src_cycle)])

    def _on_shutdown(self):
        self._running = False

    def setup(self):
        logging.basicConfig(level=logging.DEBUG if self.cfg.verbose else logging.INFO,
                            format="%(asctime)s %(levelname)s %(name)s: %(message)s")
        self.cfg.resolve_assets()
        LOG.info("unit=%s interface=%s assets=%s tx=%s",
                 self.cfg.unit, self.cfg.interface, self.cfg.assets_dir, self.cfg.tx_enabled)
        try:
            self.can.open()
        except Exception as e:
            LOG.error("CAN open failed (%s). Falling back to --sim.", e)
            self.can = SimBus(self.cfg, self.bus)
            self.can.open()
        self.can.start()
        self.hid.start()
        self.power.start()
        self.renderer.init()

    def run(self):
        pg = self.renderer.pg
        LOG.info("running; keys: %s | ESC quits", ", ".join(KEY_ACTIONS))
        while self._running:
            for ev in pg.event.get():
                if ev.type == pg.QUIT:
                    self._running = False
                elif ev.type == pg.KEYDOWN:
                    if ev.key == pg.K_ESCAPE:
                        self._running = False
                    else:
                        name = pg.key.name(ev.key)
                        act = KEY_ACTIONS.get(name)
                        if act:
                            self._on_action(act)
                elif ev.type in (pg.MOUSEBUTTONDOWN, getattr(pg, "FINGERDOWN", -1)):
                    pos = self._event_pos(ev, pg)
                    act = self.renderer.softkey_at(pos)
                    if act:
                        self._on_action(act)
            snap = self.bus.snapshot()
            page = self.sm.tick(snap, time.monotonic())
            self.renderer.draw(page, snap)
            self.renderer.flip()
        self.teardown()

    def _event_pos(self, ev, pg):
        if ev.type == pg.MOUSEBUTTONDOWN:
            return ev.pos
        # FINGERDOWN gives normalised 0..1
        return (int(ev.x * self.cfg.panel_w), int(ev.y * self.cfg.panel_h))

    def teardown(self):
        LOG.info("shutting down")
        self.can.stop()
        self.hid.stop()
        self.power.stop()
        self.renderer.quit()


# ==============================================================================
#  IDENTICAL MODE  (firmware mirror: display the real FDIM framebuffer)
# ==============================================================================
#
# The ONLY way to be pixel-identical to the factory screen is to let the factory
# code draw it. `fdim-mirror` (emu/fdim_mirror.c) runs the real firmware in real
# time, bridges MS-CAN, and streams the LCD framebuffer as "FBM1" messages:
#     "FBM1" | u16 w | u16 h | u8 fmt(0=grey0..3) | u8 flags | w*h bytes
# This mode just paints those frames and forwards touch/buttons back as presses.
# (Mono only: the colour unit's artwork lives in an external ROM not yet dumped.)

def _read_exact(stream, n):
    buf = b""
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def read_fbm1_frames(stream):
    """Yield (w, h, bytes) frames from an FBM1 stream until it closes."""
    while True:
        hdr = _read_exact(stream, 10)
        if not hdr or hdr[:4] != b"FBM1":
            return
        w = hdr[4] | (hdr[5] << 8)
        h = hdr[6] | (hdr[7] << 8)
        px = _read_exact(stream, w * h)
        if px is None or len(px) < w * h:
            return
        yield (w, h, px)


class MirrorProc:
    """Spawns fdim-mirror, reads its framebuffer stream, forwards button presses."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.proc = None
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.frames = 0

    def args(self):
        a = [self.cfg.mirror_bin, self.cfg.app_bin, "-fbhz", str(self.cfg.fps),
             "-adc", str(self.cfg.adc)]
        if self.cfg.canfile:
            a += ["-canfile", self.cfg.canfile]
        elif self.cfg.interface not in ("", "sim"):
            a += ["-canif", self.cfg.interface]
        if self.cfg.flip:
            a += ["-flip"]
        if self.cfg.max_frames:
            a += ["-frames", str(self.cfg.max_frames)]
        return a

    def start(self):
        import subprocess
        a = self.args()
        LOG.info("spawning mirror: %s", " ".join(a))
        self.proc = subprocess.Popen(a, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=None, bufsize=0)
        threading.Thread(target=self._reader, name="fb-reader", daemon=True).start()

    def _reader(self):
        for (w, h, px) in read_fbm1_frames(self.proc.stdout):
            with self._lock:
                self._latest = (w, h, px)
                self.frames += 1
            if self._stop.is_set():
                break

    def latest(self):
        with self._lock:
            return self._latest

    def press(self, n: int):
        self._send("BTN %d\n" % n)

    def release(self, n: int):
        self._send("REL %d\n" % n)

    def inject_can(self, arb_id: int, data: bytes):
        self._send("CAN %X %s\n" % (arb_id, " ".join("%02X" % b for b in data)))

    def _send(self, s: str):
        try:
            if self.proc and self.proc.stdin:
                self.proc.stdin.write(s.encode())
                self.proc.stdin.flush()
        except Exception as e:
            LOG.debug("mirror ctl write failed: %s", e)

    def alive(self):
        return self.proc and self.proc.poll() is None

    def stop(self):
        self._stop.set()
        if self.proc:
            try:
                self.proc.terminate()
            except Exception:
                pass


# grey 0..3 -> panel colour (the mono LCD look). Index by level.
MONO_PALETTE = [(6, 16, 6), (34, 96, 40), (90, 180, 96), (140, 255, 106)]


def run_identical(cfg: Config):
    """Pixel-exact mono: paint the real firmware's framebuffer, forward input."""
    if not cfg.mirror_bin or not os.path.isfile(cfg.mirror_bin):
        LOG.error("identical mode needs --mirror <path to fdim-mirror>")
        return 2
    if not cfg.app_bin or not os.path.isfile(cfg.app_bin):
        LOG.error("identical mode needs --app <firmware .bin>")
        return 2

    mir = MirrorProc(cfg)
    mir.start()

    # headless test path: just verify frames arrive, no display
    if cfg.max_frames and os.environ.get("SDL_VIDEODRIVER") == "dummy":
        import time as _t
        t0 = _t.time()
        while mir.frames < cfg.max_frames and _t.time() - t0 < 30 and mir.alive():
            _t.sleep(0.05)
        last = mir.latest()
        nonblank = last and any(last[2])
        LOG.info("identical self-test: %d frames, last non-blank=%s", mir.frames, bool(nonblank))
        mir.stop()
        return 0 if (mir.frames >= cfg.max_frames and nonblank) else 1

    import pygame
    if not cfg.windowed:
        os.environ.setdefault("SDL_VIDEODRIVER", "kmsdrm")
    pygame.display.init(); pygame.font.init()
    flags = 0 if cfg.windowed else pygame.FULLSCREEN
    screen = pygame.display.set_mode((cfg.panel_w, cfg.panel_h), flags)
    pygame.display.set_caption("FG FDIM (identical)")
    pygame.mouse.set_visible(cfg.windowed)
    clock = pygame.time.Clock()

    # 6 touch zones across the bottom -> buttons P50..P55 (confirm key order on the bench)
    nbtn = 6
    bw = cfg.panel_w // nbtn
    zones = [(i * bw, cfg.panel_h - 48, bw, 48) for i in range(nbtn)]

    def zone_at(pos):
        x, y = pos
        for i, (zx, zy, zw, zh) in enumerate(zones):
            if zx <= x <= zx + zw and zy <= y <= zy + zh:
                return i
        return None

    # keyboard -> button index (bench)
    kb = {"1": 0, "2": 1, "3": 2, "4": 3, "5": 4, "6": 5}
    running = True
    palette_surf = None
    while running and mir.alive():
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif pygame.key.name(ev.key) in kb:
                    mir.press(kb[pygame.key.name(ev.key)])
            elif ev.type == pygame.KEYUP:
                if pygame.key.name(ev.key) in kb:
                    mir.release(kb[pygame.key.name(ev.key)])
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                n = zone_at(ev.pos)
                if n is not None:
                    mir.press(n)
            elif ev.type == pygame.MOUSEBUTTONUP:
                n = zone_at(ev.pos)
                if n is not None:
                    mir.release(n)

        screen.fill((0, 0, 0))
        f = mir.latest()
        if f:
            w, h, px = f
            # build an RGB surface from grey levels via the mono palette
            surf = pygame.Surface((w, h))
            buf = bytearray(w * h * 3)
            for i, lvl in enumerate(px):
                r, g, b = MONO_PALETTE[lvl & 3]
                buf[i * 3] = r; buf[i * 3 + 1] = g; buf[i * 3 + 2] = b
            surf = pygame.image.frombuffer(bytes(buf), (w, h), "RGB")
            scale = min(cfg.panel_w // w, (cfg.panel_h - 10) // h) or 1
            big = pygame.transform.scale(surf, (w * scale, h * scale))
            screen.blit(big, big.get_rect(center=(cfg.panel_w // 2, cfg.panel_h // 2)))
        pygame.display.flip()
        clock.tick(cfg.fps)

    mir.stop()
    pygame.quit()
    return 0


# ==============================================================================
#  SELF-TEST  (no pygame / no python-can — exercises the logic only)
# ==============================================================================

def selftest() -> int:
    print("== decode + state-machine self-test ==")
    cfg = Config(unit=UNIT_COLOUR, sleep_seconds=2.0, linger_seconds=0.5,
                 splash_seconds=1.0, bus_timeout=1.2)
    bus = BusState()
    sm = FDIMStateMachine(cfg)
    t = 100.0

    def step(dt=0.1):
        nonlocal t
        t += dt
        return sm.tick(bus.snapshot(), t)

    # ignition off, no traffic -> OFF
    p = step()
    assert p == Page.OFF, p

    # ignition RUN + audio present + source RADIO -> RADIO
    decode_frame(bus, 0x406, bytes([0, 0x04, 0, 0, 0, 0, 0, 0]), t)
    decode_frame(bus, 0x311, bytes([0, 0, 0, 0, 0, 0, 0, 0x01]), t)
    decode_frame(bus, 0x2F2, bytes([0, 0, 0, 0, 0, 0, int(Source.RADIO), 0]), t)
    p = step()
    assert p == Page.RADIO, p

    # audio unit drops out -> Audio Code Error
    decode_frame(bus, 0x311, bytes([0, 0, 0, 0, 0, 0, 0, 0x00]), t)
    p = step()
    assert p == Page.AUDIO_CODE_ERROR, p

    # restore + AUX1 with iPod link -> IPOD
    decode_frame(bus, 0x311, bytes([0, 0, 0, 0, 0, 0, 0, 0x01]), t)
    decode_frame(bus, 0x2F2, bytes([0, 0, 0, 0, 0, 0, int(Source.AUX1), 0]), t)
    decode_frame(bus, 0x2D4, bytes([0, 0, 0, 0, 0, 0, 0x06, 0]), t)
    p = step()
    assert p == Page.IPOD, p

    # ignition off (stop feeding 0x406), wait past bus_timeout -> OFF/POWERSAVE,
    # then past sleep -> slept; next wake -> SPLASH
    for _ in range(20):
        p = step(0.2)           # ~4 s of silence
    assert p in (Page.OFF, Page.POWERSAVE), p
    decode_frame(bus, 0x406, bytes([0, 0x04, 0, 0, 0, 0, 0, 0]), t + 0.2)
    decode_frame(bus, 0x311, bytes([0, 0, 0, 0, 0, 0, 0, 0x01]), t + 0.2)
    decode_frame(bus, 0x2F2, bytes([0, 0, 0, 0, 0, 0, int(Source.RADIO), 0]), t + 0.2)
    p = step(0.2)
    assert p == Page.SPLASH, ("expected SPLASH on wake, got", p)
    print("  state machine: OK (OFF->RADIO->CODE_ERR->IPOD->sleep->SPLASH)")

    # ignition decode variants
    decode_frame(bus, 0x406, bytes([0x01, 0x01, 0, 0, 0, 0, 0, 0]), t)  # OFF + off-event
    s = bus.snapshot()
    assert s["ignition_on"] is False and s["ign_off_event"] is True
    print("  ignition decode: OK")

    # TX table builds valid 8-byte frames + NM progression
    tbl = _tx_table(lambda: True)
    ids = []
    for period, arb, builder in tbl:
        d = builder()
        assert isinstance(d, (bytes, bytearray)) and len(d) == 8, (hex(arb), d)
        ids.append(arb)
    assert 0x55C in ids and 0x2FC in ids and 0x315 in ids
    _NM.i = 0
    nm = [_NM.next() for _ in range(4)]
    assert nm[0][1] == 0x01 and nm[1][1] == 0x02 and nm[2][1] == 0x04 and nm[3][1] == 0x04
    print("  tx table + NM 5C01->5C02->5C04 hold: OK")

    # button reports exist for every keyboard action
    for act in set(KEY_ACTIONS.values()):
        assert act in BUTTON_REPORTS, act
    print("  button report table: OK")

    print("ALL SELF-TESTS PASSED")
    return 0


# ==============================================================================
#  CLI
# ==============================================================================

def parse_args(argv=None) -> Config:
    ap = argparse.ArgumentParser(
        description="FG Falcon FDIM replacement for Raspberry Pi (mono + colour).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--unit", choices=[UNIT_MONO, UNIT_COLOUR], default=UNIT_COLOUR)
    ap.add_argument("--interface", default="can0", help="SocketCAN iface, or 'sim'")
    ap.add_argument("--sim", action="store_true", help="synthetic bus, no hardware")
    ap.add_argument("--windowed", action="store_true", help="window instead of fullscreen/DSI")
    ap.add_argument("--assets", default="", help="path to gfx-lo (firmware bitmaps/logos)")
    ap.add_argument("--aim-assets", default="", help="path to gfx-aim (iPod logos)")
    ap.add_argument("--no-tx", action="store_true", help="do NOT transmit on the bus")
    ap.add_argument("--hid", default="", help="/dev/input/eventN for front-panel MCU")
    ap.add_argument("--fpv", action="store_true", help="FPV splash instead of Ford oval")
    # identical (firmware-mirror) mode
    ap.add_argument("--identical", action="store_true",
                    help="pixel-exact: run the real firmware via fdim-mirror and paint its framebuffer")
    ap.add_argument("--mirror", default="", help="path to the compiled fdim-mirror host")
    ap.add_argument("--app", default="", help="firmware image for the mirror (…-AC-app.bin)")
    ap.add_argument("--canfile", default="", help="candump file for bench replay (else live --interface)")
    ap.add_argument("--adc", type=int, default=430, help="supply-sense ADC override")
    ap.add_argument("--flip", action="store_true", help="panel 180° rotation (pass to mirror)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N mirror frames (testing)")
    ap.add_argument("--allow-shutdown", action="store_true",
                    help="let the power watchdog halt the Pi when the bus dies")
    ap.add_argument("--panel", default="800x480", help="panel WxH")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--selftest", action="store_true", help="run logic tests and exit")
    a = ap.parse_args(argv)

    if a.selftest:
        sys.exit(selftest())

    try:
        pw, ph = (int(x) for x in a.panel.lower().split("x"))
    except Exception:
        pw, ph = 800, 480

    return Config(
        unit=a.unit, interface=a.interface, sim=a.sim, windowed=a.windowed,
        assets_dir=a.assets, aim_assets_dir=a.aim_assets, tx_enabled=not a.no_tx,
        hid_device=a.hid, fpv_logo=a.fpv, allow_shutdown=a.allow_shutdown,
        panel_w=pw, panel_h=ph, fps=a.fps, verbose=a.verbose,
        identical=a.identical, mirror_bin=a.mirror, app_bin=a.app, canfile=a.canfile,
        adc=a.adc, flip=a.flip, max_frames=a.max_frames,
    )


def main(argv=None):
    cfg = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if cfg.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    if cfg.identical:
        sys.exit(run_identical(cfg))
    app = App(cfg)

    def on_sig(signum, frame):
        LOG.info("signal %s", signum)
        app._running = False
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    app.setup()
    app.run()


if __name__ == "__main__":
    main()
