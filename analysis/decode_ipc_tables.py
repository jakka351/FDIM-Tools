#!/usr/bin/env python3
"""Decode the data tables of the FG Falcon instrument cluster dump (GS_IPC.bin).

usage: decode_ipc_tables.py GS_IPC.bin [outdir]

Writes <outdir>/tables.json: compact vector table, both CAN channels' bit timing and
RX/TX identifier tables, the callt (kernel service) table, and the message-centre strings.
"""
import sys, os, struct, json, re

img = open(sys.argv[1], 'rb').read()
out = sys.argv[2] if len(sys.argv) > 2 else 'gfx-ipc'
os.makedirs(out, exist_ok=True)
u16 = lambda a: struct.unpack_from('<H', img, a)[0]
u32 = lambda a: struct.unpack_from('<I', img, a)[0]
def sext22(v): return v - (1 << 22) if v & (1 << 21) else v

# compact vector table @0x4000: slot k -> vec 0x10*k (k<=6) / 0x10*(k+1) (k>=7)
SPUR = 0x3D7FE
vectors = []
for k in range((0x41AC - 0x4000) // 4):
    a = 0x4000 + 4 * k; w0, w1 = u16(a), u16(a + 2)
    vec = 0x10 * k if k <= 6 else 0x10 * (k + 1)
    if (w0 & 0xFFC0) == 0x0780:
        tgt = a + sext22(((w0 & 0x3F) << 16) | (w1 & 0xFFFE))
        if tgt != SPUR:
            vectors.append(dict(vector=f"{vec:03x}", handler=f"{tgt:06x}"))

# CAN channel table @0x14574: 2 channels x {regbase, maskbase, mbbase}; timing profiles follow
chan = []
for c in range(2):
    regbase = u32(0x14574 + 8 * c + 0)            # 0x03FEC000 / 0x03FEC600
    prof = img[0x145EE + c]
    brp = u16(0x14590 + 2 * prof); btr = u16(0x14594 + 2 * prof)
    tseg1 = (btr & 0xF) + 1; tseg2 = ((btr >> 8) & 7) + 1
    tq = 16_000_000 / (brp + 1); kbit = tq / (tseg1 + tseg2 + 1) / 1000
    chan.append(dict(channel=c, regbase=f"{regbase:08x}", brp=brp, btr=f"{btr:04x}",
                     kbit_s=round(kbit, 1), bus="MS-CAN" if c == 0 else "HS-CAN"))

# RX exact-ID list @0x14764 (u16 each), split 12 MS + 11 HS; handler table @0x14D28
rx_ids = [u16(0x14764 + 2 * i) for i in range(23)]
rx = dict(ms=[f"{x:03X}" for x in rx_ids[:12]], hs=[f"{x:03X}" for x in rx_ids[12:23]],
          handlers=[f"{u32(0x14D28 + 4 * i):06x}" for i in range(22)])

# TX ID table @0x145F4 (two groups of 8)
tx = dict(group0=[f"{u16(0x145F4 + 2 * i):03X}" for i in range(8)],
          group1=[f"{u16(0x14604 + 2 * i):03X}" for i in range(8)])

# callt kernel-service table @0x3C8A4
CTBP = 0x3C8A4
callt = {}
for n in range(0x40):
    off = u16(CTBP + 2 * n)
    if off and CTBP + off < 0x40000:
        callt[f"{n:02x}"] = f"{CTBP + off:06x}"

# message-centre strings 0x10400..0x11400
strings = [m.group().decode('latin1') for m in re.finditer(rb'[\x20-\x7e]{3,}', img[0x10400:0x11400])]
# de-dup preserving order
seen = set(); uniq = [s for s in strings if not (s in seen or seen.add(s))]

json.dump(dict(vectors=vectors, can_channels=chan, can_rx=rx, can_tx=tx,
               callt_table=callt, message_centre_strings=uniq),
          open(f'{out}/tables.json', 'w'), indent=1)
print(f"{len(vectors)} vectors, 2 CAN channels, {len(rx_ids)} RX ids, "
      f"{len(callt)} callt entries, {len(uniq)} strings -> {out}/tables.json")
for c in chan:
    print(f"  CAN{c['channel']} {c['bus']}: {c['kbit_s']} kbit/s (BRP {c['brp']}, BTR {c['btr']})")
