#!/usr/bin/env python3
"""Decode the data tables of the FG Falcon AIM / Front Entertainment Module application (8R29-14F029-AB).

usage: decode_aim_tables.py aim.bin [outdir]

Writes to outdir (default ./gfx-aim):
  tables.json        compact vector table, CAN receive table (ID -> slot, RAM buffer, callback), CAN transmit table,
                     iAP Lingo-4 response list, iPod category strings
  ipod_logo_*.png    the Ford ovals the module uploads to the iPod's screen (SetDisplayImage) in each supported format
"""
import sys, os, struct, json, re
from PIL import Image

img = open(sys.argv[1], 'rb').read()
out = sys.argv[2] if len(sys.argv) > 2 else 'gfx-aim'
os.makedirs(out, exist_ok=True)
u16 = lambda a: struct.unpack_from('<H', img, a)[0]
u32 = lambda a: struct.unpack_from('<I', img, a)[0]
GP = 0x0400400C

def sext22(v): return v - (1 << 22) if v & (1 << 21) else v
vectors = []
for k in range(0, 0x170 // 4):
    slot = 0x4500 + 4 * k; w0, w1 = u16(slot), u16(slot + 2)
    if (w0 & 0xFFC0) == 0x0780:
        tgt = slot + sext22(((w0 & 0x3F) << 16) | (w1 & 0xFFFE))
        vectors.append(dict(slot=f"{slot:06x}", vector=f"{0x10 * (k + 1):03x}" if k else "entry", handler=f"{tgt:06x}"))

# CAN receive table: 10 identifiers (stored as IDH = id << 2) at 0x4B90, slot numbers at 0x4BA4, DLC at 0x4BB8,
# RAM buffer per slot at 0x4BC4, callback per slot at 0x4C14, 'received' flag byte/bit at 0x4C3C/0x4C46
rx = []
for i in range(10):
    idh = u16(0x4B90 + 2 * i); slot = u16(0x4BA4 + 2 * i); buf = u32(0x4BC4 + 4 * slot); cb = u32(0x4C14 + 4 * slot)
    rx.append(dict(id=f"{idh >> 2:03X}", slot=slot, dlc=img[0x4BB8 + slot], ram=f"{buf:08x}" if buf else None, gp_offset=f"-0x{GP - buf:x}" if buf else None,
                   callback=f"{cb:06x}" if cb else None, flag_byte=img[0x4C3C + slot], flag_bit=f"{img[0x4C46 + slot]:02x}"))
rx.append(dict(id="500-57F", note="network-management range handled by 0xE6A8 (own NM id 0x555)"))
# transmit table: IDH at 0x4AB0, DLC at 0x4AD0, data pointer at 0x4AE0 (16 entries)
tx = []
for k in range(16):
    idh = u16(0x4AB0 + 2 * k); dlc = img[0x4AD0 + k]; ptr = u32(0x4AE0 + 4 * k)
    if idh or ptr: tx.append(dict(slot=k, id=f"{idh >> 2:03X}", dlc=dlc, ram=f"{ptr:08x}", gp_offset=f"-0x{GP - ptr:x}"))
# iAP Lingo 4 responses the module waits for: 3-byte entries {lingo 04, cmd hi, cmd lo} at 0x4F50
iap = []
a = 0x4F50
while img[a] == 0x04:
    iap.append(f"{(img[a + 1] << 8) | img[a + 2]:04x}"); a += 3
strings = [dict(addr=f"{m.start():06x}", s=m.group().decode()) for m in re.finditer(rb'[\x20-\x7e]{4,}', img[0x4F90:0x50A0])]
json.dump(dict(vectors=vectors, can_rx=rx, can_tx=tx, iap_lingo4_responses=iap, ipod_strings=strings), open(f'{out}/tables.json', 'w'), indent=1)

# iPod display images (Ford oval): selected by the iPod's reported display limits
def rgb565(a, w, h):
    im = Image.new('RGB', (w, h)); px = im.load()
    for y in range(h):
        for x in range(w):
            v = struct.unpack_from('<H', img, a + (y * w + x) * 2)[0]; px[x, y] = ((v >> 11) << 3, ((v >> 5) & 0x3F) << 2, (v & 0x1F) << 3)
    return im
def grey(a, w, h, bpp):
    im = Image.new('L', (w, h)); px = im.load(); stride = w * bpp // 8
    for y in range(h):
        for x in range(w):
            b = img[a + y * stride + (x * bpp) // 8]
            v = (b >> ((3 - (x % 4)) * 2)) & 3 if bpp == 2 else (b >> (4 if x % 2 == 0 else 0)) & 15
            px[x, y] = 255 - v * (85 if bpp == 2 else 17)
    return im
images = [("ipod_logo_rgb565_120x44", 0x50A0, rgb565(0x50A0, 120, 44)), ("ipod_logo_grey4_152x108", 0x79E0, grey(0x79E0, 152, 108, 4)),
          ("ipod_logo_2bpp_160x62", 0x99F0, grey(0x99F0, 160, 62, 2)), ("ipod_logo_2bpp_128x40", 0xA3A0, grey(0xA3A0, 128, 40, 2))]
for name, addr, im in images:
    im.save(f"{out}/{name}.png")
    print(f"{name}: flash {addr:06x}, {im.size[0]}x{im.size[1]}")
print(f"{len(vectors)} vector slots, {len(rx)} rx ids, {len(tx)} tx slots, {len(iap)} iAP responses -> {out}/")
