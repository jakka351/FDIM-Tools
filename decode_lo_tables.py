#!/usr/bin/env python3
"""Decode the data tables of the FDIM "Low Display" (8R29-14D017-AC, monochrome) application image.

usage: decode_lo_tables.py lo.bin [outdir]

Writes to outdir (default ./gfx-lo):
  images.json / img_<desc>_<w>x<h>.png   every embedded bitmap (row-major 1-bpp, MSB first), found by scanning for
                                          16-byte image descriptors {u16 w, u16 h, u16 stride, u16 bpp, u32 data, u32 aux, ...}
  sheet_all.png                           contact sheet of all bitmaps
  fonts.json / fonts.png                  the four bitmap fonts (glyph table entries {u8 w, u8 advance, u16 stride, u32 data})
  tables.json                             compact vector table, CAN0 mailbox -> RAM buffer map, strings
"""
import sys, os, struct, json, re
from PIL import Image, ImageDraw

img = open(sys.argv[1], 'rb').read()
out = sys.argv[2] if len(sys.argv) > 2 else 'gfx-lo'
os.makedirs(out, exist_ok=True)
u16 = lambda a: struct.unpack_from('<H', img, a)[0]
u32 = lambda a: struct.unpack_from('<I', img, a)[0]
GP = 0x0400400C

def render_bitmap(w, h, stride, ptr):
    im = Image.new('L', (w, h), 0); px = im.load()
    for y in range(h):
        row = img[ptr + y * stride: ptr + (y + 1) * stride]
        for x in range(w):
            if (row[x >> 3] >> (7 - (x & 7))) & 1: px[x, y] = 255
    return im

# ---------------------------------------------------------------- image descriptors
descs = []
for a in range(0x4800, len(img) - 12, 2):
    w, h, st, bpp = struct.unpack_from('<HHHH', img, a)
    if not (1 <= w <= 256 and 1 <= h <= 100 and bpp in (1, 2, 4, 8)) or st != (w * bpp + 7) // 8: continue
    ptr = u32(a + 8)
    if not (0x4800 <= ptr < len(img) and ptr + st * h <= len(img)): continue
    descs.append(dict(desc=a, w=w, h=h, stride=st, bpp=bpp, data=ptr, aux=u32(a + 12)))
tiles = []
for d in descs:
    im = render_bitmap(d['w'], d['h'], d['stride'], d['data'])
    im.save(f"{out}/img_{d['desc']:06x}_{d['w']}x{d['h']}.png"); tiles.append((d, im))
cols, cw, rh = 4, 260, 115
sheet = Image.new('L', (cols * cw, ((len(tiles) + cols - 1) // cols) * rh), 64); dr = ImageDraw.Draw(sheet)
for i, (d, im) in enumerate(tiles):
    x, y = (i % cols) * cw, (i // cols) * rh
    sheet.paste(im, (x + 2, y + 12)); dr.text((x + 2, y), f"{d['desc']:06x} {im.width}x{im.height}", fill=255)
sheet.save(f'{out}/sheet_all.png')
json.dump([{k: (f"{v:06x}" if k in ('desc', 'data', 'aux') else v) for k, v in d.items()} for d in descs], open(f'{out}/images.json', 'w'), indent=1)
print(f"{len(descs)} bitmaps")

# ---------------------------------------------------------------- fonts
def glyph(a):
    w, adv, st, ptr = struct.unpack_from('<BBHI', img, a)
    return (w, adv, st, ptr) if (1 <= w <= 40 and 1 <= adv <= 40 and st == (w + 7) // 8 and 0x4800 <= ptr < len(img)) else None
fonts = []; a = 0x4800
while a < len(img) - 8:
    if glyph(a):
        b = a
        while b < len(img) - 8 and glyph(b): b += 8
        n = (b - a) // 8
        if n >= 40:
            hs = {}
            for k in range(n - 1):
                p, q = glyph(a + 8 * k), glyph(a + 8 * k + 8)
                if q[3] > p[3] and p[2]: h = (q[3] - p[3]) // p[2]; hs[h] = hs.get(h, 0) + 1
            fonts.append((a, n, max(hs, key=hs.get)))
        a = b
    else: a += 2
meta = {}
fsheet = Image.new('L', (1000, len(fonts) * 60), 64); dr = ImageDraw.Draw(fsheet)
for fi, (tab, n, h) in enumerate(fonts):
    x, y = 2, fi * 60; dr.text((x, y), f"font @{tab:06x} h={h} n={n}", fill=255); y += 12; glyphs = []
    for k in range(n):
        w, adv, st, ptr = glyph(tab + 8 * k)
        im = render_bitmap(w, h, st, ptr); glyphs.append(dict(ch=chr(0x20 + k), w=w, adv=adv, stride=st, data=f"{ptr:06x}"))
        if x + w > 995: x = 2; y += h + 2
        fsheet.paste(im, (x, y)); x += adv + 1
    meta[f"{tab:06x}"] = dict(height=h, first=0x20, count=n, glyphs=glyphs)
fsheet.save(f'{out}/fonts.png'); json.dump(meta, open(f'{out}/fonts.json', 'w'), indent=1)
print(f"{len(fonts)} fonts: " + ', '.join(f"@{t:06x} h={h} n={n}" for t, n, h in fonts))

# ---------------------------------------------------------------- vector table, CAN map, strings
def sext22(v): return v - (1 << 22) if v & (1 << 21) else v
vectors = []
for k in range(0, 0x170 // 4):
    slot = 0x4500 + 4 * k; w0, w1 = u16(slot), u16(slot + 2)
    if (w0 & 0xFFC0) == 0x0780:
        tgt = slot + sext22(((w0 & 0x3F) << 16) | (w1 & 0xFFFE))
        vectors.append(dict(slot=f"{slot:06x}", vector=f"{0x10 * (k + 1):03x}" if k else "reset", handler=f"{tgt:06x}"))
canmap = []
for mb in range(1, 30):
    slot = u16(0x4A48 + 2 * (mb + 7)); buf = u32(0x4AB8 + 4 * slot)
    canmap.append(dict(mailbox=mb, slot=slot, ram=f"{buf:08x}", gp_offset=f"-0x{GP - buf:x}" if buf else None))
strings = [dict(addr=f"{m.start():06x}", s=m.group().decode()) for m in re.finditer(rb'[\x20-\x7e]{5,}', img[0x5700:0x18200])]
json.dump(dict(vectors=vectors, can_rx_buffers=canmap, strings=strings), open(f'{out}/tables.json', 'w'), indent=1)
print(f"{len(vectors)} vector slots, {len(strings)} strings -> {out}/")
