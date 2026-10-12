#!/usr/bin/env python3
"""Compose FDIM frames from the emulator's JSON object dumps.

Uses the firmware's embedded PNET image directory + font tables to classify each object:
  - glyph images -> drawn as the real character with a substitute font
  - other images  -> placeholder box (hashed colour) labelled with the directory index
If an image ROM dump is available (-rom), real RGB565 pixels are used instead.
"""
import sys, json, struct, argparse, os
from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser()
ap.add_argument('app')                       # fdim.bin (application image at offset 0)
ap.add_argument('frames', nargs='+')         # frame_*.ppm.json
ap.add_argument('-rom', default=None)
ap.add_argument('-scale', type=int, default=2)
ap.add_argument('-w', type=int, default=480)
ap.add_argument('-H', type=int, default=234)
ap.add_argument('-out', default=None)
args = ap.parse_args()

img = open(args.app, 'rb').read()
u16 = lambda a: struct.unpack_from('<H', img, a)[0]
u32 = lambda a: struct.unpack_from('<I', img, a)[0]

# PNET directory
hdr = img.find(b'PNET', 0x40000)
n = u16(hdr + 4); base = hdr + 8
entries = []
addr2idx = {}
for i in range(n):
    e = img[base + 12*i: base + 12*i + 12]
    w = ((e[4] & 0x3F) + 1) * 8; h = e[5] + 1; addr = struct.unpack('>I', e[6:10])[0]
    entries.append((w, h, addr))
    addr2idx.setdefault((addr, w, h), i)

# fonts: count @0x4089e, pointer table @0x4088c -> 96 glyph entries {u16 image_id, u16 advance}
nf = u16(0x4089e); ftab = u32(0x408a0)
glyph_of = {}   # image index -> (font, char, advance)
for fi in range(nf):
    fbase = u32(ftab + 4*fi)
    for gidx in range(96):
        gid = u16(fbase + 4*gidx); adv = u16(fbase + 4*gidx + 2)
        idx = gid - 20000 if gid >= 20000 else gid
        if idx >= 30000: idx -= 30000
        if 0 <= idx < n: glyph_of.setdefault(idx, (fi, chr(0x20 + gidx), adv))

rom = open(args.rom, 'rb').read() if args.rom and os.path.exists(args.rom) else None

def font_for(h):
    for name in ('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'):
        if os.path.exists(name):
            return ImageFont.truetype(name, max(6, int(h * 0.95)))
    return ImageFont.load_default()

def hashcol(addr):
    v = (addr * 2654435761) & 0xFFFFFFFF
    return (60 + (v >> 24) // 2, 60 + ((v >> 16) & 0xFF) // 2, 80 + ((v >> 8) & 0xFF) // 2)

for fp in args.frames:
    d = json.load(open(fp))
    regs = d['regs']
    bg565 = (regs[0x15] << 8) | regs[0x16]
    bg = (((bg565 >> 11) << 3), (((bg565 >> 5) & 0x3F) << 2), ((bg565 & 0x1F) << 3))
    W, H = args.w, args.H
    canvas = Image.new('RGB', (W, H), bg)
    dr = ImageDraw.Draw(canvas)
    bank = (regs[0x0D] & 7)
    objs = d['objects']
    # choose displayed bank: objects 0..169 or 170..339 (reg 0x0D bit pattern unknown; show whichever bank has objects, prefer the one matching bit0)
    for o in objs:
        x, y, w, h, addr = o['x'], o['y'], o['w'], o['h'], o['addr']
        key = (addr, w, h)
        idx = addr2idx.get(key)
        if rom is not None and addr + w*h*2 <= len(rom):
            tile = Image.frombytes('RGB', (w, h), b''.join(
                bytes((((px >> 11) << 3), (((px >> 5) & 0x3F) << 2), ((px & 0x1F) << 3)))
                for px in struct.unpack('>%dH' % (w*h), rom[addr:addr + w*h*2])))
            canvas.paste(tile, (x, y))
            continue
        if idx is not None and idx in glyph_of:
            fi, ch, adv = glyph_of[idx]
            gh = 24 if fi == 3 else 16
            f = font_for(gh)
            col = [(255, 255, 255), (255, 230, 120), (160, 200, 255), (255, 255, 255)][fi]
            dr.text((x, y), ch, fill=col, font=f)
        else:
            col = hashcol(addr)
            dr.rectangle([x, y, x + w - 1, y + h - 1], fill=col, outline=(200, 200, 200))
            if w >= 24 and h >= 9:
                dr.text((x + 2, y + 1), f"#{idx}" if idx is not None else f"{addr:06x}", fill=(255, 255, 255), font=font_for(min(h, 12)))
    if args.scale != 1:
        canvas = canvas.resize((W * args.scale, H * args.scale), Image.NEAREST)
    out = args.out or fp.replace('.ppm.json', '.png')
    canvas.save(out)
    print(out, len(objs), 'objects, bank reg', hex(regs[0x0D]), 'bg', hex(bg565))
