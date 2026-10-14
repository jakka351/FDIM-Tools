#!/usr/bin/env python3
"""Convert the mono-unit emulator's frame_*.pgm / final.pgm (240x84, 4 grey levels) to PNG.

usage: render_lo.py frame.pgm [more.pgm ...] [-scale N] [-style white|blue|paper] [-sheet out.png] [-crop]
  -crop   keep only the 80 display lines (drop the icon-line page at the bottom)
  -sheet  tile all frames into one contact sheet instead of one PNG per frame
"""
import sys, argparse
from PIL import Image, ImageDraw

ap = argparse.ArgumentParser()
ap.add_argument('frames', nargs='+'); ap.add_argument('-scale', type=int, default=3)
ap.add_argument('-style', default='white', choices=['white', 'blue', 'paper']); ap.add_argument('-sheet', default=None)
ap.add_argument('-crop', action='store_true')
a = ap.parse_args()

def colorize(im):
    g = im.convert('L')
    if a.style == 'white': return g.convert('RGB')
    lut = []
    for v in range(256):
        t = v / 255
        if a.style == 'blue': lut.append((int(8 + 40 * t), int(16 + 120 * t), int(60 + 195 * t)))
        else: lut.append((int(200 - 170 * t), int(210 - 180 * t), int(190 - 170 * t)))   # positive "paper" LCD
    r = Image.new('RGB', g.size); px = r.load(); gp = g.load()
    for y in range(g.height):
        for x in range(g.width): px[x, y] = lut[gp[x, y]]
    return r

outs = []
for fp in a.frames:
    im = Image.open(fp)
    if a.crop: im = im.crop((0, 0, im.width, 80))
    im = colorize(im).resize((im.width * a.scale, im.height * a.scale), Image.NEAREST)
    outs.append((fp, im))
if a.sheet:
    cols = 2 if len(outs) > 1 else 1; W, H = outs[0][1].size; rows = (len(outs) + cols - 1) // cols
    sheet = Image.new('RGB', (cols * (W + 8), rows * (H + 20)), (90, 90, 90)); dr = ImageDraw.Draw(sheet)
    for i, (fp, im) in enumerate(outs):
        x, y = (i % cols) * (W + 8), (i // cols) * (H + 20)
        dr.text((x + 2, y + 2), fp.split('/')[-1], fill=(255, 255, 255)); sheet.paste(im, (x, y + 16))
    sheet.save(a.sheet); print(a.sheet, sheet.size)
else:
    for fp, im in outs:
        o = fp.rsplit('.', 1)[0] + '.png'; im.save(o); print(o)
