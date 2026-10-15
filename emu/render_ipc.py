#!/usr/bin/env python3
"""Render the FG IPC message-centre LCD from the emulator's final_lcdram.bin.
The panel is mounted rotated 270 deg + mirrored (controller ADC/COM reverse),
and the 4-grey data is stored as two bit-planes per 8-line page (low nibble =
plane 0, high nibble = plane 1). usage: render_ipc.py final_lcdram.bin out.png [scale]"""
import sys
from PIL import Image
ram = open(sys.argv[1], 'rb').read()
out = sys.argv[2] if len(sys.argv) > 2 else 'ipc_lcd.png'
S = int(sys.argv[3]) if len(sys.argv) > 3 else 6
COLS, W, PAGES = 256, 180, 17
im = Image.new('L', (W, PAGES * 4), 0); px = im.load()
LEVEL = [0, 110, 190, 255]
for c in range(W):
    for p in range(PAGES):
        b = ram[p * COLS + c]
        for k in range(4):                       # 4 display lines per page, 2 bit-planes
            g = ((b >> k) & 1) | (((b >> (k + 4)) & 1) << 1)
            px[c, p * 4 + k] = LEVEL[g]
im = im.rotate(-90, expand=True).transpose(Image.FLIP_LEFT_RIGHT)   # 270 + mirror -> upright
im = im.resize((im.width * S, im.height * S), Image.NEAREST)
im.save(out); print(f"{out}: {im.size[0]}x{im.size[1]} (logical {PAGES*4}x{W} before rotate)")
