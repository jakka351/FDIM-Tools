#!/usr/bin/env python3
"""Decode the FDIM application's data tables: PNET image directory, fonts, pages/lines/fields,
interrupt vector map, CAN config, diagnostic strings.  Usage: decode_tables.py fdim.bin [what]
what = all | dir | fonts | pages | vectors | can | strings
"""
import sys, struct, json, re, collections

img = open(sys.argv[1], 'rb').read()
what = sys.argv[2] if len(sys.argv) > 2 else 'all'
u16 = lambda a: struct.unpack_from('<H', img, a)[0]
s16 = lambda a: struct.unpack_from('<h', img, a)[0]
u32 = lambda a: struct.unpack_from('<I', img, a)[0]

PNET = img.find(b'PNET', 0x40000)       # embedded image directory (the one at 0x3ffdc is the empty default)
FONT_COUNT, FONT_TABLE_PTR = 0x4089e, 0x408a0
PAGE_TABLE, PAGE_COUNT = 0x45ae4, 0x6a
VECTOR_BASE = 0x4400
CAN_CFG = 0x3f35e

def directory():
    n = u16(PNET + 4); base = PNET + 8; out = []
    for i in range(n):
        e = img[base + 12*i: base + 12*i + 12]
        out.append({'idx': i, 'w': ((e[4] & 0x3F) + 1) * 8, 'h': e[5] + 1, 'addr': struct.unpack('>I', e[6:10])[0], 'raw': e.hex()})
    return out

def fonts():
    nf = u16(FONT_COUNT); ftab = u32(FONT_TABLE_PTR); out = []
    for fi in range(nf):
        fb = u32(ftab + 4*fi); glyphs = {}
        for g in range(96):
            gid = u16(fb + 4*g); adv = u16(fb + 4*g + 2)
            glyphs[chr(0x20 + g)] = {'image_id': gid, 'dir_idx': gid - 20000 if gid >= 20000 else gid, 'advance': adv}
        out.append({'font': fi, 'table': fb, 'height_px': 24 if fi == 3 else 16, 'glyphs': glyphs})
    return out

def pages():
    out = []
    for p in range(PAGE_COUNT):
        pa = PAGE_TABLE + 8*p; en = u16(pa); nl = u16(pa + 2); lp = u32(pa + 4); lines = []
        for l in range(nl):
            la = lp + 12*l; nf = u16(la); fp = u32(la + 8); fields = []
            for f in range(nf):
                fa = fp + 12*f
                fields.append({'x0': s16(fa), 'y': s16(fa + 2), 'a': u16(fa + 4), 'b': u16(fa + 6), 'x1': s16(fa + 8), 'c': u16(fa + 10), 'raw': img[fa:fa+12].hex()})
            lines.append({'line': l, 'raw': img[la:la+12].hex(), 'fields': fields})
        out.append({'page': p, 'enabled': en, 'lines': lines})
    return out

def vectors():
    def sext22(v): return v - (1 << 22) if v & (1 << 21) else v
    out = []
    for v in range(0x80, 0x5A0, 0x10):
        slot = VECTOR_BASE + v; w0 = u16(slot); w1 = u16(slot + 2)
        tgt = slot + sext22(((w0 & 0x3F) << 16) | (w1 & 0xFFFE)); h = u32(slot + 4); pri = img[slot + 15]
        kind = 'shared-ISR-entry' if tgt == 0x33526 else ('default' if tgt == 0x335d4 else f'direct:{tgt:06x}')
        out.append({'vector': v, 'ic_reg': 0xFFFFF110 + ((v - 0x80) // 0x10) * 2, 'entry': kind, 'handler': h, 'priority': pri})
    return [o for o in out if o['entry'] != 'default' or o['handler'] not in (0, 0xFFFFFFFF)]

def can_cfg():
    ie, brp, btr = u16(CAN_CFG), u16(CAN_CFG + 2), u16(CAN_CFG + 4)
    sjw = ((btr >> 12) & 3) + 1; tseg2 = ((btr >> 8) & 7) + 1; tseg1 = (btr & 0xF) + 1
    tq = 1 + tseg1 + tseg2
    return {'C1IE': ie, 'C1BRP': brp, 'C1BTR': btr, 'tq_per_bit': tq, 'bitrate_at_20MHz': 20e6 / (brp + 1) / tq}

def strings():
    return [{'addr': m.start(), 's': m.group().decode()} for m in re.finditer(rb'[\x20-\x7e]{5,}', img[0x4d900:0x4f400])]

res = {}
if what in ('all', 'dir'): d = directory(); res['directory'] = {'count': len(d), 'image_rom_bytes': max(e['addr'] + e['w']*e['h']*2 for e in d), 'entries': d}
if what in ('all', 'fonts'): res['fonts'] = fonts()
if what in ('all', 'pages'): res['pages'] = pages()
if what in ('all', 'vectors'): res['vectors'] = vectors()
if what in ('all', 'can'): res['can'] = can_cfg()
if what in ('all', 'strings'): res['strings'] = strings()
json.dump(res, sys.stdout, indent=1)
print()
