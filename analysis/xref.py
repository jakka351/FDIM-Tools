#!/usr/bin/env python3
"""Linear-sweep the image, resolve movhi/movea/ld/st pairs to absolute addresses,
and dump cross-references for peripheral regions."""
import sys, re, collections, pypcode

IMG = open(sys.argv[1], 'rb').read()
BASE, END = (int(sys.argv[3], 16), int(sys.argv[4], 16)) if len(sys.argv) > 4 else (0x4400, 0x5E000)
ctx = pypcode.Context("V850:LE:32:default")

# linear sweep, recovering from undecodable with +2
insns = []
a = BASE
while a < END:
    try:
        r = ctx.disassemble(IMG[a:a+8], base_address=a, max_instructions=1)
        if not r.instructions:
            a += 2; continue
        ins = r.instructions[0]
        insns.append((a, ins.length, ins.mnem, ins.body))
        a += ins.length
    except Exception:
        a += 2

def sext16(v): return v - 0x10000 if v & 0x8000 else v

# track movhi results per register within a small window
refs = collections.defaultdict(list)  # abs addr -> list of (pc, text)
hi = {}  # reg -> (value, pc)
reg_re = re.compile(r'^(?:0x)?([0-9a-f]+), (r\d+|sp|gp|tp|ep|lp), (r\d+|sp|gp|tp|ep|lp)$')
mem_re = re.compile(r'^(.*?)(-?0x[0-9a-f]+)\[(r\d+|sp|gp|tp|ep|lp)\](.*)$')
for pc, ln, mn, body in insns:
    if mn == 'movhi':
        m = reg_re.match(body)
        if m:
            imm, r1, r2 = m.groups()
            if r1 == 'r0':
                hi[r2] = (int(imm, 16) << 16, pc)
            else:
                hi.pop(r2, None)
        continue
    if mn == 'movea':
        m = reg_re.match(body)
        if m:
            imm, r1, r2 = m.groups()
            if r1 in hi:
                addr = (hi[r1][0] + sext16(int(imm, 16))) & 0xFFFFFFFF
                refs[addr].append((pc, f"movea -> {r2}"))
                hi[r2] = (addr, pc)   # now r2 holds an absolute pointer; keep as base with disp 0
            else:
                hi.pop(r2, None)
        continue
    if mn in ('ld.b','ld.h','ld.w','ld.bu','ld.hu','st.b','st.h','st.w'):
        m = mem_re.match(body)
        if m:
            pre, disp, base, post = m.groups()
            d = int(disp, 16)
            if base == 'r0':
                addr = d & 0xFFFFFFFF
                refs[addr].append((pc, f"{mn} {body}"))
            elif base in hi:
                addr = (hi[base][0] + d) & 0xFFFFFFFF
                refs[addr].append((pc, f"{mn} {body}  (base {base}={hi[base][0]:#x})"))
        # destination register of a load no longer holds a hi value
        if mn.startswith('ld'):
            dst = body.split(',')[-1].strip()
            hi.pop(dst, None)
        continue
    if mn == 'mov':
        # mov imm32, reg
        m = re.match(r'^(0x[0-9a-f]+), (r\d+|sp|gp|tp|ep|lp)$', body)
        if m:
            v = int(m.group(1), 16)
            hi[m.group(2)] = (v, pc)
            if 0x03FE0000 <= v <= 0x03FFFFFF or v >= 0xFFFF0000 or 0x00100000 <= v < 0x03FE0000:
                refs[v].append((pc, f"mov imm32 -> {m.group(2)}"))
            continue
        m = re.match(r'^(r\d+|sp|gp|tp|ep|lp), (r\d+|sp|gp|tp|ep|lp)$', body)
        if m:
            s, d = m.groups()
            if s in hi: hi[d] = hi[s]
            else: hi.pop(d, None)
        continue
    # any other instruction writing a register invalidates it (crudely: last operand)
    if mn in ('jarl','jmp','jr'):
        hi.clear() if mn != 'jarl' else hi.pop('lp', None)
        if mn == 'jarl':
            # calls clobber r1,r6-r19 (caller-saved) in V850 ABI
            for r in ['r1'] + [f'r{i}' for i in range(6, 20)]:
                hi.pop(r, None)
        continue
    parts = [p.strip() for p in body.split(',')]
    if parts and re.match(r'^(r\d+|sp|gp|tp|ep|lp)$', parts[-1]):
        hi.pop(parts[-1], None)

import pickle
pickle.dump((insns, dict(refs)), open(sys.argv[2], 'wb'))

def region(name, lo, hi_):
    print(f"\n== {name} [{lo:#x}-{hi_:#x}] ==")
    for addr in sorted(refs):
        if lo <= addr <= hi_:
            sites = refs[addr]
            kinds = collections.Counter(t.split()[0] for _, t in sites)
            print(f"  {addr:08x} x{len(sites):<3} {dict(kinds)}  e.g. {sites[0][0]:06x}")

region("SFR area", 0xFFFFF000, 0xFFFFFFFF)
region("mirrored RAM/hi vars", 0xFFFF8000, 0xFFFFEFFF)
region("CAN / PPA", 0x03FEC000, 0x03FEFFFF)
region("internal RAM 0x03FFxxxx", 0x03FF0000, 0x03FFEFFF)
region("ext/unknown 0x00100000-0x03FEBFFF", 0x00100000, 0x03FEBFFF)
region("low flash/boot 0x0-0x43ff", 0x0, 0x43FF)
