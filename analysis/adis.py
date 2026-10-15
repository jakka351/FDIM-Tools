#!/usr/bin/env python3
"""Annotated disassembly: resolves r0-relative / movhi+disp / mov imm32 addresses to SJ3 SFR names."""
import sys, os, re, json, struct, pypcode

IMG = open(sys.argv[1], 'rb').read()
REGS = {int(k, 16): v for k, v in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sj3_regs.json'))).items()}
# CAN1 register names (offset from 0x03FEC600) per aFCAN map
CAN = {0x000:'C1GMCTRL',0x002:'C1GMCS',0x006:'C1GMABT',0x008:'C1GMABTD',0x040:'C1MASK1L',0x042:'C1MASK1H',0x044:'C1MASK2L',0x046:'C1MASK2H',
       0x048:'C1MASK3L',0x04A:'C1MASK3H',0x04C:'C1MASK4L',0x04E:'C1MASK4H',0x050:'C1CTRL',0x052:'C1LEC',0x053:'C1INFO',0x054:'C1ERC',
       0x056:'C1IE',0x058:'C1INTS',0x05A:'C1BRP',0x05C:'C1BTR',0x05E:'C1LIPT',0x060:'C1RGPT',0x062:'C1LOPT',0x064:'C1TGPT',0x066:'C1TS'}
def name(addr):
    addr &= 0xFFFFFFFF
    if addr in REGS: return REGS[addr][0]
    if 0x03FEC600 <= addr < 0x03FECC00:
        off = addr - 0x03FEC600
        if off in CAN: return CAN[off]
        if off >= 0x100:
            mb, o = divmod(off - 0x100, 0x20)
            fld = {0:'MDATA0',1:'MDATA1',2:'MDATA2',3:'MDATA3',4:'MDATA4',5:'MDATA5',6:'MDATA6',7:'MDATA7',8:'MDLC',9:'MCONF',0xA:'MIDL',0xC:'MIDH',0xE:'MCTRL'}.get(o, f'+{o:x}')
            return f'C1MB{mb}.{fld}'
    if 0x03FEC000 <= addr < 0x03FEC600: return f'CAN0+{addr-0x03FEC000:x}'
    if 0x03FF7000 <= addr <= 0x03FFEFFF: return f'RAM_{addr:08x}'
    if 0xFFFF7000 <= addr <= 0xFFFFEFFF: return f'RAM_{(addr & 0x03FFFFFF):08x}'
    if 0x4400 <= addr < 0x5E000: return f'FLASH_{addr:05x}'
    return None

ctx = pypcode.Context("V850:LE:32:default")
def sext16(v): return v - 0x10000 if v & 0x8000 else v
reg_re = re.compile(r'^(?:0x)?(-?[0-9a-fx]+), (r\d+|sp|gp|tp|ep|lp), (r\d+|sp|gp|tp|ep|lp)$')
mem_re = re.compile(r'(-?0x[0-9a-f]+)\[(r\d+|sp|gp|tp|ep|lp)\]')

def dis(start, end=None, count=None, out=sys.stdout):
    a = start; n = 0; hi = {}
    while (end is None or a < end) and (count is None or n < count):
        try:
            r = ctx.disassemble(IMG[a:a+8], base_address=a, max_instructions=1)
            ins = r.instructions[0]
        except Exception:
            print(f"{a:06x}: ???? {IMG[a:a+2].hex()}", file=out); a += 2; n += 1; continue
        mn, body = ins.mnem, ins.body
        note = ''
        if mn == 'movhi':
            m = reg_re.match(body)
            if m and m.group(2) == 'r0':
                hi[m.group(3)] = int(m.group(1), 16) << 16
            elif m: hi.pop(m.group(3), None)
        elif mn == 'movea':
            m = reg_re.match(body)
            if m:
                imm, r1, r2 = m.groups()
                if r1 in hi:
                    v = (hi[r1] + sext16(int(imm, 16))) & 0xFFFFFFFF
                    hi[r2] = v; nm = name(v); note = f"; {r2} = {v:#x}" + (f" {nm}" if nm else '')
                else: hi.pop(r2, None)
        elif mn == 'mov':
            m = re.match(r'^(0x[0-9a-f]+), (r\d+|sp|gp|tp|ep|lp)$', body)
            if m:
                v = int(m.group(1), 16); hi[m.group(2)] = v; nm = name(v)
                if nm: note = f"; {nm}"
            else:
                m = re.match(r'^(r\d+|sp|gp|tp|ep|lp), (r\d+|sp|gp|tp|ep|lp)$', body)
                if m:
                    if m.group(1) in hi: hi[m.group(2)] = hi[m.group(1)]
                    else: hi.pop(m.group(2), None)
        elif mn.startswith(('ld.', 'st.', 'sld', 'sst', 'set1', 'clr1', 'not1', 'tst1')):
            m = mem_re.search(body)
            if m:
                d = int(m.group(1), 16); base = m.group(2)
                v = None
                if base == 'r0': v = d & 0xFFFFFFFF
                elif base in hi: v = (hi[base] + d) & 0xFFFFFFFF
                if v is not None:
                    nm = name(v); note = f"; [{v:#x}]" + (f" {nm}" if nm else '')
            if mn.startswith(('ld', 'sld')):
                hi.pop(body.split(',')[-1].strip(), None)
        elif mn in ('jarl',):
            for r in ['r1'] + [f'r{i}' for i in range(6, 20)]: hi.pop(r, None)
        else:
            parts = [p.strip() for p in body.split(',')]
            if parts and re.match(r'^(r\d+|sp|gp|tp|ep|lp)$', parts[-1]) and mn not in ('cmp', 'tst'):
                hi.pop(parts[-1], None)
        raw = IMG[a:a+ins.length].hex()
        print(f"{a:06x}: {raw:<12} {mn:<8} {body:<28} {note}", file=out)
        a += ins.length; n += 1
    return a

if __name__ == '__main__':
    for arg in sys.argv[2:]:
        s, _, e = arg.partition('-')
        s = int(s, 16)
        if e.startswith('+'): dis(s, count=int(e[1:]))
        elif e: dis(s, end=int(e, 16))
        else: dis(s, count=40)
        print()
