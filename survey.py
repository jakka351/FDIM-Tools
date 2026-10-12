#!/usr/bin/env python3
"""First-pass survey of the V850 application image."""
import struct, sys, re, collections

img = open(sys.argv[1], 'rb').read()
BASE = 0x4400
END = 0x5E000

def h16(a): return struct.unpack_from('<H', img, a)[0]
def h32(a): return struct.unpack_from('<I', img, a)[0]

def sext(v, bits):
    return v - (1 << bits) if v & (1 << (bits-1)) else v

# --- vector table: V850 vectors are 16 bytes apart, each should begin with jr disp22
print("== vector table at 0x4400 ==")
vec_names_es = {  # generic V850ES/Fx3-ish naming; verified later against the datasheet
 0x00:'RESET',0x10:'NMI',0x20:'INTWDT2',0x40:'TRAP0x',0x50:'TRAP1x',0x60:'ILGOP/DBG0',0x70:'DBG1(ES)',
}
for i in range(0, 0x400, 0x10):
    a = BASE + i
    w0 = h16(a); w1 = h16(a+2)
    if (w0 & 0x07E0) == 0x0780 and (w0 & 0xF800) == 0:  # jr disp22
        disp = sext(((w0 & 0x3F) << 16) | (w1 & 0xFFFE), 22)
        tgt = a + disp
        print(f"  {a:06x} (vec {i:03x}): jr -> {tgt:06x}")
    else:
        print(f"  {a:06x} (vec {i:03x}): {w0:04x} {w1:04x}  (not jr)")

# --- strings
print("\n== printable strings (>=6) ==")
for m in re.finditer(rb'[\x20-\x7e]{6,}', img):
    print(f"  {m.start():06x}: {m.group().decode()}")

# --- 32-bit immediates loaded with mov imm32,reg (opcode: 0000 0110 001R RRRR + 32-bit imm)
print("\n== mov imm32, reg (top values) ==")
imm = collections.Counter()
for a in range(BASE, END-6, 2):
    w = h16(a)
    if (w & 0xFFE0) == 0x0620:
        v = h32(a+2)
        reg = w & 0x1F
        imm[(v, reg)] += 1
for (v, reg), c in imm.most_common(60):
    print(f"  {v:08x} -> r{reg:<2}  x{c}")

# --- SFR-style absolute accesses: ld/st with reg1=r0 and 16-bit disp (sign-extended)
# Format VII ld.b/ld.h/ld.w/st.b/st.h/st.w : 15..11 reg2, 10..5 opcode(0x38..0x3B), 4..0 reg1, then disp16
print("\n== absolute [r0] accesses (disp16 sign-extended) ==")
abs_acc = collections.Counter()
for a in range(BASE, END-4, 2):
    w = h16(a)
    op = (w >> 5) & 0x3F
    reg1 = w & 0x1F
    if reg1 == 0 and op in (0x38, 0x39, 0x3A, 0x3B):
        d = h16(a+2)
        disp = sext(d & 0xFFFE if op in (0x39,0x3B) else d, 16)  # ld.w/st.w and ld.h/st.h have bit0 as subcode
        addr = disp & 0xFFFFFFFF
        kind = {0x38:'ld.b',0x39:'ld.h/w',0x3A:'st.b',0x3B:'st.h/w'}[op]
        abs_acc[addr] += 1
for addr, c in sorted(abs_acc.items()):
    if c >= 2:
        print(f"  {addr:08x}  x{c}")
print(f"  distinct abs addresses: {len(abs_acc)}")

# --- movhi hi16,r0,reg  (opcode 0x0640|reg2<<11 ... ) : format VI: 15..11 reg2, 10..5 = 110010 (0x32), 4..0 reg1, imm16
print("\n== movhi imm16,r0,rX (upper halves of absolute addresses) ==")
hi = collections.Counter()
for a in range(BASE, END-4, 2):
    w = h16(a)
    if ((w >> 5) & 0x3F) == 0x32 and (w & 0x1F) == 0:
        hi[h16(a+2)] += 1
for v, c in hi.most_common(30):
    print(f"  movhi 0x{v:04x} x{c}")
