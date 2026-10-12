#!/usr/bin/env python3
"""Ford PHF (binary Intel-HEX variant) -> raw binary image.

Record: 0x3A, len(1), addr(2, BE), type(1), data(len), checksum(1)
Types: 00 data, 01 EOF, 02 ext segment addr, 04 ext linear addr (same as Intel HEX)
"""
import sys, struct

def parse(path):
    data = open(path, 'rb').read()
    hdr_end = data.find(b'$')
    header = data[:hdr_end].decode('latin1')
    pos = hdr_end + 1
    base = 0
    mem = {}
    recs = {}
    bad = 0
    while pos < len(data):
        if data[pos] != 0x3A:
            # tolerate stray whitespace/newlines
            if data[pos] in (0x0A, 0x0D, 0x20):
                pos += 1
                continue
            print(f"unexpected byte {data[pos]:02x} at {pos:#x}", file=sys.stderr)
            pos += 1
            bad += 1
            continue
        ln = data[pos+1]
        addr = struct.unpack('>H', data[pos+2:pos+4])[0]
        typ = data[pos+4]
        payload = data[pos+5:pos+5+ln]
        cks = data[pos+5+ln]
        calc = (-(sum(data[pos+1:pos+5+ln]))) & 0xFF
        if calc != cks:
            bad += 1
            print(f"checksum mismatch at {pos:#x}: got {cks:02x} want {calc:02x}", file=sys.stderr)
        recs[typ] = recs.get(typ, 0) + 1
        if typ == 0x00:
            a = base + addr
            for i, b in enumerate(payload):
                mem[a+i] = b
        elif typ == 0x01:
            pass
        elif typ == 0x02:
            base = struct.unpack('>H', payload)[0] << 4
        elif typ == 0x04:
            base = struct.unpack('>H', payload)[0] << 16
        else:
            print(f"unknown record type {typ:02x} at {pos:#x} payload={payload.hex()}", file=sys.stderr)
        pos += 5 + ln + 1
    return header, mem, recs, bad

def ranges(mem):
    keys = sorted(mem)
    out = []
    start = prev = keys[0]
    for k in keys[1:]:
        if k != prev + 1:
            out.append((start, prev))
            start = k
        prev = k
    out.append((start, prev))
    return out

if __name__ == '__main__':
    src, dst = sys.argv[1], sys.argv[2]
    header, mem, recs, bad = parse(src)
    print("record types:", {f"{k:02x}": v for k, v in recs.items()}, "bad:", bad)
    rs = ranges(mem)
    print("address ranges:")
    for a, b in rs:
        print(f"  {a:08x}-{b:08x}  ({b-a+1} bytes)")
    lo, hi = rs[0][0], rs[-1][1]
    img = bytearray(b'\xff' * (hi + 1))
    for a, v in mem.items():
        img[a] = v
    open(dst, 'wb').write(img)
    print(f"wrote {dst}: {len(img)} bytes (0x0-{hi:#x}), filled from {lo:#x}")
    # Ford 16-bit checksum of all data bytes?
    s16 = sum(mem.values()) & 0xFFFF
    print(f"sum16 of data bytes = {s16:#06x}")
