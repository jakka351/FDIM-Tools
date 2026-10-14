#!/bin/bash
# Sweep the audio-source field (0x2F2 byte 6) of the mono unit and render one screen per source.
# usage: ./lo_sources.sh path/to/fdim-emu path/to/8R29-14D017-AC-app.bin
EMU=${1:-../emu/fdim-emu}; BIN=${2:-8R29-14D017-AC-app.bin}
for v in 1 2 3 4 5 6 7; do
  d=out_src_$v; rm -rf $d; mkdir -p $d; cp lo_eeprom_firstboot.bin $d/eeprom.bin
  python3 -c "
for t in range(300, 3000, 100):
    print(t, '406', '00 04 00 00 00 00 00 00')
    print(t+10, '2F2', '00 00 00 00 00 00 %02x 00' % $v)
    print(t+20, '311', '00 00 00 00 00 00 00 01')" > $d/stim.can
  $EMU $BIN -ms 2600 -can $d/stim.can -adcall 430 -out $d -quiet -log $d/run.log -eeprom $d/eeprom.bin >/dev/null 2>&1
  echo "source $v -> $d/final.pgm"
done
python3 ../emu/render_lo.py out_src_*/final.pgm -scale 2 -crop -sheet lo_sources.png
