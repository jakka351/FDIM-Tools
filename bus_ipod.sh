#!/bin/bash
# FG Falcon audio system on a virtual MS-CAN bus: mono FDIM + AIM + simulated iPod.
# usage: ./bus_ipod.sh [ms]   (run from the examples directory after building ../emu)
MS=${1:-32000}
rm -rf FDIM-AC AIM bus.log ipod_display_image.bin
cp lo_eeprom_ipod.bin fdim_ee.bin; cp aim_eeprom_paired.bin aim_ee.bin
../emu/fdim-emu -module 8R29-14D017-AC-app.bin -adcall 430 -eeprom fdim_ee.bin \
                -module 8R29-14F029-AB-app.bin -eeprom aim_ee.bin -ipod \
                -ms $MS -can lo_aim_bus.can -snap 1000 -quiet
last=$(ls FDIM-AC/frame_*.pgm | tail -1)
python3 ../emu/render_lo.py $last -scale 3 -crop
echo "FDIM screen: ${last%.pgm}.png ; bus traffic: bus.log ; AIM/run.log has the iAP exchange (IPOD RX/TX lines)"
awk '{print $2, $3}' bus.log | sort | uniq -c | sort -rn
