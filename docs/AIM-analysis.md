# FG Falcon AIM / Front Entertainment Module (8R29-14F029-AB) — analysis

Third module of the FG audio system after the colour FDIM (`FDIM-analysis.md`) and the mono FDIM
(`FDIM-AC-mono-analysis.md`). The PHF header calls it the **Front Entertainment Module (FEM)**, production part
8R29‑14F027‑AA, released 29 July 2008 by Sumitomo Wiring Systems Australia, diagnostic ID 0x767. It is the
**iPod / auxiliary‑input interface**: it talks Apple's iAP protocol to a docked iPod over a UART, routes the audio
through an SPI‑controlled switch, and feeds track information to the FDIM over MS‑CAN. Same hardware platform and
same code base family as the mono FDIM (compact vector table, EEPROM on P4, scheduler slots), which is why the
emulator needed only new peripheral models — and why the two firmwares can now be run **together on a virtual bus**
with a simulated iPod attached, with the FDIM rendering the "iPod" page from text the AIM really sends.

---

## 1. Container and image

* PHF with Intel‑HEX records as before; `FILE CHECKSUM = 0x45D1` matches, no corruption.
* Image 254 156 bytes: application **0x4500–0x1D063**, 2 bytes at 0x3DF00, 204 zero bytes at 0x3E000 (calibration block).
  Header at 0x4700 `{0x2B5, 1, 0x4500, 0x39AFF, 2, 0x3E000, 0x1FFF}` (the mono FDIM has `{0x1B5, 1, …}`; the emulator uses
  byte 0x4701 to tell the two 256 KB boards apart).
* Code 0xAE00–0x1D063, data 0x4800–0xADBA (tables, iPod strings, the iPod logo images), `CTBP = 0xAE00`, `gp = 0x0400400C`,
  `sp = 0x03FFEDD8`, RAM 0x03FFC000–0x03FFEFFF (12 KB).

## 2. MCU, vectors, scheduling

V850ES/Fx3 (FG3 lineage) 256 KB / 12 KB part, 20 MHz. Compact 4‑byte vector table at 0x4500 (slot k = vector 0x10·(k+1)):

| vector | handler | source |
|---|---|---|
| entry | 0xAFE0 | reset: gp/sp/ep, RAM clear, CTBP, init list 0xB0B6, main loop 0xB0D6 |
| 0x120 | 0x1AF66 | timer @0x540 CC0 — 1 ms tick (scheduler slots 10/20/25/30/65/100 ms) |
| 0x170 | 0x1B162 | TMP0 CC0 |
| 0x250 / 0x260 | 0x1ACF0 / 0x1ADBA | CSIB1 transfer complete / transmit empty |
| 0x290 / 0x2A0 | 0x0F36E / 0x0F298 | **UARTA1 receive / transmit** (iPod link; ring buffers at gp‑0x7574 / gp‑0x7448) |
| 0x2B0 | 0x14D26 | INTAD (channel 7 only) |
| 0x2C0–0x2F0 | 0x0CE64 / 0x0CD80 / 0x0CF30 / 0x0D038 | CAN0 ERR / WUP / REC / TRX |
| 0x320 | 0x0BE9C | INTWT (0.25 s) |
| 0x370 | 0x1AE84 | timer @0x610 CC0 — 5 ms |

No RTOS: super‑loop with a millisecond counter dispatching the 10/20/25/30/65/100 ms slots, exactly like the FDIM.

## 3. Peripherals and pins

| function | details |
|---|---|
| **CAN0** | 125 kbit/s (BRP 9, BTR 0x3508). Mailbox 0 = transmit, mailboxes 30/31 = mask buffers that accept *every* identifier; filtering is done in software (section 4). |
| **UARTA1** | 0xFFFFFA10, 57 600 baud 8N1 (CTL1=4, CTL2=0x0B) — **iPod Accessory Protocol**. |
| **CSIB1** | 0xFFFFFD10, 8‑bit SPI, master, transmit only: a control byte (idle value **0xA8**, bits toggled per audio source by 0x1AB94) sent every 500 ms and on every change → audio source switch / mute lines. |
| **EEPROM** | 25xx SPI on P4 (bit2 SCK, bit1 MOSI, bit0 MISO), **/CS = P0 bit4**; same bit‑bang driver as the FDIM (0x1461E/0x1468E/0x14752). First boot writes `[0x0D]=0x80`, `[0x55]=0x01`. Records: 0x00–0x0E header, **0x35–0x3C pairing key**, **0x54 bit0 = pairing disabled/not configured**, 0x55, 0x59, 0x63–0x66 settings. |
| **ADC** | channel 7 only (0x14CF0), raw 10‑bit at gp‑0x7D52; thresholds 0x143 / 0x15C → accessory (iPod / AUX) supply detection. |
| **detect inputs** | P12 bit6 (0 = dock/iPod connected, 0xFA46), PCD bit2 (0xFA5A), P6H bit6 (detect), P6H bit7 = accessory power output, P0 bit2 … |
| **timers** | TMQ0 1 ms, TMQ1 5 ms, TMP0–3 configured (PWM/time bases), WT 0.25 s |

## 4. CAN traffic

### Received (table at 0x4B90 → slot → RAM buffer 0x4BC4[slot], callback 0x4C14[slot])

| ID | buffer | use |
|---|---|---|
| 0x767 | — | physical diagnostic request (UDS‑style handler, responses on 0x76F) |
| **0x317** | gp‑0x7690 | **pairing / security key**: 8 bytes compared with EEPROM 0x35–0x3C (0x115B2; only if EEPROM 0x54 bit0 = 0). Match → mode 1 (iPod functions enabled), mismatch → mode 2 (locked). Nothing happens on the iPod side until this passes. |
| **0x2FC** | gp‑0x7188 | from the FDIM: byte 4 = **display screen ID** (0x21–0x24 select the four iPod page handlers 0x1951C/0x189EC/0x18018/0x176F0: now‑playing and the menu lists), byte 2 bit 5, byte 4 also used as a general "ICC state". |
| **0x2F2** | gp‑0x77F4 | from the head unit: byte 6 bits 0‑2 = **audio source** (same field the FDIM uses; non‑zero runs the accessory state machine 0x110DE), byte 7 bits 4‑7 (value 1 checked by the iPod control code). |
| 0x2E9 | gp‑0x7180 | ISO‑TP flow control from the FDIM for the 0x2E8 channel (`30 08 00`) |
| 0x2E5, 0x2DF, 0x2DC, 0x2D9, 0x2D3 | gp‑0x7178 … | further channels from the FDIM / head unit (flow control for the other data channels) |
| 0x500–0x57F | — | network management (own NM id 0x555) |

### Transmitted (table at 0x4AB0/0x4AD0/0x4AE0)

| ID | period | content |
|---|---|---|
| 0x555 | 1 s | network management (`55 01` / `55 02` alternating) |
| **0x2D4** | 250 ms + on change | **status**: byte 4 bits 4‑5 accessory detect state, byte 5 bits 4‑6, **byte 6 bits 1‑3 = iPod link state** (1 connecting, 3 identified / uploading, …; the FDIM shows the "iPod" label and page only when this is non‑zero), bytes 0‑3 / 7 further fields (setters 0xDD86…0xDEF8). Idle value `00 00 00 00 10 10 00 00`, with the iPod `… 10 10 06 00`. |
| **0x2E8**, 0x2E4, 0x2DE, 0x2DB, 0x2D8, 0x2D2 | on demand | double‑buffered **ISO‑TP data channels** to the FDIM. 0x2E8 carries the now‑playing text: `10 34 00 97 'Turbo Lag Blues' 03 00 25 'Barra Sessions' 03 00 33 'Tester Present' 03 …` — first frame + consecutive frames, the FDIM answers with flow control on 0x2E9. Field tags 0x97 title, 0x25 album, 0x33 artist, 0x03 as terminator. The other channels carry the menu lists ("Playlists", "Artists", "Albums", "Podcasts", "Composers", "Audiobooks" and the "No …" texts at 0x4F98). |
| 0x76F | — | diagnostic responses (two buffers) |

## 5. The iPod link (iAP over UARTA1)

Packet framing `FF 55 len lingo cmd … cks` (0x10014 builds packets, checksum = two's complement of len+payload; long
packets use the 3‑byte length form). Sequence observed with the simulator attached (`-ipod`):

1. Lingo 0: `IdentifyDeviceLingoes` (lingoes 0 General, 2 Simple Remote, 4 Extended Interface — mask 0x15) → ACK;
   RequestiPodName, RequestiPodModelNum, RequestiPodSoftwareVersion, RequestLingoProtocolVersion(4), RequestRemoteUIMode,
   EnterRemoteUIMode.
2. Lingo 4: `GetMonoDisplayImageLimits` (0x0033) then **SetDisplayImage (0x0032)** in 496‑byte telegrams — the module pushes a
   **Ford oval onto the iPod's own screen**. Four variants live in the data block, chosen by the reported display format:

   | flash | format | size |
   |---|---|---|
   | 0x50A0 | RGB565 (colour iPods) | 120×44 |
   | 0x79E0 | 4‑bit greyscale | 152×108 |
   | 0x99F0 | 2‑bit greyscale | 160×62 |
   | 0xA3A0 | 2‑bit greyscale | 128×40 |

   (`gfx-aim/ipod_logo_*.png`; the upload loop sends 0x1F5‑byte chunks starting at 0x50A0 regardless of the chosen
   variant, so ~60 KB go over the wire — a firmware quirk the real iPod simply ignores.)
3. Lingo 2 button reports, then Lingo 4: SetPlayStatusChangeNotification, GetShuffle/GetRepeat (0x002C/0x002F), SetRepeat,
   GetPlayStatus (0x001C) polled every 100 ms, GetCurrentPlayingTrackIndex, GetIndexedPlayingTrackInfo, and the title / album /
   artist requests (0x0020 / 0x0024 / 0x0022) for the current track — which are then packed into the 0x2E8 ISO‑TP message
   for the display. The 23 Lingo‑4 responses the module understands are listed at 0x4F50 (official Apple numbering: 0x0001 ACK,
   0x0013 protocol version, 0x0019 record count, 0x001B records, 0x001D play status, 0x001F track index, 0x0021/23/25
   title/artist/album, 0x0027 notifications, 0x002D/0x0030 shuffle/repeat, 0x0034/0x003A display limits, 0x0036 track count …).

Everything the iPod answers is validated against that table; unanswered requests are retried every 200–500 ms.

## 6. Emulation

`fdim-emu` now runs several modules in lock‑step on a virtual MS‑CAN bus and has models for UARTA1, CSIB1, the second EEPROM
wiring, mask‑mailbox identifier capture (needed for the AIM's software filtering) and a small **iPod simulator** (`emu/ipodsim.c`,
`-ipod`) that implements enough of Lingo 0/2/4 to carry a four‑track library ("Tester Present — Barra Sessions").

```
fdim-emu -module 8R29-14D017-AC-app.bin -adcall 430 -eeprom lo_eeprom_ipod.bin \
         -module 8R29-14F029-AB-app.bin -eeprom aim_eeprom_paired.bin -ipod \
         -ms 32000 -can lo_aim_bus.can -snap 1000 -quiet
python3 ../emu/render_lo.py FDIM-AC/frame_0031.pgm -scale 3 -crop     # the FDIM's "iPod" page
```
`lo_aim_bus.can` provides what the missing head unit would send (0x406 ignition RUN, 0x2F2 source 1, 0x311 audio OK,
0x317 pairing key `46 47 2D 41 49 4D 2D 31`); `aim_eeprom_paired.bin` holds the same key at 0x35 and 0x54 = 0x00;
`lo_eeprom_ipod.bin` has the FDIM's "iPod fitted" bit (record 0x64 bit 4). Timeline in the emulator: pairing and iPod
identification within 1 s, logo upload until ~23 s, track query at ~28 s, then the FDIM shows **"iPod 1 — Turbo Lag Blues /
Tester Present / Barra Sessions 0:29"** with the iPod/RPT/RDM/pause soft keys (`docs/aim-fdim-ipod.png`), driven entirely by the
two real firmwares. `bus.log` holds every frame on the virtual bus with its sender.

What is still missing: the FDIM's buttons (key‑return matrix on P5 + ADC ladders) are not modelled, so menu navigation into the
playlist/artist lists cannot be exercised yet; the head unit (ACU) firmware is not available, so 0x2F2/0x311/0x317 are
synthesised; the exact meaning of 0x2D4 bits 0‑3 / byte 5 and the CSIB1 switch bits would need the AIM's schematic.
