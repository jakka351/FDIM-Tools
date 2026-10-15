/* Minimal iPod (iAP, "Apple Accessory Protocol") responder for the AIM's UARTA1 link.
 * Parses the packets the AIM sends (FF 55 len lingo cmd ... cks) and answers with canned data:
 * Lingo 0x00 General and Lingo 0x04 Extended Interface (database browsing / play status / track names).
 * Command numbering follows the public (iPodLinux / Lingo 4 rev 1.x) documentation; everything received is logged
 * so the AIM's own expectations can be checked against the log.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include "sys.h"

static struct {
    uint8_t buf[4096]; int n; int state; int len;
    uint8_t img[65536]; int img_len, img_w, img_h, img_stride, img_fmt; int img_done;
    FILE *log;
    /* library */
    int track;            /* current track index */
    int playing;          /* 0 stopped, 1 playing, 2 paused */
    uint32_t pos_ms;
    int shuffle, repeat, notify, uimode;
    int cat;              /* currently selected category for record retrieval */
    uint64_t last_status;
} ip;

typedef struct { const char *title, *artist, *album; uint32_t len_ms; } track_t;
static const track_t lib[] = {
    { "Turbo Lag Blues",     "Tester Present", "Barra Sessions", 214000 },
    { "Boost Leak Boogie",   "Tester Present", "Barra Sessions", 187000 },
    { "ZF Six Speed Shuffle","Tester Present", "Barra Sessions", 243000 },
    { "Kalgoorlie Highway",  "Bromigo",        "Western Roads",  301000 },
};
#define NTRACK ((int)(sizeof lib / sizeof lib[0]))
static const char *cat_names[][4] = { {0}, {"Barra Sessions", "Western Roads", "On The Road Again", 0}, {"Tester Present", "Bromigo", 0, 0}, {"Barra Sessions", "Western Roads", 0, 0}, {"Rock", "Diesel", 0, 0} };

static void ipod_log(const char *fmt, ...)
{
    if (!ip.log) return;
    va_list ap; va_start(ap, fmt); fprintf(ip.log, "[%llu] IPOD ", (unsigned long long)S->cpu.cycles); vfprintf(ip.log, fmt, ap); fputc('\n', ip.log); va_end(ap);
}

static void send_packet(sys_t *s, const uint8_t *payload, int n)
{
    uint8_t pkt[320]; int k = 0; uint8_t sum = 0;
    pkt[k++] = 0xFF; pkt[k++] = 0x55;
    if (n > 255) { pkt[k++] = 0; pkt[k++] = n >> 8; pkt[k++] = n & 0xFF; sum = (n >> 8) + (n & 0xFF); } else { pkt[k++] = n; sum = n; }
    for (int i = 0; i < n; i++) { pkt[k++] = payload[i]; sum += payload[i]; }
    pkt[k++] = (uint8_t)(-sum);
    sys_uart1_inject(s, pkt, k);
    if (ip.log) { fprintf(ip.log, "[%llu] IPOD TX:", (unsigned long long)S->cpu.cycles); for (int i = 0; i < k; i++) fprintf(ip.log, " %02x", pkt[i]); fputc('\n', ip.log); }
}

static int put_str(uint8_t *d, const char *str) { int n = strlen(str) + 1; memcpy(d, str, n); return n; }
static int put_u32(uint8_t *d, uint32_t v) { d[0] = v >> 24; d[1] = v >> 16; d[2] = v >> 8; d[3] = v; return 4; }
static uint32_t get_u32(const uint8_t *d) { return ((uint32_t)d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]; }

static void l0_ack(sys_t *s, int cmd) { uint8_t p[4] = {0x00, 0x02, 0x00, cmd}; send_packet(s, p, 4); }
static void l4_ack(sys_t *s, int cmd) { uint8_t p[6] = {0x04, 0x00, 0x01, 0x00, cmd >> 8, cmd & 0xFF}; send_packet(s, p, 6); }

static void handle(sys_t *s, const uint8_t *pl, int n)
{
    int lingo = pl[0];
    if (ip.log) { fprintf(ip.log, "[%llu] IPOD RX lingo %02x:", (unsigned long long)S->cpu.cycles, lingo); for (int i = 1; i < n; i++) fprintf(ip.log, " %02x", pl[i]); fputc('\n', ip.log); }
    uint8_t p[300]; int k;
    if (lingo == 0x00) {
        int cmd = pl[1];
        switch (cmd) {
        case 0x01: case 0x13:           /* Identify / IdentifyDeviceLingoes */
            l0_ack(s, cmd); ipod_log("identified (cmd %02x, lingoes %08x)", cmd, cmd == 0x13 ? get_u32(pl + 2) : 1); break;
        case 0x05: ip.uimode = 1; l0_ack(s, cmd); break;  /* EnterRemoteUIMode */
        case 0x06: ip.uimode = 0; l0_ack(s, cmd); break;  /* ExitRemoteUIMode */
        case 0x07: k = 0; p[k++] = 0; p[k++] = 0x08; k += put_str(p + k, "Bromigo's iPod"); send_packet(s, p, k); break;
        case 0x09: k = 0; p[k++] = 0; p[k++] = 0x0A; p[k++] = 1; p[k++] = 2; p[k++] = 3; send_packet(s, p, k); break;
        case 0x0B: k = 0; p[k++] = 0; p[k++] = 0x0C; k += put_str(p + k, "8R29FALCON"); send_packet(s, p, k); break;
        case 0x0D: k = 0; p[k++] = 0; p[k++] = 0x0E; k += put_u32(p + k, 0x00070002); k += put_str(p + k, "MA450"); send_packet(s, p, k); break;
        case 0x0F: k = 0; p[k++] = 0; p[k++] = 0x10; p[k++] = pl[2]; p[k++] = 1; p[k++] = pl[2] == 4 ? 9 : 5; send_packet(s, p, k); break;
        case 0x03: k = 0; p[k++] = 0; p[k++] = 0x04; p[k++] = ip.uimode; send_packet(s, p, k); break;      /* RequestRemoteUIMode */
        default: l0_ack(s, cmd); break;
        }
        return;
    }
    if (lingo == 0x04) {   /* Extended Interface lingo, official Apple numbering */
        int cmd = (pl[1] << 8) | pl[2]; const uint8_t *a = pl + 3; const track_t *t = &lib[ip.track % NTRACK];
        #define L4(c) do { k = 0; p[k++] = 4; p[k++] = (c) >> 8; p[k++] = (c) & 0xFF; } while (0)
        switch (cmd) {
        case 0x0002: L4(0x0003); k += put_u32(p + k, 0xFFFFFFFF); k += put_u32(p + k, 0); send_packet(s, p, k); break;   /* no chapters */
        case 0x0005: L4(0x0006); k += put_u32(p + k, t->len_ms); k += put_u32(p + k, ip.pos_ms); send_packet(s, p, k); break;
        case 0x0007: L4(0x0008); k += put_str(p + k, ""); send_packet(s, p, k); break;
        case 0x0009: L4(0x000A); p[k++] = 0; send_packet(s, p, k); break;
        case 0x000C: { int type = a[0]; L4(0x000D); p[k++] = type;                        /* GetIndexedPlayingTrackInfo */
            if (type == 0) { p[k++] = 0; k += put_u32(p + k, t->len_ms); p[k++] = 0; p[k++] = 0; }
            else if (type == 1) { k += put_str(p + k, ""); }                              /* chapter times */
            else if (type == 5) k += put_str(p + k, "Rock"); else if (type == 6) k += put_str(p + k, t->artist); else k += put_str(p + k, "");
            send_packet(s, p, k); break; }
        case 0x000E: L4(0x000F); send_packet(s, p, k); break;                             /* no artwork formats */
        case 0x0012: L4(0x0013); p[k++] = 1; p[k++] = 9; send_packet(s, p, k); break;     /* protocol 1.09 */
        case 0x0014: L4(0x0015); k += put_str(p + k, "Bromigo's iPod"); send_packet(s, p, k); break;
        case 0x0016: ip.cat = 0; l4_ack(s, cmd); break;                                   /* ResetDBSelection */
        case 0x0017: ip.cat = a[0]; l4_ack(s, cmd); ipod_log("select category %d record %u", a[0], get_u32(a + 1)); break;
        case 0x0018: { int c = a[0]; uint32_t cnt = c == 5 ? NTRACK : c == 1 ? 3 : 2; L4(0x0019); k += put_u32(p + k, cnt); send_packet(s, p, k); break; }
        case 0x001A: { int c = a[0]; uint32_t start = get_u32(a + 1), cnt = get_u32(a + 5); if (cnt > 8) cnt = 8;
            for (uint32_t i = 0; i < cnt; i++) {
                const char *name = c == 5 ? lib[(start + i) % NTRACK].title : (c >= 1 && c <= 4 && cat_names[c][(start + i) % 3]) ? cat_names[c][(start + i) % 3] : "Unknown";
                L4(0x001B); k += put_u32(p + k, start + i); k += put_str(p + k, name); send_packet(s, p, k);
            }
            break; }
        case 0x001C: L4(0x001D); k += put_u32(p + k, t->len_ms); k += put_u32(p + k, ip.pos_ms); p[k++] = ip.playing; send_packet(s, p, k); break;
        case 0x001E: L4(0x001F); k += put_u32(p + k, ip.track); send_packet(s, p, k); break;
        case 0x0020: L4(0x0021); k += put_str(p + k, lib[get_u32(a) % NTRACK].title); send_packet(s, p, k); break;
        case 0x0022: L4(0x0023); k += put_str(p + k, lib[get_u32(a) % NTRACK].artist); send_packet(s, p, k); break;
        case 0x0024: L4(0x0025); k += put_str(p + k, lib[get_u32(a) % NTRACK].album); send_packet(s, p, k); break;
        case 0x0026: ip.notify = a[0]; l4_ack(s, cmd); break;                             /* SetPlayStatusChangeNotification */
        case 0x0028: ip.track = get_u32(a) % NTRACK; ip.playing = 1; ip.pos_ms = 0; l4_ack(s, cmd); break;
        case 0x0029: { int ctl = a[0]; l4_ack(s, cmd);                                    /* PlayControl */
            if (ctl == 1) ip.playing = ip.playing == 1 ? 2 : 1; else if (ctl == 2) ip.playing = 0; else if (ctl == 3 || ctl == 8) { ip.track = (ip.track + 1) % NTRACK; ip.pos_ms = 0; }
            else if (ctl == 4 || ctl == 9) { ip.track = (ip.track + NTRACK - 1) % NTRACK; ip.pos_ms = 0; } else if (ctl == 0x0A) ip.playing = 1; else if (ctl == 0x0B) ip.playing = 2;
            ipod_log("play control %02x -> state %d track %d", ctl, ip.playing, ip.track);
            if (ip.notify) { L4(0x0027); p[k++] = ip.playing == 0 ? 0x00 : 0x06; p[k++] = ip.playing; send_packet(s, p, k); }   /* 0x06 = play status ext */
            break; }
        case 0x002C: L4(0x002D); p[k++] = ip.shuffle; send_packet(s, p, k); break;
        case 0x002E: ip.shuffle = a[0]; l4_ack(s, cmd); break;
        case 0x002F: L4(0x0030); p[k++] = ip.repeat; send_packet(s, p, k); break;
        case 0x0031: ip.repeat = a[0]; l4_ack(s, cmd); break;
        case 0x0033: L4(0x0034); p[k++] = 0; p[k++] = 0x80; p[k++] = 0; p[k++] = 0x16; p[k++] = 1; send_packet(s, p, k); break;   /* mono display 128x22, format 1 */
        case 0x0032: { int tel = (a[0] << 8) | a[1]; const uint8_t *d; int dn;         /* SetDisplayImage */
            if (tel == 0) { ip.img_fmt = a[2]; ip.img_w = (a[3] << 8) | a[4]; ip.img_h = (a[5] << 8) | a[6]; ip.img_stride = get_u32(a + 7); d = a + 11; dn = n - 3 - 11; ip.img_len = 0;
                            ipod_log("SetDisplayImage: %dx%d format %d stride %d", ip.img_w, ip.img_h, ip.img_fmt, ip.img_stride); }
            else { d = a + 2; dn = n - 3 - 2; }
            if (dn > 0 && ip.img_len + dn <= (int)sizeof ip.img) { memcpy(ip.img + ip.img_len, d, dn); ip.img_len += dn; }
            { FILE *f = fopen("ipod_display_image.bin", "wb"); if (f) { fwrite(ip.img, 1, ip.img_len, f); fclose(f); } }
            if (ip.img_len >= ip.img_stride * ip.img_h && !ip.img_done) { ip.img_done = 1; ipod_log("display image: declared size reached (%d bytes), telegram %d", ip.img_len, tel); }
            l4_ack(s, cmd); break; }
        case 0x0035: L4(0x0036); k += put_u32(p + k, NTRACK); send_packet(s, p, k); break;
        case 0x0037: ip.track = get_u32(a) % NTRACK; ip.pos_ms = 0; ip.playing = 1; l4_ack(s, cmd); break;
        case 0x0039: L4(0x003A); send_packet(s, p, k); break;                             /* no colour display */
        default: l4_ack(s, cmd); break;
        }
        return;
    }
    /* other lingoes (2 = simple remote …): ack */
    uint8_t ack[4] = {lingo, 0x01, 0x00, pl[1]}; send_packet(s, ack, 4);
}

/* called for every byte the AIM transmits */
void ipodsim_rx(sys_t *s, uint8_t b)
{
    switch (ip.state) {
    case 0: if (b == 0xFF) ip.state = 1; break;
    case 1: ip.state = (b == 0x55) ? 2 : (b == 0xFF ? 1 : 0); break;
    case 2: if (b == 0) { ip.state = 5; } else { ip.len = b; ip.n = 0; ip.state = 3; } break;
    case 5: ip.len = b << 8; ip.state = 6; break;
    case 6: ip.len |= b; ip.n = 0; ip.state = 3; break;
    case 3: ip.buf[ip.n++] = b; if (ip.n >= ip.len || ip.n >= (int)sizeof ip.buf) ip.state = 4; break;
    case 4: { uint8_t sum = ip.len > 255 ? (ip.len >> 8) + (ip.len & 0xFF) : ip.len; for (int i = 0; i < ip.n; i++) sum += ip.buf[i]; sum += b;
              if (sum == 0) handle(s, ip.buf, ip.n); else ipod_log("bad checksum on %d-byte packet", ip.n); ip.state = 0; break; }
    }
}

void ipodsim_tick(sys_t *s)
{
    /* advance play position; emit a periodic PlayStatusChangeNotification (track position) every second while playing */
    if (ip.playing == 1) {
        uint64_t now = s->cpu.cycles;
        if (now - ip.last_status >= CPU_HZ) {
            ip.pos_ms += 1000; ip.last_status = now;
            const track_t *t = &lib[ip.track % NTRACK];
            if (!ip.notify) return;
            if (ip.pos_ms >= t->len_ms) { ip.pos_ms = 0; ip.track = (ip.track + 1) % NTRACK; uint8_t p[8] = {4, 0, 0x27, 0x01, 0, 0, 0, 0}; put_u32(p + 4, ip.track); send_packet(s, p, 8); }
            else { uint8_t p[8] = {4, 0, 0x27, 0x04, 0, 0, 0, 0}; put_u32(p + 4, ip.pos_ms); send_packet(s, p, 8); }
        }
    } else ip.last_status = s->cpu.cycles;
}

void ipodsim_init(FILE *log) { memset(&ip, 0, sizeof ip); ip.log = log; ip.playing = 1; }
