/* fdim_mirror.c - real-time live FDIM mirror.
 *
 * Runs the ACTUAL FG Falcon FDIM firmware in real time, bridges MS-CAN between the
 * firmware and a SocketCAN interface (the car's bus), and streams the LCD framebuffer
 * so a display front-end (fdim_pi.py --identical) paints the exact pixels the factory
 * module draws. The firmware itself does all the bus work - network-management
 * heartbeat, periodic status, button reports - so the result is pixel- AND bus-
 * identical to the factory, because it IS the factory code running.
 *
 *     can0 <==(SocketCAN)==> [ firmware in emulator ] ==(framebuffer stream)==> display
 *                                     ^
 *                        button presses injected on P5 (INTKR)
 *
 * Build:  make fdim-mirror
 * Usage:  fdim-mirror APP.bin [options]
 *   -canif NAME     SocketCAN interface for live RX/TX, e.g. can0 (Linux).
 *   -canfile FILE   replay a candump file instead of a live interface (bench).
 *   -eeprom FILE    EEPROM image (persisted on exit).
 *   -fb PATH        stream framebuffer here (default stdout; use a FIFO or file).
 *   -fbhz N         framebuffer rate, default 30.
 *   -ctl PATH       control input (default stdin): "BTN n" / "REL n" / "CAN id b..".
 *   -flip -width N -xoff N   render window (match the emulator defaults 240/16).
 *   -adc N          ADC override (supply sense); 430 keeps the panel on.
 *   -fast           run as fast as possible (no real-time pacing) - for testing.
 *   -frames N       exit after streaming N frames (testing).
 *
 * Framebuffer stream (little-endian), one message per frame:
 *   "FBM1" | u16 width | u16 height | u8 fmt(0 = grey 0..3) | u8 flags | payload
 *   payload = width*height bytes, each a grey level 0..3.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include "sys.h"

#ifdef __linux__
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#define HAVE_SOCKETCAN 1
#endif

/* ------------------------------------------------------------------ globals */
static int can_sock = -1;          /* SocketCAN fd, or -1 */
static FILE *fb_out = NULL;        /* framebuffer stream */
static int g_width = 240, g_xoff = 16, g_flip = 0;

/* ------------------------------------------------- TX: firmware -> car bus */
static void tx_to_can(sys_t *s, const can_frame_t *f)
{
    (void)s;
#ifdef HAVE_SOCKETCAN
    if (can_sock < 0) return;
    struct can_frame cf; memset(&cf, 0, sizeof cf);
    cf.can_id = f->id | (f->ext ? CAN_EFF_FLAG : 0);
    cf.can_dlc = f->dlc > 8 ? 8 : f->dlc;
    memcpy(cf.data, f->data, cf.can_dlc);
    if (write(can_sock, &cf, sizeof cf) != sizeof cf) { /* bus full/off: drop */ }
#else
    (void)f;
#endif
}

/* ------------------------------------------------- RX: car bus -> firmware */
static void drain_can_rx(sys_t *s)
{
#ifdef HAVE_SOCKETCAN
    if (can_sock < 0) return;
    struct can_frame cf;
    for (;;) {
        ssize_t n = read(can_sock, &cf, sizeof cf);
        if (n != (ssize_t)sizeof cf) break;         /* EAGAIN when drained */
        can_frame_t f; memset(&f, 0, sizeof f);
        f.id = cf.can_id & CAN_EFF_MASK;
        f.ext = !!(cf.can_id & CAN_EFF_FLAG);
        f.dlc = cf.can_dlc;
        memcpy(f.data, cf.data, cf.can_dlc > 8 ? 8 : cf.can_dlc);
        sys_can_inject(s, &f);
    }
#else
    (void)s;
#endif
}

static int open_socketcan(const char *ifname)
{
#ifdef HAVE_SOCKETCAN
    int sk = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sk < 0) { perror("socket(CAN_RAW)"); return -1; }
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(sk, SIOCGIFINDEX, &ifr) < 0) { perror(ifname); close(sk); return -1; }
    struct sockaddr_can addr; memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sk, (struct sockaddr *)&addr, sizeof addr) < 0) { perror("bind"); close(sk); return -1; }
    fcntl(sk, F_SETFL, O_NONBLOCK);
    return sk;
#else
    (void)ifname; fprintf(stderr, "SocketCAN not available on this build\n"); return -1;
#endif
}

/* --------------------------------------------------- candump bench replay */
typedef struct { double ms; can_frame_t f; } ev_t;
static ev_t *evs; static int nev;
static double replay_span_ms;

static void load_candump(const char *path)
{
    FILE *f = fopen(path, "r"); if (!f) { perror(path); exit(1); }
    int cap = 1024; evs = malloc(cap * sizeof *evs); nev = 0;
    char line[512]; double t0 = -1;
    while (fgets(line, sizeof line, f)) {
        double ts; int id, nb, j, k; char *p;
        /* "(1700000000.123456) can0 406#0404..." */
        if (sscanf(line, "(%lf)", &ts) == 1 && (p = strchr(line, '#'))) {
            char *idp = strstr(line, ") "); if (!idp) continue; idp += 2;
            while (*idp == ' ') idp++; char *sp = strchr(idp, ' '); if (!sp) continue;
            id = (int)strtol(sp + 1, NULL, 16);
            if (t0 < 0) t0 = ts;
            if (nev >= cap) { cap *= 2; evs = realloc(evs, cap * sizeof *evs); }
            ev_t *e = &evs[nev++]; memset(e, 0, sizeof *e);
            e->ms = (ts - t0) * 1000.0; e->f.id = id; e->f.ext = id > 0x7FF;
            p++; k = 0; while (k < 8 && sscanf(p, "%2x%n", &nb, &j) == 1) { e->f.data[k++] = nb; p += j; }
            e->f.dlc = k;
        } else if (sscanf(line, " %*s %x [%d]%n", &id, &nb, &j) == 2) {
            /* "can0 406 [8] 04 04 .." */
            if (t0 < 0) t0 = 0;
            if (nev >= cap) { cap *= 2; evs = realloc(evs, cap * sizeof *evs); }
            ev_t *e = &evs[nev++]; memset(e, 0, sizeof *e);
            e->ms = nev; e->f.id = id; e->f.ext = id > 0x7FF; p = line + j; k = 0;
            while (k < nb && k < 8 && sscanf(p, "%x%n", &id, &j) == 1) { e->f.data[k++] = id; p += j; }
            e->f.dlc = k;
        }
    }
    fclose(f);
    replay_span_ms = nev ? evs[nev - 1].ms + 1 : 0;
    fprintf(stderr, "candump: %d frames over %.0f ms (looping)\n", nev, replay_span_ms);
}

/* ------------------------------------------------------- control channel */
static int ctl_fd = -1;
static void handle_ctl_line(sys_t *s, char *line)
{
    int n; unsigned id, b; char *p;
    if (sscanf(line, "BTN %d", &n) == 1 && n >= 0 && n < 8) {
        s->pin_in[0x05] &= ~(1 << n);          /* active-low press */
        sys_raise(s, VEC_KR);
    } else if (sscanf(line, "REL %d", &n) == 1 && n >= 0 && n < 8) {
        s->pin_in[0x05] |= (1 << n);
        sys_raise(s, VEC_KR);
    } else if (sscanf(line, "CAN %x%n", &id, &n) == 1) {
        can_frame_t f; memset(&f, 0, sizeof f); f.id = id; f.ext = id > 0x7FF;
        p = line + n; int k = 0;
        while (k < 8 && sscanf(p, "%x%n", &b, &n) == 1) { f.data[k++] = b; p += n; }
        f.dlc = k; sys_can_inject(s, &f);
    }
}
static void drain_ctl(sys_t *s)
{
    if (ctl_fd < 0) return;
    static char buf[1024]; static int len = 0;
    for (;;) {
        ssize_t r = read(ctl_fd, buf + len, sizeof buf - len - 1);
        if (r <= 0) break;
        len += r;
        char *nl;
        while ((nl = memchr(buf, '\n', len))) {
            *nl = 0; handle_ctl_line(s, buf);
            int rest = len - (nl - buf) - 1; memmove(buf, nl + 1, rest); len = rest;
        }
        if (len >= (int)sizeof buf - 1) len = 0;    /* overlong line: drop */
    }
}

/* --------------------------------------------------------- fb streaming */
static void stream_frame(sys_t *s)
{
    static uint8_t px[256 * 200];
    if (!s->lcd) return;                 /* colour unit needs the artwork ROM */
    int w, h;
    int n = lcd_copy_frame(s->lcd, px, &w, &h, g_width, g_xoff, g_flip);
    uint8_t hdr[10] = { 'F','B','M','1', w & 0xFF, w >> 8, h & 0xFF, h >> 8, 0, 0 };
    if (fwrite(hdr, 1, 10, fb_out) != 10) return;
    fwrite(px, 1, n, fb_out);
    fflush(fb_out);
}

/* ---------------------------------------------------------------- timing */
static double now_s(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
    const char *app_path = NULL, *canif = NULL, *canfile = NULL, *eeprom = NULL;
    const char *fb_path = NULL, *ctl_path = NULL;
    int fbhz = 30, fast = 0, adc = 430, frames_limit = 0;
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] != '-' && !app_path) app_path = argv[i];
        else if (!strcmp(argv[i], "-canif") && i + 1 < argc) canif = argv[++i];
        else if (!strcmp(argv[i], "-canfile") && i + 1 < argc) canfile = argv[++i];
        else if (!strcmp(argv[i], "-eeprom") && i + 1 < argc) eeprom = argv[++i];
        else if (!strcmp(argv[i], "-fb") && i + 1 < argc) fb_path = argv[++i];
        else if (!strcmp(argv[i], "-fbhz") && i + 1 < argc) fbhz = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-ctl") && i + 1 < argc) ctl_path = argv[++i];
        else if (!strcmp(argv[i], "-flip")) g_flip = 1;
        else if (!strcmp(argv[i], "-width") && i + 1 < argc) g_width = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-xoff") && i + 1 < argc) g_xoff = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-adc") && i + 1 < argc) adc = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-fast")) fast = 1;
        else if (!strcmp(argv[i], "-frames") && i + 1 < argc) frames_limit = atoi(argv[++i]);
        else { fprintf(stderr, "unknown arg: %s\n", argv[i]); }
    }
    if (!app_path) { fprintf(stderr, "usage: fdim-mirror APP.bin [-canif can0|-canfile f] [-fb PATH] ...\n"); return 1; }

    /* load firmware image */
    FILE *af = fopen(app_path, "rb"); if (!af) { perror(app_path); return 1; }
    fseek(af, 0, SEEK_END); long alen = ftell(af); fseek(af, 0, SEEK_SET);
    uint8_t *app = malloc(alen);
    if (fread(app, 1, alen, af) != (size_t)alen) { perror("read"); return 1; } fclose(af);

    sys_t *s = calloc(1, sizeof *s); S = s;
    sys_init(s, app, alen);
    s->name = "FDIM";
    /* attach the display device the same way the emulator does */
    if (s->board == 0) { s->gdc = gdc_create(NULL, NULL); }
    else if (s->module != 2) { s->lcd = lcd_create(); lcd_set_log(s->lcd, NULL, 0); if (s->board == 2) lcd_force_gray(s->lcd, 1); }
    s->tx_hook = tx_to_can;
    s->pin_in[0x05] = 0x3F;               /* P50-55 buttons released (active-low) */
    if (adc) for (int i = 0; i < 16; i++) s->adc_val[i] = adc;
    if (eeprom) { FILE *ef = fopen(eeprom, "rb"); if (ef) { memset(s->eeprom, 0xFF, 512);
        if (fread(s->eeprom, 1, 512, ef) > 0) s->eeprom_loaded = 1; fclose(ef); } }

    if (!s->lcd)
        fprintf(stderr, "WARNING: this is the colour unit; the artwork ROM is external and "
                        "not present, so the framebuffer cannot be reproduced. Mono only for now.\n");

    /* I/O setup */
    fb_out = fb_path ? fopen(fb_path, "wb") : stdout;
    if (!fb_out) { perror(fb_path); return 1; }
    if (canif) { can_sock = open_socketcan(canif);
        if (can_sock < 0) fprintf(stderr, "continuing without live CAN\n"); }
    if (canfile) load_candump(canfile);
    ctl_fd = ctl_path ? open(ctl_path, O_RDONLY | O_NONBLOCK) : 0; /* 0 = stdin */
    if (ctl_fd == 0) fcntl(0, F_SETFL, O_NONBLOCK);

    fprintf(stderr, "fdim-mirror: %s  board=%d  can=%s  fb=%s @ %dHz  %s\n",
            app_path, s->board, canif ? canif : (canfile ? "file" : "none"),
            fb_path ? fb_path : "stdout", fbhz, fast ? "FAST" : "real-time");

    /* real-time loop: 1 ms firmware slices, paced to the wall clock */
    const uint64_t slice = CPU_HZ / 1000;
    const double wall0 = now_s();
    uint64_t sim_ms = 0, fb_period_ms = fbhz > 0 ? 1000 / fbhz : 33, next_fb = 0;
    int ei = 0, nframes = 0;
    for (;;) {
        drain_can_rx(s);
        drain_ctl(s);
        if (nev) {                         /* candump replay (looping) */
            double loop_ms = replay_span_ms > 0 ? replay_span_ms : 1;
            double pos = sim_ms - loop_ms * (int)(sim_ms / loop_ms);
            while (ei < nev && evs[ei].ms <= pos) { sys_can_inject(s, &evs[ei].f); ei++; }
            if (ei >= nev && sim_ms && (int)(sim_ms / loop_ms) != (int)((sim_ms - 1) / loop_ms)) ei = 0;
        }
        sys_run(s, slice);
        sim_ms++;

        if (sim_ms >= next_fb) {
            stream_frame(s); next_fb += fb_period_ms;
            if (frames_limit && ++nframes >= frames_limit) break;
        }
        if (!fast) {                       /* sleep until wall clock catches up */
            double target = wall0 + sim_ms / 1000.0, dt = target - now_s();
            if (dt > 0) { struct timespec t = { (time_t)dt, (long)((dt - (time_t)dt) * 1e9) }; nanosleep(&t, NULL); }
        }
    }

    if (eeprom && s->eeprom_dirty) { FILE *ef = fopen(eeprom, "wb"); if (ef) { fwrite(s->eeprom, 1, 512, ef); fclose(ef); } }
    if (fb_out && fb_out != stdout) fclose(fb_out);
    return 0;
}
