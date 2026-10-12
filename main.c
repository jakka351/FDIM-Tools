/* fdim-emu: FG Falcon FDIM (V850ES) emulator driver */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"

static sys_t sysm;

static void usage(void)
{
    fprintf(stderr, "usage: fdim-emu <app.bin> [-ms N] [-trace-at PC] [-trace-insns N] [-log file] [-rom image.bin] [-can script] [-snap ms] [-out dir] [-quiet]\n");
    exit(1);
}

/* CAN input formats (auto-detected per line):
 *   simple : "ms id d0 d1 ... d7"                 (ms = emulated time, hex id/bytes)
 *   candump: "(1700000000.123456) can0 406#0404040404040404"   (timestamps rebased to can_offset_ms)
 *   candump: "  can0  406   [8]  04 04 04 04 04 04 04 04"
 */
typedef struct { uint64_t at; can_frame_t f; } sched_t;
static sched_t sched[200000]; static int nsched;
static double can_offset_ms = 500;

static int hexval(const char *s, int n){ int v = 0; for (int i = 0; i < n; i++) { char c = s[i]; v <<= 4; if (c >= '0' && c <= '9') v |= c - '0'; else if (c >= 'a' && c <= 'f') v |= c - 'a' + 10; else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10; else return -1; } return v; }

static void load_can_script(const char *path)
{
    FILE *f = fopen(path, "r"); if (!f) { perror(path); return; }
    char line[512]; double t0 = -1;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        sched_t *s = &sched[nsched]; memset(s, 0, sizeof *s);
        char *p = line; while (*p == ' ' || *p == '\t') p++;
        if (*p == '(') {                       /* candump -L */
            double ts = atof(p + 1); char *h = strchr(p, '#'); if (!h) continue;
            char *idp = h; while (idp > p && idp[-1] != ' ') idp--;
            int idlen = h - idp; unsigned id = hexval(idp, idlen);
            if (t0 < 0) t0 = ts;
            s->at = (uint64_t)((can_offset_ms + (ts - t0) * 1000.0) * (CPU_HZ / 1000));
            s->f.id = id; s->f.ext = idlen > 3;
            int k = 0; const char *d = h + 1; if (*d == 'R') continue;
            while (k < 8 && hexval(d, 2) >= 0) { s->f.data[k++] = hexval(d, 2); d += 2; }
            s->f.dlc = k; nsched++;
        } else if (strstr(p, "[") && strncmp(p, "can", 3) == 0) {   /* candump plain */
            char *idp = strchr(p, ' '); while (*idp == ' ') idp++;
            unsigned id = strtoul(idp, NULL, 16); char *b = strchr(p, ']'); if (!b) continue; b++;
            s->at = (uint64_t)((can_offset_ms + nsched * 1.0) * (CPU_HZ / 1000));   /* no timestamps: 1 ms apart */
            s->f.id = id; s->f.ext = id > 0x7FF; int k = 0; unsigned v; int n;
            while (k < 8 && sscanf(b, "%x%n", &v, &n) == 1) { s->f.data[k++] = v; b += n; }
            s->f.dlc = k; nsched++;
        } else {
            double ms; unsigned id; int n;
            if (sscanf(p, "%lf %x%n", &ms, &id, &n) < 2) continue;
            p += n;
            s->at = (uint64_t)(ms * (CPU_HZ / 1000)); s->f.id = id; s->f.ext = id > 0x7FF;
            int k = 0; unsigned b;
            while (k < 8 && sscanf(p, "%x%n", &b, &n) == 1) { s->f.data[k++] = b; p += n; }
            s->f.dlc = k; nsched++;
        }
        if (nsched >= 200000) break;
    }
    fclose(f);
    fprintf(stderr, "loaded %d scheduled CAN frames\n", nsched);
}

int main(int argc, char **argv)
{
    if (argc < 2) usage();
    const char *bin = argv[1]; double run_ms = 10000; uint32_t trace_at = 0; uint64_t trace_insns = 0; const char *logf = NULL; const char *rom = NULL;
    const char *canscript = NULL; double snap_ms = 0; const char *outdir = "."; int quiet = 0;
    uint32_t watch[16]; int nwatch = 0; uint64_t trace_started = 0; int adc_override[16] = {0};
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "-ms") && i + 1 < argc) run_ms = atof(argv[++i]);
        else if (!strcmp(argv[i], "-trace-at") && i + 1 < argc) trace_at = strtoul(argv[++i], NULL, 16);
        else if (!strcmp(argv[i], "-trace-insns") && i + 1 < argc) trace_insns = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "-log") && i + 1 < argc) logf = argv[++i];
        else if (!strcmp(argv[i], "-rom") && i + 1 < argc) rom = argv[++i];
        else if (!strcmp(argv[i], "-can") && i + 1 < argc) canscript = argv[++i];
        else if (!strcmp(argv[i], "-snap") && i + 1 < argc) snap_ms = atof(argv[++i]);
        else if (!strcmp(argv[i], "-out") && i + 1 < argc) outdir = argv[++i];
        else if (!strcmp(argv[i], "-quiet")) quiet = 1;
        else if (!strcmp(argv[i], "-gdcv")) { extern int gdc_verbose; gdc_verbose = 1; }
        else if (!strcmp(argv[i], "-can-offset") && i + 1 < argc) can_offset_ms = atof(argv[++i]);
        else if (!strcmp(argv[i], "-watch") && i + 1 < argc && nwatch < 16) watch[nwatch++] = strtoul(argv[++i], NULL, 16);
        else if (!strcmp(argv[i], "-adc") && i + 2 < argc) { int ch = atoi(argv[++i]); int v = atoi(argv[++i]); if (ch >= 0 && ch < 16) adc_override[ch] = v + 1; }
        else if (!strcmp(argv[i], "-adcall") && i + 1 < argc) { int v = atoi(argv[++i]); for (int k = 0; k < 16; k++) adc_override[k] = v + 1; }
        else usage();
    }
    FILE *f = fopen(bin, "rb"); if (!f) { perror(bin); return 1; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *app = malloc(n); if (fread(app, 1, n, f) != (size_t)n) { perror("read"); return 1; } fclose(f);

    sys_t *s = &sysm; memset(s, 0, sizeof *s);
    sys_init(s, app, n);
    s->log = logf ? fopen(logf, "w") : stderr;
    s->log_sfr = !quiet; s->log_can = 1; s->log_gdc = 1; s->log_unknown = !quiet;
    for (int k = 0; k < 16; k++) s->adc_val[k] = adc_override[k] ? adc_override[k] - 1 : 700;
    s->gdc = gdc_create(rom, NULL);
    extern void gdc_set_log(struct gdc *g, FILE *f); gdc_set_log(s->gdc, s->log);
    if (canscript) load_can_script(canscript);

    uint64_t total = (uint64_t)(run_ms * (CPU_HZ / 1000));
    uint64_t snap_every = snap_ms > 0 ? (uint64_t)(snap_ms * (CPU_HZ / 1000)) : 0, next_snap = snap_every;
    int si = 0, frame = 0;
    uint64_t slice = CPU_HZ / 1000;   /* 1 ms slices */
    extern uint32_t g_watch[16]; extern int g_nwatch; memcpy(g_watch, watch, sizeof watch); g_nwatch = nwatch;
    if (trace_at) slice = 1;
    while (s->cpu.cycles < total && s->cpu.halted != 2) {
        /* trace control */
        if (trace_at && !s->cpu.trace && s->cpu.pc == trace_at) { s->cpu.trace = 1; trace_started = s->cpu.insns; }
        if (s->cpu.trace && trace_insns && s->cpu.insns > trace_started + trace_insns) { s->cpu.trace = 0; trace_at = 0; }
        while (si < nsched && sched[si].at <= s->cpu.cycles) { sys_can_inject(s, &sched[si].f); si++; }
        sys_run(s, slice);
        if (snap_every && s->cpu.cycles >= next_snap) {
            char path[512]; snprintf(path, sizeof path, "%s/frame_%04d.ppm", outdir, frame++);
            gdc_render(s->gdc, path); next_snap += snap_every;
        }
    }
    fprintf(stderr, "stopped at cycle %llu (%.1f ms), %llu insns, pc=%06x halted=%d\n", (unsigned long long)s->cpu.cycles, s->cpu.cycles / (CPU_HZ / 1000.0), (unsigned long long)s->cpu.insns, s->cpu.pc, s->cpu.halted);
    fprintf(stderr, "irqs:"); for (int i = 0; i < NVEC / 0x10; i++) if (s->irq_count[i]) fprintf(stderr, " %03x:%llu", 0x80 + i * 0x10, (unsigned long long)s->irq_count[i]); fprintf(stderr, "\n");
    char path[512]; snprintf(path, sizeof path, "%s/final.ppm", outdir); gdc_render(s->gdc, path);
    extern void sys_dump_can(sys_t *s, FILE *f); sys_dump_can(s, stderr);
    snprintf(path, sizeof path, "%s/objects.txt", outdir); FILE *of = fopen(path, "w"); if (of) { gdc_dump_objects(s->gdc, of); fclose(of); }
    return 0;
}
