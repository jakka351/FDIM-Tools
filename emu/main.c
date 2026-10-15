/* fdim-emu: FG Falcon FDIM / AIM (V850ES) emulator driver.
 *
 *   fdim-emu app.bin [module options] [global options]
 *   fdim-emu -module a.bin [options for a] -module b.bin [options for b] [global options]
 *
 * With several -module groups the instances run in lock-step on a virtual MS-CAN bus: every frame one module
 * transmits is delivered to all the others (and to the log); frames from -can stimulus files go to every module.
 *
 * module options : -eeprom FILE -adc CH VAL -adcall VAL -pin PORTIDX VAL -uart FILE -rom FILE -log FILE -out DIR
 *                  -name NAME -watch PC -call PC -wmem ADDR -rwatch ADDR -trace-at PC -trace-insns N -gdcv -eev -lcdv
 * global options  : -ms N -can FILE -can-offset MS -snap MS -quiet -flip -width N -xoff N -fhist FILE -busv
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"

#define MAXMOD 4
static sys_t *mods[MAXMOD]; static int nmod;

static void usage(void)
{
    fprintf(stderr, "usage: fdim-emu app.bin [options] | fdim-emu -module a.bin [opts] -module b.bin [opts] [global opts]\n"
                    "  global: -ms N -can FILE -can-offset MS -snap MS -quiet -flip -width N -xoff N -fhist FILE -busv\n"
                    "  module: -eeprom FILE -adc CH VAL -adcall VAL -pin IDX VAL -uart FILE -rom FILE -log FILE -out DIR -name NAME\n"
                    "          -watch PC -call PC -wmem ADDR -rwatch ADDR -trace-at PC -trace-insns N -gdcv -eev -ipod (attach the iPod simulator to UARTA1)\n");
    exit(1);
}

/* ------------------------------------------------------------------ stimulus files */
typedef struct { uint64_t at; can_frame_t f; } sched_t;
static sched_t sched[200000]; static int nsched;
static double can_offset_ms = 500;
typedef struct { uint64_t at; int n; uint8_t d[256]; } usched_t;

static int hexval(const char *s, int n){ int v = 0; for (int i = 0; i < n; i++) { char c = s[i]; v <<= 4; if (c >= '0' && c <= '9') v |= c - '0'; else if (c >= 'a' && c <= 'f') v |= c - 'a' + 10; else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10; else return -1; } return v; }

/* CAN input formats (auto-detected per line):
 *   simple : "ms id d0 d1 ... d7"                 (ms = emulated time, hex id/bytes)
 *   candump: "(1700000000.123456) can0 406#0404040404040404"   (timestamps rebased to can_offset_ms)
 *   candump: "  can0  406   [8]  04 04 04 04 04 04 04 04"
 */
static void load_can_script(const char *path)
{
    FILE *f = fopen(path, "r"); if (!f) { perror(path); return; }
    char line[512]; double t0 = -1;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        sched_t *s = &sched[nsched]; memset(s, 0, sizeof *s);
        char *p = line; while (*p == ' ' || *p == '\t') p++;
        /* channel routing: an interface name 'can1' (or 'hs') in the line targets CAN channel 1 (IPC HS-CAN); default channel 0 */
        if (strstr(p, " can1 ") || strncmp(p, "can1 ", 5) == 0 || strstr(p, " hs ")) s->f.ch = 1; else s->f.ch = 0;
        if (*p == '(') {
            double ts = atof(p + 1); char *h = strchr(p, '#'); if (!h) continue;
            char *idp = h; while (idp > p && idp[-1] != ' ') idp--;
            int idlen = h - idp; unsigned id = hexval(idp, idlen);
            if (t0 < 0) t0 = ts;
            s->at = (uint64_t)((can_offset_ms + (ts - t0) * 1000.0) * (CPU_HZ / 1000));
            s->f.id = id; s->f.ext = idlen > 3;
            int k = 0; const char *d = h + 1; if (*d == 'R') continue;
            while (k < 8 && hexval(d, 2) >= 0) { s->f.data[k++] = hexval(d, 2); d += 2; }
            s->f.dlc = k; nsched++;
        } else if (strstr(p, "[") && strncmp(p, "can", 3) == 0) {
            char *idp = strchr(p, ' '); while (*idp == ' ') idp++;
            unsigned id = strtoul(idp, NULL, 16); char *b = strchr(p, ']'); if (!b) continue; b++;
            s->at = (uint64_t)((can_offset_ms + nsched * 1.0) * (CPU_HZ / 1000));
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

static int cmp_sched(const void *a, const void *b) { uint64_t x = ((const sched_t *)a)->at, y = ((const sched_t *)b)->at; return x < y ? -1 : x > y; }

static int load_uart_script(const char *path, usched_t *u, int max)
{
    FILE *f = fopen(path, "r"); if (!f) { perror(path); return 0; }
    char line[2048]; int n = 0;
    while (fgets(line, sizeof line, f) && n < max) {
        if (line[0] == '#' || line[0] == '\n') continue;
        double ms; int k; if (sscanf(line, "%lf%n", &ms, &k) < 1) continue;
        u[n].at = (uint64_t)(ms * (CPU_HZ / 1000)); u[n].n = 0;
        char *p = line + k; unsigned b; int j;
        while (u[n].n < 256 && sscanf(p, "%x%n", &b, &j) == 1) { u[n].d[u[n].n++] = b; p += j; }
        if (u[n].n) n++;
    }
    fclose(f); fprintf(stderr, "loaded %d scheduled UART bursts\n", n); return n;
}

/* ------------------------------------------------------------------ per-module configuration */
typedef struct {
    const char *bin, *logf, *rom, *eeprom_path, *uartscript, *outdir, *name, *ramdump;
    uint32_t trace_at; uint64_t trace_insns; int adc_override[16]; uint32_t pins[16][2]; int npins;
    uint32_t watch[16]; int nwatch; uint32_t wmem[16], wmem_last[16]; int nwmem; uint32_t callw[16]; int ncallw; uint32_t rwatch;
    int gdcv, eev, lcdv, ipod; uint64_t trace_started; usched_t *usched; int nusched, ui;
    /* bus: frames waiting for delivery into this module */
    sched_t inq[4096]; int inq_head, inq_count;
} modcfg_t;
static modcfg_t cfg[MAXMOD];
static int quiet, flip, width = 240, xoff = 16, busv; static double snap_ms, run_ms = 10000; static const char *fhist;

static int parse_module_opt(modcfg_t *c, int argc, char **argv, int i)
{
    if (!strcmp(argv[i], "-eeprom") && i + 1 < argc) { c->eeprom_path = argv[++i]; return i; }
    if (!strcmp(argv[i], "-adc") && i + 2 < argc) { int ch = atoi(argv[++i]); int v = atoi(argv[++i]); if (ch >= 0 && ch < 16) c->adc_override[ch] = v + 1; return i; }
    if (!strcmp(argv[i], "-adcall") && i + 1 < argc) { int v = atoi(argv[++i]); for (int k = 0; k < 16; k++) c->adc_override[k] = v + 1; return i; }
    if (!strcmp(argv[i], "-pin") && i + 2 < argc && c->npins < 16) { c->pins[c->npins][0] = strtoul(argv[++i], NULL, 16); c->pins[c->npins][1] = strtoul(argv[++i], NULL, 16); c->npins++; return i; }
    if (!strcmp(argv[i], "-uart") && i + 1 < argc) { c->uartscript = argv[++i]; return i; }
    if (!strcmp(argv[i], "-rom") && i + 1 < argc) { c->rom = argv[++i]; return i; }
    if (!strcmp(argv[i], "-log") && i + 1 < argc) { c->logf = argv[++i]; return i; }
    if (!strcmp(argv[i], "-out") && i + 1 < argc) { c->outdir = argv[++i]; return i; }
    if (!strcmp(argv[i], "-ramdump") && i + 1 < argc) { c->ramdump = argv[++i]; return i; }
    if (!strcmp(argv[i], "-name") && i + 1 < argc) { c->name = argv[++i]; return i; }
    if (!strcmp(argv[i], "-trace-at") && i + 1 < argc) { c->trace_at = strtoul(argv[++i], NULL, 16); return i; }
    if (!strcmp(argv[i], "-trace-insns") && i + 1 < argc) { c->trace_insns = strtoull(argv[++i], NULL, 10); return i; }
    if (!strcmp(argv[i], "-watch") && i + 1 < argc && c->nwatch < 16) { c->watch[c->nwatch++] = strtoul(argv[++i], NULL, 16); return i; }
    if (!strcmp(argv[i], "-call") && i + 1 < argc && c->ncallw < 16) { c->callw[c->ncallw++] = strtoul(argv[++i], NULL, 16); return i; }
    if (!strcmp(argv[i], "-wmem") && i + 1 < argc && c->nwmem < 16) { c->wmem[c->nwmem] = strtoul(argv[++i], NULL, 16); c->wmem_last[c->nwmem] = 0xDEADBEEF; c->nwmem++; return i; }
    if (!strcmp(argv[i], "-rwatch") && i + 1 < argc) { c->rwatch = strtoul(argv[++i], NULL, 16); return i; }
    if (!strcmp(argv[i], "-gdcv")) { c->gdcv = 1; c->lcdv = 1; return i; }
    if (!strcmp(argv[i], "-eev")) { c->eev = 1; return i; }
    if (!strcmp(argv[i], "-ipod")) { c->ipod = 1; return i; }
    return -1;
}

static int parse_global_opt(int argc, char **argv, int i)
{
    if (!strcmp(argv[i], "-ms") && i + 1 < argc) { run_ms = atof(argv[++i]); return i; }
    if (!strcmp(argv[i], "-can") && i + 1 < argc) { load_can_script(argv[++i]); return i; }
    if (!strcmp(argv[i], "-can-offset") && i + 1 < argc) { can_offset_ms = atof(argv[++i]); return i; }
    if (!strcmp(argv[i], "-snap") && i + 1 < argc) { snap_ms = atof(argv[++i]); return i; }
    if (!strcmp(argv[i], "-quiet")) { quiet = 1; return i; }
    if (!strcmp(argv[i], "-flip")) { flip = 1; return i; }
    if (!strcmp(argv[i], "-busv")) { busv = 1; return i; }
    if (!strcmp(argv[i], "-width") && i + 1 < argc) { width = atoi(argv[++i]); return i; }
    if (!strcmp(argv[i], "-xoff") && i + 1 < argc) { xoff = atoi(argv[++i]); return i; }
    if (!strcmp(argv[i], "-fhist") && i + 1 < argc) { extern int g_flash_hist_on; g_flash_hist_on = 1; fhist = argv[++i]; return i; }
    return -1;
}

/* ------------------------------------------------------------------ virtual bus */
static FILE *busf;
static void bus_tx_hook(sys_t *from, const can_frame_t *f)
{
    int src = -1; for (int m = 0; m < nmod; m++) if (mods[m] == from) src = m;
    if (busf) { fprintf(busf, "%10.3f %-6s %03X [%d]", from->cpu.cycles / (CPU_HZ / 1000.0), cfg[src].name, f->id, f->dlc); for (int k = 0; k < f->dlc; k++) fprintf(busf, " %02X", f->data[k]); fprintf(busf, "\n"); }
    for (int m = 0; m < nmod; m++) {
        if (m == src) continue;
        modcfg_t *c = &cfg[m]; if (c->inq_count >= 4096) continue;
        sched_t *q = &c->inq[(c->inq_head + c->inq_count) % 4096]; q->at = from->cpu.cycles + CPU_HZ / 1000 / 4;  /* ~0.25 ms (frame time at 125 kbit/s) */
        q->f = *f; c->inq_count++;
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) usage();
    int i = 1;
    if (strcmp(argv[1], "-module") != 0) { cfg[0].bin = argv[1]; nmod = 1; i = 2; }
    modcfg_t *cur = nmod ? &cfg[0] : NULL;
    for (; i < argc; i++) {
        if (!strcmp(argv[i], "-module") && i + 1 < argc) { if (nmod >= MAXMOD) usage(); cur = &cfg[nmod++]; cur->bin = argv[++i]; continue; }
        int j = parse_global_opt(argc, argv, i); if (j >= 0) { i = j; continue; }
        if (cur) { j = parse_module_opt(cur, argc, argv, i); if (j >= 0) { i = j; continue; } }
        fprintf(stderr, "unknown option %s\n", argv[i]); usage();
    }
    if (!nmod) usage();
    qsort(sched, nsched, sizeof sched[0], cmp_sched);

    for (int m = 0; m < nmod; m++) {
        modcfg_t *c = &cfg[m];
        FILE *f = fopen(c->bin, "rb"); if (!f) { perror(c->bin); return 1; }
        fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
        uint8_t *app = malloc(n); if (fread(app, 1, n, f) != (size_t)n) { perror("read"); return 1; } fclose(f);
        sys_t *s = calloc(1, sizeof *s); mods[m] = s;
        if (c->eeprom_path) { FILE *ef = fopen(c->eeprom_path, "rb"); if (ef) { memset(s->eeprom, 0xFF, 512); if (fread(s->eeprom, 1, 512, ef) > 0) s->eeprom_loaded = 1; fclose(ef); } }
        s->eeprom_verbose = c->eev;
        sys_init(s, app, n);
        if (!c->name) c->name = s->board == 2 ? "IPC" : s->board == 0 ? "FDIM-CB" : s->module == 2 ? "AIM" : "FDIM-AC";
        s->name = c->name;
        if (!c->outdir) c->outdir = nmod > 1 ? c->name : ".";
        if (nmod > 1) { char cmd[600]; snprintf(cmd, sizeof cmd, "mkdir -p '%s'", c->outdir); if (system(cmd)) {} }
        if (c->logf) s->log = fopen(c->logf, "w");
        else if (nmod > 1) { char path[600]; snprintf(path, sizeof path, "%s/run.log", c->outdir); s->log = fopen(path, "w"); }
        else s->log = stderr;
        s->log_sfr = !quiet; s->log_can = 1; s->log_gdc = 1; s->log_unknown = !quiet;
        for (int k = 0; k < 16; k++) s->adc_val[k] = c->adc_override[k] ? c->adc_override[k] - 1 : 700;
        for (int k = 0; k < c->npins; k++) if (c->pins[k][0] < 0x40) s->pin_in[c->pins[k][0]] = c->pins[k][1];
        if (s->board == 0) { s->gdc = gdc_create(c->rom, NULL); extern void gdc_set_log(struct gdc *g, FILE *f); gdc_set_log(s->gdc, s->log); if (c->gdcv) { extern int gdc_verbose; gdc_verbose = 1; } }
        else if (s->module != 2) { s->lcd = lcd_create(); lcd_set_log(s->lcd, s->log, c->lcdv); if (s->board == 2) lcd_force_gray(s->lcd, 1); }
        if (c->uartscript) { c->usched = calloc(4096, sizeof(usched_t)); c->nusched = load_uart_script(c->uartscript, c->usched, 4096); }
        if (nmod > 1) s->tx_hook = bus_tx_hook;
        if (c->ipod) { ipodsim_init(s->log); s->uart_tx_hook = ipodsim_rx; }
        fprintf(stderr, "module %d: %s = %s\n", m, c->name, s->board == 2 ? "FG instrument cluster full dump (SJ3-class, CAN0=MS 125k, CAN1=HS 500k, IIC EEPROM)" : s->board == 0 ? "8R29-14D017-CB colour FDIM (GDC, CAN1)" : s->module == 2 ? "8R29-14F029-AB AIM / Front Entertainment Module (CAN0, UARTA1 iAP, CSIB1)" : "8R29-14D017-AC mono FDIM (S1D15721, CAN0)");
    }
    if (nmod > 1) { busf = fopen("bus.log", "w"); fprintf(stderr, "virtual bus: %d modules, frames logged to bus.log\n", nmod); }

    uint64_t total = (uint64_t)(run_ms * (CPU_HZ / 1000));
    uint64_t snap_every = snap_ms > 0 ? (uint64_t)(snap_ms * (CPU_HZ / 1000)) : 0, next_snap = snap_every;
    int si = 0, frame = 0;
    uint64_t slice = CPU_HZ / 1000, now = 0;   /* 1 ms lock-step slices */
    for (int m = 0; m < nmod; m++) if (cfg[m].trace_at) slice = 1;
    extern uint32_t g_watch[16]; extern int g_nwatch; extern uint32_t g_callw[16]; extern int g_ncallw; extern uint32_t g_rwatch;
    while (now < total) {
        /* external stimulus to every module */
        while (si < nsched && sched[si].at <= now) { for (int m = 0; m < nmod; m++) { S = mods[m]; sys_can_inject(mods[m], &sched[si].f); } si++; }
        for (int m = 0; m < nmod; m++) {
            sys_t *s = mods[m]; modcfg_t *c = &cfg[m]; S = s;
            if (s->cpu.halted == 2) continue;
            memcpy(g_watch, c->watch, sizeof g_watch); g_nwatch = c->nwatch; memcpy(g_callw, c->callw, sizeof g_callw); g_ncallw = c->ncallw; g_rwatch = c->rwatch;
            if (c->trace_at && !s->cpu.trace && s->cpu.pc == c->trace_at) { s->cpu.trace = 1; c->trace_started = s->cpu.insns; }
            if (s->cpu.trace && c->trace_insns && s->cpu.insns > c->trace_started + c->trace_insns) { s->cpu.trace = 0; c->trace_at = 0; }
            while (c->inq_count && c->inq[c->inq_head].at <= s->cpu.cycles) { sys_can_inject(s, &c->inq[c->inq_head].f); c->inq_head = (c->inq_head + 1) % 4096; c->inq_count--; }
            while (c->ui < c->nusched && c->usched[c->ui].at <= s->cpu.cycles) { sys_uart1_inject(s, c->usched[c->ui].d, c->usched[c->ui].n); c->ui++; }
            sys_run(s, slice);
            if (c->ipod) ipodsim_tick(s);
            for (int k = 0; k < c->nwmem; k++) { uint32_t v = bus_read(&s->cpu, c->wmem[k], 4); if (v != c->wmem_last[k]) { fprintf(s->log, "[%llu] WMEM %08x = %08x (was %08x)\n", (unsigned long long)s->cpu.cycles, c->wmem[k], v, c->wmem_last[k]); c->wmem_last[k] = v; } }
        }
        now += slice;
        if (snap_every && now >= next_snap) {
            for (int m = 0; m < nmod; m++) { sys_t *s = mods[m]; S = s; char path[600];
                if (s->gdc) { snprintf(path, sizeof path, "%s/frame_%04d.ppm", cfg[m].outdir, frame); gdc_render(s->gdc, path); }
                else if (s->lcd) { snprintf(path, sizeof path, "%s/frame_%04d.pgm", cfg[m].outdir, frame); lcd_render(s->lcd, path, width, xoff, flip); } }
            frame++; next_snap += snap_every;
        }
    }
    for (int m = 0; m < nmod; m++) {
        sys_t *s = mods[m]; modcfg_t *c = &cfg[m]; S = s; char path[600];
        fprintf(stderr, "[%s] stopped at cycle %llu (%.1f ms), %llu insns, pc=%06x halted=%d\n", c->name, (unsigned long long)s->cpu.cycles, s->cpu.cycles / (CPU_HZ / 1000.0), (unsigned long long)s->cpu.insns, s->cpu.pc, s->cpu.halted);
        fprintf(stderr, "[%s] irqs:", c->name); for (int k = 0; k < NVEC / 0x10; k++) if (s->irq_count[k]) fprintf(stderr, " %03x:%llu", 0x80 + k * 0x10, (unsigned long long)s->irq_count[k]); fprintf(stderr, "\n");
        extern void sys_dump_can(sys_t *s, FILE *f); sys_dump_can(s, stderr);
        if (c->eeprom_path && s->eeprom_dirty) { FILE *ef = fopen(c->eeprom_path, "wb"); if (ef) { fwrite(s->eeprom, 1, 512, ef); fclose(ef); fprintf(stderr, "[%s] EEPROM image updated: %s\n", c->name, c->eeprom_path); } }
        if (c->ramdump) { FILE *rf = fopen(c->ramdump, "wb"); if (rf) { fwrite(s->ram, 1, s->ram_size, rf); fclose(rf); fprintf(stderr, "[%s] RAM dumped (%u bytes @%08x): %s\n", c->name, s->ram_size, s->ram_base, c->ramdump); } }
        if (s->gdc) {
            snprintf(path, sizeof path, "%s/final.ppm", c->outdir); gdc_render(s->gdc, path);
            snprintf(path, sizeof path, "%s/objects.txt", c->outdir); FILE *of = fopen(path, "w"); if (of) { gdc_dump_objects(s->gdc, of); fclose(of); }
        } else if (s->lcd) {
            snprintf(path, sizeof path, "%s/final.pgm", c->outdir); lcd_render(s->lcd, path, width, xoff, flip);
            snprintf(path, sizeof path, "%s/final_lcdram.bin", c->outdir); lcd_save_ram(s->lcd, path);
            lcd_dump_state(s->lcd, stderr);
        }
        if (s->log && s->log != stderr) fclose(s->log);
    }
    if (fhist) { extern uint32_t g_flash_hist[]; FILE *hf = fopen(fhist, "w"); if (hf) { for (int k = 0; k < FLASH_SIZE / 64; k++) if (g_flash_hist[k]) fprintf(hf, "%06x %u\n", k * 64, g_flash_hist[k]); fclose(hf); } }
    if (busf) fclose(busf);
    return 0;
}
