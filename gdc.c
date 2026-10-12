/* Graphics display controller model (sprite/object engine inferred from FDIM firmware).
 * Bus: A0-A2 select "ports":
 *   0: OAM data window      (auto-inc when reg4 bit1)   address regs 5(hi),6(lo)   4 KB
 *   1: region-1 data window (auto-inc when reg4 bit0)   address regs 7(hi),8(lo)   1 KB  (CLUT?)
 *   2: status (bit6 = busy)
 *   3: image memory window  (auto-inc when reg4 bit2)   address regs 9..12 (32-bit BE)  32 MB
 *   4: register index (bit7 = burst/auto-increment)
 *   5: register data
 * Object descriptor (12 bytes, big-endian fields):
 *   b0: [7:4] flags  [3:0] y[11:8]   b1: y[7:0]
 *   b2: [7:4] flags  [3:0] x[11:8]   b3: x[7:0]
 *   b4: bit6 visible, [5:0] width/8-1      b5: height-1
 *   b6..b9: image address               b10,b11: attributes (0x40 0x40)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"

#define OAM_SIZE   0x1000
#define MEM1_SIZE  0x400
#define IMG_SIZE   0x400000
#define NOBJ       340
#define CANVAS_W   640
#define CANVAS_H   320

typedef struct gdc {
    uint8_t regs[0x30];
    uint8_t oam[OAM_SIZE];
    uint8_t mem1[MEM1_SIZE];
    uint8_t *img; size_t imglen; int have_rom;
    uint32_t a_oam, a_mem1, a_img;
    int reg_idx, reg_burst;
    uint64_t busy_until;
    int irq, irq_enabled; uint64_t next_vsync;
    int in_reset;
    unsigned long writes_oam, writes_mem1, writes_img, writes_reg;
    FILE *log;
} gdc_t;

static uint32_t rd32be(const uint8_t *p){ return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }

gdc_t *gdc_create(const char *image_rom, const char *dir_file)
{
    (void)dir_file;
    gdc_t *g = calloc(1, sizeof *g);
    g->img = calloc(IMG_SIZE, 1); g->imglen = IMG_SIZE;
    if (image_rom) {
        FILE *f = fopen(image_rom, "rb");
        if (f) { size_t n = fread(g->img, 1, IMG_SIZE, f); fclose(f); g->have_rom = 1; fprintf(stderr, "GDC: loaded %zu bytes of image ROM\n", n); }
    }
    g->next_vsync = ~0ULL;
    return g;
}

void gdc_reset(gdc_t *g)
{
    memset(g->regs, 0, sizeof g->regs);
    g->a_oam = g->a_mem1 = g->a_img = 0; g->reg_idx = 0; g->reg_burst = 0; g->busy_until = 0; g->irq = 0;
    g->in_reset = 1; g->irq_enabled = 0; g->next_vsync = ~0ULL;
}

int gdc_ready(gdc_t *g){ (void)g; return 1; }   /* READY pin high = ready */
int gdc_irq_pending(gdc_t *g){ int r = g->irq; g->irq = 0; return r; }

void gdc_tick(gdc_t *g, uint64_t cycles)
{
    if (g->next_vsync != ~0ULL && cycles >= g->next_vsync) { g->next_vsync += CPU_HZ / 60; if (g->irq_enabled) g->irq = 1; }
}

static void reg_write(gdc_t *g, int idx, uint8_t v)
{
    if (idx >= 0x30) return;
    uint8_t old = g->regs[idx];
    g->regs[idx] = v; g->writes_reg++;
    switch (idx) {
    case 5: g->a_oam = (g->a_oam & 0x00FF) | (v << 8); break;
    case 6: g->a_oam = (g->a_oam & 0xFF00) | v; break;
    case 7: g->a_mem1 = (g->a_mem1 & 0x00FF) | (v << 8); break;
    case 8: g->a_mem1 = (g->a_mem1 & 0xFF00) | v; break;
    case 9: g->a_img = (g->a_img & 0x00FFFFFF) | ((uint32_t)v << 24); break;
    case 10: g->a_img = (g->a_img & 0xFF00FFFF) | (v << 16); break;
    case 11: g->a_img = (g->a_img & 0xFFFF00FF) | (v << 8); break;
    case 12: g->a_img = (g->a_img & 0xFFFFFF00) | v; break;
    case 0: /* control: any enable bit -> start vsync interrupts */
        if (v) { if (g->next_vsync == ~0ULL) g->next_vsync = S->cpu.cycles + CPU_HZ / 60; g->irq_enabled = 1; }
        else { g->irq_enabled = 0; }
        break;
    }
    if (g->log && (idx < 5 || idx > 12) && old != v) fprintf(g->log, "[%llu] GDC reg[%02x] = %02x\n", (unsigned long long)S->cpu.cycles, idx, v);
}

int gdc_verbose;
void gdc_bus_write(gdc_t *g, int addr, uint8_t data)
{
    if (gdc_verbose && g->log && (addr & 7) != 0) fprintf(g->log, "[%llu]   W port%d %02x%s\n", (unsigned long long)S->cpu.cycles, addr & 7, data, (addr&7)==5 ? "" : "");
    switch (addr & 7) {
    case 0: if (g->a_oam < OAM_SIZE) g->oam[g->a_oam] = data; g->writes_oam++; if (g->regs[4] & 2) g->a_oam++; break;
    case 1: if (g->a_mem1 < MEM1_SIZE) g->mem1[g->a_mem1] = data; g->writes_mem1++; if (g->regs[4] & 1) g->a_mem1++; break;
    case 3: if (g->a_img < g->imglen) g->img[g->a_img] = data; g->writes_img++; if (g->regs[4] & 4) g->a_img++; g->busy_until = S->cpu.cycles + 40; break;
    case 4: g->reg_idx = data & 0x7F; g->reg_burst = !!(data & 0x80); break;
    case 5: reg_write(g, g->reg_idx, data); if (g->reg_burst) g->reg_idx = (g->reg_idx + 1) & 0x7F; break;
    default:
        if (g->log) fprintf(g->log, "[%llu] GDC write port %d = %02x\n", (unsigned long long)S->cpu.cycles, addr & 7, data);
        break;
    }
}

uint8_t gdc_bus_read(gdc_t *g, int addr)
{
    uint8_t rv = 0;
    if (gdc_verbose && g->log && (addr & 7) != 0) {
        switch (addr & 7) { case 2: rv = (S->cpu.cycles < g->busy_until) ? 0x40 : 0x00; break; case 5: rv = g->reg_idx < 0x30 ? g->regs[g->reg_idx] : 0; break; default: rv = 0; }
        fprintf(g->log, "[%llu]   R port%d -> %02x (regidx %02x)\n", (unsigned long long)S->cpu.cycles, addr & 7, rv, g->reg_idx);
    }
    switch (addr & 7) {
    case 0: { uint8_t v = g->a_oam < OAM_SIZE ? g->oam[g->a_oam] : 0; if (g->regs[4] & 2) g->a_oam++; return v; }
    case 1: { uint8_t v = g->a_mem1 < MEM1_SIZE ? g->mem1[g->a_mem1] : 0; if (g->regs[4] & 1) g->a_mem1++; return v; }
    case 2: return (S->cpu.cycles < g->busy_until) ? 0x40 : 0x00;
    case 3: {
        uint8_t v;
        if (g->a_img >= 0x01FFFFF0) v = (S->portc[0x0C] & 0x20) ? 0xAA : 0x55;   /* bus test pattern selected by PCM5 */
        else v = g->a_img < g->imglen ? g->img[g->a_img] : 0;
        if (g->regs[4] & 4) g->a_img++; return v; }
    case 5: { uint8_t v = g->reg_idx < 0x30 ? g->regs[g->reg_idx] : 0; if (g->reg_burst) g->reg_idx = (g->reg_idx + 1) & 0x7F; return v; }
    default:
        if (g->log) fprintf(g->log, "[%llu] GDC read port %d\n", (unsigned long long)S->cpu.cycles, addr & 7);
        return 0;
    }
}

/* ---- rendering ---- */
static int sx12(int v){ v &= 0xFFF; return v & 0x800 ? v - 0x1000 : v; }

void gdc_dump_objects(gdc_t *g, FILE *f)
{
    int n = 0;
    for (int i = 0; i < NOBJ; i++) {
        const uint8_t *d = &g->oam[i * 12];
        int vis = d[4] & 0x40; int w = ((d[4] & 0x3F) + 1) * 8, h = d[5] + 1;
        if (!vis && !d[0] && !d[2] && !d[4] && !d[5]) continue;
        fprintf(f, "obj %3d: %s x=%4d y=%4d %3dx%-3d addr=%06x fl=%x/%x attr=%02x%02x raw=", i, vis ? "VIS" : "   ", sx12((d[2] << 8) | d[3]), sx12((d[0] << 8) | d[1]), w, h, rd32be(d + 6), d[0] >> 4, d[2] >> 4, d[10], d[11]);
        for (int k = 0; k < 12; k++) fprintf(f, "%02x", d[k]);
        fprintf(f, "\n"); n++;
    }
    fprintf(f, "(%d objects defined, regs:", n);
    for (int i = 0; i < 0x30; i++) fprintf(f, " %02x", g->regs[i]);
    fprintf(f, ")\n");
}

static void put_px(uint8_t *cv, int x, int y, uint8_t r, uint8_t gg, uint8_t b)
{
    if (x < 0 || y < 0 || x >= CANVAS_W || y >= CANVAS_H) return;
    uint8_t *p = cv + (y * CANVAS_W + x) * 3; p[0] = r; p[1] = gg; p[2] = b;
}

void gdc_render_json(gdc_t *g, const char *path)
{
    FILE *f = fopen(path, "w"); if (!f) return;
    fprintf(f, "{\"cycle\": %llu, \"regs\": [", (unsigned long long)S->cpu.cycles);
    for (int i = 0; i < 0x30; i++) fprintf(f, "%s%d", i ? "," : "", g->regs[i]);
    fprintf(f, "], \"objects\": [");
    int first = 1;
    for (int i = 0; i < NOBJ; i++) {
        const uint8_t *d = &g->oam[i * 12];
        if (!(d[4] & 0x40)) continue;
        int w = ((d[4] & 0x3F) + 1) * 8, h = d[5] + 1;
        fprintf(f, "%s{\"i\":%d,\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d,\"addr\":%u,\"fy\":%d,\"fx\":%d,\"a\":%d}", first ? "" : ",", i,
                sx12((d[2] << 8) | d[3]), sx12((d[0] << 8) | d[1]), w, h, rd32be(d + 6), d[0] >> 4, d[2] >> 4, (d[10] << 8) | d[11]);
        first = 0;
    }
    fprintf(f, "]}\n"); fclose(f);
}

void gdc_render(gdc_t *g, const char *path)
{
    { char jp[600]; snprintf(jp, sizeof jp, "%s.json", path); gdc_render_json(g, jp); }
    uint8_t *cv = calloc(CANVAS_W * CANVAS_H * 3, 1);
    int minx = CANVAS_W, miny = CANVAS_H, maxx = -1, maxy = -1;
    /* objects drawn in index order (later = on top) */
    for (int i = 0; i < NOBJ; i++) {
        const uint8_t *d = &g->oam[i * 12];
        if (!(d[4] & 0x40)) continue;
        int w = ((d[4] & 0x3F) + 1) * 8, h = d[5] + 1;
        int x0 = sx12((d[2] << 8) | d[3]), y0 = sx12((d[0] << 8) | d[1]);
        uint32_t addr = rd32be(d + 6);
        if (x0 < minx) minx = x0; if (y0 < miny) miny = y0; if (x0 + w > maxx) maxx = x0 + w; if (y0 + h > maxy) maxy = y0 + h;
        for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
            uint8_t r, gg, b;
            size_t off = addr + (size_t)(y * w + x) * 2;
            if (g->have_rom && off + 1 < g->imglen) {
                uint16_t px = (g->img[off] << 8) | g->img[off + 1];
                r = (px >> 11) << 3; gg = ((px >> 5) & 0x3F) << 2; b = (px & 0x1F) << 3;
            } else {
                /* placeholder: hashed colour per image, border, diagonal for big ones */
                uint32_t hsh = addr * 2654435761u;
                r = 40 + (hsh >> 24) / 2; gg = 40 + ((hsh >> 16) & 0xFF) / 2; b = 60 + ((hsh >> 8) & 0xFF) / 2;
                if (x == 0 || y == 0 || x == w - 1 || y == h - 1) { r = gg = b = 230; }
            }
            put_px(cv, x0 + x, y0 + y, r, gg, b);
        }
    }
    FILE *f = fopen(path, "wb");
    if (f) {
        fprintf(f, "P6\n%d %d\n255\n", CANVAS_W, CANVAS_H);
        fwrite(cv, 1, CANVAS_W * CANVAS_H * 3, f);
        fclose(f);
    }
    if (g->log) fprintf(g->log, "[%llu] GDC render -> %s  extents x=%d..%d y=%d..%d\n", (unsigned long long)S->cpu.cycles, path, minx, maxx, miny, maxy);
    free(cv);
}

void gdc_set_log(gdc_t *g, FILE *f){ g->log = f; }
