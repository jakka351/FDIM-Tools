/* Epson S1D15721 (256 SEG x 80 COM + icon line, 4-gray/binary) LCD controller model.
 * Driven by the FDIM "Low Display" firmware over an 8080-style bus on port DL:
 *   PDL0-7 = D0-7, PDLH bit0 = /RD, bit1 = /WR, bit2 = A0 (1 = data/parameter), bit3 = /CS, bit4 = /RES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include "sys.h"

#define LCD_PAGES 21
#define LCD_COLS  256

struct lcd {
    uint8_t ram[LCD_PAGES][LCD_COLS];
    int page, col;
    int start_line, com_rev, adc_rev, input_dir;      /* input_dir: 0 = column, 1 = page */
    int mode_k;              /* display mode K1:K0 (bit0: 1 = binary, 0 = 4-gray) */
    int mode_cfg;            /* last mode set by cmd 66 (survives a controller reset; what the glass shows) */
    int force_gray;          /* IPC panel: data is 4-grey regardless of the 66 parameter */
    int gray_pattern;
    int lines_u, start_s;    /* display line number set */
    int on, reverse, all_on, osc_on, power_save, discharge, temp_sensor;
    int volume, v3_range, bias, osc_freq, power_ctl, temp_grad, mls, nline, nline_on;
    int cmd;                 /* current command */
    int params_left;         /* parameters still expected for cmd */
    int param_idx;
    int dummy_read;
    int in_reset;
    /* stats / logging */
    uint64_t writes, cmds;
    int burst;               /* data bytes in the current 1D burst */
    int burst_page, burst_col;
    FILE *log; int verbose;
    uint64_t last_change;    /* cycle of last RAM change */
    uint64_t frame_seq;
};

static const struct { int code; const char *name; int nparam; } cmdtab[] = {
    {0xAE,"DISPLAY OFF",0},{0xAF,"DISPLAY ON",0},{0xA6,"NORMAL",0},{0xA7,"REVERSE",0},{0xA4,"ALL-ON OFF",0},{0xA5,"ALL-ON",0},
    {0xC4,"COM NORMAL",0},{0xC5,"COM REVERSE",0},{0xCC,"COM REVERSE",0},{0x8A,"START LINE",1},{0xB1,"PAGE",1},{0x13,"COLUMN",1},
    {0x1D,"DATA WRITE",0},{0x1C,"DATA READ",0},{0x84,"INPUT DIR=COLUMN",0},{0x85,"INPUT DIR=PAGE",0},{0xA0,"ADC NORMAL",0},{0xA1,"ADC REVERSE",0},
    {0x36,"N-LINE INV SET",1},{0xE4,"N-LINE OFF",0},{0xE5,"N-LINE ON",0},{0x66,"DISPLAY MODE",1},{0x39,"GRAY PATTERN",1},{0x6D,"DISPLAY LINES",2},
    {0xE0,"RMW",0},{0xEE,"END",0},{0xAA,"OSC OFF",0},{0xAB,"OSC ON",0},{0x5F,"OSC FREQ",1},{0x25,"POWER CTL",1},{0x2B,"V3 RANGE",1},
    {0xA2,"LCD BIAS",1},{0x81,"VOLUME",1},{0xE2,"DISCHARGE OFF",0},{0xE3,"DISCHARGE ON",0},{0xA8,"POWER SAVE OFF",0},{0xA9,"POWER SAVE ON",0},
    {0x4E,"TEMP GRADIENT",1},{0x8E,"STATUS READ",0},{0x28,"TEMP SENSOR OFF",0},{0x29,"TEMP SENSOR ON",0},{0xE7,"MLS DRIVE",1},{0xE3,"NOP",0},
    {-1,NULL,0}
};

struct lcd *lcd_create(void)
{
    struct lcd *l = calloc(1, sizeof *l);
    l->lines_u = 0x13; l->in_reset = 1;
    return l;
}
void lcd_set_log(struct lcd *l, FILE *f, int verbose) { l->log = f; l->verbose = verbose; }

static void lcd_log(struct lcd *l, const char *fmt, ...)
{
    if (!l->log) return;
    va_list ap; va_start(ap, fmt); fprintf(l->log, "[%llu] LCD ", (unsigned long long)S->cpu.cycles); vfprintf(l->log, fmt, ap); fputc('\n', l->log); va_end(ap);
}

static void flush_burst(struct lcd *l)
{
    if (l->burst) { if (l->verbose || l->burst > 0) lcd_log(l, "data burst: %d bytes from page %d col %d", l->burst, l->burst_page, l->burst_col); l->burst = 0; }
}

void lcd_reset(struct lcd *l, int asserted)
{
    if (asserted) {
        flush_burst(l);
        lcd_log(l, "RESET asserted");
        l->page = l->col = 0; l->start_line = 0; l->com_rev = 0; l->adc_rev = 0; l->input_dir = 0; l->mode_k = 0;
        l->lines_u = 0x13; l->start_s = 0; l->on = 0; l->reverse = 0; l->all_on = 0; l->osc_on = 0; l->power_save = 0;
        l->cmd = -1; l->params_left = 0; l->in_reset = 1;
    } else { lcd_log(l, "RESET released"); l->in_reset = 0; }
}

static void advance(struct lcd *l)
{
    if (l->input_dir == 0) { l->col = (l->col + 1) & 0xFF; if (l->col == 0) l->page = (l->page + 1) % LCD_PAGES; }
    else { l->page++; if (l->page >= LCD_PAGES) { l->page = 0; l->col = (l->col + 1) & 0xFF; } }
}

void lcd_write(struct lcd *l, int a0, uint8_t d)
{
    if (!a0) {   /* command */
        if (l->cmd != 0x1D) flush_burst(l); else if (d != 0x1D) flush_burst(l);
        l->cmds++;
        int np = 0; const char *name = "?";
        for (int i = 0; cmdtab[i].code >= 0; i++) if (cmdtab[i].code == d) { np = cmdtab[i].nparam; name = cmdtab[i].name; break; }
        l->cmd = d; l->params_left = np; l->param_idx = 0; l->dummy_read = 1;
        switch (d) {
        case 0xAE: l->on = 0; break; case 0xAF: l->on = 1; break;
        case 0xA6: l->reverse = 0; break; case 0xA7: l->reverse = 1; break;
        case 0xA4: l->all_on = 0; break; case 0xA5: l->all_on = 1; break;
        case 0xC4: l->com_rev = 0; break; case 0xC5: case 0xCC: case 0xCD: l->com_rev = 1; break;
        case 0x84: l->input_dir = 0; break; case 0x85: l->input_dir = 1; break;
        case 0xA0: l->adc_rev = 0; break; case 0xA1: l->adc_rev = 1; break;
        case 0xE4: l->nline_on = 0; break; case 0xE5: l->nline_on = 1; break;
        case 0xAA: l->osc_on = 0; break; case 0xAB: l->osc_on = 1; break;
        case 0xA8: l->power_save = 0; break; case 0xA9: l->power_save = 1; break;
        case 0xE2: l->discharge = 0; break; case 0xE3: l->discharge = 1; break;
        case 0x28: l->temp_sensor = 0; break; case 0x29: l->temp_sensor = 1; break;
        }
        if (d != 0x1D && (l->verbose || np == 0)) lcd_log(l, "cmd %02x %s", d, name);
        return;
    }
    /* parameter / data */
    if (l->cmd == 0x1D) {
        if (!l->burst) { l->burst_page = l->page; l->burst_col = l->col; }
        if (l->ram[l->page][l->col] != d) l->last_change = S->cpu.cycles;
        l->ram[l->page][l->col] = d; l->writes++; l->burst++;
        advance(l);
        return;
    }
    if (l->params_left > 0) {
        l->params_left--;
        switch (l->cmd) {
        case 0x8A: l->start_line = d; break;
        case 0xB1: l->page = d & 0x1F; if (l->page >= LCD_PAGES) l->page = LCD_PAGES - 1; break;
        case 0x13: l->col = d; break;
        case 0x36: l->nline = d; break;
        case 0x66: l->mode_k = d & 3; l->mode_cfg = l->force_gray ? 0 : l->mode_k; break;
        case 0x39: l->gray_pattern = d; break;
        case 0x6D: if (l->param_idx == 0) l->lines_u = d & 0x3F; else l->start_s = d & 0x3F; break;
        case 0x5F: l->osc_freq = d; break;
        case 0x25: l->power_ctl = d; break;
        case 0x2B: l->v3_range = d; break;
        case 0xA2: l->bias = d; break;
        case 0x81: l->volume = d & 0x7F; break;
        case 0x4E: l->temp_grad = d; break;
        case 0xE7: l->mls = d; break;
        }
        l->param_idx++;
        if (l->verbose || l->params_left == 0) {
            const char *name = "?"; for (int i = 0; cmdtab[i].code >= 0; i++) if (cmdtab[i].code == l->cmd) { name = cmdtab[i].name; break; }
            if (l->cmd == 0x6D) lcd_log(l, "cmd %02x %s: lines=%d (%d lines, 1/%d duty) start block %d", l->cmd, name, l->lines_u, 4 * l->lines_u + 5, 4 * l->lines_u + 12, l->start_s);
            else if (l->cmd == 0x66) lcd_log(l, "cmd 66 DISPLAY MODE: %s, dummy period %s", (d & 1) ? "binary" : "4-gray", (d & 2) ? "lit-off" : "lit");
            else if (l->cmd != 0xB1 && l->cmd != 0x13) lcd_log(l, "cmd %02x %s = %02x", l->cmd, name, d);
            else if (l->verbose) lcd_log(l, "cmd %02x %s = %d", l->cmd, name, d);
        }
        return;
    }
    lcd_log(l, "unexpected data byte %02x after cmd %02x", d, l->cmd);
}

uint8_t lcd_read(struct lcd *l, int a0)
{
    if (!a0) {   /* status: bit7 busy=0, bit6 ADC, bit5 ON/OFF, bit4 reset */
        return (l->adc_rev << 6) | ((!l->on) << 5) | (l->in_reset << 4);
    }
    if (l->cmd == 0x1C) {
        uint8_t v = l->ram[l->page][l->col];
        if (l->dummy_read) { l->dummy_read = 0; if (l->verbose) lcd_log(l, "dummy read page %d col %d", l->page, l->col); return v; }
        advance(l);
        if (l->verbose) lcd_log(l, "read page %d col %d = %02x", l->page, l->col, v);
        return v;
    }
    return 0xFF;
}

uint64_t lcd_last_change(struct lcd *l) { return l->last_change; }
int lcd_is_on(struct lcd *l) { return l->on; }
void lcd_force_gray(struct lcd *l, int on) { l->force_gray = on; if (on) l->mode_cfg = 0; }

/* Render the display: 256 x 81 logical lines in 4-gray mode (4 lines/page, page 20 = icon line),
 * 256 x 168 in binary mode (8 lines/page).  gray level 0..3 -> pixel intensity (3 = fully lit). */
void lcd_render(struct lcd *l, const char *path, int width, int xoff, int flip)
{
    int binary = l->mode_cfg & 1;
    int lpp = binary ? 8 : 4;                     /* lines per page */
    int height = lpp * 20 + (binary ? 8 : 4);     /* pages 0..19 + icon page */
    int W = width > 0 ? width : LCD_COLS;
    FILE *f = fopen(path, "wb"); if (!f) return;
    fprintf(f, "P5\n%d %d\n255\n", W, height);
    for (int y = 0; y < height; y++) {
        int yy = flip ? height - 1 - y : y;
        int page = yy / lpp, line = yy % lpp;
        for (int x = 0; x < W; x++) {
            int xx = (flip ? (W - 1 - x) : x) + xoff;
            uint8_t b = l->ram[page][xx & 0xFF];
            int g = binary ? ((b >> line) & 1) * 3 : ((b >> (2 * line)) & 3);
            if (l->all_on) g = 3;
            if (l->reverse) g = 3 - g;
            if (!l->on) g = g / 3;      /* display off: show RAM contents faintly */
            fputc(g * 85, f);
        }
    }
    fclose(f);
}

/* Copy the current display into a caller buffer as grey levels 0..3 (one byte per
 * pixel), using the exact same mapping as lcd_render. Returns the pixel count and
 * reports width/height. buf must hold at least width * (binary?168:84) bytes.
 * Used by the live mirror to stream the framebuffer with no file I/O. */
int lcd_copy_frame(struct lcd *l, uint8_t *buf, int *out_w, int *out_h, int width, int xoff, int flip)
{
    int binary = l->mode_cfg & 1;
    int lpp = binary ? 8 : 4;
    int height = lpp * 20 + (binary ? 8 : 4);
    int W = width > 0 ? width : LCD_COLS;
    int n = 0;
    for (int y = 0; y < height; y++) {
        int yy = flip ? height - 1 - y : y;
        int page = yy / lpp, line = yy % lpp;
        for (int x = 0; x < W; x++) {
            int xx = (flip ? (W - 1 - x) : x) + xoff;
            uint8_t b = l->ram[page][xx & 0xFF];
            int g = binary ? ((b >> line) & 1) * 3 : ((b >> (2 * line)) & 3);
            if (l->all_on) g = 3;
            if (l->reverse) g = 3 - g;
            if (!l->on) g = g / 3;
            buf[n++] = (uint8_t)g;
        }
    }
    if (out_w) *out_w = W;
    if (out_h) *out_h = height;
    return n;
}

void lcd_dump_state(struct lcd *l, FILE *f)
{
    fprintf(f, "LCD: on=%d mode=%s start_line=%d lines=%d(+icon) start_block=%d com_rev=%d adc_rev=%d input_dir=%s reverse=%d all_on=%d osc=%d/%02x volume=%d v3=%d bias=%d power=%02x temp_grad=%d mls=%02x gray=%02x power_save=%d writes=%llu cmds=%llu\n",
            l->on, (l->mode_k & 1) ? "binary" : "4-gray", l->start_line, 4 * l->lines_u + 4, l->start_s, l->com_rev, l->adc_rev, l->input_dir ? "page" : "column",
            l->reverse, l->all_on, l->osc_on, l->osc_freq, l->volume, l->v3_range, l->bias, l->power_ctl, l->temp_grad, l->mls, l->gray_pattern, l->power_save,
            (unsigned long long)l->writes, (unsigned long long)l->cmds);
}

void lcd_save_ram(struct lcd *l, const char *path)
{
    FILE *f = fopen(path, "wb"); if (!f) return; fwrite(l->ram, 1, sizeof l->ram, f); fclose(f);
}
