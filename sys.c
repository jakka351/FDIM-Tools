/* FDIM system model: memory map, INTC, timers, serial, CAN1, ADC, ports, GDC bus glue */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"

sys_t *S;

#define LOG(...) do { if (S->log) fprintf(S->log, __VA_ARGS__); } while (0)
#define NOW (S->cpu.cycles)

/* ------------------------------------------------------------------ INTC */
void sys_raise(sys_t *s, int vec)
{
    int i = IC_INDEX(vec);
    s->ic[i] |= 0x80;
}

static int ispr_level(sys_t *s)
{
    for (int p = 0; p < 8; p++) if (s->ispr & (1 << p)) return p;
    return 8;
}

int sys_pending_irq(sys_t *s)
{
    uint32_t psw = s->cpu.sr[SR_PSW];
    if (psw & (PSW_ID | PSW_NP)) return 0;
    int best = -1, bestpri = 8, cur = ispr_level(s);
    for (int i = 0; i < NVEC / 0x10 - 8; i++) {
        uint8_t ic = s->ic[i];
        if ((ic & 0xC0) == 0x80) {
            int pri = ic & 7;
            if (pri < bestpri) { bestpri = pri; best = i; }
        }
    }
    if (best < 0 || bestpri >= cur) return 0;
    s->ic[best] &= ~0x80;
    s->ispr |= 1 << bestpri;
    int vec = 0x80 + best * 0x10;
    s->irq_count[best]++;
    cpu_interrupt(&s->cpu, vec, vec);
    return 1;
}

void sys_reti_hook(cpu_t *c, int was_ep, int was_np)
{
    (void)c; (void)was_np;
    if (!was_ep) {   /* returning from a maskable interrupt: clear highest-priority in-service bit */
        for (int p = 0; p < 8; p++) if (S->ispr & (1 << p)) { S->ispr &= ~(1 << p); break; }
    }
}

/* ------------------------------------------------------------------ timers */
static void timer_init(hwtimer_t *t, const char *name, int cc0, int cc1, int ov)
{
    memset(t, 0, sizeof *t); t->name = name; t->vec_cc0 = cc0; t->vec_cc1 = cc1; t->vec_ov = ov; t->next_match = ~0ULL;
}
static uint64_t timer_period(hwtimer_t *t)
{
    return ((uint64_t)t->ccr[0] + 1) << (t->ctl0 & 7);
}
static void timer_write(hwtimer_t *t, int off, uint32_t v, int size)
{
    switch (off) {
    case 0: {
        int was = t->ctl0 & 0x80; t->ctl0 = v;
        if ((v & 0x80) && !was) { t->start_cycle = NOW; t->next_match = NOW + timer_period(t); LOG("[%llu] %s start mode=%d cks=%d ccr0=%u ccr1=%u period=%llu cyc\n", (unsigned long long)NOW, t->name, t->ctl1 & 7, v & 7, t->ccr[0], t->ccr[1], (unsigned long long)timer_period(t)); }
        if (!(v & 0x80)) t->next_match = ~0ULL;
        break; }
    case 1: t->ctl1 = v; break;
    case 2: t->ioc0 = v; break;
    case 3: t->ioc1 = v; break;
    case 4: t->ioc2 = v; break;
    case 5: t->opt0 = v; break;
    case 6: case 8: case 0xA: case 0xC: {
        int k = (off - 6) / 2; if (size == 1) { t->ccr[k] = (t->ccr[k] & 0xFF00) | (v & 0xFF); } else t->ccr[k] = v & 0xFFFF; break; }
    case 7: case 9: case 0xB: case 0xD: { int k = (off - 7) / 2; t->ccr[k] = (t->ccr[k] & 0x00FF) | ((v & 0xFF) << 8); break; }
    default: break;
    }
}
static uint32_t timer_read(hwtimer_t *t, int off, int size)
{
    switch (off) {
    case 0: return t->ctl0; case 1: return t->ctl1; case 2: return t->ioc0; case 3: return t->ioc1; case 4: return t->ioc2; case 5: return t->opt0;
    case 6: case 8: case 0xC: return t->ccr[(off - 6) / 2];
    case 0xA: {  /* TPnCNT (for TMP) / CCR2 (for TMQ) : return counter for TMP */
        if (t->vec_cc1 >= 0 && t->ccr[2] == 0) {
            if (!(t->ctl0 & 0x80)) return 0;
            uint64_t ticks = (NOW - t->start_cycle) >> (t->ctl0 & 7);
            return (uint32_t)(ticks % ((uint64_t)t->ccr[0] + 1));
        }
        return t->ccr[2];
    }
    case 0xE: { if (!(t->ctl0 & 0x80)) return 0; uint64_t ticks = (NOW - t->start_cycle) >> (t->ctl0 & 7); return (uint32_t)(ticks % ((uint64_t)t->ccr[0] + 1)); }
    default: (void)size; return 0;
    }
}
static void timer_tick(hwtimer_t *t)
{
    if (NOW >= t->next_match) {
        int mode = t->ctl1 & 7;
        if (mode == 0 && t->vec_cc0 >= 0) sys_raise(S, t->vec_cc0);   /* interval mode: CC0 match */
        t->next_match += timer_period(t);
        if (t->next_match <= NOW) t->next_match = NOW + timer_period(t);
    }
}

/* ------------------------------------------------------------------ CAN1 */
static void can_push_rhl(sys_t *s, int mb)
{
    if (s->rhl_count >= 32) { s->rovf = 1; s->rhl[(s->rhl_head + 31) % 32] = mb; return; }
    s->rhl[(s->rhl_head + s->rhl_count) % 32] = mb; s->rhl_count++;
}
static void can_push_thl(sys_t *s, int mb)
{
    if (s->thl_count >= 32) { s->tovf = 1; return; }
    s->thl[(s->thl_head + s->thl_count) % 32] = mb; s->thl_count++;
}
static uint16_t mb16(sys_t *s, int m, int off) { return s->c_mb[m][off] | (s->c_mb[m][off + 1] << 8); }

static void can_transmit(sys_t *s, int m)
{
    uint8_t *b = s->c_mb[m];
    can_frame_t f; memset(&f, 0, sizeof f);
    uint16_t idh = mb16(s, m, 0xC), idl = mb16(s, m, 0xA);
    f.ext = !!(idh & 0x8000);
    f.id = f.ext ? (((idh & 0x1FFF) << 16) | idl) : ((idh >> 2) & 0x7FF);
    f.dlc = b[8] & 0xF; if (f.dlc > 8) f.dlc = 8;
    f.rtr = !!(b[9] & 0x40);
    memcpy(f.data, b, 8);
    if (s->log_can) {
        LOG("[%llu] CAN TX mb%d id=%03X dlc=%d data=", (unsigned long long)NOW, m, f.id, f.dlc);
        for (int i = 0; i < f.dlc; i++) LOG("%02X ", f.data[i]);
        LOG("\n");
    }
    if (s->tx_count < 64) s->tx_queue[s->tx_count++] = f;
    if (s->tx_hook) s->tx_hook(s, &f);
    /* completion */
    b[0xE] &= ~0x02;              /* clear TRQ */
    can_push_thl(s, m);
    s->c_ints |= 0x01;
    if (s->c_ie & 0x01) sys_raise(s, VEC_CAN_TRX);
}

void sys_can_inject(sys_t *s, const can_frame_t *f)
{
    int hit = 0;
    if (s->c_ctrl & 0x18) {   /* PSMODE != 0 (sleep/stop): bus activity wakes the module and raises INTCnWUP */
        if (s->log_can) LOG("[%llu] CAN RX id=%03X while in power-save mode %d: wake-up\n", (unsigned long long)NOW, f->id, (s->c_ctrl >> 3) & 3);
        s->c_ctrl &= ~0x18u;
        s->c_ints |= 0x20;
        if (s->c_ie & 0x20) sys_raise(s, VEC_CAN_WUP);
        s->wakeup_pending = 1;
    }
    for (int m = 0; m < 32; m++) {
        uint8_t *b = s->c_mb[m];
        uint8_t conf = b[9]; uint16_t ctrl = mb16(s, m, 0xE);
        if (!(conf & 1)) continue;                    /* MA0 */
        int mt = (conf >> 3) & 7;
        if (mt == 0) continue;                        /* transmit buffer */
        if (!(ctrl & 1)) continue;                    /* RDY */
        uint16_t idh = mb16(s, m, 0xC), idl = mb16(s, m, 0xA);
        int ext = !!(idh & 0x8000);
        if (ext != f->ext) continue;
        uint32_t mbid = ext ? (((idh & 0x1FFF) << 16) | idl) : ((idh >> 2) & 0x7FF);
        uint32_t dontcare = 0;
        if (mt >= 2 && mt <= 5) {   /* aFCAN: CMID bit 1 = masked (not compared) */
            int k = mt - 2; uint16_t ml = s->c_mask[k * 2], mh = s->c_mask[k * 2 + 1];
            uint32_t mval = ((mh & 0x1FFF) << 16) | ml;
            dontcare = ext ? mval : ((mval >> 18) & 0x7FF);
        }
        if (((mbid ^ f->id) & ~dontcare) != 0) continue;
        if ((ctrl & 0x04) && !(conf & 0x80)) { /* DN set and OWS=0 -> discard */ continue; }
        if (ctrl & 0x04) b[0xE] |= 0x10;      /* MOW */
        memcpy(b, f->data, 8);
        b[8] = f->dlc;
        if (mt >= 2) {                        /* mask mailbox: the hardware stores the received identifier */
            uint16_t nidh = ext ? (0x8000 | ((f->id >> 16) & 0x1FFF)) : ((f->id & 0x7FF) << 2), nidl = ext ? (f->id & 0xFFFF) : 0;
            b[0xC] = nidh & 0xFF; b[0xD] = nidh >> 8; b[0xA] = nidl & 0xFF; b[0xB] = nidl >> 8;
        }
        b[0xE] |= 0x04;                       /* DN */
        s->c_ctrl |= 0x20;                    /* VALID */
        can_push_rhl(s, m);
        hit = 1;
        if (s->log_can) LOG("[%llu] CAN RX id=%03X -> mb%d (ie=%d)\n", (unsigned long long)NOW, f->id, m, !!(ctrl & 8));
        if (ctrl & 0x08) { s->c_ints |= 0x02; if (s->c_ie & 0x02) sys_raise(s, VEC_CAN_REC); }
        break;
    }
    if (!hit && s->log_can) LOG("[%llu] CAN RX id=%03X : no matching buffer\n", (unsigned long long)NOW, f->id);
}

static uint32_t can_read(sys_t *s, uint32_t off, int size)
{
    if (off >= 0x100 && off < 0x500) {
        int m = (off - 0x100) / 0x20, o = (off - 0x100) % 0x20;
        if (size == 1) return s->c_mb[m][o];
        if (size == 2) return mb16(s, m, o);
        return mb16(s, m, o) | (mb16(s, m, o + 2) << 16);
    }
    switch (off) {
    case 0x00: return s->c_gmctrl | 0x100;   /* MBON */
    case 0x02: return s->c_gmcs;
    case 0x40: case 0x42: case 0x44: case 0x46: case 0x48: case 0x4A: case 0x4C: case 0x4E: return s->c_mask[(off - 0x40) / 2];
    case 0x50: return s->c_ctrl;
    case 0x52: return s->c_lec;
    case 0x53: return s->c_info;
    case 0x54: return s->c_erc;
    case 0x56: return s->c_ie;
    case 0x58: return s->c_ints;
    case 0x5A: return s->c_brp;
    case 0x5C: return s->c_btr;
    case 0x5E: return 0;
    case 0x60: {  /* RGPT: read pops */
        if (s->rhl_count == 0) return 0x02 | (s->rovf ? 1 : 0);
        int mb = s->rhl[s->rhl_head]; s->rhl_head = (s->rhl_head + 1) % 32; s->rhl_count--;
        return (mb << 8) | (s->rovf ? 1 : 0);
    }
    case 0x62: return 0;
    case 0x64: {  /* TGPT */
        if (s->thl_count == 0) return 0x02 | (s->tovf ? 1 : 0);
        int mb = s->thl[s->thl_head]; s->thl_head = (s->thl_head + 1) % 32; s->thl_count--;
        return (mb << 8) | (s->tovf ? 1 : 0);
    }
    case 0x66: return s->c_ts;
    }
    return 0;
}
static void can_write(sys_t *s, uint32_t off, uint32_t v, int size)
{
    if (off >= 0x100 && off < 0x500) {
        int m = (off - 0x100) / 0x20, o = (off - 0x100) % 0x20;
        if (o == 0xE) {   /* MCTRL set/clear */
            uint16_t cur = mb16(s, m, 0xE);
            uint16_t set = (v >> 8) & 0x1F, clr = v & 0x1F;
            cur = (cur & ~clr) | (set & 0x0B);   /* DN(bit2) and MOW cannot be set by software */
            s->c_mb[m][0xE] = cur & 0xFF; s->c_mb[m][0xF] = 0;
            if ((set & 0x02) && (cur & 0x01)) can_transmit(s, m);
            return;
        }
        if (size == 1) s->c_mb[m][o] = v;
        else if (size == 2) { s->c_mb[m][o] = v; s->c_mb[m][o + 1] = v >> 8; }
        else { for (int i = 0; i < 4; i++) s->c_mb[m][o + i] = v >> (8 * i); }
        return;
    }
    switch (off) {
    case 0x00: s->c_gmctrl = (s->c_gmctrl & ~(v & 0xFF)) | ((v >> 8) & 0x03); break;
    case 0x02: s->c_gmcs = v; break;
    case 0x40: case 0x42: case 0x44: case 0x46: case 0x48: case 0x4A: case 0x4C: case 0x4E: s->c_mask[(off - 0x40) / 2] = v; break;
    case 0x50: {
        uint16_t old = s->c_ctrl;
        s->c_ctrl = (s->c_ctrl & ~(v & 0xFF)) | ((v >> 8) & 0xFF);
        s->c_ctrl &= ~0x20u; s->c_ctrl |= old & 0x20;   /* VALID is read-only here */
        if (s->log_can && ((old ^ s->c_ctrl) & 0x1F)) LOG("[%llu] CAN C1CTRL opmode=%d psmode=%d\n", (unsigned long long)NOW, s->c_ctrl & 7, (s->c_ctrl >> 3) & 3);
        break; }
    case 0x52: s->c_lec = v; break;
    case 0x54: s->c_erc = 0; break;
    case 0x56: s->c_ie = (s->c_ie & ~(v & 0xFF)) | ((v >> 8) & 0x3F); break;
    case 0x58: s->c_ints &= ~(v & 0x3F); break;
    case 0x5A: s->c_brp = v; break;
    case 0x5C: s->c_btr = v; break;
    case 0x60: if (v & 1) s->rovf = 0; break;
    case 0x64: if (v & 1) s->tovf = 0; break;
    default: break;
    }
}

/* ------------------------------------------------------------------ ports & GDC bus */
static uint8_t port_read(uint8_t *latch, uint8_t *pm, uint8_t *in) { return (*latch & ~*pm) | (*in & *pm); }

static void gdc_bus_update(sys_t *s, uint8_t old_pcs, uint8_t new_pcs)
{
    if (!s->gdc) return;
    int cs = !(new_pcs & 1);
    uint8_t addr = s->portc[0x0E] & 7;
    /* /WR falling edge (bit2) */
    if (cs && (old_pcs & 4) && !(new_pcs & 4)) {
        uint8_t data = s->port[0x18];
        gdc_bus_write(s->gdc, addr, data);
    }
    /* /RD falling edge (bit3) : latch data for the read that follows */
    if (cs && (old_pcs & 8) && !(new_pcs & 8)) {
        s->pin_in[0x18] = gdc_bus_read(s->gdc, addr);
    }
}

/* S1D15721 on port DL: PDLH bit0 = /RD, bit1 = /WR, bit2 = A0, bit3 = /CS, bit4 = /RES */
static void lcd_bus_update(sys_t *s, uint8_t old, uint8_t new_)
{
    if (!s->lcd) return;
    if ((old ^ new_) & 0x10) lcd_reset(s->lcd, !(new_ & 0x10));
    int cs = !(new_ & 8), a0 = !!(new_ & 4);
    if (cs && !(old & 2) && (new_ & 2)) lcd_write(s->lcd, a0, s->portc[0x04]);          /* /WR rising edge latches data */
    if (cs && (old & 1) && !(new_ & 1)) s->pinc_in[0x04] = lcd_read(s->lcd, a0);        /* /RD falling edge: drive bus */
}

/* SPI EEPROM (25xx-style, 256 bytes, 8-byte pages) on the mono unit: P4 bit2 = SCK, bit1 = MOSI, bit0 = MISO, P9H bit5 = /CS.
 * Output bits change on the falling SCK edge (D7 of a byte appears on the falling edge that follows the last input bit). */
#define ee (s->ee)
#define g_eeprom (s->eeprom)
#define g_eeprom_dirty (s->eeprom_dirty)
#define g_eeprom_verbose (s->eeprom_verbose)
static void ee_present(sys_t *s, int bit) { s->pin_in[0x08] = (s->pin_in[0x08] & ~1) | (bit & 1); }
static void ee_next_out(sys_t *s)
{
    /* called on each falling edge once a byte has been loaded (outidx >= -1) */
    if (ee.outidx < -1) return;
    ee.outidx++;
    if (ee.outidx == 8) {
        if (ee.cmd == 3) { ee.addr = (ee.addr + 1) & 511; ee.shout = g_eeprom[ee.addr]; }
        else if (ee.cmd == 5) ee.shout = ee.wel ? 0x02 : 0x00;
        else ee.shout = 0xFF;
        ee.outidx = 0;
    }
    ee_present(s, (ee.shout >> (7 - ee.outidx)) & 1);
}
static void eeprom_update(sys_t *s, uint8_t p4, uint8_t csport)
{
    int cs = !(csport & s->ee_cs_bit), sck = !!(p4 & 4), mosi = !!(p4 & 2);
    if (!cs) { if (ee.cs && ee.cmd == 2 && ee.phase == 2) ee.wel = 0;      /* WRITE completes when /CS rises */
               ee.cs = 0; ee.bitcnt = 0; ee.phase = 0; ee.shin = 0; ee.outidx = -2; ee.sck = sck; ee_present(s, 1); return; }
    if (!ee.cs) { ee.cs = 1; ee.bitcnt = 0; ee.phase = 0; ee.shin = 0; ee.outidx = -2; }
    if (!ee.sck && sck) {                     /* rising edge: sample MOSI */
        ee.shin = (ee.shin << 1) | mosi; ee.bitcnt++;
        if (ee.bitcnt == 8) {
            ee.bitcnt = 0;
            switch (ee.phase) {
            case 0:   /* instruction */
                ee.cmd = ee.shin & 7;
                if (g_eeprom_verbose) LOG("[%llu] EEPROM cmd %02x\n", (unsigned long long)NOW, ee.shin);
                if (ee.cmd == 6) ee.wel = 1;
                else if (ee.cmd == 4) ee.wel = 0;
                else if (ee.cmd == 5) { ee.shout = ee.wel ? 0x02 : 0x00; ee.outidx = -1; ee.phase = 3; }
                else if (ee.cmd == 3 || ee.cmd == 2) { ee.phase = 1; ee.addr = (ee.shin & 8) ? 256 : 0; }
                else if (ee.cmd == 1) ee.phase = 4;
                break;
            case 1:   /* address byte */
                ee.addr |= ee.shin; ee.phase = 2;
                if (g_eeprom_verbose) LOG("[%llu] EEPROM %s addr %03x\n", (unsigned long long)NOW, ee.cmd == 3 ? "READ" : "WRITE", ee.addr);
                if (ee.cmd == 3) { ee.shout = g_eeprom[ee.addr & 511]; ee.outidx = -1; }
                break;
            case 2:   /* data */
                if (ee.cmd == 2) {
                    if (ee.wel) { if (g_eeprom[ee.addr & 511] != ee.shin) g_eeprom_dirty = 1; g_eeprom[ee.addr & 511] = ee.shin; }
                    if (g_eeprom_verbose || s->log_gdc) LOG("[%llu] EEPROM write [%03x] = %02x%s\n", (unsigned long long)NOW, ee.addr & 511, ee.shin, ee.wel ? "" : " (ignored: WEL clear)");
                    ee.addr = (ee.addr & ~7) | ((ee.addr + 1) & 7);
                }
                break;
            case 4: ee.status = ee.shin; break;
            }
            ee.shin = 0;
        }
    } else if (ee.sck && !sck) {              /* falling edge: next output bit */
        ee_next_out(s);
    }
    ee.sck = sck;
}

/* 3-wire device on P9 (bit2 = CE, bit3 = CLK, bit4 = DIO) */
static struct { int bitcnt; uint8_t shift; int phase; uint8_t cmd; uint8_t regs[16]; uint8_t outbits; int outcnt; } p9dev;
static void p9_update(sys_t *s, uint8_t old, uint8_t new_)
{
    int ce = new_ & 4;
    if (!ce) { p9dev.bitcnt = 0; p9dev.phase = 0; return; }
    if (!(old & 8) && (new_ & 8)) {   /* clock rising edge */
        if (p9dev.phase == 2) {   /* device drives data out, LSB first */
            s->pin_in[0x12] = (s->pin_in[0x12] & ~0x10) | (((p9dev.outbits >> p9dev.outcnt) & 1) << 4);
            p9dev.outcnt++;
            return;
        }
        int bit = !!(new_ & 0x10);
        p9dev.shift |= bit << p9dev.bitcnt; p9dev.bitcnt++;
        if (p9dev.bitcnt == 8) {
            if (p9dev.phase == 0) {
                p9dev.cmd = p9dev.shift; p9dev.shift = 0; p9dev.bitcnt = 0;
                if ((p9dev.cmd & 0xF) == 0xC) { p9dev.phase = 2; p9dev.outbits = p9dev.regs[p9dev.cmd >> 4]; p9dev.outcnt = 0;
                    LOG("[%llu] P9DEV read reg %d -> %02x\n", (unsigned long long)NOW, p9dev.cmd >> 4, p9dev.outbits);
                    s->pin_in[0x12] = (s->pin_in[0x12] & ~0x10) | ((p9dev.outbits & 1) << 4); p9dev.outcnt = 1; }
                else p9dev.phase = 1;
            } else if (p9dev.phase == 1) {
                p9dev.regs[p9dev.cmd >> 4] = p9dev.shift;
                LOG("[%llu] P9DEV write reg %d = %02x (cmd %02x)\n", (unsigned long long)NOW, p9dev.cmd >> 4, p9dev.shift, p9dev.cmd);
                p9dev.shift = 0; p9dev.bitcnt = 0; p9dev.phase = 0;
            }
        }
    }
}

/* ------------------------------------------------------------------ SFR access */
static const char *sfr_name(uint32_t off);

static uint32_t sfr_read(sys_t *s, uint32_t off, int size)
{
    /* interrupt controller */
    if (off >= 0x110 && off < 0x110 + (NVEC / 0x10 - 8) * 2) {
        int i = (off - 0x110) / 2;
        if (off & 1) return 0;
        return size == 2 ? (s->ic[i] | (s->ic[i + 1] << 8)) : s->ic[i];
    }
    if (off >= 0x100 && off < 0x10C) {   /* IMRn: mask bits */
        int base = (off - 0x100) / 2 * 16; uint32_t v = 0;
        for (int k = 0; k < 16; k++) { int i = base + k; if (i < NVEC / 0x10 - 8 && (s->ic[i] & 0x40)) v |= 1 << k; }
        return size == 1 && (off & 1) ? (v >> 8) & 0xFF : v;
    }
    if (off == 0x1FA) return s->ispr;
    if (off >= 0x400 && off < 0x440) {
        int p = off - 0x400;
        if (p < 0x20) {   /* Pn */
            uint8_t pm = s->port[0x20 + p];
            if (size == 2 && (p & 1) == 0) return port_read(&s->port[p], &s->port[0x20 + p], &s->pin_in[p]) | (port_read(&s->port[p + 1], &s->port[0x21 + p], &s->pin_in[p + 1]) << 8);
            return port_read(&s->port[p], &pm, &s->pin_in[p]);
        }
        return s->port[p];
    }
    if (off < 0x40) {
        int p = off;
        if (p < 0x20) {
            if (p == 0x0A) {   /* PCT: bit2 = GDC READY input */
                uint8_t in = s->pinc_in[0x0A];
                if (s->gdc) in = (in & ~4) | (gdc_ready(s->gdc) ? 4 : 0);
                s->pinc_in[0x0A] = in;
            }
            uint8_t pm = s->portc[0x20 + p];
            return port_read(&s->portc[p], &pm, &s->pinc_in[p]);
        }
        return s->portc[p];
    }
    if (off >= 0x540 && off < 0x550) return timer_read(&s->tmq[0], off - 0x540, size);
    if (off >= 0x610 && off < 0x620) return timer_read(&s->tmq[1], off - 0x610, size);
    if (off >= 0x620 && off < 0x630) return timer_read(&s->tmq[2], off - 0x620, size);
    if (off >= 0x590 && off < 0x5D0) return timer_read(&s->tmp[(off - 0x590) / 0x10], (off - 0x590) % 0x10, size);
    switch (off) {
    case 0x680: return s->wtm;
    case 0x824: return 0;              /* LOCKR: 0 = PLL locked */
    case 0x828: return (s->pcc & ~0x10) | ((s->pcc & 0x08) ? 0x10 : 0);   /* CLS (status, bit 4) follows CK3 (bit 3): 1 = running on the sub-clock */
    case 0x82C: return s->pllctl;
    case 0x1FE: return s->psc;
    case 0x8B0: return s->prsm0;
    case 0x8B1: return s->prscm0;
    case 0x888: return 0x01;           /* RESF: power-on reset */
    case 0xD00: return s->cb0ctl0;
    case 0xD01: return s->cb0ctl1;
    case 0xD02: return s->cb0ctl2;
    case 0xD03: return s->cb0str | ((s->cb0_done > NOW) ? 0x80 : 0);
    case 0xD04: return s->cb0rx;
    case 0xD06: return s->cb0tx;
    case 0xD10: return s->cb1ctl0;
    case 0xD11: return s->cb1ctl1;
    case 0xD12: return s->cb1ctl2;
    case 0xD13: return s->cb1str | ((s->cb1_done > NOW) ? 0x80 : 0);
    case 0xD14: return s->cb1rx;
    case 0xD16: return s->cb1tx;
    case 0xA10: return s->ua1ctl0;
    case 0xA11: return s->ua1ctl1;
    case 0xA12: return s->ua1ctl2;
    case 0xA13: return s->ua1opt0;
    case 0xA14: return s->ua1str | ((s->ua1_tx_done > NOW) ? 0x80 : 0);
    case 0xA16: return s->ua1rx;
    case 0xA17: return s->ua1tx;
    case 0x200: return s->ada0m0 & ~(s->adc_busy ? 0 : 0);
    case 0x201: return s->ada0m1;
    case 0x202: return s->ada0s;
    case 0x203: return s->ada0m2;
    }
    if (off >= 0x210 && off < 0x230) {   /* ADA0CRn 16-bit left aligned, or ADA0CRnH byte */
        int ch = (off - 0x210) / 2;
        uint16_t v = (s->adc_val[ch] & 0x3FF) << 6;
        if (size == 1) return (off & 1) ? (v >> 8) : (v & 0xFF);
        return v;
    }
    if (s->log_unknown) LOG("[%llu] SFR read  %08x (%s) size %d pc=%06x\n", (unsigned long long)NOW, 0xFFFFF000 + off, sfr_name(off), size, s->cpu.last_pc);
    return 0;
}

static void sfr_write(sys_t *s, uint32_t off, uint32_t v, int size)
{
    if (off >= 0x110 && off < 0x110 + (NVEC / 0x10 - 8) * 2) {
        int i = (off - 0x110) / 2;
        if (off & 1) return;
        s->ic[i] = (s->ic[i] & 0x80 & ~(v & 0x80 ? 0 : 0x80)) | (v & 0x7F) | ((v & 0x80) ? 0x80 : 0);
        /* writing 0 to IF clears the request; writing 1 sets it */
        s->ic[i] = (v & 0xC7) | ((v & 0x80) ? 0x80 : 0);
        if (size == 2 && i + 1 < NVEC / 0x10 - 8) s->ic[i + 1] = (v >> 8) & 0xC7;
        return;
    }
    if (off >= 0x100 && off < 0x10C) {
        int base = (off - 0x100) / 2 * 16;
        for (int k = 0; k < 16; k++) { int i = base + k; if (i >= NVEC / 0x10 - 8) break;
            int bit = (size == 1 && (off & 1)) ? ((v >> (k - 8)) & 1) : ((v >> k) & 1);
            if (size == 1 && (off & 1) && k < 8) continue;
            if (size == 1 && !(off & 1) && k >= 8) continue;
            if (bit) s->ic[i] |= 0x40; else s->ic[i] &= ~0x40; }
        return;
    }
    if (off >= 0x400 && off < 0x440) {
        int p = off - 0x400;
        uint8_t old = s->port[p];
        if (size == 2) { s->port[p] = v; s->port[p + 1] = v >> 8; } else s->port[p] = v;
        if (p == 0x12 && s->board == 0) p9_update(s, old, s->port[0x12]);
        if (s->board == 1 && (p == 0x08 || p == s->ee_cs_port)) eeprom_update(s, s->port[0x08], s->port[s->ee_cs_port]);
        if (p == 0x0D && s->log_gdc && old != s->port[p]) LOG("[%llu] P6H = %02x (display power bits 6/7 = %d/%d)\n", (unsigned long long)NOW, s->port[p], !!(s->port[p] & 0x40), !!(s->port[p] & 0x80));
        if (s->board == 1 && s->log_gdc && old != s->port[p] && (p == 0x13 || p == 0x12 || p == 0x0E || p == 0x08 || p == 0x00)) LOG("[%llu] P%s = %02x\n", (unsigned long long)NOW, p == 0x13 ? "9H" : p == 0x12 ? "9L" : p == 0x0E ? "7" : p == 0x08 ? "4" : "0", s->port[p]);
        return;
    }
    if (off < 0x40) {
        int p = off; uint8_t old = s->portc[p];
        if (size == 2 && p + 1 < 0x40) { s->portc[p] = v; s->portc[p + 1] = v >> 8; } else s->portc[p] = v;
        if (p == 0x08) gdc_bus_update(s, old, v);
        if (p == 0x05 && s->lcd) lcd_bus_update(s, old, s->portc[0x05]);
        if (p == 0x04 && size == 2 && s->lcd) lcd_bus_update(s, s->portc[0x05], s->portc[0x05]);
        if (p == 0x0C && s->log_gdc && old != s->portc[p]) LOG("[%llu] PCM = %02x\n", (unsigned long long)NOW, s->portc[p]);
        if (p == 0x0A && s->gdc && ((old ^ v) & 0x20)) { if (!(v & 0x20)) { LOG("[%llu] GDC reset asserted\n", (unsigned long long)NOW); gdc_reset(s->gdc); } else LOG("[%llu] GDC reset released\n", (unsigned long long)NOW); }
        return;
    }
    if (off >= 0x540 && off < 0x550) { timer_write(&s->tmq[0], off - 0x540, v, size); return; }
    if (off >= 0x610 && off < 0x620) { timer_write(&s->tmq[1], off - 0x610, v, size); return; }
    if (off >= 0x620 && off < 0x630) { timer_write(&s->tmq[2], off - 0x620, v, size); return; }
    if (off >= 0x590 && off < 0x5D0) { timer_write(&s->tmp[(off - 0x590) / 0x10], (off - 0x590) % 0x10, v, size); return; }
    switch (off) {
    case 0x680: { uint8_t old = s->wtm; s->wtm = v;
        if (v & 2) { if (!(old & 2)) { s->wt_next = NOW + ((1ULL << (4 + ((v >> 4) & 7))) * CPU_HZ / 32768); LOG("[%llu] WT interval enabled, period %llu cyc\n", (unsigned long long)NOW, (unsigned long long)((1ULL << (4 + ((v >> 4) & 7))) * CPU_HZ / 32768)); } }
        else s->wt_next = ~0ULL;
        if (v & 1) { static const int sh[4] = {14, 13, 5, 4}; s->wtov_period = (1ULL << sh[(v >> 2) & 3]) * CPU_HZ / 32768; if (!(old & 1)) { s->wtov_next = NOW + s->wtov_period; LOG("[%llu] WT overflow enabled, period %llu cyc\n", (unsigned long long)NOW, (unsigned long long)s->wtov_period); } }
        else s->wtov_next = ~0ULL;
        return; }
    case 0x828: if (s->log_sfr || ((s->pcc ^ v) & 0x58)) LOG("[%llu] PCC = %02x (%s)\n", (unsigned long long)NOW, v, (v & 0x08) ? ((v & 0x40) ? "sub-clock, main osc stopped" : "sub-clock") : "main clock"); s->pcc = v; return;
    case 0x82C: s->pllctl = v; return;
    case 0x1FC: return; /* PRCMD */
    case 0x1FE: s->psc = v; return;
    case 0x8B0: s->prsm0 = v; return;
    case 0x8B1: s->prscm0 = v; return;
    case 0xD00: s->cb0ctl0 = v; return;
    case 0xD01: s->cb0ctl1 = v; return;
    case 0xD02: s->cb0ctl2 = v; return;
    case 0xD03: s->cb0str = v & 0x7F; return;
    case 0xD10: s->cb1ctl0 = v; return;
    case 0xD11: s->cb1ctl1 = v; return;
    case 0xD12: s->cb1ctl2 = v; return;
    case 0xD13: s->cb1str = v & 0x7F; return;
    case 0xD14: return;
    case 0xD16: { int cl = s->cb1ctl2 & 0xF; int bits = cl == 0 ? 8 : cl == 8 ? 16 : cl;
        s->cb1tx = v; s->cb1_done = NOW + 40 * bits; s->cb1_count++;
        if (s->log_gdc) LOG("[%llu] CSIB1 TX %0*x (%d bits, P9H=%02x P4=%02x PCM=%02x)\n", (unsigned long long)NOW, bits > 8 ? 4 : 2, v & (bits > 8 ? 0xFFFF : 0xFF), bits, s->port[0x13], s->port[0x08], s->portc[0x0C]);
        return; }
    case 0xA10: { uint8_t old = s->ua1ctl0; s->ua1ctl0 = v; if ((old ^ v) & 0xE0) LOG("[%llu] UARTA1 CTL0=%02x (PWR=%d TXE=%d RXE=%d) CTL1=%02x CTL2=%02x -> %llu baud\n", (unsigned long long)NOW, v, !!(v & 0x80), !!(v & 0x40), !!(v & 0x20), s->ua1ctl1, s->ua1ctl2, (unsigned long long)(s->ua1ctl2 ? (CPU_HZ >> (s->ua1ctl1 & 7)) / (2 * s->ua1ctl2) : 0)); 
        s->ua1_byte_cycles = s->ua1ctl2 ? (uint64_t)(2 * s->ua1ctl2) * 10 * (1ULL << (s->ua1ctl1 & 7)) : 2000; return; }
    case 0xA11: s->ua1ctl1 = v; return;
    case 0xA12: s->ua1ctl2 = v; return;
    case 0xA13: s->ua1opt0 = v; return;
    case 0xA14: s->ua1str &= ~(v & 0x07); return;
    case 0xA17: { s->ua1tx = v; s->ua1_tx_done = NOW + (s->ua1_byte_cycles ? s->ua1_byte_cycles : 2000); s->ua1_tx_count++;
        if (s->uart_tx_hook) s->uart_tx_hook(s, v);
        /* collect bytes into lines for the log (flush on a 20 ms gap or 64 bytes) */
        if (s->ua1_txline_len && (NOW - s->ua1_txline_t0 > CPU_HZ / 50 || s->ua1_txline_len >= 64)) { LOG("[%llu] UARTA1 TX:", (unsigned long long)s->ua1_txline_t0); for (int k = 0; k < s->ua1_txline_len; k++) LOG(" %02x", s->ua1_txline[k]); LOG("\n"); s->ua1_txline_len = 0; }
        if (!s->ua1_txline_len) s->ua1_txline_t0 = NOW;
        s->ua1_txline[s->ua1_txline_len++] = v;
        return; }
    case 0xD06: s->cb0tx = v; s->cb0_done = NOW + 400;   /* 16 bits @ ~1 MHz */
        if (s->log_sfr) LOG("[%llu] CSIB0 TX %04x (reg %02x <- %02x) P4=%02x\n", (unsigned long long)NOW, v & 0xFFFF, v & 0xFF, (v >> 8) & 0xFF, s->port[0x08]);
        return;
    case 0x200: {
        uint8_t old = s->ada0m0; s->ada0m0 = v;
        if ((v & 0x80) && !(old & 0x80)) { s->adc_busy = 1; s->adc_done = NOW + 400; }
        return; }
    case 0x201: s->ada0m1 = v; return;
    case 0x202: s->ada0s = v; return;
    case 0x203: s->ada0m2 = v; return;
    case 0x204: case 0x205: return;
    }
    /* write-only / don't-care registers we accept silently */
    switch (off) {
    case 0x0E0: case 0x0E2: case 0x0E4: case 0x0E6: case 0x9FC: case 0x06E: case 0x6C0: case 0x6C1: case 0x300: case 0x891: case 0x890: case 0x318:
    case 0x064: case 0x066: case 0x484: case 0x488: case 0x48A: case 0x892: case 0x308: case 0x30A: case 0x690: case 0x694: case 0x80C: case 0x6D0:
    case 0x802: case 0x810: case 0x812: case 0x814: case 0x816: case 0x82F: case 0x870: case 0x888: case 0x8B2:
        return;
    }
    if (off >= 0xA00 && off < 0xA40) return;   /* UARTA0,2,3 */
    if (off >= 0xB00 && off < 0xB80) return;   /* PFCE etc. */
    if (off >= 0xC00 && off < 0xC40) return;   /* INTFn/INTRn */
    if (off >= 0xD20 && off < 0xD30) return;   /* CSIB2 */
    if (off >= 0x5D0 && off < 0x610) return;   /* TMP4.. */
    if (s->log_unknown) LOG("[%llu] SFR write %08x (%s) = %x size %d pc=%06x\n", (unsigned long long)NOW, 0xFFFFF000 + off, sfr_name(off), v, size, s->cpu.last_pc);
}

/* ------------------------------------------------------------------ bus */
static inline uint32_t rd(const uint8_t *p, int size)
{
    if (size == 1) return p[0];
    if (size == 2) return p[0] | (p[1] << 8);
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void wr(uint8_t *p, uint32_t v, int size)
{
    p[0] = v; if (size >= 2) p[1] = v >> 8; if (size == 4) { p[2] = v >> 16; p[3] = v >> 24; }
}

uint32_t bus_fetch16(cpu_t *c, uint32_t a)
{
    (void)c; a &= 0x03FFFFFF;
    if (a + 1 < FLASH_SIZE) return S->flash[a] | (S->flash[a + 1] << 8);
    if (a >= RAM_BASE && a + 1 < RAM_BASE + RAM_SIZE) return S->ram[a - RAM_BASE] | (S->ram[a - RAM_BASE + 1] << 8);
    return 0xFFFF;
}

uint32_t g_flash_hist[FLASH_SIZE / 64]; int g_flash_hist_on; uint32_t g_rwatch;
uint32_t bus_read(cpu_t *c, uint32_t addr, int size)
{
    (void)c; uint32_t a = addr & 0x03FFFFFF;
    if (g_flash_hist_on && a < FLASH_SIZE) g_flash_hist[a / 64]++;
    if (g_rwatch && a >= g_rwatch && a < g_rwatch + 256) LOG("[%llu] RWATCH read %08x size %d pc=%06x r6=%08x r7=%08x r8=%08x r9=%08x r10=%08x r11=%08x r12=%08x r13=%08x lp=%06x\n", (unsigned long long)NOW, addr, size, S->cpu.pc, S->cpu.r[6], S->cpu.r[7], S->cpu.r[8], S->cpu.r[9], S->cpu.r[10], S->cpu.r[11], S->cpu.r[12], S->cpu.r[13], S->cpu.r[31]);
    if (a >= 0x03FFF000) return sfr_read(S, a - 0x03FFF000, size);
    if (a >= RAM_BASE && a < RAM_BASE + RAM_SIZE) return rd(&S->ram[a - RAM_BASE], size);
    if (a < FLASH_SIZE) return rd(&S->flash[a], size);
    if (a >= S->can_base && a < S->can_base + 0x600) return can_read(S, a - S->can_base, size);
    if (a >= 0x03FEC000 && a < 0x03FECC00) return 0;      /* the other CAN controller (unused) */
    if (S->log_unknown) LOG("[%llu] unmapped read %08x size %d pc=%06x\n", (unsigned long long)NOW, addr, size, S->cpu.last_pc);
    return 0;
}

void bus_write(cpu_t *c, uint32_t addr, uint32_t val, int size)
{
    (void)c; uint32_t a = addr & 0x03FFFFFF;
    if (a >= 0x03FFF000) { sfr_write(S, a - 0x03FFF000, val, size); return; }
    if (a >= RAM_BASE && a < RAM_BASE + RAM_SIZE) { wr(&S->ram[a - RAM_BASE], val, size); return; }
    if (a >= S->can_base && a < S->can_base + 0x600) { can_write(S, a - S->can_base, val, size); return; }
    if (a >= 0x03FEC000 && a < 0x03FECC00) return;
    if (a < FLASH_SIZE) { if (S->log_unknown) LOG("[%llu] flash write %08x = %x (ignored) pc=%06x\n", (unsigned long long)NOW, addr, val, S->cpu.last_pc); return; }
    if (S->log_unknown) LOG("[%llu] unmapped write %08x = %x size %d pc=%06x\n", (unsigned long long)NOW, addr, val, size, S->cpu.last_pc);
}

/* ------------------------------------------------------------------ init / run */
void sys_init(sys_t *s, const uint8_t *app, size_t applen)
{
    S = s;
    memset(s->flash, 0xFF, sizeof s->flash);
    if (applen > FLASH_SIZE) applen = FLASH_SIZE;
    memcpy(s->flash, app, applen);
    /* board detection: the colour unit image is 384 KB with its vector table (16-byte slots) at 0x4400,
     * the mono unit image is 256 KB with a compact table (4-byte slots, slot k = vector 0x10*(k+1)) at 0x4500 */
    s->board = (applen <= 0x40000) ? 1 : 0;
    s->module = s->board ? app[0x4701] : 0;
    if (s->module == 2) { s->ee_cs_port = 0x00; s->ee_cs_bit = 0x10; } else { s->ee_cs_port = 0x13; s->ee_cs_bit = 0x20; }
    s->can_base = s->board ? 0x03FEC000 : 0x03FEC600;
    s->vec_can  = s->board ? 0x2C0 : 0x3D0;
    for (uint32_t v = 0; v < NVEC; v += 0x10) {
        uint32_t target = s->board ? (v == 0 ? 0x4500 : 0x4500 + 4 * (v / 0x10 - 1)) : 0x4400 + v;
        if (s->board && target >= 0x4670) continue;              /* beyond the app's table */
        uint32_t disp = (target - v) & 0x3FFFFF;                 /* jr disp22 */
        s->flash[v + 0] = 0x80 | ((disp >> 16) & 0x3F); s->flash[v + 1] = 0x07;
        s->flash[v + 2] = disp & 0xFF; s->flash[v + 3] = (disp >> 8) & 0xFF;
    }
    /* plausible bootloader data the app reads */
    if (s->board == 0) memcpy(&s->flash[0x720], "8R29-14D017-AA", 15);
    else {
        /* 0x700: pointer to the bootloader's reprogramming entry (we point it at a 'jmp [lp]'), 0x704-0x706: FBL version bytes */
        s->flash[0x700] = 0x10; s->flash[0x701] = 0x07; s->flash[0x702] = 0; s->flash[0x703] = 0;
        s->flash[0x710] = 0x7F; s->flash[0x711] = 0x00;                       /* jmp [lp] */
        s->flash[0x704] = 1; s->flash[0x705] = 0; s->flash[0x706] = 0;
    }
    memset(s->ram, 0, sizeof s->ram);
    memset(s->ic, 0x47, sizeof s->ic);
    s->ispr = 0;
    memset(s->port, 0, sizeof s->port); memset(s->portc, 0, sizeof s->portc);
    for (int i = 0x20; i < 0x40; i++) { s->port[i] = 0xFF; s->portc[i] = 0xFF; }  /* PM: all inputs after reset */
    s->portc[0x0A] = 0; s->pinc_in[0x0A] = 0x04;
    for (int i = 0; i < 16; i++) s->adc_val[i] = 0x200;
    if (!s->eeprom_loaded) memset(s->eeprom, 0xFF, sizeof s->eeprom);
    s->pin_in[0x08] |= 1;   /* MISO idle high */
    timer_init(&s->tmp[0], "TMP0", 0x170, 0x180, 0x160);
    timer_init(&s->tmp[1], "TMP1", 0x1A0, 0x1B0, 0x190);
    timer_init(&s->tmp[2], "TMP2", VEC_TICK, 0x1E0, 0x1C0);
    timer_init(&s->tmp[3], "TMP3", 0x200, 0x210, 0x1F0);
    timer_init(&s->tmq[0], "TMQ0", 0x120, 0x130, 0x110); s->tmq[0].ccr[2] = 1;
    timer_init(&s->tmq[1], "TMQ1", 0x370, 0x380, 0x360); s->tmq[1].ccr[2] = 1;
    timer_init(&s->tmq[2], "TMQ2", -1, -1, -1); s->tmq[2].ccr[2] = 1;
    s->wt_next = ~0ULL; s->wtov_next = ~0ULL; s->cb0_done = 0; s->adc_done = ~0ULL;
    s->c_ctrl = 0; s->c_ie = 0; s->c_ints = 0;
    cpu_reset(&s->cpu, 0x0000);
}

static void sys_events(sys_t *s)
{
    for (int i = 0; i < 4; i++) if (s->tmp[i].ctl0 & 0x80) timer_tick(&s->tmp[i]);
    for (int i = 0; i < 3; i++) if (s->tmq[i].ctl0 & 0x80) timer_tick(&s->tmq[i]);
    if (NOW >= s->wt_next) { sys_raise(s, VEC_WTI); s->wt_next += (1ULL << (4 + ((s->wtm >> 4) & 7))) * CPU_HZ / 32768; }
    if (NOW >= s->wtov_next) { sys_raise(s, VEC_WTOV); s->wtov_next += s->wtov_period; }
    if (s->cb0_done && NOW >= s->cb0_done) { s->cb0_done = 0; if (s->cb0ctl0 & 0x80) sys_raise(s, VEC_CSIB0); }
    if (s->cb1_done && NOW >= s->cb1_done) { s->cb1_done = 0; if (s->cb1ctl0 & 0x80) { sys_raise(s, VEC_CSIB1_R); sys_raise(s, VEC_CSIB1_T); } }
    if (s->ua1_tx_done && NOW >= s->ua1_tx_done) { s->ua1_tx_done = 0; if (s->ua1ctl0 & 0x80) sys_raise(s, VEC_UA1_T); }
    if (s->ua1_rxq_count && NOW >= s->ua1_rx_next) {
        s->ua1rx = s->ua1_rxq[s->ua1_rxq_head]; s->ua1_rxq_head = (s->ua1_rxq_head + 1) % (int)sizeof s->ua1_rxq; s->ua1_rxq_count--; s->ua1_rx_count++;
        s->ua1_rx_next = NOW + (s->ua1_byte_cycles ? s->ua1_byte_cycles : 2000);
        if ((s->ua1ctl0 & 0xA0) == 0xA0) sys_raise(s, VEC_UA1_R);
    }
    if (s->adc_busy && NOW >= s->adc_done) { s->adc_busy = 0; s->adc_done = ~0ULL; sys_raise(s, VEC_INTAD); }
    if (s->gdc) { gdc_tick(s->gdc, NOW); if (gdc_irq_pending(s->gdc)) sys_raise(s, VEC_INTP0); }
}

uint32_t g_watch[16]; int g_nwatch; uint32_t g_callw[16]; int g_ncallw;
static void watch_check(sys_t *s)
{
    for (int i = 0; i < g_ncallw; i++) if (s->cpu.pc == g_callw[i]) {
        LOG("[%llu] CALL %06x(r6=%x r7=%x r8=%x r9=%x | stk", (unsigned long long)NOW, s->cpu.pc, s->cpu.r[6], s->cpu.r[7], s->cpu.r[8], s->cpu.r[9]);
        for (int k = 0; k < 8; k++) LOG(" %x", bus_read(&s->cpu, s->cpu.r[3] + 4 * k, 4));
        LOG(") lp=%06x\n", s->cpu.r[31]);
    }
    for (int i = 0; i < g_nwatch; i++) if (s->cpu.pc == g_watch[i]) {
        LOG("[%llu] WATCH pc=%06x r6=%08x r7=%08x r8=%08x r9=%08x r10=%08x r11=%08x r12=%08x r13=%08x r29=%08x lp=%06x\n", (unsigned long long)NOW, s->cpu.pc,
            s->cpu.r[6], s->cpu.r[7], s->cpu.r[8], s->cpu.r[9], s->cpu.r[10], s->cpu.r[11], s->cpu.r[12], s->cpu.r[13], s->cpu.r[29], s->cpu.r[31]);
    }
}
void sys_run(sys_t *s, uint64_t cycles)
{
    uint64_t end = NOW + cycles;
    while (NOW < end) {
        if (s->cpu.halted == 2) return;
        if (s->cpu.halted) {
            /* idle: jump to next event */
            uint64_t nxt = end;
            for (int i = 0; i < 4; i++) if ((s->tmp[i].ctl0 & 0x80) && s->tmp[i].next_match < nxt) nxt = s->tmp[i].next_match;
            for (int i = 0; i < 3; i++) if ((s->tmq[i].ctl0 & 0x80) && s->tmq[i].next_match < nxt) nxt = s->tmq[i].next_match;
            if (s->wt_next < nxt) nxt = s->wt_next;
            if (s->wtov_next < nxt) nxt = s->wtov_next;
            if (s->cb0_done && s->cb0_done < nxt) nxt = s->cb0_done;
            if (s->cb1_done && s->cb1_done < nxt) nxt = s->cb1_done;
            if (s->ua1_tx_done && s->ua1_tx_done < nxt) nxt = s->ua1_tx_done;
            if (s->ua1_rxq_count && s->ua1_rx_next < nxt) nxt = s->ua1_rx_next;
            if (s->adc_busy && s->adc_done < nxt) nxt = s->adc_done;
            if (nxt > NOW) s->cpu.cycles = nxt; else s->cpu.cycles++;
            sys_events(s);
            if (sys_pending_irq(s)) s->cpu.halted = 0;
            continue;
        }
        if (g_nwatch | g_ncallw) watch_check(s);
        cpu_step(&s->cpu);
        if ((s->cpu.insns & 15) == 0) sys_events(s);
        sys_pending_irq(s);
    }
}

static const char *sfr_name(uint32_t off)
{
    switch (off) {
    case 0x008: return "PCS"; case 0x00A: return "PCT"; case 0x00C: return "PCM"; case 0x00E: return "PCD";
    case 0x1FA: return "ISPR"; case 0x200: return "ADA0M0"; case 0x680: return "WTM";
    default: return "?";
    }
}

void sys_uart1_inject(sys_t *s, const uint8_t *data, int n)
{
    for (int i = 0; i < n; i++) { if (s->ua1_rxq_count >= (int)sizeof s->ua1_rxq) break; s->ua1_rxq[(s->ua1_rxq_head + s->ua1_rxq_count) % (int)sizeof s->ua1_rxq] = data[i]; s->ua1_rxq_count++; }
    if (s->ua1_rx_next < NOW) s->ua1_rx_next = NOW;
}

void sys_dump_can(sys_t *s, FILE *f)
{
    if (s->ua1_txline_len) { LOG("[%llu] UARTA1 TX:", (unsigned long long)s->ua1_txline_t0); for (int k = 0; k < s->ua1_txline_len; k++) LOG(" %02x", s->ua1_txline[k]); LOG("\n"); s->ua1_txline_len = 0; }
    if (s->ua1_tx_count || s->cb1_count) fprintf(f, "UARTA1: %d bytes sent, %d received; CSIB1: %d transfers\n", s->ua1_tx_count, s->ua1_rx_count, s->cb1_count);
    fprintf(f, "CAN%d: ctrl=%04x ie=%04x ints=%04x brp=%02x btr=%04x masks:", s->board ? 0 : 1, s->c_ctrl, s->c_ie, s->c_ints, s->c_brp, s->c_btr);
    for (int k = 0; k < 4; k++) fprintf(f, " m%d=%04x%04x", k + 1, s->c_mask[k * 2 + 1], s->c_mask[k * 2]);
    fprintf(f, "\n");
    for (int m = 0; m < 32; m++) {
        uint8_t *b = s->c_mb[m]; uint16_t ctrl = mb16(s, m, 0xE), idh = mb16(s, m, 0xC), idl = mb16(s, m, 0xA);
        if (!(b[9] & 1) && !ctrl && !idh) continue;
        int ext = !!(idh & 0x8000); uint32_t id = ext ? (((idh & 0x1FFF) << 16) | idl) : ((idh >> 2) & 0x7FF);
        int mt = (b[9] >> 3) & 7;
        fprintf(f, "  mb%02d: %s id=%0*X mt=%d(%s) ctrl=%04x dlc=%d MA0=%d\n", m, ext ? "ext" : "std", ext ? 8 : 3, id, mt,
                mt == 0 ? "TX" : mt == 1 ? "RX" : "RXmask", ctrl, b[8] & 0xF, b[9] & 1);
    }
}
