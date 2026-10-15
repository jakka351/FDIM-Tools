/* V850ES (V850E1 ISA) interpreter + disassembler.
 * Encodings cross-checked against Ghidra's V850 SLEIGH spec; semantics per NEC V850E1 architecture manual.
 */
#include <stdio.h>
#include <string.h>
#include "v850.h"

static const char *regnames[32] = {
 "r0","r1","r2","sp","gp","tp","r6","r7","r8","r9","r10","r11","r12","r13","r14","r15",
 "r16","r17","r18","r19","r20","r21","r22","r23","r24","r25","r26","r27","r28","r29","ep","lp"};
const char *cpu_regname(int r){ return regnames[r&31]; }

static const char *srnames[32] = {
 "EIPC","EIPSW","FEPC","FEPSW","ECR","PSW","FPSR","FPEPC","FPST","FPCC","FPCFG","SCCFG","SCBP","EIIC","FEIC","DBIC",
 "CTPC","CTPSW","DBPC","DBPSW","CTBP","DIR","DBG22","DBG23","DBG24","DBG25","DBG26","DBG27","EIWR","FEWR","DBWR","BSEL"};

static const char *condnames[16] = {"v","c","e","nh","n","t","lt","le","nv","nc","ne","h","p","sa","ge","gt"};

enum op {
 OP_ILLEGAL=0, OP_NOP, OP_MOV_RR, OP_MOV_I5, OP_MOV_I32, OP_NOT, OP_DIVH_RR, OP_SWITCH, OP_JMP, OP_SATSUBR, OP_SATSUB, OP_SATADD_RR,
 OP_MULH_RR, OP_ZXB, OP_SXB, OP_ZXH, OP_SXH, OP_OR, OP_XOR, OP_AND, OP_TST, OP_SUBR, OP_SUB, OP_ADD_RR, OP_CMP_RR,
 OP_CALLT, OP_SATADD_I5, OP_ADD_I5, OP_CMP_I5, OP_SHR_I5, OP_SAR_I5, OP_SHL_I5, OP_MULH_I5,
 OP_SLD_B, OP_SLD_BU, OP_SLD_H, OP_SLD_HU, OP_SLD_W, OP_SST_B, OP_SST_H, OP_SST_W, OP_BCOND,
 OP_ADDI, OP_MOVEA, OP_MOVHI, OP_SATSUBI, OP_ORI, OP_XORI, OP_ANDI, OP_MULHI,
 OP_LD_B, OP_LD_H, OP_LD_W, OP_LD_BU, OP_LD_HU, OP_ST_B, OP_ST_H, OP_ST_W,
 OP_JR, OP_JARL, OP_PREPARE, OP_DISPOSE, OP_SET1_I, OP_NOT1_I, OP_CLR1_I, OP_TST1_I,
 OP_LDSR, OP_STSR, OP_SHR_RR, OP_SAR_RR, OP_SHL_RR, OP_SET1_R, OP_NOT1_R, OP_CLR1_R, OP_TST1_R,
 OP_TRAP, OP_HALT, OP_RETI, OP_CTRET, OP_DI, OP_EI, OP_SETF, OP_SASF, OP_RIE, OP_SYNC,
 OP_SHR_3, OP_SAR_3, OP_SHL_3, OP_CAXI, OP_MUL_RR, OP_MULU_RR, OP_MUL_I9, OP_MULU_I9, OP_DIVH_3, OP_DIVHU, OP_DIV, OP_DIVU,
 OP_BSW, OP_BSH, OP_HSW, OP_HSH, OP_SCH0R, OP_SCH1R, OP_SCH0L, OP_SCH1L, OP_SATSUB_3, OP_SATADD_3,
 OP_CMOV_I5, OP_CMOV_RR, OP_SBF, OP_ADF, OP_MAC, OP_MACU
};

typedef struct {
    int op; int len;
    int r1, r2, r3;      /* reg fields */
    int cond;            /* condition code */
    int32_t imm;         /* immediate / displacement (sign-extended where applicable) */
    uint32_t list;       /* prepare/dispose list bits: bit k = register k */
    int sub;             /* prepare sub-op / dispose reg */
    int bitno;
} insn_t;

static inline int32_t sx(uint32_t v, int bits){ uint32_t m = 1u<<(bits-1); return (int32_t)((v ^ m) - m); }

/* list12 decoding: hw1 = first halfword, hw2 = second */
static uint32_t decode_list(uint32_t hw1, uint32_t hw2)
{
    uint32_t L = 0;
    /* prep token bits: 0 = hw1 bit0 (ep), 21 = hw2 bit5 (lp), 22..31 = hw2 bits 6..15 */
    if (hw1 & 1)        L |= 1u<<30;             /* ep (r30) */
    if (hw2 & (1<<5))   L |= 1u<<31;             /* lp */
    if (hw2 & (1<<6))   L |= 1u<<29;             /* prep22 -> r29 */
    if (hw2 & (1<<7))   L |= 1u<<28;             /* prep23 -> r28 */
    if (hw2 & (1<<8))   L |= 1u<<23;             /* prep24 -> r23 */
    if (hw2 & (1<<9))   L |= 1u<<22;             /* prep25 -> r22 */
    if (hw2 & (1<<10))  L |= 1u<<21;             /* prep26 -> r21 */
    if (hw2 & (1<<11))  L |= 1u<<20;             /* prep27 -> r20 */
    if (hw2 & (1<<12))  L |= 1u<<27;             /* prep28 -> r27 */
    if (hw2 & (1<<13))  L |= 1u<<26;             /* prep29 -> r26 */
    if (hw2 & (1<<14))  L |= 1u<<25;             /* prep30 -> r25 */
    if (hw2 & (1<<15))  L |= 1u<<24;             /* prep31 -> r24 */
    return L;
}

static int decode(cpu_t *c, uint32_t pc, insn_t *d)
{
    uint32_t w0 = bus_fetch16(c, pc);
    uint32_t w1 = 0, w2 = 0, w3 = 0;
    int reg1 = w0 & 0x1F, reg2 = (w0 >> 11) & 0x1F, op = (w0 >> 5) & 0x3F;
    memset(d, 0, sizeof *d);
    d->len = 2; d->r1 = reg1; d->r2 = reg2; d->op = OP_ILLEGAL;

    if (w0 == 0x0040) { d->op = OP_RIE; return 2; }
    if (op <= 0x0F) {
        switch (op) {
        case 0x00:
            if (reg2 == 0) {
                if (reg1 >= 0x1D) d->op = OP_SYNC; else d->op = OP_NOP; /* mov rX,r0 == nop */
            } else d->op = OP_MOV_RR;
            return 2;
        case 0x01: d->op = OP_NOT; return 2;
        case 0x02: d->op = reg2 ? OP_DIVH_RR : OP_SWITCH; return 2;
        case 0x03:
            if (reg2 == 0) { d->op = OP_JMP; return 2; }
            if (reg1 & 0x10) { d->op = OP_SLD_HU; d->imm = (reg1 & 0xF) * 2; } else { d->op = OP_SLD_BU; d->imm = reg1 & 0xF; }
            return 2;
        case 0x04: d->op = reg2 ? OP_SATSUBR : OP_ZXB; return 2;
        case 0x05: d->op = reg2 ? OP_SATSUB : OP_SXB; return 2;
        case 0x06: d->op = reg2 ? OP_SATADD_RR : OP_ZXH; return 2;
        case 0x07: d->op = reg2 ? OP_MULH_RR : OP_SXH; return 2;
        case 0x08: d->op = OP_OR; return 2;
        case 0x09: d->op = OP_XOR; return 2;
        case 0x0A: d->op = OP_AND; return 2;
        case 0x0B: d->op = OP_TST; return 2;
        case 0x0C: d->op = OP_SUBR; return 2;
        case 0x0D: d->op = OP_SUB; return 2;
        case 0x0E: d->op = OP_ADD_RR; return 2;
        case 0x0F: d->op = OP_CMP_RR; return 2;
        }
    }
    if (op == 0x10 || op == 0x11) {
        if (reg2 == 0) { d->op = OP_CALLT; d->imm = w0 & 0x3F; return 2; }
        d->imm = sx(reg1, 5);
        d->op = (op == 0x10) ? OP_MOV_I5 : OP_SATADD_I5; return 2;
    }
    if (op >= 0x12 && op <= 0x17) {
        static const int ops[] = {OP_ADD_I5, OP_CMP_I5, OP_SHR_I5, OP_SAR_I5, OP_SHL_I5, OP_MULH_I5};
        d->op = ops[op - 0x12];
        d->imm = (op == 0x12 || op == 0x13 || op == 0x17) ? sx(reg1, 5) : reg1;
        return 2;
    }
    if (op >= 0x18 && op <= 0x2F) {
        int o4 = (w0 >> 7) & 0xF;   /* op0710 */
        int d7 = w0 & 0x7F;
        switch (o4) {
        case 0x6: d->op = OP_SLD_B; d->imm = d7; return 2;
        case 0x7: d->op = OP_SST_B; d->imm = d7; return 2;
        case 0x8: d->op = OP_SLD_H; d->imm = d7 * 2; return 2;
        case 0x9: d->op = OP_SST_H; d->imm = d7 * 2; return 2;
        case 0xA: d->imm = (d7 & 0x7E) * 2; d->op = (w0 & 1) ? OP_SST_W : OP_SLD_W; return 2;
        case 0xB: d->op = OP_BCOND; d->cond = w0 & 0xF;
                  d->imm = sx(((w0 >> 11) & 0x1F) << 4 | ((w0 >> 4) & 0x7) << 1, 9); return 2;
        }
        d->op = OP_ILLEGAL; return 2;
    }
    /* 32-bit (or longer) formats */
    w1 = bus_fetch16(c, pc + 2);
    d->len = 4;
    switch (op) {
    case 0x30: d->op = OP_ADDI; d->imm = sx(w1, 16); return 4;
    case 0x31:
        if (reg2 == 0) { w2 = bus_fetch16(c, pc + 4); d->op = OP_MOV_I32; d->imm = (int32_t)((w2 << 16) | w1); d->len = 6; return 6; }
        d->op = OP_MOVEA; d->imm = sx(w1, 16); return 4;
    case 0x32: case 0x33:
        if (reg2 == 0) { /* dispose */
            d->op = OP_DISPOSE; d->imm = (w0 >> 1) & 0x1F; d->list = decode_list(w0, w1); d->sub = w1 & 0x1F; return 4;
        }
        if (op == 0x32) { d->op = OP_MOVHI; d->imm = sx(w1, 16); } else { d->op = OP_SATSUBI; d->imm = sx(w1, 16); }
        return 4;
    case 0x34: d->op = OP_ORI;  d->imm = w1; return 4;
    case 0x35: d->op = OP_XORI; d->imm = w1; return 4;
    case 0x36: d->op = OP_ANDI; d->imm = w1; return 4;
    case 0x37:
        if (reg2 == 0) { d->op = OP_ILLEGAL; return 4; }   /* jmp disp32[reg1] : V850E2 */
        d->op = OP_MULHI; d->imm = sx(w1, 16); return 4;
    case 0x38: d->op = OP_LD_B; d->imm = sx(w1, 16); return 4;
    case 0x39: d->imm = sx(w1 & 0xFFFE, 16); d->op = (w1 & 1) ? OP_LD_W : OP_LD_H; return 4;
    case 0x3A: d->op = OP_ST_B; d->imm = sx(w1, 16); return 4;
    case 0x3B: d->imm = sx(w1 & 0xFFFE, 16); d->op = (w1 & 1) ? OP_ST_W : OP_ST_H; return 4;
    case 0x3C: case 0x3D:
        if ((w1 & 1) == 0) {  /* jr / jarl disp22 */
            d->imm = sx(((w0 & 0x3F) << 16) | (w1 & 0xFFFE), 22);
            d->op = reg2 ? OP_JARL : OP_JR; return 4;
        }
        if (reg2 == 0) {      /* prepare */
            d->op = OP_PREPARE; d->imm = (w0 >> 1) & 0x1F; d->list = decode_list(w0, w1); d->sub = w1 & 0x1F;
            if (d->sub == 0x0B || d->sub == 0x13) { w2 = bus_fetch16(c, pc + 4); d->len = 6; d->r3 = 0;
                d->imm = d->imm; d->list = d->list; /* ep value kept in bitno field via imm2 */ d->bitno = (int)sx(w2, 16); return 6; }
            if (d->sub == 0x1B) { w2 = bus_fetch16(c, pc + 4); w3 = bus_fetch16(c, pc + 6); d->len = 8; d->bitno = (int)((w3 << 16) | w2); return 8; }
            return 4;
        }
        d->op = OP_LD_BU; d->imm = sx((w1 & 0xFFFE) | (op & 1), 16); return 4;
    case 0x3E: {
        d->imm = sx(w1, 16); d->bitno = (w0 >> 11) & 7;
        static const int ops[] = {OP_SET1_I, OP_NOT1_I, OP_CLR1_I, OP_TST1_I};
        d->op = ops[(w0 >> 14) & 3]; return 4;
    }
    case 0x3F: {
        if (w1 & 1) { d->op = OP_LD_HU; d->imm = sx(w1 & 0xFFFE, 16); return 4; }
        d->r3 = (w1 >> 11) & 0x1F;
        uint32_t sub = w1 & 0x7FF;         /* op1626 */
        /* setf / sasf: first hw bits 10..4 == 0x7E */
        if (((w0 >> 4) & 0x7F) == 0x7E && w1 == 0x0000) { d->op = OP_SETF; d->cond = w0 & 0xF; return 4; }
        if (((w0 >> 4) & 0x7F) == 0x7E && w1 == 0x0200) { d->op = OP_SASF; d->cond = w0 & 0xF; return 4; }
        if (w1 == 0x0020) { d->op = OP_LDSR; return 4; }
        if (w1 == 0x0040) { d->op = OP_STSR; return 4; }
        if (w1 == 0x0080) { d->op = OP_SHR_RR; return 4; }
        if (w1 == 0x00A0) { d->op = OP_SAR_RR; return 4; }
        if (w1 == 0x00C0) { d->op = OP_SHL_RR; return 4; }
        if (w1 == 0x00E0) { d->op = OP_SET1_R; return 4; }
        if (w1 == 0x00E2) { d->op = OP_NOT1_R; return 4; }
        if (w1 == 0x00E4) { d->op = OP_CLR1_R; return 4; }
        if (w1 == 0x00E6) { d->op = OP_TST1_R; return 4; }
        if (w1 == 0x0100 && reg2 == 0) { d->op = OP_TRAP; d->imm = reg1; return 4; }
        if (w0 == 0x07E0 && w1 == 0x0120) { d->op = OP_HALT; return 4; }
        if (w0 == 0x07E0 && w1 == 0x0140) { d->op = OP_RETI; return 4; }
        if (w0 == 0x07E0 && w1 == 0x0144) { d->op = OP_CTRET; return 4; }
        if (w0 == 0x07E0 && w1 == 0x0160) { d->op = OP_DI; return 4; }
        if (w0 == 0x87E0 && w1 == 0x0160) { d->op = OP_EI; return 4; }
        if (((w0 >> 4) & 0x7F) == 0x7F && w1 == 0) { d->op = OP_RIE; return 4; }
        switch (sub) {
        case 0x082: d->op = OP_SHR_3; return 4;
        case 0x0A2: d->op = OP_SAR_3; return 4;
        case 0x0C2: d->op = OP_SHL_3; return 4;
        case 0x0EE: d->op = OP_CAXI; return 4;
        case 0x220: d->op = OP_MUL_RR; return 4;
        case 0x222: d->op = OP_MULU_RR; return 4;
        case 0x280: d->op = OP_DIVH_3; return 4;
        case 0x282: d->op = OP_DIVHU; return 4;
        case 0x2C0: d->op = OP_DIV; return 4;
        case 0x2C2: d->op = OP_DIVU; return 4;
        case 0x2FC: d->op = OP_DIV; return 4;   /* divq */
        case 0x2FE: d->op = OP_DIVU; return 4;  /* divqu */
        case 0x340: d->op = OP_BSW; return 4;
        case 0x342: d->op = OP_BSH; return 4;
        case 0x344: d->op = OP_HSW; return 4;
        case 0x346: d->op = OP_HSH; return 4;
        case 0x360: d->op = OP_SCH0R; return 4;
        case 0x362: d->op = OP_SCH1R; return 4;
        case 0x364: d->op = OP_SCH0L; return 4;
        case 0x366: d->op = OP_SCH1L; return 4;
        case 0x39A: d->op = OP_SATSUB_3; return 4;
        case 0x3BA: d->op = OP_SATADD_3; return 4;
        }
        if (((w1 >> 6) & 0x1F) == 0x9) {  /* mul/mulu imm9 */
            int hi = (w1 >> 2) & 0xF;
            if ((w1 & 3) == 0) { d->op = OP_MUL_I9; d->imm = sx((hi << 5) | reg1, 9); return 4; }
            if ((w1 & 3) == 2) { d->op = OP_MULU_I9; d->imm = (hi << 5) | reg1; return 4; }
        }
        if ((w1 & 1) == 0) {
            int g = (w1 >> 5) & 0x3F;  /* op2126 */
            d->cond = (w1 >> 1) & 0xF;
            if (g == 0x18) { d->op = OP_CMOV_I5; d->imm = sx(reg1, 5); return 4; }
            if (g == 0x19) { d->op = OP_CMOV_RR; return 4; }
            if (g == 0x1C) { d->op = OP_SBF; return 4; }
            if (g == 0x1D) { d->op = OP_ADF; return 4; }
            if (g == 0x1E) { d->op = OP_MAC; d->r3 = (w1 >> 11) & 0x1E; d->bitno = (w1 >> 1) & 0x1E; return 4; }
            if (g == 0x1F) { d->op = OP_MACU; d->r3 = (w1 >> 11) & 0x1E; d->bitno = (w1 >> 1) & 0x1E; return 4; }
        }
        d->op = OP_ILLEGAL; return 4;
    }
    }
    d->op = OP_ILLEGAL; return 2;
}

/* ---------------- disassembler (pypcode-compatible formatting) ---------------- */
static void fmt_imm(char *b, size_t n, int32_t v)
{
    if (v < 0) snprintf(b, n, "-0x%x", (unsigned)(-v)); else snprintf(b, n, "0x%x", (unsigned)v);
}
static void fmt_list(char *b, size_t n, uint32_t list)
{
    /* order as pypcode prints: r20..r29, ep, lp */
    char *p = b; *p = 0; int first = 1;
    for (int r = 20; r < 32; r++) if (list & (1u << r)) {
        p += snprintf(p, n - (p - b), "%s%s", first ? "" : ",", regnames[r]); first = 0;
    }
}

int cpu_disasm(cpu_t *c, uint32_t pc, char *out, size_t n)
{
    insn_t d; decode(c, pc, &d);
    char a[40], b[40], l[96];
    const char *R1 = regnames[d.r1], *R2 = regnames[d.r2], *R3 = regnames[d.r3];
    uint32_t tgt = pc + d.imm;
#define P(...) snprintf(out, n, __VA_ARGS__)
    switch (d.op) {
    case OP_ILLEGAL: P("???"); break;
    case OP_NOP: P("nop"); break;
    case OP_SYNC: P(d.r1 == 0x1D ? "synce" : d.r1 == 0x1E ? "syncm" : "syncp"); break;
    case OP_MOV_RR: P("mov %s, %s", R1, R2); break;
    case OP_MOV_I5: fmt_imm(a, 40, d.imm); P("mov %s, %s", a, R2); break;
    case OP_MOV_I32: P("mov 0x%x, %s", (unsigned)d.imm, R1); break;
    case OP_NOT: P("not %s, %s", R1, R2); break;
    case OP_DIVH_RR: P("divh %s, %s", R1, R2); break;
    case OP_SWITCH: P("switch %s", R1); break;
    case OP_JMP: P("jmp [%s]", R1); break;
    case OP_SATSUBR: P("satsubr %s, %s", R1, R2); break;
    case OP_SATSUB: P("satsub %s, %s", R1, R2); break;
    case OP_SATADD_RR: P("satadd %s, %s", R1, R2); break;
    case OP_MULH_RR: P("mulh %s, %s", R1, R2); break;
    case OP_ZXB: P("zxb %s", R1); break;
    case OP_SXB: P("sxb %s", R1); break;
    case OP_ZXH: P("zxh %s", R1); break;
    case OP_SXH: P("sxh %s", R1); break;
    case OP_OR: P("or %s, %s", R1, R2); break;
    case OP_XOR: P("xor %s, %s", R1, R2); break;
    case OP_AND: P("and %s, %s", R1, R2); break;
    case OP_TST: P("tst %s, %s", R1, R2); break;
    case OP_SUBR: P("subr %s, %s", R1, R2); break;
    case OP_SUB: P("sub %s, %s", R1, R2); break;
    case OP_ADD_RR: P("add %s, %s", R1, R2); break;
    case OP_CMP_RR: P("cmp %s, %s", R1, R2); break;
    case OP_CALLT: P("callt 0x%x", (unsigned)d.imm); break;
    case OP_SATADD_I5: fmt_imm(a, 40, d.imm); P("satadd %s, %s", a, R2); break;
    case OP_ADD_I5: fmt_imm(a, 40, d.imm); P("add %s, %s", a, R2); break;
    case OP_CMP_I5: fmt_imm(a, 40, d.imm); P("cmp %s, %s", a, R2); break;
    case OP_SHR_I5: P("shr 0x%x, %s", (unsigned)d.imm, R2); break;
    case OP_SAR_I5: P("sar 0x%x, %s", (unsigned)d.imm, R2); break;
    case OP_SHL_I5: P("shl 0x%x, %s", (unsigned)d.imm, R2); break;
    case OP_MULH_I5: fmt_imm(a, 40, d.imm); P("mulh %s, %s", a, R2); break;
    case OP_SLD_B: P("sld.b 0x%x[ep], %s", (unsigned)d.imm, R2); break;
    case OP_SLD_BU: P("sld.bu 0x%x[ep], %s", (unsigned)d.imm, R2); break;
    case OP_SLD_H: P("sld.h 0x%x[ep], %s", (unsigned)d.imm, R2); break;
    case OP_SLD_HU: P("sld.hu 0x%x[ep], %s", (unsigned)d.imm, R2); break;
    case OP_SLD_W: P("sld.w 0x%x[ep], %s", (unsigned)d.imm, R2); break;
    case OP_SST_B: P("sst.b %s, 0x%x[ep]", R2, (unsigned)d.imm); break;
    case OP_SST_H: P("sst.h %s, 0x%x[ep]", R2, (unsigned)d.imm); break;
    case OP_SST_W: P("sst.w %s, 0x%x[ep]", R2, (unsigned)d.imm); break;
    case OP_BCOND: if (d.cond == 5) P("br 0x%x", tgt); else P("b%s 0x%x", condnames[d.cond], tgt); break;
    case OP_ADDI: fmt_imm(a, 40, d.imm); P("addi %s, %s, %s", a, R1, R2); break;
    case OP_MOVEA: fmt_imm(a, 40, d.imm); P("movea %s, %s, %s", a, R1, R2); break;
    case OP_MOVHI: fmt_imm(a, 40, d.imm); P("movhi %s, %s, %s", a, R1, R2); break;
    case OP_SATSUBI: fmt_imm(a, 40, d.imm); P("satsubi %s, %s, %s", a, R1, R2); break;
    case OP_ORI: P("ori 0x%x, %s, %s", (unsigned)d.imm, R1, R2); break;
    case OP_XORI: P("xori 0x%x, %s, %s", (unsigned)d.imm, R1, R2); break;
    case OP_ANDI: P("andi 0x%x, %s, %s", (unsigned)d.imm, R1, R2); break;
    case OP_MULHI: fmt_imm(a, 40, d.imm); P("mulhi %s, %s, %s", a, R1, R2); break;
    case OP_LD_B: fmt_imm(a, 40, d.imm); P("ld.b %s[%s], %s", a, R1, R2); break;
    case OP_LD_H: fmt_imm(a, 40, d.imm); P("ld.h %s[%s], %s", a, R1, R2); break;
    case OP_LD_W: fmt_imm(a, 40, d.imm); P("ld.w %s[%s], %s", a, R1, R2); break;
    case OP_LD_BU: fmt_imm(a, 40, d.imm); P("ld.bu %s[%s], %s", a, R1, R2); break;
    case OP_LD_HU: fmt_imm(a, 40, d.imm); P("ld.hu %s[%s], %s", a, R1, R2); break;
    case OP_ST_B: fmt_imm(a, 40, d.imm); P("st.b %s, %s[%s]", R2, a, R1); break;
    case OP_ST_H: fmt_imm(a, 40, d.imm); P("st.h %s, %s[%s]", R2, a, R1); break;
    case OP_ST_W: fmt_imm(a, 40, d.imm); P("st.w %s, %s[%s]", R2, a, R1); break;
    case OP_JR: P("jr 0x%x", tgt); break;
    case OP_JARL: P("jarl 0x%x, %s", tgt, R2); break;
    case OP_PREPARE: fmt_list(l, 96, d.list);
        if (d.sub == 1) P("prepare { %s }, 0x%x", l, (unsigned)d.imm);
        else if (d.sub == 3) P("prepare { %s }, 0x%x, sp", l, (unsigned)d.imm);
        else { fmt_imm(a, 40, d.bitno); P("prepare { %s }, 0x%x, %s", l, (unsigned)d.imm, a); }
        break;
    case OP_DISPOSE: fmt_list(l, 96, d.list);
        if (d.sub) P("dispose 0x%x, { %s }, [%s]", (unsigned)d.imm, l, regnames[d.sub]); else P("dispose 0x%x, { %s }", (unsigned)d.imm, l); break;
    case OP_SET1_I: fmt_imm(a, 40, d.imm); P("set1 0x%x, %s[%s]", d.bitno, a, R1); break;
    case OP_NOT1_I: fmt_imm(a, 40, d.imm); P("not1 0x%x, %s[%s]", d.bitno, a, R1); break;
    case OP_CLR1_I: fmt_imm(a, 40, d.imm); P("clr1 0x%x, %s[%s]", d.bitno, a, R1); break;
    case OP_TST1_I: fmt_imm(a, 40, d.imm); P("tst1 0x%x, %s[%s]", d.bitno, a, R1); break;
    case OP_LDSR: P("ldsr %s, %s", R1, srnames[d.r2]); break;
    case OP_STSR: P("stsr %s, %s", srnames[d.r1], R2); break;
    case OP_SHR_RR: P("shr %s, %s", R1, R2); break;
    case OP_SAR_RR: P("sar %s, %s", R1, R2); break;
    case OP_SHL_RR: P("shl %s, %s", R1, R2); break;
    case OP_SET1_R: P("set1 %s, [%s]", R2, R1); break;
    case OP_NOT1_R: P("not1 %s, [%s]", R2, R1); break;
    case OP_CLR1_R: P("clr1 %s, [%s]", R2, R1); break;
    case OP_TST1_R: P("tst1 %s, [%s]", R1, R2); break;
    case OP_TRAP: P("trap 0x%x", (unsigned)d.imm); break;
    case OP_HALT: P("halt"); break;
    case OP_RETI: P("reti"); break;
    case OP_CTRET: P("ctret"); break;
    case OP_DI: P("di"); break;
    case OP_EI: P("ei"); break;
    case OP_SETF: P("setf%s %s", condnames[d.cond], R2); break;
    case OP_SASF: P("sasf%s %s", condnames[d.cond], R2); break;
    case OP_RIE: P("rie"); break;
    case OP_SHR_3: P("shr %s, %s, %s", R1, R2, R3); break;
    case OP_SAR_3: P("sar %s, %s, %s", R1, R2, R3); break;
    case OP_SHL_3: P("shl %s, %s, %s", R1, R2, R3); break;
    case OP_CAXI: P("caxi [%s], %s, %s", R1, R2, R3); break;
    case OP_MUL_RR: P("mul %s, %s, %s", R1, R2, R3); break;
    case OP_MULU_RR: P("mulu %s, %s, %s", R1, R2, R3); break;
    case OP_MUL_I9: fmt_imm(a, 40, d.imm); P("mul %s, %s, %s", a, R2, R3); break;
    case OP_MULU_I9: P("mulu 0x%x, %s, %s", (unsigned)d.imm, R2, R3); break;
    case OP_DIVH_3: P("divh %s, %s, %s", R1, R2, R3); break;
    case OP_DIVHU: P("divhu %s, %s, %s", R1, R2, R3); break;
    case OP_DIV: P("div %s, %s, %s", R1, R2, R3); break;
    case OP_DIVU: P("divu %s, %s, %s", R1, R2, R3); break;
    case OP_BSW: P("bsw %s, %s", R2, R3); break;
    case OP_BSH: P("bsh %s, %s", R2, R3); break;
    case OP_HSW: P("hsw %s, %s", R2, R3); break;
    case OP_HSH: P("hsh %s, %s", R2, R3); break;
    case OP_SCH0R: P("sch0r %s, %s", R2, R3); break;
    case OP_SCH1R: P("sch1r %s, %s", R2, R3); break;
    case OP_SCH0L: P("sch0l %s, %s", R2, R3); break;
    case OP_SCH1L: P("sch1l %s, %s", R2, R3); break;
    case OP_SATSUB_3: P("satsub %s, %s, %s", R1, R2, R3); break;
    case OP_SATADD_3: P("satadd %s, %s, %s", R1, R2, R3); break;
    case OP_CMOV_I5: fmt_imm(a, 40, d.imm); P("cmov%s %s, %s, %s", condnames[d.cond], a, R2, R3); break;
    case OP_CMOV_RR: P("cmov%s %s, %s, %s", condnames[d.cond], R1, R2, R3); break;
    case OP_SBF: P("sbf%s %s, %s, %s", condnames[d.cond], R1, R2, R3); break;
    case OP_ADF: P("adf%s %s, %s, %s", condnames[d.cond], R1, R2, R3); break;
    case OP_MAC: P("mac %s, %s, r%d, r%d", R1, R2, d.r3, d.bitno); break;
    case OP_MACU: P("macu %s, %s, r%d, r%d", R1, R2, d.r3, d.bitno); break;
    default: P("???"); break;
    }
#undef P
    (void)b;
    return d.len;
}

/* ---------------- execution ---------------- */
static inline void set_flags_add(cpu_t *c, uint32_t a, uint32_t b, uint32_t res)
{
    uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY);
    if (res == 0) f |= PSW_Z;
    if (res & 0x80000000u) f |= PSW_S;
    if (((a ^ res) & (b ^ res)) & 0x80000000u) f |= PSW_OV;
    if (res < a) f |= PSW_CY;
    c->sr[SR_PSW] = f;
}
static inline void set_flags_sub(cpu_t *c, uint32_t a, uint32_t b, uint32_t res) /* res = a - b */
{
    uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY);
    if (res == 0) f |= PSW_Z;
    if (res & 0x80000000u) f |= PSW_S;
    if (((a ^ b) & (a ^ res)) & 0x80000000u) f |= PSW_OV;
    if (a < b) f |= PSW_CY;
    c->sr[SR_PSW] = f;
}
static inline void set_flags_logic(cpu_t *c, uint32_t res)
{
    uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV);
    if (res == 0) f |= PSW_Z;
    if (res & 0x80000000u) f |= PSW_S;
    c->sr[SR_PSW] = f;
}
static inline void set_flags_shift(cpu_t *c, uint32_t res, int cy)
{
    uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY);
    if (res == 0) f |= PSW_Z;
    if (res & 0x80000000u) f |= PSW_S;
    if (cy) f |= PSW_CY;
    c->sr[SR_PSW] = f;
}
static inline int cond_true(cpu_t *c, int cond)
{
    uint32_t p = c->sr[SR_PSW];
    int Z = !!(p & PSW_Z), S = !!(p & PSW_S), OV = !!(p & PSW_OV), CY = !!(p & PSW_CY), SAT = !!(p & PSW_SAT);
    switch (cond) {
    case 0x0: return OV; case 0x8: return !OV;
    case 0x1: return CY; case 0x9: return !CY;
    case 0x2: return Z;  case 0xA: return !Z;
    case 0x3: return CY || Z; case 0xB: return !(CY || Z);
    case 0x4: return S;  case 0xC: return !S;
    case 0x5: return 1;  case 0xD: return SAT;
    case 0x6: return S ^ OV; case 0xE: return !(S ^ OV);
    case 0x7: return (S ^ OV) || Z; case 0xF: return !((S ^ OV) || Z);
    }
    return 0;
}
static inline uint32_t saturate(cpu_t *c, int64_t v)
{
    if (v > 0x7FFFFFFFLL) { c->sr[SR_PSW] |= PSW_SAT; return 0x7FFFFFFFu; }
    if (v < -0x80000000LL) { c->sr[SR_PSW] |= PSW_SAT; return 0x80000000u; }
    return (uint32_t)v;
}
static inline void sat_flags(cpu_t *c, uint32_t a, uint32_t b, uint32_t res, int sub)
{
    if (sub) set_flags_sub(c, a, b, res); else set_flags_add(c, a, b, res);
}

void cpu_reset(cpu_t *c, uint32_t pc)
{
    memset(c->r, 0, sizeof c->r); memset(c->sr, 0, sizeof c->sr);
    c->sr[SR_PSW] = PSW_ID | PSW_NP;   /* reset: NP=1? ID=1 */
    c->pc = pc; c->halted = 0; c->cycles = 0; c->insns = 0;
}

void cpu_interrupt(cpu_t *c, uint32_t vector_offset, uint32_t exception_code)
{
    c->sr[SR_EIPC] = c->pc;
    c->sr[SR_EIPSW] = c->sr[SR_PSW];
    c->sr[SR_ECR] = (c->sr[SR_ECR] & 0xFFFF0000u) | (exception_code & 0xFFFF);
    c->sr[SR_PSW] = (c->sr[SR_PSW] & ~PSW_EP) | PSW_ID;
    c->pc = vector_offset;
    c->halted = 0;
}

#define R(x) c->r[(x)]
#define WR(x, v) do { if ((x) != 0) c->r[(x)] = (v); } while (0)

int cpu_step(cpu_t *c)
{
    insn_t d;
    uint32_t pc = c->pc;
    c->last_pc = pc;
    decode(c, pc, &d);
    uint32_t next = pc + d.len;
    uint32_t a, b, res; int32_t s; int64_t w;
    int cyc = 1;
    if (c->trace) { char buf[128]; cpu_disasm(c, pc, buf, sizeof buf); fprintf(stderr, "%06x: %s\n", pc, buf); }
    c->insns++;
    switch (d.op) {
    case OP_NOP: case OP_SYNC: break;
    case OP_MOV_RR: WR(d.r2, R(d.r1)); break;
    case OP_MOV_I5: WR(d.r2, (uint32_t)d.imm); break;
    case OP_MOV_I32: WR(d.r1, (uint32_t)d.imm); break;
    case OP_NOT: res = ~R(d.r1); WR(d.r2, res); set_flags_logic(c, res); break;
    case OP_DIVH_RR: {
        int32_t dv = (int16_t)R(d.r1); int32_t nd = (int32_t)R(d.r2);
        c->sr[SR_PSW] &= ~PSW_OV;
        if (dv == 0) { c->sr[SR_PSW] |= PSW_OV; res = R(d.r2); }
        else if (nd == (int32_t)0x80000000 && dv == -1) { c->sr[SR_PSW] |= PSW_OV; res = 0x80000000u; }
        else res = (uint32_t)(nd / dv);
        WR(d.r2, res); set_flags_logic(c, res); cyc = 35; break;
    }
    case OP_SWITCH: {
        uint32_t adr = next + (R(d.r1) << 1);
        int16_t off = (int16_t)bus_read(c, adr, 2);
        next = next + ((int32_t)off << 1); cyc = 5; break;
    }
    case OP_JMP: next = R(d.r1); cyc = 3; break;
    case OP_SATSUBR: a = R(d.r1); b = R(d.r2); w = (int64_t)(int32_t)a - (int32_t)b; res = saturate(c, w); sat_flags(c, a, b, (uint32_t)w, 1); WR(d.r2, res); break;
    case OP_SATSUB:  a = R(d.r2); b = R(d.r1); w = (int64_t)(int32_t)a - (int32_t)b; res = saturate(c, w); sat_flags(c, a, b, (uint32_t)w, 1); WR(d.r2, res); break;
    case OP_SATADD_RR: a = R(d.r2); b = R(d.r1); w = (int64_t)(int32_t)a + (int32_t)b; res = saturate(c, w); sat_flags(c, a, b, (uint32_t)w, 0); WR(d.r2, res); break;
    case OP_SATADD_I5: a = R(d.r2); b = (uint32_t)d.imm; w = (int64_t)(int32_t)a + d.imm; res = saturate(c, w); sat_flags(c, a, b, (uint32_t)w, 0); WR(d.r2, res); break;
    case OP_SATSUBI: a = R(d.r1); b = (uint32_t)d.imm; w = (int64_t)(int32_t)a - d.imm; res = saturate(c, w); sat_flags(c, a, b, (uint32_t)w, 1); WR(d.r2, res); break;
    case OP_SATSUB_3: a = R(d.r2); b = R(d.r1); w = (int64_t)(int32_t)a - (int32_t)b; res = saturate(c, w); sat_flags(c, a, b, (uint32_t)w, 1); WR(d.r3, res); break;
    case OP_SATADD_3: a = R(d.r2); b = R(d.r1); w = (int64_t)(int32_t)a + (int32_t)b; res = saturate(c, w); sat_flags(c, a, b, (uint32_t)w, 0); WR(d.r3, res); break;
    case OP_MULH_RR: WR(d.r2, (uint32_t)((int32_t)(int16_t)R(d.r2) * (int32_t)(int16_t)R(d.r1))); break;
    case OP_MULH_I5: WR(d.r2, (uint32_t)((int32_t)(int16_t)R(d.r2) * d.imm)); break;
    case OP_MULHI: WR(d.r2, (uint32_t)((int32_t)(int16_t)R(d.r1) * d.imm)); break;
    case OP_ZXB: WR(d.r1, R(d.r1) & 0xFF); break;
    case OP_SXB: WR(d.r1, (uint32_t)(int32_t)(int8_t)R(d.r1)); break;
    case OP_ZXH: WR(d.r1, R(d.r1) & 0xFFFF); break;
    case OP_SXH: WR(d.r1, (uint32_t)(int32_t)(int16_t)R(d.r1)); break;
    case OP_OR:  res = R(d.r2) | R(d.r1); WR(d.r2, res); set_flags_logic(c, res); break;
    case OP_XOR: res = R(d.r2) ^ R(d.r1); WR(d.r2, res); set_flags_logic(c, res); break;
    case OP_AND: res = R(d.r2) & R(d.r1); WR(d.r2, res); set_flags_logic(c, res); break;
    case OP_TST: res = R(d.r2) & R(d.r1); set_flags_logic(c, res); break;
    case OP_ORI:  res = R(d.r1) | (uint32_t)d.imm; WR(d.r2, res); set_flags_logic(c, res); break;
    case OP_XORI: res = R(d.r1) ^ (uint32_t)d.imm; WR(d.r2, res); set_flags_logic(c, res); break;
    case OP_ANDI: res = R(d.r1) & (uint32_t)d.imm; WR(d.r2, res); set_flags_logic(c, res); break;
    case OP_SUBR: a = R(d.r1); b = R(d.r2); res = a - b; set_flags_sub(c, a, b, res); WR(d.r2, res); break;
    case OP_SUB:  a = R(d.r2); b = R(d.r1); res = a - b; set_flags_sub(c, a, b, res); WR(d.r2, res); break;
    case OP_ADD_RR: a = R(d.r2); b = R(d.r1); res = a + b; set_flags_add(c, a, b, res); WR(d.r2, res); break;
    case OP_ADD_I5: a = R(d.r2); b = (uint32_t)d.imm; res = a + b; set_flags_add(c, a, b, res); WR(d.r2, res); break;
    case OP_ADDI:   a = R(d.r1); b = (uint32_t)d.imm; res = a + b; set_flags_add(c, a, b, res); WR(d.r2, res); break;
    case OP_CMP_RR: a = R(d.r2); b = R(d.r1); set_flags_sub(c, a, b, a - b); break;
    case OP_CMP_I5: a = R(d.r2); b = (uint32_t)d.imm; set_flags_sub(c, a, b, a - b); break;
    case OP_CALLT: {
        c->sr[SR_CTPC] = next; c->sr[SR_CTPSW] = c->sr[SR_PSW];
        uint32_t adr = c->sr[SR_CTBP] + (d.imm << 1);
        next = c->sr[SR_CTBP] + bus_read(c, adr, 2); cyc = 5; break;
    }
    case OP_SHR_I5: case OP_SHR_RR: case OP_SHR_3: {
        uint32_t sh = (d.op == OP_SHR_I5) ? (uint32_t)d.imm : (R(d.r1) & 0x1F);
        a = R(d.r2); res = sh ? a >> sh : a; int cy = sh ? ((a >> (sh - 1)) & 1) : 0;
        set_flags_shift(c, res, cy); WR(d.op == OP_SHR_3 ? d.r3 : d.r2, res); break;
    }
    case OP_SAR_I5: case OP_SAR_RR: case OP_SAR_3: {
        uint32_t sh = (d.op == OP_SAR_I5) ? (uint32_t)d.imm : (R(d.r1) & 0x1F);
        a = R(d.r2); res = sh ? (uint32_t)((int32_t)a >> sh) : a; int cy = sh ? ((a >> (sh - 1)) & 1) : 0;
        set_flags_shift(c, res, cy); WR(d.op == OP_SAR_3 ? d.r3 : d.r2, res); break;
    }
    case OP_SHL_I5: case OP_SHL_RR: case OP_SHL_3: {
        uint32_t sh = (d.op == OP_SHL_I5) ? (uint32_t)d.imm : (R(d.r1) & 0x1F);
        a = R(d.r2); res = sh ? a << sh : a; int cy = sh ? ((a >> (32 - sh)) & 1) : 0;
        set_flags_shift(c, res, cy); WR(d.op == OP_SHL_3 ? d.r3 : d.r2, res); break;
    }
    case OP_SLD_B:  WR(d.r2, (uint32_t)(int32_t)(int8_t)bus_read(c, R(30) + d.imm, 1)); break;
    case OP_SLD_BU: WR(d.r2, bus_read(c, R(30) + d.imm, 1)); break;
    case OP_SLD_H:  WR(d.r2, (uint32_t)(int32_t)(int16_t)bus_read(c, R(30) + d.imm, 2)); break;
    case OP_SLD_HU: WR(d.r2, bus_read(c, R(30) + d.imm, 2)); break;
    case OP_SLD_W:  WR(d.r2, bus_read(c, R(30) + d.imm, 4)); break;
    case OP_SST_B:  bus_write(c, R(30) + d.imm, R(d.r2), 1); break;
    case OP_SST_H:  bus_write(c, R(30) + d.imm, R(d.r2), 2); break;
    case OP_SST_W:  bus_write(c, R(30) + d.imm, R(d.r2), 4); break;
    case OP_BCOND: if (cond_true(c, d.cond)) { next = pc + d.imm; cyc = 3; } break;
    case OP_MOVEA: WR(d.r2, R(d.r1) + (uint32_t)d.imm); break;
    case OP_MOVHI: WR(d.r2, R(d.r1) + ((uint32_t)d.imm << 16)); break;
    case OP_LD_B:  WR(d.r2, (uint32_t)(int32_t)(int8_t)bus_read(c, R(d.r1) + d.imm, 1)); break;
    case OP_LD_H:  WR(d.r2, (uint32_t)(int32_t)(int16_t)bus_read(c, R(d.r1) + d.imm, 2)); break;
    case OP_LD_W:  WR(d.r2, bus_read(c, R(d.r1) + d.imm, 4)); break;
    case OP_LD_BU: WR(d.r2, bus_read(c, R(d.r1) + d.imm, 1)); break;
    case OP_LD_HU: WR(d.r2, bus_read(c, R(d.r1) + d.imm, 2)); break;
    case OP_ST_B:  bus_write(c, R(d.r1) + d.imm, R(d.r2), 1); break;
    case OP_ST_H:  bus_write(c, R(d.r1) + d.imm, R(d.r2), 2); break;
    case OP_ST_W:  bus_write(c, R(d.r1) + d.imm, R(d.r2), 4); break;
    case OP_JR:   next = pc + d.imm; cyc = 3; break;
    case OP_JARL: WR(d.r2, next); next = pc + d.imm; cyc = 3; break;
    case OP_PREPARE: {
        uint32_t sp = R(3);
        for (int r = 20; r < 32; r++) if (d.list & (1u << r)) { sp -= 4; bus_write(c, sp, R(r), 4); }
        sp -= (uint32_t)d.imm << 2;
        R(3) = sp;
        if (d.sub == 3) R(30) = sp;
        else if (d.sub == 0x0B) R(30) = (uint32_t)d.bitno;
        else if (d.sub == 0x13) R(30) = (uint32_t)d.bitno << 16;
        else if (d.sub == 0x1B) R(30) = (uint32_t)d.bitno;
        cyc = 2 + __builtin_popcount(d.list); break;
    }
    case OP_DISPOSE: {
        uint32_t sp = R(3) + ((uint32_t)d.imm << 2);
        for (int r = 31; r >= 20; r--) if (d.list & (1u << r)) { uint32_t v = bus_read(c, sp, 4); sp += 4; c->r[r] = v; }
        R(3) = sp;
        if (d.sub) { next = R(d.sub); cyc += 2; }
        cyc += 1 + __builtin_popcount(d.list); break;
    }
    case OP_SET1_I: case OP_NOT1_I: case OP_CLR1_I: case OP_TST1_I: {
        uint32_t adr = R(d.r1) + d.imm; uint32_t v = bus_read(c, adr, 1); uint32_t m = 1u << d.bitno;
        if (v & m) c->sr[SR_PSW] &= ~PSW_Z; else c->sr[SR_PSW] |= PSW_Z;
        if (d.op == OP_SET1_I) bus_write(c, adr, v | m, 1);
        else if (d.op == OP_CLR1_I) bus_write(c, adr, v & ~m, 1);
        else if (d.op == OP_NOT1_I) bus_write(c, adr, v ^ m, 1);
        cyc = 3; break;
    }
    case OP_SET1_R: case OP_NOT1_R: case OP_CLR1_R: case OP_TST1_R: {
        uint32_t adr = R(d.r1); uint32_t v = bus_read(c, adr, 1); uint32_t m = 1u << (R(d.r2) & 7);
        if (v & m) c->sr[SR_PSW] &= ~PSW_Z; else c->sr[SR_PSW] |= PSW_Z;
        if (d.op == OP_SET1_R) bus_write(c, adr, v | m, 1);
        else if (d.op == OP_CLR1_R) bus_write(c, adr, v & ~m, 1);
        else if (d.op == OP_NOT1_R) bus_write(c, adr, v ^ m, 1);
        cyc = 3; break;
    }
    case OP_LDSR: {
        uint32_t v = R(d.r1); int id = d.r2;
        if (id == SR_PSW) v &= 0xFF;
        if (id == SR_ECR) break;   /* read-only */
        c->sr[id] = v; break;
    }
    case OP_STSR: WR(d.r2, c->sr[d.r1]); break;
    case OP_TRAP: {
        c->sr[SR_EIPC] = next; c->sr[SR_EIPSW] = c->sr[SR_PSW];
        c->sr[SR_ECR] = (c->sr[SR_ECR] & 0xFFFF0000u) | (0x40 + d.imm);
        c->sr[SR_PSW] |= PSW_EP | PSW_ID;
        next = (d.imm <= 15) ? 0x40 : 0x50; cyc = 4; break;
    }
    case OP_HALT: c->halted = 1; next = pc + 4; break;   /* stays halted until interrupt */
    case OP_RETI: {
        uint32_t p = c->sr[SR_PSW];
        if (p & PSW_EP) { next = c->sr[SR_EIPC]; c->sr[SR_PSW] = c->sr[SR_EIPSW] & 0xFF; }
        else if (p & PSW_NP) { next = c->sr[SR_FEPC]; c->sr[SR_PSW] = c->sr[SR_FEPSW] & 0xFF; }
        else { next = c->sr[SR_EIPC]; c->sr[SR_PSW] = c->sr[SR_EIPSW] & 0xFF; }
        extern void sys_reti_hook(cpu_t *c, int was_ep, int was_np);
        sys_reti_hook(c, !!(p & PSW_EP), !!(p & PSW_NP));
        cyc = 4; break;
    }
    case OP_CTRET: next = c->sr[SR_CTPC]; c->sr[SR_PSW] = c->sr[SR_CTPSW] & 0xFF; cyc = 4; break;
    case OP_DI: c->sr[SR_PSW] |= PSW_ID; break;
    case OP_EI: c->sr[SR_PSW] &= ~PSW_ID; break;
    case OP_SETF: WR(d.r2, cond_true(c, d.cond) ? 1 : 0); break;
    case OP_SASF: WR(d.r2, (R(d.r2) << 1) | (cond_true(c, d.cond) ? 1 : 0)); break;
    case OP_RIE: fprintf(stderr, "RIE (illegal instruction) at %06x\n", pc); c->halted = 2; break;
    case OP_CAXI: {
        uint32_t adr = R(d.r1) & ~3u; uint32_t tkn = bus_read(c, adr, 4);
        a = R(d.r2); set_flags_sub(c, a, tkn, a - tkn);
        if (a == tkn) bus_write(c, adr, R(d.r3), 4); else bus_write(c, adr, tkn, 4);
        WR(d.r3, tkn); break;
    }
    case OP_MUL_RR: w = (int64_t)(int32_t)R(d.r2) * (int32_t)R(d.r1); WR(d.r2, (uint32_t)w); WR(d.r3, (uint32_t)(w >> 32)); cyc = 2; break;
    case OP_MULU_RR: { uint64_t u = (uint64_t)R(d.r2) * (uint64_t)R(d.r1); WR(d.r2, (uint32_t)u); WR(d.r3, (uint32_t)(u >> 32)); cyc = 2; break; }
    case OP_MUL_I9: w = (int64_t)(int32_t)R(d.r2) * d.imm; WR(d.r2, (uint32_t)w); WR(d.r3, (uint32_t)(w >> 32)); cyc = 2; break;
    case OP_MULU_I9: { uint64_t u = (uint64_t)R(d.r2) * (uint32_t)d.imm; WR(d.r2, (uint32_t)u); WR(d.r3, (uint32_t)(u >> 32)); cyc = 2; break; }
    case OP_DIVH_3: case OP_DIV: {
        int32_t nd = (int32_t)R(d.r2); int32_t dv = (d.op == OP_DIVH_3) ? (int32_t)(int16_t)R(d.r1) : (int32_t)R(d.r1);
        c->sr[SR_PSW] &= ~PSW_OV; uint32_t q, m;
        if (dv == 0) { c->sr[SR_PSW] |= PSW_OV; q = (uint32_t)nd; m = 0; }
        else if (nd == (int32_t)0x80000000 && dv == -1) { c->sr[SR_PSW] |= PSW_OV; q = 0x80000000u; m = 0; }
        else { q = (uint32_t)(nd / dv); m = (uint32_t)(nd % dv); }
        WR(d.r2, q); WR(d.r3, m); set_flags_logic(c, q); cyc = 35; break;
    }
    case OP_DIVHU: case OP_DIVU: {
        uint32_t nd = R(d.r2); uint32_t dv = (d.op == OP_DIVHU) ? (R(d.r1) & 0xFFFF) : R(d.r1);
        c->sr[SR_PSW] &= ~PSW_OV; uint32_t q, m;
        if (dv == 0) { c->sr[SR_PSW] |= PSW_OV; q = nd; m = 0; } else { q = nd / dv; m = nd % dv; }
        WR(d.r2, q); WR(d.r3, m); set_flags_logic(c, q); cyc = 35; break;
    }
    case OP_BSW: { a = R(d.r2); res = __builtin_bswap32(a); set_flags_logic(c, res);
        int cy = !(a & 0xFF) || !(a & 0xFF00) || !(a & 0xFF0000) || !(a & 0xFF000000u); if (cy) c->sr[SR_PSW] |= PSW_CY; else c->sr[SR_PSW] &= ~PSW_CY; WR(d.r3, res); break; }
    case OP_BSH: { a = R(d.r2); res = ((a & 0xFF) << 8) | ((a >> 8) & 0xFF) | ((a & 0xFF0000) << 8) | ((a >> 8) & 0xFF0000);
        uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY); if (res & 0x80000000u) f |= PSW_S;
        if (!(a & 0xFF) && !(a & 0xFF00)) f |= PSW_Z; if (!(a & 0xFF) || !(a & 0xFF00)) f |= PSW_CY; c->sr[SR_PSW] = f; WR(d.r3, res); break; }
    case OP_HSW: { a = R(d.r2); res = (a << 16) | (a >> 16); set_flags_logic(c, res);
        if (!(a & 0xFFFF) || !(a >> 16)) c->sr[SR_PSW] |= PSW_CY; else c->sr[SR_PSW] &= ~PSW_CY; WR(d.r3, res); break; }
    case OP_HSH: { a = R(d.r2); res = a; uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY); if (res & 0x80000000u) f |= PSW_S;
        if (!(a & 0xFFFF)) f |= PSW_Z | PSW_CY; c->sr[SR_PSW] = f; WR(d.r3, res); break; }
    case OP_SCH0R: case OP_SCH1R: case OP_SCH0L: case OP_SCH1L: {
        a = R(d.r2); uint32_t v = (d.op == OP_SCH0R || d.op == OP_SCH0L) ? ~a : a; int i;
        uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY);
        if (v == 0) { res = 0; f |= PSW_Z; }
        else { if (d.op == OP_SCH0R || d.op == OP_SCH1R) { for (i = 0; !((v >> i) & 1); i++); } else { for (i = 0; !((v << i) & 0x80000000u); i++); }
               res = i + 1; f |= PSW_CY; }
        c->sr[SR_PSW] = f; WR(d.r3, res); break;
    }
    case OP_CMOV_I5: WR(d.r3, cond_true(c, d.cond) ? (uint32_t)d.imm : R(d.r2)); break;
    case OP_CMOV_RR: WR(d.r3, cond_true(c, d.cond) ? R(d.r1) : R(d.r2)); break;
    case OP_ADF: { uint32_t cf = cond_true(c, d.cond); a = R(d.r2); b = R(d.r1); uint64_t u = (uint64_t)a + b + cf; res = (uint32_t)u;
        uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY); if (!res) f |= PSW_Z; if (res & 0x80000000u) f |= PSW_S;
        if (u >> 32) f |= PSW_CY; int64_t sw = (int64_t)(int32_t)a + (int32_t)b + cf; if (sw != (int32_t)sw) f |= PSW_OV; c->sr[SR_PSW] = f; WR(d.r3, res); break; }
    case OP_SBF: { uint32_t cf = cond_true(c, d.cond); a = R(d.r2); b = R(d.r1); uint64_t u = (uint64_t)a - b - cf; res = (uint32_t)u;
        uint32_t f = c->sr[SR_PSW] & ~(PSW_Z|PSW_S|PSW_OV|PSW_CY); if (!res) f |= PSW_Z; if (res & 0x80000000u) f |= PSW_S;
        if (u >> 32) f |= PSW_CY; int64_t sw = (int64_t)(int32_t)a - (int32_t)b - cf; if (sw != (int32_t)sw) f |= PSW_OV; c->sr[SR_PSW] = f; WR(d.r3, res); break; }
    case OP_MAC: case OP_MACU: {
        int64_t acc = ((int64_t)(uint64_t)R(d.r3 + 1) << 32) | R(d.r3);
        int64_t p = (d.op == OP_MAC) ? (int64_t)(int32_t)R(d.r2) * (int32_t)R(d.r1) : (int64_t)((uint64_t)R(d.r2) * R(d.r1));
        uint64_t r = (uint64_t)(acc + p); c->r[d.bitno] = (uint32_t)r; c->r[d.bitno + 1] = (uint32_t)(r >> 32); break;
    }
    default:
        fprintf(stderr, "ILLEGAL/unimplemented instruction at %06x: %04x %04x\n", pc, bus_fetch16(c, pc), bus_fetch16(c, pc + 2));
        c->halted = 2; break;
    }
    c->r[0] = 0;
    c->pc = next;
    c->cycles += cyc;
    return cyc;
}
