/* FDIM system model: V850ES (ES/x2-class) peripherals + graphics controller */
#ifndef SYS_H
#define SYS_H
#include <stdint.h>
#include <stdio.h>
#include "v850.h"

#define FLASH_SIZE   0x60000
#define RAM_BASE     0x03FF9000u
#define RAM_SIZE     0x6000          /* 24 KB */
#define CPU_HZ       20000000ULL

/* interrupt vectors (firmware-derived) */
#define VEC_INTP0     0x090   /* graphics controller IRQ */
#define VEC_INTP4     0x0D0
#define VEC_INTP5     0x0E0
#define VEC_INTP6     0x0F0
#define VEC_TICK      0x1D0   /* TMP2 CC0 : RTOS 1 ms tick */
#define VEC_CSIB0     0x230
#define VEC_UNK290    0x290
#define VEC_UNK2A0    0x2A0
#define VEC_INTAD     0x2B0
#define VEC_WT        0x310
#define VEC_CAN_ERR   0x3D0
#define VEC_CAN_WUP   0x3E0
#define VEC_CAN_REC   0x3F0
#define VEC_CAN_TRX   0x400
#define VEC_EXT450    0x450
#define VEC_EXT460    0x460
#define VEC_EXT470    0x470
#define NVEC          0x5A0
#define IC_INDEX(v)   (((v) - 0x80) / 0x10)

typedef struct {
    uint8_t ctl0, ctl1, ioc0, ioc1, ioc2, opt0;
    uint16_t ccr[4];
    uint64_t start_cycle;      /* cycle when counter started/cleared */
    uint64_t next_match;       /* absolute cycle of next CC0 match */
    int vec_cc0, vec_cc1, vec_ov;
    const char *name;
} hwtimer_t;

typedef struct {
    uint16_t id; int ext; int rtr; int dlc; uint8_t data[8];
} can_frame_t;

typedef struct sys {
    cpu_t cpu;
    uint8_t flash[FLASH_SIZE];
    uint8_t ram[RAM_SIZE];
    /* INTC */
    uint8_t ic[NVEC / 0x10];         /* interrupt control registers, index = (vec-0x80)/0x10 */
    uint16_t ispr;
    /* ports (output latches) + mode regs, keyed by port register address low byte */
    uint8_t port[0x40];   /* 0xFFFFF400 .. 0xFFFFF43F : Pn (even) and PMn */
    uint8_t portc[0x40];  /* 0xFFFFF000 .. 0xFFFFF03F : PCS/PCT/PCM/PCD/PDL/PDH + PM/PMC */
    uint8_t pin_in[0x40]; /* external input levels for Pn (same indexing as port[]) */
    uint8_t pinc_in[0x40];
    /* timers */
    hwtimer_t tmp[4];       /* TMP0..3 at 0xFFFFF590.. */
    hwtimer_t tmq[3];       /* TMQ0 at 0x540, TMQ1 at 0x610, TMQ2 at 0x620 */
    uint8_t wtm; uint64_t wt_next;
    /* CSIB0 */
    uint8_t cb0ctl0, cb0ctl1, cb0ctl2, cb0str; uint16_t cb0tx, cb0rx; uint64_t cb0_done;
    /* ADC */
    uint8_t ada0m0, ada0m1, ada0s, ada0m2; uint16_t adc_val[16]; uint64_t adc_done; int adc_busy;
    /* CAN1 */
    uint16_t c_gmctrl, c_gmcs, c_mask[8], c_ctrl, c_lec, c_info, c_erc, c_ie, c_ints, c_brp, c_btr, c_ts;
    uint8_t  c_mb[32][0x20];
    uint8_t  rhl[32]; int rhl_head, rhl_count, rovf;   /* receive history list */
    uint8_t  thl[32]; int thl_head, thl_count, tovf;   /* transmit history list */
    /* misc SFR */
    uint8_t prsm0, prscm0, pllctl, pcc, psc, ckc;
    /* GDC */
    struct gdc *gdc;
    /* logging */
    FILE *log; int log_sfr; int log_can; int log_gdc; int log_unknown;
    uint64_t next_event;
    /* stats */
    uint64_t irq_count[NVEC / 0x10];
    can_frame_t tx_queue[64]; int tx_count;
} sys_t;

extern sys_t *S;

void sys_init(sys_t *s, const uint8_t *app, size_t applen);
void sys_run(sys_t *s, uint64_t cycles);
void sys_can_inject(sys_t *s, const can_frame_t *f);
void sys_raise(sys_t *s, int vec);     /* set interrupt request flag */
int  sys_pending_irq(sys_t *s);

/* GDC interface */
struct gdc *gdc_create(const char *image_rom, const char *dir_file);
void gdc_bus_write(struct gdc *g, int addr, uint8_t data);
uint8_t gdc_bus_read(struct gdc *g, int addr);
int  gdc_ready(struct gdc *g);
void gdc_reset(struct gdc *g);
void gdc_render(struct gdc *g, const char *png_path);
int  gdc_irq_pending(struct gdc *g);
void gdc_tick(struct gdc *g, uint64_t cycles);
void gdc_dump_objects(struct gdc *g, FILE *f);

#endif
