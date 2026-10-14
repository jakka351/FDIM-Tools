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
#define VEC_CSIB1_R   0x250   /* CSIB1 transfer complete (AIM) */
#define VEC_CSIB1_T   0x260
#define VEC_UA1_R     0x290   /* UARTA1 receive (AIM) */
#define VEC_UA1_T     0x2A0   /* UARTA1 transmit complete (AIM) */
#define VEC_UNK290    0x290
#define VEC_UNK2A0    0x2A0
#define VEC_INTAD     0x2B0
#define VEC_WT        0x310
#define VEC_WTI       0x310   /* watch timer interval */
#define VEC_WTOV      0x320   /* watch timer overflow */
#define VEC_KR        0x300   /* key return (mono unit buttons) */
/* CAN vectors: CAN1 (colour unit) 0x3D0.., CAN0 (mono unit) 0x2C0.. ; selected per board in sys_init */
#define VEC_CAN_ERR   (S->vec_can)
#define VEC_CAN_WUP   (S->vec_can + 0x10)
#define VEC_CAN_REC   (S->vec_can + 0x20)
#define VEC_CAN_TRX   (S->vec_can + 0x30)
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
    uint8_t wtm; uint64_t wt_next, wtov_next, wtov_period;
    /* CSIB0 */
    uint8_t cb0ctl0, cb0ctl1, cb0ctl2, cb0str; uint16_t cb0tx, cb0rx; uint64_t cb0_done;
    /* CSIB1 (AIM: SPI to the audio switch/DSP) */
    uint8_t cb1ctl0, cb1ctl1, cb1ctl2, cb1str; uint16_t cb1tx, cb1rx; uint64_t cb1_done; int cb1_count;
    /* UARTA1 (AIM: iPod / accessory link) */
    uint8_t ua1ctl0, ua1ctl1, ua1ctl2, ua1opt0, ua1str, ua1rx, ua1tx; uint64_t ua1_tx_done; uint64_t ua1_byte_cycles;
    uint8_t ua1_rxq[4096]; int ua1_rxq_head, ua1_rxq_count; uint64_t ua1_rx_next; int ua1_tx_count, ua1_rx_count;
    uint8_t ua1_txline[256]; int ua1_txline_len; uint64_t ua1_txline_t0;
    /* ADC */
    uint8_t ada0m0, ada0m1, ada0s, ada0m2; uint16_t adc_val[16]; uint64_t adc_done; int adc_busy;
    /* CAN1 */
    uint16_t c_gmctrl, c_gmcs, c_mask[8], c_ctrl, c_lec, c_info, c_erc, c_ie, c_ints, c_brp, c_btr, c_ts;
    uint8_t  c_mb[32][0x20];
    uint8_t  rhl[32]; int rhl_head, rhl_count, rovf;   /* receive history list */
    uint8_t  thl[32]; int thl_head, thl_count, tovf;   /* transmit history list */
    int wakeup_pending;
    /* misc SFR */
    uint8_t prsm0, prscm0, pllctl, pcc, psc, ckc;
    /* board: 0 = 8R29-14D017-CB colour unit (GDC on ports 0-6, CAN1), 1 = -AC mono unit (S1D15721 on port DL, CAN0) */
    int board; uint32_t can_base; int vec_can;
    int module;            /* application header byte 0x4701 on the 256 KB boards: 1 = FDIM mono (8R29-14D017-AC), 2 = AIM/FEM (8R29-14F029-AB) */
    int ee_cs_port, ee_cs_bit;   /* EEPROM /CS: FDIM mono = P9H bit5, AIM = P0 bit4 */
    /* GDC (colour unit) */
    struct gdc *gdc;
    /* LCD (mono unit) */
    struct lcd *lcd;
    /* logging */
    FILE *log; int log_sfr; int log_can; int log_gdc; int log_unknown;
    uint64_t next_event;
    /* stats */
    uint64_t irq_count[NVEC / 0x10];
    can_frame_t tx_queue[64]; int tx_count;
    void (*tx_hook)(struct sys *s, const can_frame_t *f);     /* virtual bus: called for every transmitted frame */
    void (*uart_tx_hook)(struct sys *s, uint8_t b);           /* UARTA1 transmit byte hook (iPod simulator) */
    const char *name;                                          /* instance name for logs */
    /* SPI EEPROM state (per instance) */
    struct { int cs, sck, bitcnt, phase, cmd, addr, wel, outidx; uint8_t shin, shout, status; } ee;
    uint8_t eeprom[512]; int eeprom_dirty, eeprom_loaded, eeprom_verbose;
} sys_t;

extern sys_t *S;

void sys_init(sys_t *s, const uint8_t *app, size_t applen);
void sys_run(sys_t *s, uint64_t cycles);
void sys_can_inject(sys_t *s, const can_frame_t *f);
void sys_uart1_inject(sys_t *s, const uint8_t *data, int n);
void ipodsim_init(FILE *log); void ipodsim_rx(sys_t *s, uint8_t b); void ipodsim_tick(sys_t *s);
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

/* LCD interface (mono unit) */
struct lcd *lcd_create(void);
void lcd_set_log(struct lcd *l, FILE *f, int verbose);
void lcd_reset(struct lcd *l, int asserted);
void lcd_write(struct lcd *l, int a0, uint8_t d);
uint8_t lcd_read(struct lcd *l, int a0);
void lcd_render(struct lcd *l, const char *path, int width, int xoff, int flip);
void lcd_dump_state(struct lcd *l, FILE *f);
void lcd_save_ram(struct lcd *l, const char *path);
uint64_t lcd_last_change(struct lcd *l);
int lcd_is_on(struct lcd *l);

#endif
