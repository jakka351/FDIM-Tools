/* FDIM system model: V850ES (ES/x2-class) peripherals + graphics controller */
#ifndef SYS_H
#define SYS_H
#include <stdint.h>
#include <stdio.h>
#include "v850.h"

#define FLASH_SIZE   0x60000
#define RAM_BASE     0x03FF9000u
#define RAM_SIZE     0x10000         /* buffer: up to 64 KB (per-board ram_base/ram_size select the window) */
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
#define NVEC          0x6C0
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
    int ch;                 /* CAN channel (0 = CAN0 / only channel, 1 = CAN1) */
} can_frame_t;

/* one aFCAN channel */
typedef struct {
    uint16_t gmctrl, gmcs, mask[8], ctrl, lec, info, erc, ie, ints, brp, btr, ts;
    uint8_t  mb[32][0x20];
    uint8_t  rhl[32]; int rhl_head, rhl_count, rovf;   /* receive history list */
    uint8_t  thl[32]; int thl_head, thl_count, tovf;   /* transmit history list */
    uint32_t base;          /* register base (0x03FEC000 CAN0, 0x03FEC600 CAN1) */
    int vec;                /* ERR vector; WUP/REC/TRX follow at +0x10/+0x20/+0x30 */
    int ch;
} can_t;

/* 16-bit interval timer block as found on the IPC (SJ3-class part): cmp16 at +4, ctl8 at +6 (bit7 = enable) */
typedef struct { uint16_t cmp; uint8_t ctl; uint64_t next; int vec; } itm_t;

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
    hwtimer_t tmp[8];       /* TMP0..3 at 0xFFFFF590.. (IPC: TMP0..6 at 0x590..0x5FF) */
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
    /* CAN: one channel on the FDIM/AIM boards, two on the IPC */
    can_t can[2]; int ncan;
    uint32_t can_base; int vec_can;      /* = can[0] (compat) */
    int wakeup_pending;
    /* RAM window */
    uint32_t ram_base, ram_size;
    /* IPC interval-timer bank at 0xFFFFF600 + 8k and the board's INTAD vector */
    itm_t itm[6]; int vec_intad;
    uint8_t sfr_echo[0x1000]; uint8_t sfr_echo_seen[0x1000], sfr_echo_wseen[0x1000];
    uint32_t sfr_wcount[0x1000], sfr_rcount[0x1000];   /* access histogram (board 2) */
    /* IIC (I2C) master channels with a 24Cxx EEPROM slave (IPC) */
    struct iic {
        uint8_t data, ctl, cl, x, status, flag;      /* IICn, IICCn, IICCLn, IICXn, IICSn, IICFn */
        int state;                                    /* 0 idle, 1 started (await address), 2 addressed-write, 3 addressed-read, 4 stopped */
        int slave, rw, nbytes, ack;                   /* current transaction */
        uint32_t eeaddr; int eeaddr_bytes, eeaddr_got;/* EEPROM pointer, address width (1 or 2), bytes of address received */
        uint64_t done;                                /* cycle when the byte transfer completes */
        int vec; const char *name;
        uint8_t txn[64]; int txn_len;                 /* transaction log */
    } iic[2];
    uint8_t i2c_eeprom[8192]; int i2c_eeprom_dirty, i2c_eeprom_addr_bytes, i2c_slave_addr;
    /* misc SFR */
    uint8_t prsm0, prscm0, pllctl, pcc, psc, ckc;
    /* board: 0 = 8R29-14D017-CB colour unit (GDC on ports 0-6, CAN1), 1 = -AC mono unit (S1D15721 on port DL, CAN0) */
    int board;
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
void sys_can_inject_ch(sys_t *s, int ch, const can_frame_t *f);
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
int  lcd_copy_frame(struct lcd *l, uint8_t *buf, int *out_w, int *out_h, int width, int xoff, int flip);
void lcd_dump_state(struct lcd *l, FILE *f);
void lcd_save_ram(struct lcd *l, const char *path);
uint64_t lcd_last_change(struct lcd *l);
int lcd_is_on(struct lcd *l);
void lcd_force_gray(struct lcd *l, int on);

#endif
