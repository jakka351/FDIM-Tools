/* V850ES CPU core (V850E1 integer ISA) — FDIM emulator */
#ifndef V850_H
#define V850_H
#include <stdint.h>
#include <stddef.h>

#define PSW_Z   (1u<<0)
#define PSW_S   (1u<<1)
#define PSW_OV  (1u<<2)
#define PSW_CY  (1u<<3)
#define PSW_SAT (1u<<4)
#define PSW_ID  (1u<<5)
#define PSW_EP  (1u<<6)
#define PSW_NP  (1u<<7)

enum { SR_EIPC=0, SR_EIPSW=1, SR_FEPC=2, SR_FEPSW=3, SR_ECR=4, SR_PSW=5, SR_CTPC=16, SR_CTPSW=17, SR_DBPC=18, SR_DBPSW=19, SR_CTBP=20 };

typedef struct cpu {
    uint32_t r[32];
    uint32_t pc;
    uint32_t sr[32];          /* system registers, indexed by regID */
    uint64_t cycles;          /* approximate cycle counter (1/instruction + mem penalties) */
    uint64_t insns;
    int      halted;
    int      trace;
    /* exception bookkeeping */
    uint32_t last_pc;
} cpu_t;

/* bus interface implemented by the system model */
uint32_t bus_read(cpu_t *c, uint32_t addr, int size);          /* size 1,2,4 ; returns zero-extended */
void     bus_write(cpu_t *c, uint32_t addr, uint32_t val, int size);
uint32_t bus_fetch16(cpu_t *c, uint32_t addr);

void cpu_reset(cpu_t *c, uint32_t pc);
int  cpu_step(cpu_t *c);                 /* executes one instruction, returns cycles */
void cpu_interrupt(cpu_t *c, uint32_t vector_offset, uint32_t exception_code);  /* maskable interrupt entry */
int  cpu_disasm(cpu_t *c, uint32_t pc, char *out, size_t outlen);  /* returns length in bytes */
const char *cpu_regname(int r);

#endif
