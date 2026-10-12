/* Decoder differential test: prints our disassembly for every address listed on stdin */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v850.h"

static uint8_t *img; static size_t imglen;
uint32_t bus_fetch16(cpu_t *c, uint32_t a){ (void)c; if (a + 1 < imglen) return img[a] | (img[a+1] << 8); return 0xFFFF; }
uint32_t bus_read(cpu_t *c, uint32_t a, int s){ (void)c; (void)a; (void)s; return 0; }
void bus_write(cpu_t *c, uint32_t a, uint32_t v, int s){ (void)c; (void)a; (void)v; (void)s; }
void sys_reti_hook(cpu_t *c, int a, int b){ (void)c; (void)a; (void)b; }

int main(int argc, char **argv)
{
    FILE *f = fopen(argv[1], "rb"); fseek(f, 0, SEEK_END); imglen = ftell(f); fseek(f, 0, SEEK_SET);
    img = malloc(imglen); fread(img, 1, imglen, f); fclose(f);
    cpu_t c; memset(&c, 0, sizeof c);
    char line[64], out[160];
    while (fgets(line, sizeof line, stdin)) {
        uint32_t a = strtoul(line, NULL, 16);
        int len = cpu_disasm(&c, a, out, sizeof out);
        printf("%06x %d %s\n", a, len, out);
    }
    return 0;
}
