CFLAGS=-O2 -Wall -Wno-misleading-indentation -Wno-unused-variable -Wno-unused-function
fdim-emu: main.c sys.c gdc.c lcd.c ipodsim.c v850.c sys.h v850.h
	gcc $(CFLAGS) -o fdim-emu main.c sys.c gdc.c lcd.c ipodsim.c v850.c
fdim-mirror: fdim_mirror.c sys.c gdc.c lcd.c ipodsim.c v850.c sys.h v850.h
	gcc $(CFLAGS) -o fdim-mirror fdim_mirror.c sys.c gdc.c lcd.c ipodsim.c v850.c
distest: distest.c v850.c
	gcc $(CFLAGS) -o distest distest.c v850.c
clean:
	rm -f fdim-emu distest
