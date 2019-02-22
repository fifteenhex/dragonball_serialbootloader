.SUFFIXES:
BIN2C=../../../tools/bin2c
CFLAGS=-Wall -std=gnu99 -ggdb

all: bootloader

readbytes.c: readbytes.S
	m68k-uclinux-gcc -m68000 -c $<  -o readbytes.o	
	m68k-uclinux-objcopy -O binary readbytes.o readbytes.bin
	$(BIN2C) readbytes.bin readbytes instrbuffer_readbytes

bootloader: bootloader.c readbytes.c
	$(CC) $(CFLAGS) bootloader.c readbytes.c ./Musashi/m68kdasm.o -o $@
	
	
.PHONY: clean

clean:
	rm -f bootloader *.o *.bin
