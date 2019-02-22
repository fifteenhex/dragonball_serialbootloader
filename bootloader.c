/*
 * bootloader.c
 *
 *  Created on: 26 Feb 2014
 *      Author: daniel
 */

#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include <poll.h>
#include <assert.h>

#include "readbytes.h"
#include "../headers/bootloader.h"
#include "../headers/systemcontrol.h"
#include "../headers/gpio.h"
#include "../headers/dramcontroller.h"
#include "../headers/chipselect.h"
#include "../headers/ice.h"
#include "../headers/rtc.h"
#include "../headers/interruptcontroller.h"

#include "Musashi/m68k.h"

static int uartfd;

#define DATABRECORDLEN(payloadlen) (8 + 2 + (payloadlen * 2) + 1 +1)
#define BRECORDMAXPAYLOAD 0xff
#define BIGGESTBRECORD (DATABRECORDLEN(BRECORDMAXPAYLOAD))

#define FILTERPRINTABLE(c) ((c >= 0x20 && c <= 0x7F) ? c : ' ')

static void printblock(uint32_t offset, uint8_t* buffer, int len,
bool printheaders) {
	int width = 8;

	if (printheaders) {
		printf("        \t");
		for (int i = 0; i < width; i++) {
			printf("0x%02x\t", i);
		}
		printf("\n\n");
	}

	int prepad = offset % width;
	int padleft = prepad;

	for (int r = 0; r < len; r += width) {
		printf("0x%08"PRIx32"\t", offset + r);
		for (int i = 0; i < width; i++) {
			if (padleft > 0) {
				printf("     ");
				padleft--;
			} else {
				int b = (r + i) - prepad;
				if (b == len)
					break;
				printf("0x%02x[%c]\t", buffer[b], FILTERPRINTABLE(buffer[b]));
			}
		}
		printf("\n");
	}
}

/*
 * ********************************************************
 * VZ-ADS Init B-Record
 *
 ********************************************************
 FFFFF0000118  SCR init Disable Double Map
 FFFFFB0B0100  Disable WD
 FFFFF42B0103  enable clko
 FFFFF40B0100  enable chip select
 FFFFFD0D0108  disable hardmap
 FFFFFD0E0107  clear level 7 interrupt
 FFFFF4230100  set PE3 as *DWE

 FFFFF3000140        IVR
 FFFFF30404007FFFFF  IMR

 ***
 CSA
 ***
 FFFFF100020800 Group Base Add 16M
 FFFFF110020199 Chip Sel

 ************
 SDRAM Config
 ************
 FFFFF44301F1 PKSEL
 FFFFF44B0100 PMSEL

 CSD
 FFFFF106020000 Group Base Add
 FFFFF116020281 Chip Sel
 FFFFF10A020040 Chip Sel Control

 DRAM Contorller
 FFFFFC02020000 DRAMC
 FFFFFC0402C03F SDRAM Control
 FFFFFC00024020 DRAMMC
 FFFFFC02028000 DRAMC

 SDRAM Control
 FFFFFC0402C83F issue precharge command
 FFFFFC0402D03F enable refresh
 FFFFFC0402D43F issue mode command

 *********
 Init LCDC
 *********
 FFFFF4130100        Disable Port C
 FFFFFA00040000403E  LSSA=0x403E
 FFFFFA05010A        LVPW
 FFFFFA080200A0      LXMAX
 FFFFFA0A0200EF      LYMAX
 FFFFFA200108        LPICF
 FFFFFA210101        LPOLCF
 FFFFFA230100        LACD
 FFFFFA250102        LPXCD
 FFFFFA290114        LRRA
 FFFFFA2B0118        LOTCR
 FFFFFA2D0100        LPOSR
 FFFFFA270100        Disable LCD
 FFFFFA270182        Enable LCD
 *
 */

typedef enum {
	INIT
} state_t;

#define brecord_stepto38400 "FFFFF9020100"
#define brecord_stepto115200 "FFFFF9030138"

//static uint8_t writeuart[] = { 0x11, 0xF9, 0x00, 0x00, 0x00, 0x00, 0xF9, 0x07,
//		0x4E, 0xF8, 0xFF, 0x5A };

static uint8_t writeuart[] = { 0x3e, 0x3c, 0x00, 0x00, //
		0x4d, 0xf9, 0x00, 0x00, 0x00, 0x00, //
		0x11, 0xde, 0xf9, 0x07, //
		0x08, 0x38, 0x00, 0x02, 0xf9, 0x06, //
		0x66, 0xf8, //
		0x53, 0x47, //
		0x66, 0xf0, //
		0x4e, 0xf8, 0xff, 0x5a //
		};

//static uint8_t instructionbuffertest[] = { 0x49, 0xf8, 0xf4, 0x19 };

//static uint8_t instructionbuffertest[] = { 0x49, 0xf8, 0xf4, 0x19, //
//		0x1e, 0x3c, 0x00, 0x01, //
//		0x18, 0x87 };

//#define PROTOCOLDEBUG

static uint8_t writeandreadbackwithdifference(int uartfd, char* buff, int len,
		int readbackdifference, uint8_t* outputbuff, int inputlen,
		uint8_t* inputbuff) {

	// if we have an input buffer we need to save the last char for later
	int amounttowrite = (inputlen > 0) ? len - 1 : len;
	int wrote = write(uartfd, buff, amounttowrite);
	assert(wrote == amounttowrite);

#ifdef PROTOCOLDEBUG
	printf("wrote: %s\n", buff);
#endif

	char c[BIGGESTBRECORD + 1];

	int totalneeded = len + readbackdifference;

	assert(totalneeded < sizeof(c));

	int totalread = 0;
	int r = 0;

	struct pollfd uartpollfd;
	uartpollfd.fd = 0;
	uartpollfd.events = POLLIN;

	while (totalread < totalneeded) {
		poll(&uartpollfd, 1, 10);
		if ((r = read(uartfd, &(c[totalread]), totalneeded - totalread)) > 0) {
			totalread += r;
#ifdef PROTOCOLDEBUG
			c[totalread] = '\0';
			printf("read %d of %d\n", totalread, totalneeded);
			printblock(0x0, c, totalread, false);

#endif

			if ((totalread == (len - 1)) && (inputlen > 0)) {
				sleep(2);
				for (int i = 0; i < inputlen; i++) {
					write(uartfd, inputbuff + 1, 1);
					usleep(50000);
				}
				// write the last char of the command
				write(uartfd, &buff[len - 1], 1);
			}
		}
	}

	c[totalread] = '\0';

	if (readbackdifference > 0 && outputbuff != NULL) {
		memcpy(outputbuff, &(c[len - 1]), readbackdifference);
	}
#ifdef PROTOCOLDEBUG
	printf("readback: %s\n", c);
#endif
	return (uint8_t) c[totalread - 2];
}

static uint8_t writeandreadback(int uartfd, char* buff, int len) {
	return writeandreadbackwithdifference(uartfd, buff, len, 0,
	NULL, 0,
	NULL);
}

static int createbrecord(char* buff, uint32_t address, int count, uint8_t* data) {

	assert(count <= BRECORDMAXPAYLOAD);

	int pos = sprintf(buff, "%08X%02X", address, count);
	for (int i = 0; i < count;) {
		if ((count - i) > 4) {
			pos += sprintf(buff + pos, "%02X%02X%02X%02X", data[i], data[i + 1],
					data[i + 2], data[i + 3]);
			i += 4;
		} else {
			pos += sprintf(buff + pos, "%02X", data[i]);
			i++;
		}
	}
	pos += sprintf(buff + pos, "\n");
	return pos;
}

static int createbrecord_execute(char* buff, uint32_t address) {
	return createbrecord(buff, address, 0, NULL);
}

static int createbrecord_byte(char* buff, uint32_t address, uint8_t byte) {
	return createbrecord(buff, address, 1, &byte);
}

static int createbrecord_word(char* buff, uint32_t address, uint16_t word) {
	uint8_t data[2] = { (word >> 8) & 0xff, word & 0xff };
	return createbrecord(buff, address, 2, data);
}

static int createbrecord_double(char* buff, uint32_t address, uint32_t word) {
	uint8_t data[4] = { (word >> 24) & 0xff, (word >> 16) & 0xff, (word >> 8)
			& 0xff, word & 0xff };
	return createbrecord(buff, address, 4, data);
}

#define NOP 0x4E71

static void clearinstructionbuffer(int uartfd) {
	char buff[64];
	for (int i = 0; i < INSTRUCTIONBUFFERSZ; i += 2) {
		int len = createbrecord_word(buff, INSTRUCTIONBUFFER + i,
		NOP);
		writeandreadback(uartfd, buff, len);
	}
}

static void loadinstructionsintomemory(int uartfd, uint32_t loadaddress,
		uint8_t* buffer, int len) {
	char buff[256];
	if (loadaddress == INSTRUCTIONBUFFER) {
		if (len > INSTRUCTIONBUFFERSZ) {
			printf("instruction buffer is too long\n");
			return;
		}
		if (len != INSTRUCTIONBUFFERSZ)
			clearinstructionbuffer(uartfd);
	}
	int buflen = createbrecord(buff, loadaddress, len, buffer);
	writeandreadback(uartfd, buff, buflen);
}

static void loadinstructionbuffer(int uartfd, uint8_t* buffer, int len) {
	loadinstructionsintomemory(uartfd, INSTRUCTIONBUFFER, buffer, len);
}

static void runinstructionsinmemory(int uartfd, uint32_t loadaddress,
		int returnlen, uint8_t* outputbuff, int inputlen, uint8_t* inputbuff) {
	char buff[64];
	int len = createbrecord_execute(buff, loadaddress);
	writeandreadbackwithdifference(uartfd, buff, len, returnlen, outputbuff,
			inputlen, inputbuff);
}

static void runinstructionbuffer(int uartfd, int returnlen, uint8_t* outputbuff,
		int inputlen, uint8_t* inputbuff) {
	runinstructionsinmemory(uartfd, INSTRUCTIONBUFFER, returnlen, outputbuff,
			inputlen, inputbuff);
}

static void readmemoryblock(int uartfd, uint32_t address, int len,
		uint8_t* dest) {
	assert(len <= 0x7fff);
	writeuart[2] = (len >> 8) & 0x7f;
	writeuart[3] = len & 0xff;
	writeuart[6] = (address >> 24) & 0xff;
	writeuart[7] = (address >> 16) & 0xff;
	writeuart[8] = (address >> 8) & 0xff;
	writeuart[9] = address & 0xff;
	loadinstructionbuffer(uartfd, writeuart, sizeof(writeuart));
	runinstructionbuffer(uartfd, len, dest, -1, NULL);
}

static uint8_t readmemory(int uartfd, uint32_t address, int len, uint8_t* dest) {
	int remainder = len % 0x7fff;
	len -= remainder;
	for (int i = 0; i < len; i += 0x7fff) {
		readmemoryblock(uartfd, address, 0x7fff, dest);
		address += 0x7fff;
		dest += 0x7fff;
	}
	if (remainder > 0)
		readmemoryblock(uartfd, address, remainder, dest);
	return 0;
}

static void writememoryblock(int uartfd, uint32_t address, int len,
		uint8_t* src) {

	assert(len <= 0x7fff);
	uint32_t end = address + len;

	_binary_instrbuffer_readbytes_start[2] = (address >> 24) & 0xff;
	_binary_instrbuffer_readbytes_start[3] = (address >> 16) & 0xff;
	_binary_instrbuffer_readbytes_start[4] = (address >> 8) & 0xff;
	_binary_instrbuffer_readbytes_start[5] = address & 0xff;

	_binary_instrbuffer_readbytes_start[8] = (end >> 24) & 0xff;
	_binary_instrbuffer_readbytes_start[9] = (end >> 16) & 0xff;
	_binary_instrbuffer_readbytes_start[10] = (end >> 8) & 0xff;
	_binary_instrbuffer_readbytes_start[11] = end & 0xff;

	loadinstructionbuffer(uartfd, _binary_instrbuffer_readbytes_start,
			sizeof(_binary_instrbuffer_readbytes_start));
	runinstructionbuffer(uartfd, 0, NULL, len, src);
}

static uint8_t writememory(int uartfd, uint32_t address, int len, uint8_t* src) {
	int remainder = len % 0x7fff;
	len -= remainder;
	for (int i = 0; i < len; i += 0x7fff) {
		writememoryblock(uartfd, address, 0x7fff, src);
		address += 0x7fff;
		src += 0x7fff;
	}
	if (remainder > 0)
		writememoryblock(uartfd, address, remainder, src);
	return 0;
}

static void uartsetup(int uartfd, tcflag_t baud) {
	struct termios tio;
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = baud | CS8 | CLOCAL | CREAD;
	tio.c_iflag = IGNPAR;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tcflush(uartfd, TCIFLUSH);
	tcsetattr(uartfd, TCSANOW, &tio);
}

static void speedup(int uartfd) {
	write(uartfd, brecord_stepto38400, sizeof(brecord_stepto38400));
	sleep(1);
	uartsetup(uartfd, B38400);
	write(uartfd, "\n", 1);
	char c = 0;
	while (c != '\n')
		read(uartfd, &c, 1);

	write(uartfd, brecord_stepto115200, sizeof(brecord_stepto115200));
	sleep(1);
	uartsetup(uartfd, B115200);
	write(uartfd, "\n", 1);
	c = 0;
	while (c != '\n')
		read(uartfd, &c, 1);
}

static void runinit(int uartfd) {
	char buff[64];

	int len;

	printf("init\n");

	//len = createbrecord_byte(buff, SCR, SCR_BETEN | SCR_SO);
	len = createbrecord_byte(buff, SCR, 0);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_byte(buff, WATCHDOG + 1, 0);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_byte(buff, PFSEL, 0x03);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_byte(buff, PBSEL, 0x00);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_byte(buff, ICEMCR + 1, 0x08);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_byte(buff, ICEMSR, 0x07);
	writeandreadback(uartfd, buff, len);
	printf("%s\n", buff);

	len = createbrecord_byte(buff, PESEL, 0);
	writeandreadback(uartfd, buff, len);
	printf("%s\n", buff);

	len = createbrecord_byte(buff, IVR, 0x40);
	writeandreadback(uartfd, buff, len);
	printf("%s\n", buff);

	len = createbrecord_double(buff, IMR, 0x007FFFFF);
	writeandreadback(uartfd, buff, len);
	printf("%s\n", buff);

//FLASH

	len = createbrecord_word(buff, CSGBA, CHIPSELECTBASE(0x2000000));
	writeandreadback(uartfd, buff, len);
	printf("%x %s\n", CHIPSELECTBASE(0x2000000), buff);

	len = createbrecord_word(buff, CSA,
	CSA_FLASH | CSA_BSW | CSA_WS31_12 | CSA_SIZ_8MBYTE | CSA_EN);
	writeandreadback(uartfd, buff, len);

	//NVRAM

	len = createbrecord_word(buff, CSGBB,
			CHIPSELECTBASE(0x2000000 + (0x800000 * 2)));
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, CSB,
			CSB_BSW | CSB_WS31_12 | CSA_SIZ_1MBYTE | CSB_EN);
	writeandreadback(uartfd, buff, len);

	//SDRAM

	len = createbrecord_byte(buff, PKSEL, 0xf1);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_byte(buff, PMSEL, 0x00);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, CSGBD, 0x0000);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, CSD,
	CSD_DRAM | CSD_BSW | CSD_SIZ_64K16MBYTE | CSD_EN | CSD_COMB);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, CSCTRL1,
	CSCTRL1_DSIZ3 | CSCTRL1_AWSO);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, DRAMC, 0x0000);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, SDCTRL, 0xC03C);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, DRAMMC,
			DRAMMC_ROW12_PA10 | DRAMMC_ROW0_PA11 | DRAMMC_ROW11_PA22
					| DRAMMC_ROW10_PA21 | DRAMMC_ROW9_PA19 | DRAMMC_ROW8_PA20
					| DRAMMC_COL9_PA0 | DRAMMC_COL10_PA0);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, DRAMC, 0x8000);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, SDCTRL,
			0xC800 | SDCTRL_BNKADDH_PA24 | SDCTRL_BNKADDL_PA23);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, SDCTRL,
			0xD000 | SDCTRL_BNKADDH_PA24 | SDCTRL_BNKADDL_PA23);
	writeandreadback(uartfd, buff, len);

	len = createbrecord_word(buff, SDCTRL,
			0xD400 | SDCTRL_BNKADDH_PA24 | SDCTRL_BNKADDL_PA23);
	writeandreadback(uartfd, buff, len);

	// enable the TX pin on uart 2
	len = createbrecord_byte(buff, PJSEL, 0xCF);
	writeandreadback(uartfd, buff, len);

}

static void ledson(int uartfd) {
	char buff[32];
	int len = createbrecord_byte(buff, PDDATA, 0x03);
	writeandreadback(uartfd, buff, len);
}

static void ledsoff(int uartfd) {
	char buff[32];
	int len = createbrecord_byte(buff, PDDATA, 0);
	writeandreadback(uartfd, buff, len);

}

static void printhelp() {
	printf("md\t- memory dump:\t<start address> <len> [file]\n"
			"mm\t- memory modify:\t<start address> <value> <size> <count>\n"
			"ub\t- upload binary:\t<start address> <file>\n"
			"ue\t- upload elf\n"
			"fw\t- flash write:\t<src start> <dst start> <len>\n"
			"d\t- disassemble:\t<start address> <len>\n"
			"r\t- run, start executing from address, read input:\t <address>\n"
			"g\t- go, start executing from address and exit:\t<address>\n"
			"e\t- exit\n"
			"?\t- help\n");

}

static void cmd_memorymodify(char* command) {
	uint32_t address;
	uint32_t value;
	uint8_t width;
	uint32_t count = 0;
	char buff[256];
	if (sscanf(command + 2, " 0x%"SCNx32" 0x%"SCNx32" %"SCNu8" %"SCNu32,
			&address, &value, &width, &count) == 4) {
		printf("writing 0x%"PRIx32" %"PRIu32" times starting at 0x%"PRIx32"\n",
				value, count, address);
		if (width == 1 || width == 2 || width == 4) {
			int len;
			for (int i = 0; i < count; i++) {
				switch (width) {
				case 1:
					len = createbrecord_byte(buff, address, value & 0xff);
					break;
				case 2:
					len = createbrecord_word(buff, address, value & 0xffff);
					break;
				case 4:
					len = createbrecord_double(buff, address, value);
					break;
				}
				writeandreadback(uartfd, buff, len);
				address += width;
			}
		} else
			printf("bad width\n");
	}
}

static bool checkblock(uint32_t addr, uint32_t* values, uint32_t* readback,
		int len) {
	if (memcmp(values, readback, len) != 0) {
		printf("memcmp check failed, looking for failure point\n");
		for (int i = 0; i < len / sizeof(values[0]); i++) {
			if (readback[i] != values[i]) {
				printf("failed at %x, wanted %x got %x\n", (addr + i),
						values[i], readback[i]);
				return false;
			}
		}
	}
	return true;
}

static void cmd_memorytest(char* command) {

	const uint32_t startaddr = 0x0;
	const uint32_t end = startaddr + (32 * 1024 * 1024);
	printf("\n");
	uint32_t values[BRECORDMAXPAYLOAD / sizeof(uint32_t)];
	uint32_t readback[BRECORDMAXPAYLOAD / sizeof(uint32_t)];
	bool failed = false;
	int value = 0;
	char brecordbuff[DATABRECORDLEN(BRECORDMAXPAYLOAD)];
	for (uint32_t addr = startaddr; addr < end; addr += sizeof(values)) {
		for (int i = 0; i < (sizeof(values) / sizeof(values[0])); i++) {
			values[i] = value;
			value++;
		}
		printf("\33[2K\rwrite %x", addr);
		fflush(stdout);
		int len = createbrecord(brecordbuff, addr, sizeof(values),
				(uint8_t*) values);
		writeandreadback(uartfd, brecordbuff, len);
		readmemory(uartfd, addr, sizeof(readback), (uint8_t*) readback);
		if (!checkblock(addr, values, readback, sizeof(values)))
			break;
	}

//if (!failed) {
	value = startaddr;
	for (uint32_t addr = startaddr; addr < end; addr += sizeof(values)) {
		for (int i = 0; i < (sizeof(values) / sizeof(values[0])); i++) {
			values[i] = value;
			value++;
		}
		printf("\33[2K\rreadback %x", addr);
		fflush(stdout);
		readmemory(uartfd, addr, sizeof(readback), (uint8_t*) readback);
		if (!checkblock(addr, values, readback, sizeof(values)))
			break;
	}
//}
	printf("\n");
}

static void cmd_memorydump(char* command) {

	uint32_t address = 0;
	uint32_t len = 0;
	char filepath[256];
	uint8_t memblock[256];

	int args = sscanf(command + 2, " 0x%"SCNx32" %"SCNu32" %255[^\n]s",
			&address, &len, filepath);
	if (args == 2 || args == 3) {

		FILE* file = NULL;

		if (args == 2)
			printf("reading %d bytes starting at 0x%"PRIx32"\n", len, address);
		else {
			printf("reading %d bytes starting at 0x%"PRIx32" into file %s\n",
					len, address, filepath);
			file = fopen(filepath, "w");
			if (file == NULL) {
				printf("failed to open output file\n");
				return;
			}
		}

		int partial = len % sizeof(memblock);
		int loops;
		if (partial != 0)
			loops = (len + sizeof(memblock) - partial) / sizeof(memblock);
		else
			loops = len / sizeof(memblock);
		printf("loops %d\n", loops);
		for (int l = 0; l < loops; l++) {
			uint32_t offset = sizeof(memblock) * l;
			int readlen = sizeof(memblock);
			if (readlen + offset > len)
				readlen = len % sizeof(memblock);

			memset(memblock, 0xff, sizeof(memblock));

			readmemory(uartfd, address + offset, readlen, memblock);
			if (file != NULL)
				fwrite(memblock, 1, readlen, file);
			else
				printblock(address + offset, memblock, readlen, l == 0);
		}
		if (file != NULL)
			fclose(file);
	} else
		printf("bad input\n");
}

//#define USEFASTERUPLOAD

static void cmd_uploadbinary(char* command) {
	uint32_t address = 0;
	char brecordbuff[DATABRECORDLEN(BRECORDMAXPAYLOAD)];
	char file[256];
	if (sscanf(command + 2, " 0x%"SCNx32" %255[^\n]s", &address, file) == 2) {
		printf("loading \"%s\" to 0x%"PRIx32"\n", file, address);
		FILE* f = fopen(file, "r");
		int written = 0;
		if (f != NULL) {
			uint8_t buff[BRECORDMAXPAYLOAD];
			size_t read;
			printf("\n");
			while ((read = fread(buff, 1, sizeof(buff), f)) != 0) {
				if (read > 0) {
#ifdef USEFASTERUPLOAD
					writememory(uartfd, address + written, read, buff);
#else
					int len = createbrecord(brecordbuff, address + written,
							read, buff);
					writeandreadback(uartfd, brecordbuff, len);
#endif
					written += read;
					printf("\33[2K\r%d bytes", written);
					fflush(stdout);
					//break;
				}
			}
			printf("\n");
			fclose(f);
			printf("wrote %d bytes\n", written);
		} else
			printf("failed to open \"%s\"\n", file);
	}
}

unsigned int m68k_read_disassembler_8(unsigned int address) {
	uint8_t byte;
	readmemory(uartfd, address, 1, &byte);
	return byte;
}
unsigned int m68k_read_disassembler_16(unsigned int address) {
	uint8_t word[2];
	readmemory(uartfd, address, 2, word);
	return (word[0] << 8) | word[1];
}
unsigned int m68k_read_disassembler_32(unsigned int address) {
	uint8_t lon[4];
	readmemory(uartfd, address, 4, lon);
	return (lon[0] << 24) | (lon[1] << 16) | (lon[3] << 8) | lon[4];
}

static void cmd_disassemble(char* command) {
	uint32_t address = 0;
	uint32_t len = 0;
	if (sscanf(command + 2, " 0x%"SCNx32" %"SCNu32, &address, &len) == 2) {
		printf("disassembling code at 0x%"PRIx32"\n", address);
		uint32_t pc = address;
		char disbuff[256];
		while (pc < address + len) {
			int pcoffset = m68k_disassemble(disbuff, pc, M68K_CPU_TYPE_68000);
			printf("0%"PRIx32": %s\n", pc, disbuff);
			pc += pcoffset;
		}
	}
}

static void cmd_go(char* command, bool readinput) {
	uint32_t address = 0;
	if (sscanf(command + 2, " 0x%"SCNx32, &address) == 1) {
		printf("jumping to code at 0x%"PRIx32"\n", address);
		char brecordbuff[256];
		int len = createbrecord_execute(brecordbuff, address);
		writeandreadbackwithdifference(uartfd, brecordbuff, len, -1,
		NULL, -1,
		NULL);

		if (readinput) {
			printf("Reading input from board\n");
			struct termios tty, otty;
			tcgetattr(0, &otty);
			tty = otty;
			tty.c_lflag = tty.c_lflag & ~(ECHO | ECHOK | ICANON);
			tty.c_cc[VTIME] = 1;
			tcsetattr(0, TCSANOW, &tty);
			char c;
			struct pollfd fds[2];
			fds[0].fd = 0;
			fds[0].events = POLLIN;
			fds[1].fd = uartfd;
			fds[1].events = POLLIN;
			while (true) {
				if (poll(fds, 2, 0)) {
					if ((fds[0].revents & POLLIN)
							&& fread(&c, 1, 1, stdin) == 1)
						write(uartfd, &c, 1);
					if ((fds[1].revents & POLLIN) && read(uartfd, &c, 1) == 1)
						printf("%c", c);
				}
			}
			tcsetattr(0, TCSANOW, &otty);
		}
	}
}

static bool parsecmd(char* command) {
	printf("parsing command %s\n", command);
	bool ret = true;
	switch (command[0]) {
	case 'e':
		return false;
	case 'm':
		switch (command[1]) {
		case 'm':
			cmd_memorymodify(command);
			break;
		case 'd':
			cmd_memorydump(command);
			break;
		case 't':
			cmd_memorytest(command);
		}
		break;
	case 'u':
		switch (command[1]) {
		case 'b':
			cmd_uploadbinary(command);
			break;
		}
		break;

	case 'r':
		cmd_go(command, true);
		break;
	case 'g':
		cmd_go(command, false);
		ret = false;
		break;
	case '?':
		printhelp();
		break;
	case 'd':
		cmd_disassemble(command);
		break;
	default:
		printf("bad input\n");
		break;
	}
	return ret;
}

int main(int argc, char** argv) {

	char buff[64];

	int len;

	uartfd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (uartfd < 0) {
		printf("failed to open uart\n");
		return 1;
	}

	uartsetup(uartfd, B19200);

	printf("press reset button now!\n");
	sleep(15);
	printf("starting bootloader init\n");

	write(uartfd, ".", 1);
	char buffer[2];
	while (buffer[0] != '@') {
		memset(buffer, '\0', sizeof(buffer));
		read(uartfd, buffer, 1);
		//printf("got %s\n", buffer);
	}

	printf("got @ from bootloader\n");

	printf("increasing baud rate\n");
	speedup(uartfd);

	printf("flashing the leds a bit to confirm...\n");
	len = createbrecord_byte(buff, PDDIR, 0x03);
	writeandreadback(uartfd, buff, len);

	for (int i = 0; i < 4; i++) {
		ledsoff(uartfd);
		sleep(1);
		ledson(uartfd);
		sleep(1);
	}

	printf("running init brecords\n");
	ledsoff(uartfd);
	runinit(uartfd);
	ledson(uartfd);
	printf("init brecords done..\n");

//	printf("testing instruction buffer\n");
//	ledsoff(uartfd);
//	clearinstructionbuffer(uartfd);
//	loadinstructionbuffer(uartfd, instructionbuffertest,
//			sizeof(instructionbuffertest));
//	runinstructionbuffer(uartfd, false);
//	sleep(2);
//	ledson(uartfd);

	ledson(uartfd);
	printf("done\n");

	bool exit = false;
	char cmdbuff[256];
	while (!exit) {
		fputs(">", stdout);
		fgets(cmdbuff, sizeof(cmdbuff), stdin);
		exit = !parsecmd(cmdbuff);
	}

	close(uartfd);

	return 0;
}

