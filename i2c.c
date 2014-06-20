#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define CLKCTRL_FRQ_1024 0x6
#define CLKCTRL_SRC_PLL  0x8
#define CLKCTRL_DISABLE  0x0

static int verbose = 0;

static int pll_read(int fd) {
	int ret = 0;
	uint8_t buf[8];
	
	ret = ioctl(fd, I2C_SLAVE, 0x38);
	if (ret < 0) { printf("Wrong Address?\n"); close(fd); return ret; }

	memset(buf, 0, 8);
	buf[0] = 0x40;
	buf[1] = 0x02;
	
	ret = read(fd, buf, 8);
	if (ret < 0) { printf("Read failed, errno = %d\n", errno); }
	else {
		if (verbose) {
				printf("Read ret: %d\n", ret);
				printf("PLL: (%02x%02x) %02x%02x %02x%02x %02x%02x\n",
						 buf[0], buf[1], buf[2], buf[3], 
						 buf[4], buf[5], buf[6], buf[7]);
		}
	}

	return ret;
}

static int write_command(int fd, uint8_t *cmd, int len, const char *name) {
	int ret;

	ret = ioctl(fd, I2C_SLAVE, 0x38);
	if (ret < 0) { printf("Wrong Address?\n"); close(fd); return ret; }

	ret = write(fd, cmd, len);
	if (ret < len) { printf("Can't write: %d\n", errno); close(fd); return ret; }

	if (verbose) printf("Command: \"%s\" Written successfully\n", name);
	return 0;
}

static void construct_cmd(uint8_t msb, uint8_t cmd, uint8_t *buf) {
	buf[0] = 0x40;
	buf[1] = msb;
	buf[2] = cmd;	
}

int main(int argc, char **argv) {
	int fd, ret;
	uint8_t *buf;
	uint8_t mx0, mx1;	
	int val, i;

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) verbose = 1;
	}	
	
	fd = open("/dev/i2c-0", O_RDWR);
	if (fd < 0) return 1;

	buf = malloc(sizeof(uint8_t) * 8);
	if (!buf) { printf("Malloc failed\n"); return -1; }

	/* Set up PLL by first turning it off */
	construct_cmd(0x00, 0x0E, buf);
	write_command(fd, buf, 3, "SET UP PLL");

	usleep(10000);

#if 1

	/* Write configuration values */
	uint8_t pllbuf[] = { 0x40, 0x02, 0x02, 0x71, 0x01, 0xDD, 0x19, 0x01 };
	if (argc > 1 && !strcmp(argv[1], "48khz")) {
		pllbuf[2] = 0x00; pllbuf[3] = 0x7D;
		pllbuf[4] = 0x00; pllbuf[5] = 0x0C;
		pllbuf[6] = 0x21; pllbuf[7] = 0x01;
	}
	write_command(fd, pllbuf, 8, "WRITE PLL CMD");

	/* wait */
	usleep(65000);

	/* Enable the core */
	construct_cmd(0x00, 0x0F, buf);
	write_command(fd, buf, 3, "ENABLE CORE");

	/* wait */
	usleep(10000);
	
	/* Read out of curiosity */
	//pll_read(fd);

	usleep(10000);
	
#if 0
	usleep(10000);
	construct_cmd(0x2F, 0xA0, buf);
	write_command(fd, buf, 3, "CONTROL PAD CONTROL 0");

	usleep(10000);
	construct_cmd(0x30, 0x01, buf);
	write_command(fd, buf, 3, "CONTROL PAD CONTROL 1");
#endif

	usleep (10000);
	/* Become I2S Master */
	construct_cmd(0x15, 0x01, buf);
	write_command(fd, buf, 3, "BECOME I2S MASTER");

	usleep(10000);
	
	/* Setting input mixers */
	/* Enable left mixer */
	construct_cmd(0x0A, 0x01, buf);
	write_command(fd, buf, 3, "ENABLE RECORD MIXER LEFT");
	
	usleep(10000);
	
	/* Digital volume (AD) Attenuator (Negative dB) */
	construct_cmd(0x1A, 0x00, buf);
	write_command(fd, buf, 3, "DIGITAL VOLUME (LADVOL)");
	
	usleep(10000);

	if (argc > 2 && !strcmp(argv[1], "davol")) {
		int vol = atoi(argv[2]);
		if (vol <= 0) {
			val = -1 * vol;
		}
	} else {
		val = 0x50;
	}

	/* Digital volume (DA) Attenuator (Negative dB) */
	construct_cmd(0x2B, val, buf);
	write_command(fd, buf, 3, "DIGITAL VOLUME (LDAVOL)");
	
	usleep(10000);
	
#define PGA
#ifdef PGA
	/*	LDBOOST = 0dB */
	if (argc > 1 && !strcmp(argv[1], "0dB")) {
		val = 0x08;
	} else {
		val = 0x10;
	}
	construct_cmd(0x0B, val, buf);
	write_command(fd, buf, 3, "MIXER LEFT LDBOOST 0DB");

	usleep(10000);

	/* -----> LDVOL ----> LDBOOST ---> Mixer 3 |-> ADC 		  */
   /*			                                  |-> to playback */
	/* LDVOL Control (Differential Control) */
	/* LDVOL[5:0] | LDMUTE | LDEN */
	val = (0x3F << 2) | (1 << 1) | 1;
	construct_cmd(0x0E, val, buf);
	write_command(fd, buf, 3, "LEFT DIFF INPUT VOL CONTROL");

	usleep(10000);

//#define ALC
#ifdef ALC
	/* PGASLEW[1:0] | ALCMAX[2:0] | ALCSEL[2:0] */
	/* ALCSEL : 000 => off, 001 => Right, 010 => Left, 011 => Stereo */
	val = (0 << 6) | (2 << 3) | (3 << 0);
	construct_cmd(0x11, val, buf);
	write_command(fd, buf, 3, "LEFT DIFF INPUT ALC");

	usleep(10000);

	/* NGTYP[1:0] | NGEN | NGTHR[4:0] */
	val = (3 << 6) | (1 << 5) | (0x08); 
	construct_cmd(0x14, val, buf);
	write_command(fd, buf, 3, "ALC NOISE GATE");

	usleep(10000);
#endif
#endif

	/* Setting up output mixers */

	/* LEFT INPUT -->|
		RIGHT INPUT ->|
		LAUX -------->|-> Left Mixer->|
		LEFT DAC ---->|               |--> MX5G3 --> L/R Mixer --> LOUTVOL |---> LOUTP
		RIGHT DAC --->|               |                                    |---> ~LOUTN
   */

	if (argc > 1 && !strcmp(argv[1], "analog")) {
		mx0 = 0x01; mx1 = 0x06;
	} else if (argc > 2 && !strcmp(argv[1], "both")) {
		mx0 = 0x61; mx1 = 0x00;
	} else {
		mx0 = 0x21; mx1 = 0x00;
	}

	/* Mixer 3 Control 0 */
	/* 0 | MX3RM | MX3LM | MX3AUG[3:0] | MX3EN */
	construct_cmd(0x1C, mx0, buf);
	write_command(fd, buf, 3, "LEFT MIXER CONTROL 0");
	
	usleep(10000);

	/* Mixer 3 Control 1 */
	/* RIGHTINPUT [3:0] | LEFTINPUT [3:0] */
	construct_cmd(0x1D, mx1, buf);
	write_command(fd, buf, 3, "LEFT MIXER CONTROL 1");
	
	usleep(10000);


	/* Mixer 4 Control 0 */
	construct_cmd(0x1E, mx0, buf);
	write_command(fd, buf, 3, "RIGHT MIXER CONTROL 0");

	usleep(10000);

	/* Mixer 4 Control 1 */
	construct_cmd(0x1F, mx1, buf);
	write_command(fd, buf, 3, "RIGHT MIXER CONTROL 0");

	usleep(10000);

	/* Playback L/R Mixer Left */
	/* 0[2:0] | MX5G4[1:0] | MX5G3[1:0] | MX5EN */
	val = (2 << 3) | (2 << 1) | 1; 
	construct_cmd(0x20, val, buf);
	write_command(fd, buf, 3, "PLAYBACK MX5G3 ENABLE");
	
	usleep(10000);
#if 0
	/* Left Headphone Volume */
	construct_cmd(0x23, 0xFE, buf);
	write_command(fd, buf, 3, "ENABLE HP VOLUME LEFT");

	usleep(10000);
	/* Right Headphone Volume */
	construct_cmd(0x24, 0xFE, buf);
	write_command(fd, buf, 3, "ENABLE HP VOLUME RIGHT");

	usleep(10000);
#endif
	/* Left Line Out Volume */
	/* this make difference */
	construct_cmd(0x25, 0xAE, buf);
	write_command(fd, buf, 3, "ENABLE LINEOUT VOLUME LEFT");
	
	usleep(10000);
	/* Set Up ADC */
	construct_cmd(0x19, 0x01, buf);
	write_command(fd, buf, 3, "ADC ENABLE (BOTH)");

	usleep(10000);
	/* Enable DAC */
	construct_cmd(0x2a, 0x01, buf);
	write_command(fd, buf, 3, "dac enable (both)");

	usleep(10000);

#if 1
	/* Converter */
	/* 0 | DAPAIR[1:0] | DAOSR | ADOSR | CONVSR[2:0] */
	val =  (0 << 5) | (0 << 4) | (0 << 3) | 0;
	construct_cmd(0x17, val, buf);
	write_command(fd, buf, 3, "Converter");

	usleep(10000);

	/* Serial Control 1 */
	/* BPF[2:0] | ADTDM | DATDM | MSBP | LRDEL[1:0] */
	//val = 0;
	val = (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | 0;
	construct_cmd(0x16, val, buf);
	write_command(fd, buf, 3, "Serial Control 1");

	usleep(10000);
#endif

	/* Set up record management */
	construct_cmd(0x09, 0x2A << 1, buf);
	write_command(fd, buf, 3, "Record Power MGMT");

	usleep(10000);

#if 0
	/* Pop/click suppress */
	construct_cmd(0x28, 0x04, buf);
	write_command(fd, buf, 3, "Pop/Click suppress");

	usleep(10000);
#endif

	/* Set up playback power management */
	construct_cmd(0x29, 0x03, buf);
	write_command(fd, buf, 3, "PLAYBACK POWER ON");
	
#if 0
	usleep(10000);
	construct_cmd(0x2D, 0xBA, buf);
	write_command(fd, buf, 3, "DAC_DATA PULL DOWN");

	usleep(10000);
	/* Setup signal routing */
	/* Serial input */
	construct_cmd(0xF2, 0x01, buf);
	write_command(fd, buf, 3, "SERIAL INPUT ROUTE");
	
	usleep(10000);
	/* Serial output */
	construct_cmd(0xF3, 0x01, buf);
	write_command(fd, buf, 3, "SERIAL OUTPUT ROUTE");
#endif
#endif
	printf("Set OK.\n");
	
	close(fd);
	return 0;
}
