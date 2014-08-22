#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define CODEC3_I2C_CS 158
#define CODEC2_I2C_CS 157
#define CODEC1_I2C_CS 156
#define I2C_CLEAR 999

#define GPIO_PATH	 "/sys/class/gpio"

#define I2C_ADDRESS 0x38

static int verbose = 0;

int master = 0;

static int write_command(int fd, uint8_t *cmd, int len, const char *name) 
{
	int ret;

	ret = ioctl(fd, I2C_SLAVE, I2C_ADDRESS);
	if (ret < 0) { printf("Wrong Address? %x\n", I2C_ADDRESS); close(fd); return ret; }

	ret = write(fd, cmd, len);
	if (ret < len) { printf("Can't write: %s\n", strerror(errno)); close(fd); return ret; }

	usleep(10000);

	if (verbose) printf("Command: \"%s\" Written successfully\n", name);
	return 0;
}

static void construct_cmd(uint8_t msb, uint8_t cmd, uint8_t *buf) 
{
	buf[0] = 0x40;
	buf[1] = msb;
	buf[2] = cmd;	
}

static void gpio_init(void) {
	char path[60];
	FILE* fd;
	int i;

	sprintf(path, "%s/export", GPIO_PATH);
	fd = fopen(path, "w");
	if (!fd) {
		printf("Can't open export file\n");
		return;
	}

	rewind(fd);
	fprintf(fd, "%d\n", CODEC3_I2C_CS);
	rewind(fd);
	fprintf(fd, "%d\n", CODEC2_I2C_CS);
	rewind(fd);
	fprintf(fd, "%d\n", CODEC1_I2C_CS);

	fclose(fd);

	for (i = CODEC3_I2C_CS; i >= CODEC1_I2C_CS; i--) {
		sprintf(path, "%s/gpio%d/direction", GPIO_PATH, i);
		
		printf("opening %s..\n", path);
		fd = fopen(path, "w");
		if (!fd) {
			printf("Can't open direction file for gpio%d\n", i);
			return;
		}

		rewind(fd);
		fprintf(fd, "out");
		fclose(fd);
	}

}

static void select_gpio(int codec, int select) {
	char path[50];
	FILE * fd;

	sprintf(path, "%s/gpio%d/value", GPIO_PATH, codec);

	if (access(path, F_OK) < 0) gpio_init();

	fd = fopen(path, "w");
	if (!fd) {
		printf("Failed to initialize gpio\n");			
		return;
	}

	printf("writing %d to %s\n", select, path);
	rewind(fd);
	fprintf(fd, "%d\n", select);

	fclose(fd);
}

static void gpio_mux(int codec) {
	switch(codec) {
		case CODEC1_I2C_CS:
			select_gpio(CODEC1_I2C_CS, 0);
			select_gpio(CODEC2_I2C_CS, 1);
			select_gpio(CODEC3_I2C_CS, 1);
			break;
		case CODEC2_I2C_CS:
			select_gpio(CODEC1_I2C_CS, 1);
			select_gpio(CODEC2_I2C_CS, 0);
			select_gpio(CODEC3_I2C_CS, 1);
			break;
		case CODEC3_I2C_CS:
			select_gpio(CODEC1_I2C_CS, 1);
			select_gpio(CODEC2_I2C_CS, 1);
			select_gpio(CODEC3_I2C_CS, 0);
			break;
		case I2C_CLEAR:
			select_gpio(CODEC1_I2C_CS, 0);
			select_gpio(CODEC2_I2C_CS, 0);
			select_gpio(CODEC3_I2C_CS, 0);
			break;
		default:
			break;
	}
}

static void i2s_disable(int mnum) {
	int fd;
	unsigned long base;
	unsigned long *addr;

	if (mnum = 1) {
		base = 0xff240000;
	} else {
		base = 0xff250000;
	}

	fd = open("/dev/mem", O_RDWR);

	if (!fd) {
		printf("Can't open /dev/mem\n");
		return;
	}
	
	addr = mmap(NULL, getpagesize(), PROT_WRITE | PROT_READ, MAP_SHARED,
							fd, base);	
	
	if (addr == MAP_FAILED) {
		printf("Failed to map address\n");
		return;
	}

	// disable
	addr[4] = 0x0;
}

int main(int argc, char **argv) {
	int fd, ret, m1cfgd, m2cfgd;
	unsigned long *m1cfg, *m2cfg;
	uint8_t *buf;
	uint8_t mx0, mx1;	
	int val, i;

	fd = open("/dev/i2c-0", O_RDWR);
	if (fd < 0) return 1;

	buf = malloc(sizeof(uint8_t) * 8);
	if (!buf) { printf("Malloc failed\n"); return -1; }


	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) verbose = 1;
		if (!strcmp(argv[i], "-master")) master = 1;
		if (!strcmp(argv[i], "-3")) {
			gpio_mux(CODEC3_I2C_CS);
			i2s_disable(2);
		}
		if (!strcmp(argv[i], "-2")) {
			gpio_mux(CODEC2_I2C_CS);
		}
		if (!strcmp(argv[i], "-1")) {
			gpio_mux(CODEC1_I2C_CS);
			i2s_disable(1);
		}
		if (!strcmp(argv[i], "-turnoff")) {
			construct_cmd(0x00, 0x00, buf);
			write_command(fd, buf, 3, "TURN OFF PLL");
		}
	}	

	/* Set up PLL by first turning it off */
	construct_cmd(0x00, 0x0E, buf);
	if (write_command(fd, buf, 3, "TURN OFF PLL")) goto exit;

	usleep(65000);

	/* Write configuration values */
	uint8_t pllbuf[] = { 0x40, 0x02, 0x02, 0x71, 0x01, 0xDD, 0x19, 0x01 };
	if (argc > 1 && !strcmp(argv[1], "48khz")) {
		pllbuf[2] = 0x00; pllbuf[3] = 0x7D;
		pllbuf[4] = 0x00; pllbuf[5] = 0x0C;
		pllbuf[6] = 0x21; pllbuf[7] = 0x01;
	}
	if (write_command(fd, pllbuf, 8, "WRITE PLL CMD")) goto exit;

	/* wait */
	usleep(65000);

	/* Enable the core */
	construct_cmd(0x00, 0x0F, buf);
	if (write_command(fd, buf, 3, "ENABLE CORE")) goto exit;

	construct_cmd(0x2F, 0xA0, buf);
	if (write_command(fd, buf, 3, "CONTROL PAD CONTROL 0")) goto exit;

	construct_cmd(0x30, 0x01, buf);
	if (write_command(fd, buf, 3, "CONTROL PAD CONTROL 1")) goto exit;

	/* Became I2S master */

	if (master)
		construct_cmd(0x15, 0x01, buf);
	else 
		construct_cmd(0x15, 0x00, buf);
	if (write_command(fd, buf, 3, "BECOME I2S SLAVE")) goto exit;

	/* Setting input mixers */
	/* Enable left mixer */
	construct_cmd(0x0A, 0x01, buf);
	if (write_command(fd, buf, 3, "ENABLE RECORD MIXER LEFT")) goto exit;
	
	/* Digital volume (AD) Attenuator (Negative dB) */
	construct_cmd(0x1A, 0x16, buf);
	if (write_command(fd, buf, 3, "DIGITAL VOLUME (LADVOL)")) goto exit;
	
	val = 10;
	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "davol")) {
			int vol = atoi(argv[i+1]);
			printf("davol %d\n", vol);
			if (vol <= 0) val = -1 * vol;
		}
	}

	/* Digital volume (DA) Attenuator (Negative dB) */
	construct_cmd(0x2B, val, buf);
	if (write_command(fd, buf, 3, "DIGITAL VOLUME (LDAVOL)")) goto exit;
	
#define PGA
#ifdef PGA
	/*	LDBOOST = 0dB */
	if (argc > 1 && !strcmp(argv[1], "0dB")) {
		val = 0x08;
	} else {
		val = 0x10;
	}
	construct_cmd(0x0B, val, buf);
	if (write_command(fd, buf, 3, "MIXER LEFT LDBOOST 0DB")) goto exit;

	/* -----> LDVOL ----> LDBOOST ---> Mixer 3 |-> ADC 		  */
   /*			                                  |-> to playback */
	/* LDVOL Control (Differential Control) */
	/* LDVOL[5:0] | LDMUTE | LDEN */
	val = (0x20 << 2) | (1 << 1) | 1;
	construct_cmd(0x0E, val, buf);
	if (write_command(fd, buf, 3, "LEFT DIFF INPUT VOL CONTROL")) goto exit;

#define ALC
#ifdef ALC
	/* PGASLEW[1:0] | ALCMAX[2:0] | ALCSEL[2:0] */
	/* ALCSEL : 000 => off, 001 => Right, 010 => Left, 011 => Stereo */
	val = (0 << 6) | (2 << 3) | (0 << 0);
	construct_cmd(0x11, val, buf);
	if (write_command(fd, buf, 3, "LEFT DIFF INPUT ALC")) goto exit;

	/* NGTYP[1:0] | NGEN | NGTHR[4:0] */
	val = (3 << 6) | (0 << 5) | (0x00); 
	construct_cmd(0x14, val, buf);
	if (write_command(fd, buf, 3, "ALC NOISE GATE")) goto exit;
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
	if (write_command(fd, buf, 3, "LEFT MIXER CONTROL 0")) goto exit;
	
	/* Mixer 3 Control 1 */
	/* RIGHTINPUT [3:0] | LEFTINPUT [3:0] */
	construct_cmd(0x1D, mx1, buf);
	if (write_command(fd, buf, 3, "LEFT MIXER CONTROL 1")) goto exit;
	

	/* Mixer 4 Control 0 */
	construct_cmd(0x1E, mx0, buf);
	if (write_command(fd, buf, 3, "RIGHT MIXER CONTROL 0")) goto exit;

	/* Mixer 4 Control 1 */
	construct_cmd(0x1F, mx1, buf);
	if (write_command(fd, buf, 3, "RIGHT MIXER CONTROL 0")) goto exit;

	/* Playback L/R Mixer Left */
	/* 0[2:0] | MX5G4[1:0] | MX5G3[1:0] | MX5EN */
	val = (2 << 3) | (2 << 1) | 1; 
	construct_cmd(0x20, val, buf);
	if (write_command(fd, buf, 3, "PLAYBACK MX5G3 ENABLE")) goto exit;
	
	/* Left Line Out Volume */
	/* this make difference */
	construct_cmd(0x25, 0xFE, buf);
	if (write_command(fd, buf, 3, "ENABLE LINEOUT VOLUME LEFT")) goto exit;
	
	/* Set Up ADC */
	/* Invert polarity (it comes from LINN) */
	/* HPF enabled */
	construct_cmd(0x19, 0x41, buf);
	if (write_command(fd, buf, 3, "ADC ENABLE (BOTH)")) goto exit;

	/* Enable DAC */
	construct_cmd(0x2a, 0x01, buf);
	if (write_command(fd, buf, 3, "dac enable (both)")) goto exit;

	/* Converter */
	/* 0 | DAPAIR[1:0] | DAOSR | ADOSR | CONVSR[2:0] */
	val =  (0 << 5) | (0 << 4) | (0 << 3) | 0;
	construct_cmd(0x17, val, buf);
	if (write_command(fd, buf, 3, "Converter")) goto exit;

	/* Serial Control 1 */
	/* BPF[2:0] | ADTDM | DATDM | MSBP | LRDEL[1:0] */
	val = (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | 0;
	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-msb")) {
			val = (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | 0;
		}
	}
	construct_cmd(0x16, val, buf);
	if (write_command(fd, buf, 3, "Serial Control 1")) goto exit;


	/* Set up record management */
	construct_cmd(0x09, 0x2A << 1, buf);
	if (write_command(fd, buf, 3, "Record Power MGMT")) goto exit;

#if 0
	/* Pop/click suppress */
	construct_cmd(0x28, 0x04, buf);
	if (write_command(fd, buf, 3, "Pop/Click suppress")) goto exit;

#endif

	/* Set up playback power management */
	construct_cmd(0x29, 0x03, buf);
	if (write_command(fd, buf, 3, "PLAYBACK POWER ON")) goto exit;
	

	if (!master) {
		construct_cmd(0x2D, 0xA8, buf);
		if (write_command(fd, buf, 3, "BCLK PULL-UP ON")) goto exit;

		/* Dejitter control */
		construct_cmd(0x36, 0x05, buf);
		if (write_command(fd, buf, 3, "DEJITTER CONTROL")) goto exit;
	}


#if 0

	/* Setup signal routing */
	/* Serial input */
	construct_cmd(0xF2, 0x01, buf);
	if (write_command(fd, buf, 3, "SERIAL INPUT ROUTE")) goto exit;
	
	/* Serial output */
	construct_cmd(0xF3, 0x01, buf);
	if (write_command(fd, buf, 3, "SERIAL OUTPUT ROUTE")) goto exit;
#endif
#endif
	printf("Set OK.\n");

exit:
	gpio_mux(I2C_CLEAR);

	close(fd);
	return 0;
}
