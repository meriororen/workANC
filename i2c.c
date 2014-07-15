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

#define CODEC3_I2C_CS 158
#define CODEC2_I2C_CS 157
#define CODEC1_I2C_CS
#define I2C_CLEAR 999

#define GPIO_PATH	 "/sys/class/gpio"

#define I2C_ADDRESS 0x38

static int verbose = 0;

static int pll_read(int fd) {
	int ret = 0;
	uint8_t buf[8];
	
	ret = ioctl(fd, I2C_SLAVE, I2C_ADDRESS);
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

	ret = ioctl(fd, I2C_SLAVE, I2C_ADDRESS);
	if (ret < 0) { printf("Wrong Address? %x\n", I2C_ADDRESS); close(fd); return ret; }

	ret = write(fd, cmd, len);
	if (ret < len) { printf("Can't write: %s\n", strerror(errno)); close(fd); return ret; }

	usleep(10000);

	if (verbose) printf("Command: \"%s\" Written successfully\n", name);
	return 0;
}

static void construct_cmd(uint8_t msb, uint8_t cmd, uint8_t *buf) {
	buf[0] = 0x40;
	buf[1] = msb;
	buf[2] = cmd;	
}

static void gpio_init(void) {
	char path[60];
	FILE* fd;

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

	fclose(fd);

	sprintf(path, "%s/gpio%d/direction", GPIO_PATH, CODEC3_I2C_CS);
	
	printf("opening %s..\n", path);
	fd = fopen(path, "w");
	if (!fd) {
		printf("Can't open direction file for gpio%d\n", CODEC3_I2C_CS);
		return;
	}

	rewind(fd);
	fprintf(fd, "out");

	fclose(fd);

	sprintf(path, "%s/gpio%d/direction", GPIO_PATH, CODEC2_I2C_CS);

	printf("opening %s..\n", path);
	fd = fopen(path, "w");
	if (!fd) {
		printf("Can't open direction file for gpio%d\n", CODEC2_I2C_CS);
		return;
	}

	rewind(fd);
	fprintf(fd, "out");

	fclose(fd);
}

static void gpio_mux(int codec) {
	char path[50];
	FILE * fd;

	sprintf(path, "%s/gpio%d/value", GPIO_PATH, CODEC2_I2C_CS);

	if (access(path, F_OK) < 0) gpio_init();

	fd = fopen(path, "w");
	if (!fd) {
		printf("Failed to initialize gpio\n");			
		return;
	}

	printf("writing %d to %s\n", codec == CODEC2_I2C_CS ? 0 : 1, path);
	rewind(fd);
	if (codec == I2C_CLEAR) fprintf(fd, "0");
	else fprintf(fd, "%d", codec == CODEC2_I2C_CS ? 0 : 1);

	fclose(fd);

	sprintf(path, "%s/gpio%d/value", GPIO_PATH, CODEC3_I2C_CS);

	fd = fopen(path, "w");
	if (!fd) {
		gpio_init();
		printf("Can't open %s\n", path);
		if (!fd) return;
	}

	printf("writing %d to %s\n", codec == CODEC3_I2C_CS ? 0 : 1, path);
	rewind(fd);
	if (codec == I2C_CLEAR) fprintf(fd, "0");
	else fprintf(fd, "%d", codec == CODEC3_I2C_CS ? 0 : 1);

	fclose(fd);
}

int main(int argc, char **argv) {
	int fd, ret;
	uint8_t *buf;
	uint8_t mx0, mx1;	
	int val, i;
	int mode = 1; //master

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) verbose = 1;
		if (!strcmp(argv[i], "-1")) {
			gpio_mux(CODEC3_I2C_CS);
		}
		if (!strcmp(argv[i], "-2")) {
			mode = 0; //slave
			gpio_mux(CODEC2_I2C_CS);
		}
	}	

	
	fd = open("/dev/i2c-0", O_RDWR);
	if (fd < 0) return 1;

	buf = malloc(sizeof(uint8_t) * 8);
	if (!buf) { printf("Malloc failed\n"); return -1; }

	/* Set up PLL by first turning it off */
	construct_cmd(0x00, 0x0E, buf);
	if (write_command(fd, buf, 3, "SET UP PLL")) goto exit;

	usleep(65000);

#if 1

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

#if 0
	construct_cmd(0x2F, 0xA0, buf);
	if (write_command(fd, buf, 3, "CONTROL PAD CONTROL 0")) goto exit;

	construct_cmd(0x30, 0x01, buf);
	if (write_command(fd, buf, 3, "CONTROL PAD CONTROL 1")) goto exit;
#endif

	if (mode) {
		/* Become I2S Master */
		construct_cmd(0x15, 0x01, buf);
		if (write_command(fd, buf, 3, "BECOME I2S MASTER")) goto exit;
	} else {
		construct_cmd(0x15, 0x00, buf);
		if (write_command(fd, buf, 3, "BECOME I2S SLAVE")) goto exit;
	}

	/* Setting input mixers */
	/* Enable left mixer */
	construct_cmd(0x0A, 0x01, buf);
	if (write_command(fd, buf, 3, "ENABLE RECORD MIXER LEFT")) goto exit;
	
	/* Digital volume (AD) Attenuator (Negative dB) */
	construct_cmd(0x1A, 0x00, buf);
	if (write_command(fd, buf, 3, "DIGITAL VOLUME (LADVOL)")) goto exit;
	
	val = 0;
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
	val = (0x3F << 2) | (1 << 1) | 1;
	construct_cmd(0x0E, val, buf);
	if (write_command(fd, buf, 3, "LEFT DIFF INPUT VOL CONTROL")) goto exit;

//#define ALC
#ifdef ALC
	/* PGASLEW[1:0] | ALCMAX[2:0] | ALCSEL[2:0] */
	/* ALCSEL : 000 => off, 001 => Right, 010 => Left, 011 => Stereo */
	val = (0 << 6) | (2 << 3) | (3 << 0);
	construct_cmd(0x11, val, buf);
	if (write_command(fd, buf, 3, "LEFT DIFF INPUT ALC")) goto exit;

	/* NGTYP[1:0] | NGEN | NGTHR[4:0] */
	val = (3 << 6) | (1 << 5) | (0x08); 
	construct_cmd(0x14, val, buf);
	if (write_command(fd, buf, 3, "ALC NOISE GATE")) goto exit;
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
	
#if 0
	/* Left Headphone Volume */
	construct_cmd(0x23, 0xFE, buf);
	if (write_command(fd, buf, 3, "ENABLE HP VOLUME LEFT")) goto exit;

	/* Right Headphone Volume */
	construct_cmd(0x24, 0xFE, buf);
	if (write_command(fd, buf, 3, "ENABLE HP VOLUME RIGHT")) goto exit;

#endif
	/* Left Line Out Volume */
	/* this make difference */
	construct_cmd(0x25, 0xFE, buf);
	if (write_command(fd, buf, 3, "ENABLE LINEOUT VOLUME LEFT")) goto exit;
	
	/* Set Up ADC */
	construct_cmd(0x19, 0x03, buf);
	if (write_command(fd, buf, 3, "ADC ENABLE (BOTH)")) goto exit;

	/* Enable DAC */
	construct_cmd(0x2a, 0x01, buf);
	if (write_command(fd, buf, 3, "dac enable (both)")) goto exit;


#if 1
	/* Converter */
	/* 0 | DAPAIR[1:0] | DAOSR | ADOSR | CONVSR[2:0] */
	val =  (0 << 5) | (0 << 4) | (0 << 3) | 0;
	construct_cmd(0x17, val, buf);
	if (write_command(fd, buf, 3, "Converter")) goto exit;


	/* Serial Control 1 */
	/* BPF[2:0] | ADTDM | DATDM | MSBP | LRDEL[1:0] */
	//val = 0;
	val = (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | 0;
	construct_cmd(0x16, val, buf);
	if (write_command(fd, buf, 3, "Serial Control 1")) goto exit;

#endif

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
	
#if 0
	construct_cmd(0x2D, 0xBA, buf);
	if (write_command(fd, buf, 3, "DAC_DATA PULL DOWN")) goto exit;

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
