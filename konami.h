#ifndef ANC_KONAMI_H
#define ANC_KONAMI_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <pthread.h>

#include "orz_lms.h"

#define MODULENUM 2

#define PAGE_SIZE 0x1000

#define KHZ			44100
#define CHANNELS  2
#define WORDSIZE  4

#define RX_BASE 	0xC0000000
#define TX_BASE 	0xD0000000
#define RX_BASE2  0xC1000000
#define TX_BASE2  0xD1000000

// to keep it balanced, it must be (CHNNL * even number * wordsize) ~ 0.01 second
#define BUFSIZE CHANNELS * 55 * WORDSIZE 
#define BUFCOUNT 1

/* ioctl definition */
#define KONAMI_MAGIC 0xBA
#define KONAMI_PREPARE _IOW(KONAMI_MAGIC, 1, struct descriptor)
#define KONAMI_START_RECORD _IO(KONAMI_MAGIC, 2)
#define KONAMI_START_PLAY _IO(KONAMI_MAGIC, 3)

/* MSGDMA descriptor bits */
#define DESC_CTL_GO 		(1 << 31)
#define DESC_CTL_CMPL_IRQ (1 << 14)

/* Sine generator */
#define PI (3.14159)
#define SAMPLE_BIT 24
#define AMPLITUDE (pow(2, SAMPLE_BIT- 1))

/* FIR Coefficient */
#define COEF_COUNT 64
#define I2S_BASE 0xFF240000

/* Sign extend */
#define sext(x) (x & 0x800000 ? x | 0xFF000000 : x & 0x00FFFFFF)

struct descriptor {
	uint32_t read_address;
	uint32_t write_address;
	uint32_t length;
	uint32_t control;
};

struct runtime {
	int fd;
	int modnum;
	int count;
	int enable;
	FILE *wav_file;
	unsigned long current;
	unsigned long *buf;
	unsigned int filesize; /* for rx only */
	unsigned int delaycount;
	unsigned long base;
};

typedef enum {
	MODE_NONE,
	MODE_PLAY,
	MODE_PLAY_FIX,
	MODE_PLAY_RECORD,
	MODE_PLAY_FIX_RECORD,
	MODE_RECORD,
	MODE_REALTIME,
	MODE_LMS_LEARN,
	MODE_EQUAL_FILTER,
} audio_mode_t;

typedef enum {
	EQ_IDLE,
	EQ_START,
	EQ_CALIBRATE_DELAY,
	EQ_LMS_LEARN,
	EQ_FINISH,
} eq_state_t;

static audio_mode_t mode = MODE_NONE;
static eq_state_t eq_stat = EQ_IDLE;

#define FIX_SINE 1
#define FIX_IMPULSE 2

int calc_filter_lsq(double *impulse, double *result);

static inline double fix2fl(int s) 
{
	const double Q = 1.0F / (0x00000000007fffff + 0.5);
	return (double) s * Q;
}

static inline int fl2fix(double s)
{
	return (int) ceil(s * (0x00000000007fffffF + 0.5));
}

static inline int fl2fix26(double s)
{
	//return (int) ceil(s * (0x00000000007fffffF + 0.5));
	return (int) ceil(s * ((double ) pow(2, 21) + 0.5));
}

static FILE * open_file(const char *filename, const char *mode) 
{
	FILE *file;
	file = fopen(filename, mode);
	if (!file) {
		printf("Cannot open \"%s\"\n", filename);
		return NULL;
	}
	
	return file;
}

static size_t init_wav(FILE *file, int stereo)
{
	/* Create header */
	char sample_header[] = {
		0x52, 0x49, 0x46, 0x46, /* "RIFF" */
		0x00, 0x00, 0x00, 0x00, /* Size */
		0x57, 0x41, 0x56, 0x45, /* "WAVE" */
		0x66, 0x6D, 0x74, 0x20, /* "fmt " */
		0x10, 0x00, 0x00, 0x00, /* size of fmt chunk (16) */
		0x01, 0x00, 0x02, 0x00, /* PCM; 2-channels */
		0x44, 0xAC, 0x00, 0x00, /* 44100 Hz */	
		0x98, 0x09, 0x04, 0x00, /* Byte Rate */
		0x06, 0x00, 0x18, 0x00, /* 6 Bytes Align, 24 bits per sample */
		0x64, 0x61, 0x74, 0x61, /* "data" */
		0x00, 0x00, 0x00, 0x00, /* Data chunk size (Size - 0x24) */
	};

	if (!stereo) {
		sample_header[22] = 0x01;
		sample_header[28] = 0xCC;
		sample_header[29] = 0x04;
		sample_header[30] = 0x02;
		sample_header[32] = 0x03;
	}

	return fwrite(sample_header, 1, sizeof(sample_header), file);
}

static void finalize_wav(FILE *file, int filesize) 
{
		filesize -= 4;
		if (filesize < 0) return;

		printf("size: %d\n", filesize);
		fseek(file, 0x4, SEEK_SET);
		fwrite(&filesize, 4, 1, file);
		filesize += 0x24; // header size
		fseek(file, 0x28, SEEK_SET);
		fwrite(&filesize, 4, 1, file);
}

static void init_mode(struct runtime *rx, struct runtime *tx, int modnum)
{
	tx->enable = 0;
	rx->enable = 0;

	switch (mode) {
		case MODE_PLAY:
		case MODE_PLAY_FIX:
			if (modnum == tx->modnum) tx->enable = 1;
			break;
		case MODE_RECORD:
		case MODE_REALTIME:
			if (modnum == rx->modnum) rx->enable = 1;
			break;
		case MODE_PLAY_RECORD:
		case MODE_PLAY_FIX_RECORD:
		case MODE_LMS_LEARN:
			if (modnum == tx->modnum) {
				tx->enable = 1;
				rx->enable = 1;
			}
			break;
		default:
			break;
	}

	rx->count = 0;
	tx->count = 0;	
}

static void init_module(struct runtime *rx, struct runtime *tx, int modnum) 
{
	rx->wav_file = NULL; 
	tx->wav_file = NULL; 
	rx->base = (modnum == 1) ? RX_BASE : RX_BASE2; 
	tx->base = (modnum == 1) ? TX_BASE : TX_BASE2; 
	rx->current = rx->base; 
	tx->current = tx->base;
	rx->modnum = modnum; 
	tx->modnum = modnum;
}

#define launch_thread(th_id, loop, run) \
	if (run->enable) {\
		pthread_create(&th_id, NULL, loop, (void *)run); \
		unstarted++; \
	}

#define join_thread(th_id, run) \
	if (run->enable) \
		pthread_join(th_id, NULL)

#endif
