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

#define KHZ			88200
#define CHANNELS  2
#define WORDSIZE  4

#define RX_BASE 	0xC0000000
#define TX_BASE 	0xD0000000
#define RX_BASE2  0xC1000000
#define TX_BASE2  0xD1000000

#define CODEC1_COEF_BASE 0xFF241000
#define CODEC2_COEF_BASE 0xFF251000
#define CODEC1_I2S_BASE 0xFF240000
#define CODEC2_I2S_BASE 0xFF250000

// to keep it balanced, it must be (CHNNL * even number * wordsize) ~ 0.01 second
#define BUFSIZE CHANNELS * 110 * WORDSIZE 
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
#define AMPLITUDE (pow(2, SAMPLE_BIT- 2))

/* FIR Coefficient */
#define COEF_COUNT 128
#define I2S_BASE 0xFF240000

/* Sign extend */
#define sext(x) (x & 0x800000 ? x | 0xFF000000 : x & 0x00FFFFFF)

#define GPIO_PATH	 "/sys/class/gpio"

/* GPIO buttons */
#define BUTTON0 154
#define BUTTON1 155

struct descriptor {
	uint32_t read_address;
	uint32_t write_address;
	uint32_t length;
	uint32_t control;
};

struct module {
	int modnum;
	unsigned long *coefbuf;
	unsigned long *i2sconfig;
	int i2c_fd;
	struct runtime *rx;
	struct runtime *tx;
	struct module *m;
};

struct runtime {
	int fd;
	int count;
	int enable;
	FILE *wav_file;
	int stereo;
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
	MODE_ANC,
} audio_mode_t;

typedef enum {
	EQ_IDLE,
	EQ_START,
	EQ_LMS_LEARN,
	EQ_TEST,
} eq_state_t;

typedef enum {
	ANC_IDLE,
	ANC_CALIBRATE_DELAY,
	ANC_CANCELLATION,
	ANC_FINISH,
} anc_state_t;

static audio_mode_t mode = MODE_NONE;
static eq_state_t eq_stat = EQ_IDLE;
static anc_state_t anc_stat = ANC_IDLE;
extern int button_fd;

#define FIX_SINE 1
#define FIX_IMPULSE 2

int calc_filter_lsq(double *impulse, double *result);
int monitor_button(int num, struct module *m1, struct module *m2);
void btn_gpio_init(void);
int adjust_mic_vol(int fd, int modnum, int ldvol);

static inline double fix2fl(int s) 
{
	const double Q = 1.0F / (0x00000000007fffff + 0.5);
	return (double) s * Q;
}

static inline int fl2fix(double s)
{
	return (int) ceil(s * (0x00000000007fffffF + 0.5));
}

static inline int fl2fix24(double s)
{
	//return (int) ceil(s * (0x0000000001fffffF + 0.5));
	return (int) ceil(s * ((double) pow(2, 23) + 0.5));
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
		0x88, 0x58, 0x01, 0x00, /* 44100 Hz */	
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

static void finalize_wav(struct runtime *run)
{
	FILE *file = run->wav_file;

	run->filesize -= 4;
	if (run->filesize < 0) return;

	//printf("size: %d\n", filesize);
	fseek(file, 0x4, SEEK_SET);
	fwrite(&run->filesize, 4, 1, file);
	run->filesize += 0x24; // header size
	fseek(file, 0x28, SEEK_SET);
	fwrite(&run->filesize, 4, 1, file);
}


static void init_module(struct module *m) 
{
	struct runtime *rx = m->rx;
	struct runtime *tx = m->tx;

	rx->wav_file = NULL;
	tx->wav_file = NULL;
	rx->base = (m->modnum == 1) ? RX_BASE : RX_BASE2;
	tx->base = (m->modnum == 1) ? TX_BASE : TX_BASE2; 
}

#define launch_thread_sync(th_id, loop, x, mod) \
	if (mod->x->enable) {\
		pthread_create(&th_id, NULL, loop, (void *)mod); \
		unstarted++; \
	}

#define join_thread(th_id, x, mod) \
	if (mod->x->enable) \
		pthread_join(th_id, NULL)

#endif
