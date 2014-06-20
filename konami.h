#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <unistd.h>
#include <asm/ioctl.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <pthread.h>

#define PAGE_SIZE 0x1000

#define KHZ			44100
#define CHANNELS  2
#define WORDSIZE  4

#define RX_BASE 	0xC0000000
#define TX_BASE 	0xD0000000

// to keep it balanced, it must be (CHNNL * even number * wordsize) ~ 0.01 second
#define BUFSIZE 	CHANNELS * 440 * WORDSIZE 
#define BUFCOUNT  3

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
#define COEF_BASE 0xFF241000
#define COEF_COUNT 37

/* Ping-pong buffer for delay */
#define MAX_DELAY 1024
#define DELAYSIZE 5500

/* Sign extend */
#define sext(a) \
	(a & 0x800000) ? a |= 0xFF000000 : a

struct descriptor {
	uint32_t read_address;
	uint32_t write_address;
	uint32_t length;
	uint32_t control;
};

struct runtime {
	unsigned long current;
	unsigned long *buf;
	int fd;
	int count;
	int enable;
	FILE *wav_file;
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
} audio_mode_t;

typedef enum {
	STATE_IDLE,
	STATE_LEARNING,
} audio_state_t;

typedef enum {
	FIX_SINE,
	FIX_IMPULSE,
} fix_type_t;

static int *delay;
static int delaycount;
static int samplecount = 0;
static volatile int signal_received;
static audio_mode_t mode = MODE_NONE;
static int sinmult;
static int verbose = 0;
static int fix_samples[BUFSIZE/4]; // for left and right
fix_type_t fixtype = FIX_SINE;
static int interval = 0;
static int filter_taps[COEF_COUNT];
static audio_state_t state = STATE_IDLE;

static FILE * open_file(const char *filename, const char *mode) 
{
	FILE *file;
	file = fopen(filename, mode);
	if (!file) {
		printf("Can't open \"%s\"\n", filename);
		return NULL;
	}
	
	return file;
}

static size_t init_wav(FILE *file)
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

static void init_mode(struct runtime *rxrun, struct runtime *txrun)
{
	switch (mode) {
		case MODE_PLAY:
		case MODE_PLAY_FIX:
			txrun->enable = 1;
			break;
		case MODE_RECORD:
		case MODE_REALTIME:
			rxrun->enable = 1;
			break;
		case MODE_PLAY_RECORD:
		case MODE_PLAY_FIX_RECORD:
		case MODE_LMS_LEARN:
			txrun->enable = 1;
			rxrun->enable = 1;
			break;
		default:
			rxrun->enable = 0;
			txrun->enable = 0;
			break;
	}

	rxrun->current = RX_BASE; 
	txrun->current = TX_BASE; 
	rxrun->count = 0;
	txrun->count = 0;	
}

