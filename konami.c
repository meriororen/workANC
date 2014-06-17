#include <stdio.h>
#include <stdlib.h>
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

#include <pthread.h>

#include "coef.h"

#define PAGE_SIZE 0x1000

#define KHZ			44100
#define CHANNELS 	2

#define RX_BASE 	0xC0000000
#define TX_BASE 	0xD0000000
#define BUFSIZE 	CHANNELS * 440 * 4 // to keep it balanced, it must be (CH * even# * 4) ~ 0.01 second
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
} audio_mode_t;

static volatile int signal_received;
static audio_mode_t mode = MODE_NONE;
static int sinmult;
static int verbose = 0;

typedef enum {
	FIX_SINE,
	FIX_IMPULSE,
} fix_type_t;

static fix_type_t fixtype;
static int fix_samples[BUFSIZE/4]; // for left and right

static void fix_data_generate(void)
{
	float phaseInc;
	float currentPhase = 0.0;
	int i;

	phaseInc = sinmult*(2*PI)/(BUFSIZE/4);
	
	for (i = 0; i < BUFSIZE/4; i++) {
		if (fixtype == FIX_SINE) {
			fix_samples[i] = (int) AMPLITUDE * sin(currentPhase);
			currentPhase += phaseInc;
//			printf("samples[%d] = %08x\n", i, fix_samples[i]);
		} else if (fixtype == FIX_IMPULSE) {
			if (i == 0) fix_samples[i] = 0x800000;
			else fix_samples[i] = 0;
		}
	}
}

static int map_buffers(struct runtime *rx, struct runtime *tx)
{
	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open memory\n");
		return -1;
	}
	
	rx->buf = mmap(NULL, BUFSIZE * BUFCOUNT, PROT_WRITE | PROT_READ, 
							MAP_SHARED, fd, RX_BASE);
	if (rx->buf == MAP_FAILED) {
		printf("Failed to map rx buffer\n");
		return -1;
	} 

	tx->buf = mmap(NULL, BUFSIZE * BUFCOUNT, PROT_WRITE | PROT_READ, 
							MAP_SHARED, fd, TX_BASE);
	if (tx->buf == MAP_FAILED) {
		printf("Failed to map tx buffer\n");
		return -1;
	} 

	close(fd);
	return 0;
}


static int map_coef_buffer(unsigned long **addr) 
{
	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open /dev/mem\n");
		return -1;
	}
	
	*addr = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
					MAP_SHARED, fd, COEF_BASE);

	if (addr == MAP_FAILED) {
		printf("Failed to map FIR coefficient buffer\n");
		return -1;
	}

	close(fd);
	return 0;
}

static void sigint_handler(int sig)
{
	signal_received = sig;
}

static inline void next_addr(struct runtime *run, unsigned long addr)
{
	run->current = addr + run->count * BUFSIZE;
	if (run->count == BUFCOUNT) { 
		run->count = 0; 
		run->current = addr;
	}
}

static int play_next_period(struct runtime *run, int fix)
{
	int ret;
	struct descriptor desc;
	
	if (!run->enable) return 0;
	if (!fix) next_addr(run, TX_BASE);

	desc.read_address = run->current;
	desc.length = BUFSIZE;
	desc.control = DESC_CTL_GO | DESC_CTL_CMPL_IRQ;
	//if (fix) desc.control |= (1 << 10); // park reads

	ret = ioctl(run->fd, KONAMI_PREPARE, &desc);
	if (ret) {
		printf("PLAY: failed ioctl: %d, fd: %d\n", ret * errno, run->fd);
	}

	if (verbose) printf("Start Playing.. %08x count %d\n", run->current, run->count);
	ret = ioctl(run->fd, KONAMI_START_PLAY);
	if (ret) {
		printf("Failed ioctl: %d\n", ret * errno);
	}

	return 1;
}

static int record_next_period(struct runtime *run)
{
	int ret;
	struct descriptor desc;

	if (!run->enable) return 0;
	next_addr(run, RX_BASE);

	desc.write_address = run->current;
	desc.length = BUFSIZE;
	desc.control = DESC_CTL_GO | DESC_CTL_CMPL_IRQ;

	ret = ioctl(run->fd, KONAMI_PREPARE, &desc);
	if (ret) {
		printf("REC: failed ioctl: %d\n", ret * errno);
	}

	if (verbose) printf("Start Recording.. %08x count: %d\n", run->current, run->count);
	ret = ioctl(run->fd, KONAMI_START_RECORD);
	if (ret) {
		printf("Failed ioctl: %d\n", ret * errno);
	}

	return 1;
}

static int init_devices(struct runtime *rx, struct runtime *tx)
{
	rx->fd = open("/dev/c1capt", O_RDWR);
	if (rx->fd < 0) {
		printf("Can't open /dev/c1capt\n");
		return -1;
	} 

	tx->fd = open("/dev/c1play", O_RDWR);
	if (tx->fd < 0) {
		printf("Can't open /dev/c1play\n");
		return -1;
	}
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

static int write_period(struct runtime *run, FILE *file)
{
	int in, i, c;
	char buf[3];
	int total = 0;
	in = i = run->count * BUFSIZE/4;

	while (i < in + BUFSIZE/4) {
		buf[0] = run->buf[i] & 0xff;
		buf[1] = (run->buf[i] >> 8) & 0xff;
		buf[2] = (run->buf[i] >> 16) & 0xff;
		total += fwrite(buf, sizeof(buf), 1, file);
		i++;
	}
	
	return total * (sizeof(buf));
}

static FILE * open_wav(const char *filename, const char *mode) 
{
	FILE *wavf;
	wavf = fopen(filename, mode);
	if (!wavf) {
		printf("Can't open \"%s\"\n", filename);
		return NULL;
	}
	
	return wavf;
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
		0x00, 0x65, 0x04, 0x00, /* Byte Rate */
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

static void record_loop(struct runtime *rxrun, struct runtime *txrun)
{
	struct pollfd pfd;
	struct sigaction sa;
	int ret = 0;
	int filesize = 0;

	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = SA_NOCLDSTOP;
	sa.sa_handler = sigint_handler;
	sigaction(SIGINT, &sa, NULL);

	if (mode == MODE_RECORD) printf("Recording..\n");

	for(;;) {
		if (!record_next_period(rxrun)) goto cont;

		pfd.fd = rxrun->fd;
		pfd.events = POLLIN | POLLRDNORM;

		ret = poll(&pfd, 1, 500);

		rxrun->enable = 0; 
		if (txrun) txrun->enable = 0;

		if (ret > 0) {
			if (pfd.revents & POLLRDNORM) {
				if (mode == MODE_RECORD || mode == MODE_PLAY_RECORD ||
					 mode == MODE_PLAY_FIX_RECORD) {
					filesize += write_period(rxrun, rxrun->wav_file);
					rxrun->enable = 1;
					rxrun->count++;
				}
				if (mode == MODE_REALTIME) {
					txrun->current = rxrun->current;
					txrun->enable = 1;
					play_next_period(txrun, 1);
					rxrun->enable = 1;
					rxrun->count++;
				}
			}
		} else {
			printf("timeout\n");
		}

cont:
		if (signal_received == SIGINT) break;
	}

	printf("Interrupted\n");
	/* finalize -- add size */
	if (mode == MODE_RECORD) {
		finalize_wav(rxrun->wav_file, filesize);
	}
}


static int set_audio_data(struct runtime *tx)
{
	unsigned long d;
	int i, offset, ret; 

	if (tx->count == 3) i = offset = 0;
	else i = offset = tx->count * BUFSIZE/4;

	ret = 0; d = 0;
	while (i < offset + BUFSIZE/4) {
		if (mode == MODE_PLAY || mode == MODE_PLAY_RECORD) {
			ret = fread(&d, 3, 1, tx->wav_file);
			if (!ret) { ret = -1; break; } // end of file
			tx->buf[i] = d;   			
		} else if (mode == MODE_PLAY_FIX || mode == MODE_PLAY_FIX_RECORD) {
			tx->buf[i] = fix_samples[i - offset];
			//if (i == 0) printf("%08x\n", tx->buf[i]);
		}
		i++; 
	}

	return ret;
}


static void play_loop(struct runtime *tx)
{
	struct pollfd pfd;
	struct sigaction sa;
	int ret = 0; int i;

	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = SA_NOCLDSTOP;
	sa.sa_handler = sigint_handler;
	sigaction(SIGINT, &sa, NULL);

	if (mode == MODE_PLAY || mode == MODE_PLAY_RECORD) {
		fseek(tx->wav_file, 0x2c, SEEK_SET); // skip to content
		set_audio_data(tx);
	}

	for (;;) {
		if (!play_next_period(tx, 0)) goto cont;
	
		pfd.fd = tx->fd;
		pfd.events = POLLOUT | POLLWRNORM;
		
		ret = poll(&pfd, 1, 500);

		tx->enable = 0;
		if (ret > 0) {
			if (verbose) printf("TX polled\n");
			if (pfd.revents & POLLWRNORM) {
				if (set_audio_data(tx) < 0) break;
				tx->enable = 1;
				tx->count++;
			}
		} else {
			printf("timeout\n"); break;
		}

cont:
		if (signal_received == SIGINT) break;
	} 
}

/* standalone record thread for simultaneous play-record */
void *record_thread(void *arg) 
{
	struct runtime *rx = arg;
	printf("entering record thread\n");

	record_loop(rx, NULL);
}

int main(int argc, char **argv)
{
	struct runtime *rxrun, *txrun;
	pthread_t record_thread_id;
	int i, j, ret;
	unsigned long *coefbuf;

	if (argc < 2) {
		printf("usage: %s <mode>\n", argv[0]);	
		return 1;
	}

	rxrun = malloc(sizeof(*rxrun));
	txrun = malloc(sizeof(*txrun));
	if (!rxrun || !txrun) {
		printf("Can't allocate runtime structure\n");
		return -1;
	}
	
	rxrun->wav_file = NULL; txrun->wav_file = NULL;

	if (!strcmp(argv[1], "real")) {
		mode = MODE_REALTIME;
	} else if (!strcmp(argv[1], "fix")) {
		if (argc < 3) {
			printf("usage: %s fix [-1|sine freq multiplier] (-1 for impulse)\n");
			return 1;
		}
		mode = MODE_PLAY_FIX; fixtype = FIX_SINE;
		sinmult = atoi(argv[2]);
		printf("sinmult : %d\n", sinmult);
		if (sinmult == -1) fixtype = FIX_IMPULSE;
	} else if (!strcmp(argv[1], "rec")) {
		mode = MODE_RECORD;
		if (argc < 3) {
			printf("Specify file name\n");
			return 1;
		}
		if ((rxrun->wav_file = open_wav(argv[2], "wb+")) == NULL) return -1;
		if (init_wav(rxrun->wav_file) <= 0) { printf("File init failed\n"); return -1; }
	} else if (!strcmp(argv[1], "play")) {
		mode = MODE_PLAY;
		if (argc < 3) {
			printf("Specify file name\n");
			return 1;
		}
		if ((txrun->wav_file = open_wav(argv[2], "r")) == NULL) return -1;
	} else if (!strcmp(argv[1], "playrec")) {
		mode = MODE_PLAY_RECORD;
		if (argc < 4) {
			printf("usage: %s playrec <play file> <rec file>\n");
			return 1;
		}
		if (!strcmp(argv[2], "imp")) { 
			fixtype = FIX_IMPULSE; 
			fix_data_generate(); 
			mode = MODE_PLAY_FIX_RECORD;
		} else {
			if ((txrun->wav_file = open_wav(argv[2], "r")) == NULL) return -1;
		}

		if ((rxrun->wav_file = open_wav(argv[3], "wb+")) == NULL) return -1;
		if (init_wav(rxrun->wav_file) <= 0) { printf("File init failed\n"); return -1; }
	}

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "--no-filter") || !strcmp(argv[i], "-n")) {
			printf("No Filter Mode\n");
			for(j = 0; j < COEF_COUNT; j++) {
				filter_taps[j] = (j == 1) ? (1 << 25) : 0;
			} 
		}
		if (!strcmp(argv[i], "-v")) {
			verbose = 1;
		}
	}

	/* Allocate buffers */
	if(map_buffers(rxrun, txrun) < 0) return -1;

	/* Map coefficient buffer */
	if (map_coef_buffer(&coefbuf) < 0) return -1;

	/* Open devices */
	if(init_devices(rxrun, txrun) < 0) return -1;

	/* Begin */
	init_mode(rxrun, txrun);

	printf("Setting Coefficients.. \n");
	for (i = 0; i < COEF_COUNT; i++) {
		coefbuf[i] = filter_taps[i];
		//printf("Coefficient %d : 0x%08x\n", i, *(coefbuf + i));
	}

	if (mode == MODE_PLAY_FIX) {
		fix_data_generate();
	}

	if (mode == MODE_NONE) goto finish;

	if (mode == MODE_RECORD || mode == MODE_REALTIME)
		record_loop(rxrun, txrun);
	else if (mode == MODE_PLAY || mode == MODE_PLAY_FIX)
		play_loop(txrun);
	else if (mode == MODE_PLAY_RECORD || mode ==  MODE_PLAY_FIX_RECORD) {
		/* use pthread */	
		if (pthread_create(&record_thread_id, NULL, record_thread, (void *)rxrun)) {
			fprintf(stderr, "Error creating thread\n");
			return 1;
		} else {
			play_loop(txrun);
		}

		if (pthread_join(record_thread_id, NULL)) {
			fprintf(stderr, "Error joining thread\n");
			return 2;
		}
	}

	if (txrun->wav_file) fclose(txrun->wav_file);
	if (rxrun->wav_file) fclose(rxrun->wav_file);

finish:
	free(rxrun); free(txrun);
	return 0;
}
