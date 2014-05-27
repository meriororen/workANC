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
#include <math.h>
#include <unistd.h>
#include <asm/ioctl.h>
#include <poll.h>
#include <string.h>

#define PAGE_SIZE 0x1000

#define RX_BASE 	0xC0000000
#define TX_BASE 	0xD0000000
#define BUFSIZE 	480 * 4 // 0.01sec
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
};

typedef enum {
	MODE_NONE,
	MODE_PLAY,
	MODE_PLAY_FIX,
	MODE_PLAY_IMPULSE,
	MODE_RECORD,
	MODE_REALTIME,
} audio_mode_t;

/* Generate 1 period of sinewave and put at TX_BASE */
static void generate_sine(unsigned long *buf)
{
	int samples[BUFSIZE/4];
	float phaseInc;
	float currentPhase = 0.0;
	int i;

	phaseInc = PI * 2/BUFSIZE/4;
	
	for (i = 0; i < BUFSIZE/4; i++) {
		samples[i] = (int) AMPLITUDE * sin(currentPhase);
		currentPhase += phaseInc;
	}

	for (i = 0; i < BUFSIZE/4; i++) {
		buf[i] = samples[i];
		printf("%08x %08x\n", samples[i], buf[i]);
	}
}

static volatile int signal_received;

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

	ret = ioctl(run->fd, KONAMI_PREPARE, &desc);
	if (ret) {
		printf("PLAY: failed ioctl: %d, fd: %d\n", ret * errno, run->fd);
	}

//	printf("Start Playing.. %08x count %d\n", run->current, run->count);
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

	//printf("Start Recording.. %08x count: %d\n", run->current, run->count);
	ret = ioctl(run->fd, KONAMI_START_RECORD);
	if (ret) {
		printf("Failed ioctl: %d\n", ret * errno);
	}

	return 1;
}

static int allocate_buffer(struct runtime *rx, struct runtime *tx)
{
	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open memory\n");
		return -1;
	}
	
	rx->buf = mmap(NULL, BUFSIZE * BUFCOUNT, PROT_WRITE | PROT_READ, 
							MAP_SHARED, fd, RX_BASE);
	if (rx->buf == MAP_FAILED) {
		printf("Failed to allocate buffer\n");
		return -1;
	} 

	tx->buf = mmap(NULL, BUFSIZE * BUFCOUNT, PROT_WRITE | PROT_READ, 
							MAP_SHARED, fd, TX_BASE);
	if (tx->buf == MAP_FAILED) {
		printf("Failed to allocate buffer\n");
		return -1;
	} 
}

static int init_devices(struct runtime *rx, struct runtime *tx)
{
	rx->fd = open("/dev/c1capt", O_RDWR);
	if (!rx->fd) {
		printf("Can't open /dev/c1capt");
		return -1;
	} 

	tx->fd = open("/dev/c1play", O_RDWR);
	if (!tx->fd) {
		printf("Can't open /dev/c1play");
		return -1;
	}
}

static void init_mode(struct runtime *rxrun, struct runtime *txrun,
								audio_mode_t mode) 
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
		0x80, 0xBB, 0x00, 0x00, /* 48000 Hz */	
		0x00, 0x65, 0x04, 0x00, /* Byte Rate */
		0x06, 0x00, 0x18, 0x00, /* 6 Bytes Align, 24 bits per sample */
		0x64, 0x61, 0x74, 0x61, /* "data" */
		0x00, 0x00, 0x00, 0x00, /* Data chunk size (Size - 0x24) */
	};

	return fwrite(sample_header, 1, sizeof(sample_header), file);
}

static void record_loop(struct runtime *rxrun, struct runtime *txrun,
								FILE *wavf, audio_mode_t mode)
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
		txrun->enable = 0;

		if (ret > 0) {
			if (pfd.revents & POLLRDNORM) {
				if (mode == MODE_RECORD) {
					filesize += write_period(rxrun, wavf);
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
		filesize -= 4;
		printf("size: %d\n", filesize);
		fseek(wavf, 0x4, SEEK_SET);
		fwrite(&filesize, 4, 1, wavf);
		filesize += 0x24; // header size
		fseek(wavf, 0x28, SEEK_SET);
		fwrite(&filesize, 4, 1, wavf);
	}
}


static int set_audio_data(struct runtime *tx, FILE *wavf)
{
	unsigned long d;
	int i, in, ret; 

	if (tx->count == 3) i = in = 0;
	else i = in = tx->count * BUFSIZE/4;

	ret = 0; d = 0;
	while (i < in + BUFSIZE/4) {
		if (i % 2) {                  // left
			ret = fread(&d, 3, 1, wavf);
			if (!ret) { ret = -1; break; } // end of file
			// to set 1st 4 Bytes sample as left 
			tx->buf[i] = d;   			
			//printf("%08x\n", tx->buf[i-1]);
		} else {
			if (i > 0) tx->buf[i] = 0;			// right
			fseek(wavf, ftell(wavf) + 3, SEEK_SET); // skip 3bytes
		}
		i++; 
	}

	return ret;
}


static void play_loop(struct runtime *tx, FILE *wavf)
{
	struct pollfd pfd;
	struct sigaction sa;
	int ret = 0; int i;

	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = SA_NOCLDSTOP;
	sa.sa_handler = sigint_handler;
	sigaction(SIGINT, &sa, NULL);

	fseek(wavf, 0x2C, SEEK_SET); // skip to content
	set_audio_data(tx, wavf);

	for (;;) {
		if (!play_next_period(tx, 0)) goto cont;
	
		pfd.fd = tx->fd;
		pfd.events = POLLOUT | POLLWRNORM;
		
		ret = poll(&pfd, 1, 500);

		tx->enable = 0;
		if (ret > 0) {
			if (pfd.revents & POLLWRNORM) {
				if (set_audio_data(tx, wavf) < 0) break;
				tx->enable = 1;
			}
		} else {
			printf("timeout\n"); break;
		}

cont:
		if (signal_received == SIGINT) break;
	} 
}

int main(int argc, char **argv)
{
	struct runtime *rxrun, *txrun;
	audio_mode_t mode;
	FILE *wavf;

	int i, ret;

	if (argc < 2) {
		printf("usage: %s <mode>\n", argv[0]);	
		return 1;
	}

	if (!strcmp(argv[1], "real")) mode = MODE_REALTIME;
	else if (!strcmp(argv[1], "fix")) mode = MODE_PLAY_FIX;
	else if (!strcmp(argv[1], "rec")) {
		if (argc < 3) {
			printf("Specify file name\n");
			return 1;
		}
		wavf = fopen(argv[2], "wb+");
		if (!wavf) {
			printf("Can't open \"%s\"\n", argv[2]);
			return -1;
		}
		if (init_wav(wavf) <= 0) return -1;
		mode = MODE_RECORD;
	} else if (!strcmp(argv[1], "play")) {
		if (argc < 3) {
			printf("Specify file name\n");
			return 1;
		}
		wavf = fopen(argv[2], "r");
		if (!wavf) {
			printf("Can't open \"%s\"\n", argv[2]);
			return -1;
		}
		mode = MODE_PLAY;
	} else { 
		mode = MODE_REALTIME;
	}

	rxrun = malloc(sizeof(*rxrun));
	txrun = malloc(sizeof(*txrun));
	if (!rxrun || !txrun) {
		printf("Can't allocate runtime structure\n");
		return -1;
	}

	/* Allocate buffers */
	if(allocate_buffer(rxrun, txrun) < 0) return -1;

	/* Open devices */
	if(init_devices(rxrun, txrun) < 0) return -1;

	/* Begin */
	init_mode(rxrun, txrun, mode);

	if (mode == MODE_PLAY_FIX) {
		generate_sine(txrun->buf);
		txrun->current = TX_BASE;
		play_next_period(txrun, 1);
		free(rxrun); free(txrun);
		return 0;
	}

	if (mode == MODE_RECORD || mode == MODE_REALTIME)
		record_loop(rxrun, txrun, wavf, mode);	
	else if (mode == MODE_PLAY)
		play_loop(txrun, wavf);

	if (mode == MODE_RECORD || mode == MODE_PLAY) fclose(wavf);
	free(rxrun); free(txrun);
	return 0;
}
