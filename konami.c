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
	run->current += run->count * BUFSIZE;
	if (run->count > BUFCOUNT) { 
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
	//if (fix) desc.control |= (1 << 10);

	ret = ioctl(run->fd, KONAMI_PREPARE, &desc);
	if (ret) {
		printf("PLAY: failed ioctl: %d, fd: %d\n", ret * errno, run->fd);
	}

	printf("Start Playing.. %08x\n", run->current);
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

	printf("Start Recording.. %08x count: %d\n", run->current, run->count);
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

static void initialize(struct runtime *rxrun, struct runtime *txrun,
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

static unsigned long write_period(struct runtime *run, FILE *file)
{
	int i;
	char buf[3];
	unsigned long total = 0;

	for(i = 0; i < BUFSIZE/4; i++) {
		buf[0] = run->buf[i] & 0xff;
		buf[1] = (run->buf[i] >> 8) & 0xff;
		buf[2] = (run->buf[i] >> 16) & 0xff;
		total += fwrite(buf, 3, 1, file);
	}
	//printf("%d\n", total);
	
	return total * 3;
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
		0x06, 0x00, 0x18, 0x00, /* 6 Bytes Align, 24 bits per sample */
		0x64, 0x61, 0x74, 0x61, /* "data" */
		0x00, 0x00, 0x00, 0x00, /* Data chunk size (Size - 0x24) */
	};

	return fwrite(sample_header, 1, sizeof(sample_header), file);
}

int main(int argc, char **argv)
{
	struct runtime *rxrun, *txrun;
	struct pollfd fds[2];
	audio_mode_t mode;
	FILE *wavf;

	int i, ret;
	unsigned long filesize;

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
		filesize = 0;
	}
	else mode = MODE_REALTIME;

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
	initialize(rxrun, txrun, mode);

	if (mode == MODE_PLAY_FIX) {
		generate_sine(txrun->buf);
		txrun->current = TX_BASE;
		play_next_period(txrun, 1);
		free(rxrun); free(txrun);
		return 0;
	}

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = SA_NOCLDSTOP;
	sa.sa_handler = sigint_handler;
	sigaction(SIGINT, &sa, NULL);

	for(;;) {
		record_next_period(rxrun);

		fds[0].fd = rxrun->fd;
		fds[0].events = POLLIN | POLLRDNORM;
		fds[1].fd = txrun->fd;
		fds[1].events = POLLOUT | POLLWRNORM;

		ret = poll(fds, 1, 500);

		rxrun->enable = 0; 
		txrun->enable = 0;

		if (ret > 0) {
			printf("polled\n");
			if (fds[0].revents & POLLRDNORM) {
				printf("RX\n");
				if (mode == MODE_PLAY) continue;
				if (mode == MODE_RECORD) {
					filesize += write_period(rxrun, wavf);
					rxrun->enable = 1;
					rxrun->count++;
#if 0
					for (i = 0; i < BUFSIZE/4; i++) 
						printf("%d: %08x\n", i, rxrun->buf[i]);
#endif
				}
				if (mode == MODE_REALTIME) {
					txrun->current = rxrun->current;
					txrun->enable = 1;
					play_next_period(txrun, 1);
					rxrun->enable = 1;
					rxrun->count++;
				}
			}
			if (fds[1].revents & POLLWRNORM) {
				printf("TX\n");
				if (mode == MODE_PLAY) {
					txrun->count++;
					txrun->enable = 1;
				}
			}
		} else {
			printf("timeout\n");
		}

		if (signal_received == SIGINT) break;
	}

	printf("Interrupted\n");
	if (mode == MODE_RECORD) {
		char buf[8];
		printf("size: %d\n", filesize);
		fseek(wavf, 0x24, SEEK_SET);
		fwrite(&filesize, 4, 1, wavf);
		filesize += 0x24;
		fseek(wavf, 0x4, SEEK_SET);
		fwrite(&filesize, 4, 1, wavf);
	}

	fclose(wavf);
	return 0;
}
