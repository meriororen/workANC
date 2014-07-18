#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include "konami.h"

#define IMP_INTERVAL 500

static int sinmult;
static int filter_taps[COEF_COUNT];

audio_mode_t mode = MODE_NONE;
int fix_samples[BUFSIZE/4]; 
int fixtype = 0;
int delaysize = 0;
static int delaycount;
static unsigned int samplecount = 0;
static volatile int signal_received;
static int xfilled = 0;
static int yfilled = 0;
static int interval;

int verbose = 0;
#define IMP_DATA 0x7fffff

static void fix_data_generate(int type)
{
	float phaseInc;
	float currentPhase = 0.0;
	int i;

	phaseInc = sinmult * (2*PI) / (BUFSIZE/4);

	for (i = 0; i < BUFSIZE/4; i++) {
		if (type == FIX_SINE) {
			fix_samples[i] = (int) AMPLITUDE * sin(currentPhase);
			currentPhase += phaseInc;
		} else if (type == FIX_IMPULSE) {
			if (i == 0 || i == 1) fix_samples[i] = IMP_DATA;
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

static int trigger = 0;

static void sigusr2_handler(int sig)
{
	trigger = 1;
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
	
	//printf("run %s\n", run->enable ? "enabled" : "disabled");
	
	if (!run->enable) return 0;
	if (!fix) next_addr(run, TX_BASE);

	desc.read_address = run->current;
	desc.length = BUFSIZE;
	desc.control = DESC_CTL_GO | DESC_CTL_CMPL_IRQ;

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

/* Receive one period (1 buffer full = 0.01 s) of data from ADC */

static int receive_period(struct runtime *run, FILE *file)
{
	int offset, i, c, j;
	char buf[3];
	int total = 0;
	offset = i = run->count * BUFSIZE/4;
	int wbuf[BUFSIZE/4];

	while (i < offset + BUFSIZE/4) {
		int x = run->buf[i];
		buf[0] = x & 0xff;
		buf[1] = (x >> 8) & 0xff;
		buf[2] = (x >> 16) & 0xff;
	
		// be sure to set DA volume attenuation to 0 before calibrating delay
		if (buf[2] < 0xb0 && x < 0 && mode == MODE_PLAY_FIX_RECORD) {
			//delaysize = samplecount;
			printf("samplecount: %d [dc: %d] x: %08x i: %d count: %d\n", 
				samplecount, delaycount, x, i - offset, run->count);
		}

#if 1
		if (mode == MODE_LMS_LEARN && samplecount > delaysize) {
			//
			if (target->vcount <= target->max) {
				if (!(i % 2) && !yfilled) 
					target->v[target->vcount++] = fix2fl(sext(x));
			} else {
				yfilled = 1;
			}

			if (xfilled && yfilled && !target->ready) {
				lms_learn(target);
#if 1
				for (j = 0; j < target->max; j++) {
					fwrite(&target->x[j], sizeof(double), 1, buffer1);
					fwrite(&target->v[j], sizeof(double), 1, buffer2);
				}
#endif
				lms_complete(target);
				signal_received = SIGINT;
			}
		}
#endif
	
		if (samplecount > delaysize && 
			 mode != MODE_PLAY_FIX_RECORD) {
				total += fwrite(buf, sizeof(buf), 1, file);
		}
		if (i % 2) samplecount ++;
		i++;
	}

	return total * (sizeof(buf));
}

/* Set one period (1 buffer full = 0.01 s) of data to DAC */
static int set_period(struct runtime *tx)
{
	unsigned long d;
	int i, offset, ret; 
	int c;

	if (tx->count == BUFCOUNT) i = offset = 0;
	else i = offset = tx->count * BUFSIZE/4;

	ret = 0; d = 0;
	if (verbose) printf("i = %d\n", i);
	while (i < offset + BUFSIZE/4) {
		switch (mode) {
			case MODE_PLAY:
		 	case MODE_PLAY_RECORD:
			case MODE_LMS_LEARN:
				if (feof(tx->wav_file)) { 
					if (verbose) printf("End of File\n");
					ret = -1; break; 
				}
				fread(&d, 3, 1, tx->wav_file);
				tx->buf[i] = d;   			
				if (!(i % 2) && mode == MODE_LMS_LEARN) {
					if (target->xcount < target->max && !xfilled) { 
						target->x[target->xcount++] = fix2fl(sext(d));
					} else {
						xfilled = 1;
					}
				}
				//if (interval == 10) samplecount = 0;
				break;
			case MODE_PLAY_FIX:
			case MODE_PLAY_FIX_RECORD:
				tx->buf[i] = fix_samples[i - offset];
				if (mode == MODE_PLAY_FIX_RECORD || mode == MODE_LMS_LEARN) {
					if (trigger == 1) {
						samplecount = 0;
						trigger = 0;
					}
					if (fixtype == FIX_IMPULSE && fix_samples[i - offset] == IMP_DATA) {
						if (interval == IMP_INTERVAL) samplecount = 0;
						else tx->buf[i] = 0;
					}
				}
				break;
			default:
				break;
		}

		i++; 
	}

	return ret;
}

void record_loop(struct runtime *rxrun, struct runtime *txrun)
{
	struct pollfd pfd;
	struct sigaction sa;
	int ret = 0;

	yfilled = 0;
	rxrun->filesize = 0;

	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = SA_NOCLDSTOP;
	sa.sa_handler = sigint_handler;
	sigaction(SIGINT, &sa, NULL);

	if (rxrun->enable) printf("Recording..\n");

	while(signal_received != SIGINT) {
		if (!record_next_period(rxrun)) continue;

		pfd.fd = rxrun->fd;
		pfd.events = POLLIN | POLLRDNORM;

		ret = poll(&pfd, 1, 11); // timeout >10ms (1 period)

		rxrun->enable = 0; 
		if (txrun) txrun->enable = 0;

		if (ret > 0) {
			if (pfd.revents & POLLRDNORM) {
				if (mode == MODE_RECORD || mode == MODE_PLAY_RECORD ||
					 mode == MODE_PLAY_FIX_RECORD || mode == MODE_LMS_LEARN) {
					rxrun->filesize += receive_period(rxrun, rxrun->wav_file);
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
	}

	printf("Interrupted\n");
	/* finalize -- add size information to wav file */
	if (mode == MODE_RECORD || mode == MODE_PLAY_RECORD ||
		 mode == MODE_PLAY_FIX_RECORD || mode == MODE_LMS_LEARN) {
		printf("Finalizing wav.. filesize: %d\n", rxrun->filesize);
		finalize_wav(rxrun->wav_file, rxrun->filesize);
	}
}

void play_loop(struct runtime *tx)
{
	struct pollfd pfd;
	struct sigaction sa;
	struct sigaction su;
	int ret = 0; int i;

	memset(&sa, 0, sizeof(sa));
	memset(&su, 0, sizeof(su));
	sa.sa_flags = su.sa_flags = SA_NOCLDSTOP;
	sa.sa_handler = sigint_handler;
	su.sa_handler = sigusr2_handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGUSR2, &su, NULL);
	
	tx->count = 0;

	if (mode == MODE_PLAY || mode == MODE_PLAY_RECORD || mode == MODE_LMS_LEARN) {
		fseek(tx->wav_file, 0x2c, SEEK_SET); // skip to content
		set_period(tx);
	}

	while (signal_received != SIGINT) {
		if (mode == MODE_PLAY_FIX_RECORD) {
			interval = interval == IMP_INTERVAL ? 0 : interval + 1;
		}

		if (!play_next_period(tx, 0)) goto cont;
	
		pfd.fd = tx->fd;
		pfd.events = POLLOUT | POLLWRNORM;
		
		ret = poll(&pfd, 1, 11); // timeout >10 ms (1 period)

		tx->enable = 0;
		if (ret > 0) {
			if (verbose) printf("TX polled\n");
			if (pfd.revents & POLLWRNORM) {
				tx->count++;
				if (set_period(tx) < 0) break;
				if (mode == MODE_PLAY_FIX_RECORD && fixtype == FIX_SINE) tx->enable = 0; 
				else tx->enable = 1;
			}
		} else {
			printf("timeout\n"); break;
		}

cont:
		if (trigger == 1){
			 tx->enable = 1;
		}	
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
	int i, j, ret, coef;
	unsigned long *coefbuf;
	FILE *coeff = NULL;

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
			printf("usage: %s fix [imp | <sine freq multiplier>]\n");
			return 1;
		}
		mode = MODE_PLAY_FIX; fixtype = FIX_SINE;
		if (!strcmp(argv[2], "imp")) fixtype = FIX_IMPULSE; 
		else sinmult = atoi(argv[2]);

		fix_data_generate(fixtype);
	} else if (!strcmp(argv[1], "rec")) {
		mode = MODE_RECORD;
		if (argc < 3) {
			printf("Specify file name\n");
			return 1;
		}
		if ((rxrun->wav_file = open_file(argv[2], "wb+")) == NULL) return -1;
		if (init_wav(rxrun->wav_file, 1) <= 0) { printf("File init failed\n"); return -1; }
	} else if (!strcmp(argv[1], "play")) {
		mode = MODE_PLAY;
		if (argc < 3) {
			printf("Specify file name\n");
			return 1;
		}
		if ((txrun->wav_file = open_file(argv[2], "r")) == NULL) return -1;
	} else if (!strcmp(argv[1], "playrec")) {
		mode = MODE_PLAY_RECORD;
		if (argc < 4) {
			printf("usage: %s playrec [imp | sine | <play file>] <rec file>\n");
			return 1;
		}
		if (!strcmp(argv[2], "imp")) { 
			fixtype = FIX_IMPULSE; 
			mode = MODE_PLAY_FIX_RECORD;
		} else if (!strcmp(argv[2], "sine")) {
			sinmult = 6;
			fixtype = FIX_SINE;
			mode = MODE_PLAY_FIX_RECORD;
		} else {
			if ((txrun->wav_file = open_file(argv[2], "r")) == NULL) return -1;
		}

		if ((rxrun->wav_file = open_file(argv[3], "wb+")) == NULL) return -1;
		if (init_wav(rxrun->wav_file, 1) <= 0) { printf("File init failed\n"); return -1; }
	} else if (!strcmp(argv[1], "learn")) {
		if (argc < 6) {
			printf("usage: %s learn <play file> <rec file>\n");
		}

		mode = MODE_LMS_LEARN;
		target = lms_init(COEF_COUNT, 1000 * (BUFSIZE/4), MU); /* lms_init(tap, max, mu) */

		if ((txrun->wav_file = open_file(argv[2], "r")) == NULL) return -1;
		if ((rxrun->wav_file = open_file(argv[3], "wb+")) == NULL) return -1;
		if (init_wav(rxrun->wav_file, 1) <= 0) { printf("File init failed\n"); return -1; }
		if ((buffer1 = open_file(argv[4], "wb+")) == NULL) return -1;
		if ((buffer2 = open_file(argv[5], "wb+")) == NULL) return -1;
	}

	if (mode == MODE_PLAY_FIX_RECORD) {
		fix_data_generate(fixtype); 
	}

	/* Open coefficient file */
	if((coeff = open_file("coef.txt", "r")) == NULL) return -1;

	printf("Reading Coefficients.. \n");
	i = 0;
	while (!feof(coeff)) {
		fscanf(coeff, "%d\n", &coef);
		filter_taps[i] = coef;
		i++;
	}

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-n")) {
			printf("No Filter Mode\n");
			for(j = 0; j < COEF_COUNT; j++) {
				filter_taps[j] = (j == 1) ? 0x800000 : 0;
			} 
		}
		if (!strcmp(argv[i], "-v")) {
			verbose = 1;
		}
		if (!strcmp(argv[i], "-d")) {
			delaysize = atoi(argv[i+1]);
			printf("delay: %d\n", delaysize);
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

	printf("Setting Coefficients..\n");
	for (i = 0; i < COEF_COUNT; i++) {
		coefbuf[i] = filter_taps[i];
		//printf("%d\n", coefbuf[i]);
	}

	/* clear buffer */
	for (i = 0; i < (BUFSIZE * BUFCOUNT)/4; i++) {
		rxrun->buf[i] = 0;
		txrun->buf[i] = 0;
	}

	if (mode == MODE_NONE) goto finish;

	if (mode == MODE_RECORD || mode == MODE_REALTIME)
		record_loop(rxrun, txrun);
	else if (mode == MODE_PLAY || mode == MODE_PLAY_FIX)
		play_loop(txrun);
	else if (mode == MODE_PLAY_RECORD || 
				mode ==  MODE_PLAY_FIX_RECORD || 
				mode == MODE_LMS_LEARN) {

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

	if (mode == MODE_LMS_LEARN) {
		fclose(coeff);
		coeff = open_file("coef.txt", "w");
		for (i = 0; i < COEF_COUNT; i++) {
			fprintf(coeff, "%d\n", fl2fix26(target->h[i]));
			//printf("%d\n", fl2fix26(target->h[i]));
		}
		fclose(buffer1); fclose(buffer2);
	}
finish:
	free(rxrun); free(txrun);
	fclose(coeff);
	return 0;
}
