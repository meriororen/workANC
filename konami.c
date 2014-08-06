#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include "konami.h"

#define IMP_INTERVAL 400

static int sinmult;
static int filter_taps[COEF_COUNT];

int fix_samples[BUFSIZE/4]; 
int fixtype = 0;
int delaysize = 0;
static int delaycount;
static volatile int signal_received;
static int interval;
static unsigned int samplecount = 0;

extern char *optarg;

#define CALIBUFSIZE 512
int calibuf0[CALIBUFSIZE];
int calibuf1[CALIBUFSIZE];
int calibc0 = 0; int calibc1 = 0;

int literal = 0;

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
			if (i == 0) fix_samples[i] = IMP_DATA;
			else fix_samples[i] = 0;
		}
	}

//
//	for (i = 0; i < BUFSIZE/4; i++) {
//		printf("%08x\n", fix_samples[i]);
//	}
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

static int map_coef_buffer(unsigned long **addr, int modnum) 
{
	unsigned long coef_base;

	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open /dev/mem\n");
		return -1;
	}

	if (modnum == 1) coef_base = 0xff241000;
	else coef_base = 0xff251000;
	
	*addr = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
					MAP_SHARED, fd, coef_base);

	if (addr == MAP_FAILED) {
		printf("Failed to map FIR coefficient buffer\n");
		return -1;
	}

	close(fd);
	return 0;
}

//#define I2S_MASTER
#ifdef I2S_MASTER
static int enable_i2s(void) 
{
	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open /dev/mem\n");
		return -1;
	}
	
	unsigned long *addr = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
					MAP_SHARED, fd, I2S_BASE);

	if (addr == MAP_FAILED) {
		printf("Failed to map I2S config\n");
		return -1;
	}

	/* enable */
	addr[4] = 0x1;	

	munmap(addr, 0x1000);
	
	close(fd);
	
	return 0;
}
#endif

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

static int init_devices(struct runtime *rx, struct runtime *tx, int modulenum)
{
	char devpath[16];
	
	sprintf(devpath, "/dev/c%dcapt", modulenum);

	rx->fd = open(devpath, O_RDWR);
	if (rx->fd < 0) {
		printf("Can't open %s\n", devpath);
		return -1;
	} 

	sprintf(devpath, "/dev/c%dplay", modulenum);

	tx->fd = open(devpath, O_RDWR);
	if (tx->fd < 0) {
		printf("Can't open %s\n", devpath);
		return -1;
	}
}

/* Receive one period (1 buffer full = 0.01 s) of data from ADC */

static void receive_period(struct runtime *rx, struct runtime *tx)
{
	int offset, i, c, j;
	char buf[3];
	int total = 0;
	offset = i = rx->count * BUFSIZE/4;
	static int _x = 0;

	while (i < offset + BUFSIZE/4) {
		int x = rx->buf[i];
		int left = !(i % 2);
		bool waitfordelay = (samplecount <= delaysize);
		buf[0] = x & 0xff;
		buf[1] = (x >> 8) & 0xff;
		buf[2] = (x >> 16) & 0xff;
	
		if (signal_received == SIGINT) break;
		switch (mode) {
			case MODE_PLAY_FIX_RECORD:
				if (x - _x > 0x600000 && x > 0) {
					printf("samplecount: %d [dc: %d] x: %08x _x: %08x dx: %08x i: %d count: %d\n", 
						samplecount, delaycount, x, _x, x - _x, i - offset, rx->count);
				}
				_x = x;	
				break;
			case MODE_LMS_LEARN:
				if ((!waitfordelay && left) && target->xcount < target->max)
					target->x[target->xcount++] = fix2fl(sext(x));
				break;
			case MODE_PLAY_RECORD:
				if (calibc1 < CALIBUFSIZE) {
					if (left) calibuf1[calibc1++] = sext(x);
				} else {
					int q;
					for(j = 0; j < CALIBUFSIZE; j++) {
						if (calibuf0[j] == 0x7fffff) q = 0;
						if (calibuf1[j] - calibuf1[j-1] > 0x200000) {
							 printf("delay = %d\n", q);
						}
						q++;
					}
					signal_received = SIGINT;
				}
				break;
			default:
				break;
		}


		if (!waitfordelay) {
			rx->filesize += fwrite(buf, 1, sizeof(buf), rx->wav_file);
		}

		if (left) samplecount++;
		i++;
	}
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
		int left = !(i % 2);
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
				if (calibc0 < CALIBUFSIZE && left) calibuf0[calibc0++] = d;
				if (left && mode == MODE_LMS_LEARN) {
					if (target->vcount < target->max && !target->ready) { 
						target->v[target->vcount++] = fix2fl(sext(d));
					} 
				}
				break;
			case MODE_PLAY_FIX:
			case MODE_PLAY_FIX_RECORD:
				tx->buf[i] = fix_samples[i - offset];
				if (mode == MODE_PLAY_FIX_RECORD && fixtype == FIX_IMPULSE) {
					if (tx->buf[i] == IMP_DATA && interval == IMP_INTERVAL) {
						// samplecount = 0;
					}
					else tx->buf[i] = 0;
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

		ret = poll(&pfd, 1, 200); // timeout >10ms (1 period)

		rxrun->enable = 0; 
		if (txrun) txrun->enable = 0;

		if (mode == MODE_LMS_LEARN && target->xcount >= target->max) {
			int j;
			FILE *errf;

			lms_learn(target);

			errf = fopen("error", "w");
			for (j = 0; j < target->max; j++) {
				if (literal) {
					fprintf(buffer1, "%.7g\n", target->v[j]);
					fprintf(buffer2, "%.7g\n", target->x[j]);
				} else {
					fwrite(&target->v[j], sizeof(double), 1, buffer1);
					fwrite(&target->x[j], sizeof(double), 1, buffer2);
				}
				fprintf(errf, "%.7g\n", target->e[j]);
				//printf("%.7g\n", target->y[j]);
			}
			fclose(errf);
			lms_complete(target);
			signal_received = SIGINT;
		}

		if (ret > 0) {
			if (pfd.revents & POLLRDNORM) {
				if (mode == MODE_RECORD || mode == MODE_PLAY_RECORD ||
					 mode == MODE_PLAY_FIX_RECORD || mode == MODE_LMS_LEARN) {
					receive_period(rxrun, txrun);
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
	int ret = 0; int i;

	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = SA_NOCLDSTOP;
	sa.sa_handler = sigint_handler;
	sigaction(SIGINT, &sa, NULL);
	
	tx->count = 0;

	if (mode == MODE_PLAY || mode == MODE_PLAY_RECORD || mode == MODE_LMS_LEARN) {
		fseek(tx->wav_file, 0x2c, SEEK_SET); // skip to content
		set_period(tx);
	}

	while (signal_received != SIGINT) {
		if (mode == MODE_PLAY_FIX_RECORD) 
			interval = (interval == IMP_INTERVAL ? 0 : interval + 1);

		if (!play_next_period(tx, 0)) continue;
	
		pfd.fd = tx->fd;
		pfd.events = POLLOUT | POLLWRNORM;
		
		ret = poll(&pfd, 1, 200); // timeout >10 ms (1 period)

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
	} 
}

/* standalone record thread for simultaneous play-record */

void *record_thread(void *arg) 
{
	struct runtime *rx = arg;

	record_loop(rx, NULL);
}

int main(int argc, char **argv)
{
	struct runtime *rxrun, *txrun;
	int i, j, ret, coef;
	pthread_t record_thread_id;
	unsigned long *coefbuf;
	FILE *coeff = NULL;
	int modnum = 1;
	char record_filename[32] = "recorded.wav";
	char response_data[16] = "res";
	char reference_data[16] = "ref"; 
	int nofilter = 0;

	int option_index;
	int c;
	static const char short_options[] = "er::p:c:n:vd:i:m:N";
	static const struct option long_options[] = {
		{"real", 0, 0, 'e'},
		{"record", 2, 0, 'r'},
		{"play", 1, 0, 'p'},
		{"playrec", 1, 0, 'c'},
		{"learn", 1, 0, 'n'},
		{"fix", 1, 0, 'f'},
		{"verbose", 0, 0, 'v'},
		{"delay", 1, 0, 'd'},
		{"module", 1, 0, 'm'},
		{"literal", 1, 0, 't'},
		{"nofilter", 0, 0, 'N'},
		{0, 0, 0, 0},
	};

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

	while ((c = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1) {
		switch (c) {
			case 't':
				literal = 1;
				break;
			case 'm':
				modnum = atoi(optarg);
				break;
			case 'v':
				verbose = 1;
				break;
			case 'd':
				delaysize = atoi(optarg);
				break;
			case 'e':
				mode = MODE_REALTIME;
				break;
			case 'r':
				mode = MODE_RECORD;
				//strcpy(record_filename, optarg);
				break;
			case 'p':
				mode = MODE_PLAY;
				break;
			case 'c':
				mode = MODE_PLAY_RECORD;
				if (strcmp(optarg, "imp") == 0) {
					mode = MODE_PLAY_FIX_RECORD;
				}
				break;
			case 'n':
				mode = MODE_LMS_LEARN;
				target = lms_init(COEF_COUNT, 40000, MU); /* lms_init(tap, max, mu) */
				break;
			case 'N':
				nofilter = 1;
				break;
			case 'f':
				if (strcmp(optarg, "imp") == 0) {
					fixtype = FIX_IMPULSE;	
				} else {
					sinmult = atoi(optarg);
				}
				printf("fix mode %d\n", sinmult);
				mode = MODE_PLAY_FIX;
				fix_data_generate(fixtype);
				break;
			default:
				break;
		}

		if (mode == MODE_PLAY || mode == MODE_PLAY_RECORD || mode == MODE_LMS_LEARN) {
			if (txrun->wav_file == NULL) {
				if((txrun->wav_file = open_file(optarg, "r")) == NULL) return -1;
			}
		}
	
		if (mode == MODE_RECORD || mode == MODE_PLAY_RECORD || mode == MODE_LMS_LEARN) {
			if (rxrun->wav_file == NULL) {
				if((rxrun->wav_file = open_file(record_filename, "wb+")) == NULL) return -1;
				init_wav(rxrun->wav_file, 1);
			}
		}

		if (mode == MODE_LMS_LEARN) {
			if (buffer1 == NULL || buffer2 == NULL) {
				buffer1 = open_file(reference_data, "wb+");
				buffer2 = open_file(response_data, "wb+");
			}
		}
	}

#ifdef I2S_MASTER
	enable_i2s();
#endif

	/* Open coefficient file */
	if((coeff = open_file("coef.txt", "r")) == NULL) return -1;

	printf("Reading Coefficients.. \n");
	i = 0;
	while (!feof(coeff)) {
		fscanf(coeff, "%d\n", &coef);
		filter_taps[i] = coef;
		i++;
	}

	if (nofilter) {
		printf("No Filter Mode\n");
		for(j = 0; j < COEF_COUNT; j++) {
			filter_taps[j] = (j == 1) ? 0x800000 : 0;
		} 
	}

	/* Allocate buffers */
	if(map_buffers(rxrun, txrun) < 0) return -1;

	/* Map coefficient buffer */
	if (map_coef_buffer(&coefbuf, modnum) < 0) return -1;

	/* Open devices */
	if(init_devices(rxrun, txrun, modnum) < 0) return -1;

	/* Begin */
	init_mode(rxrun, txrun);

	printf("Setting Coefficients..\n");
	for (i = 0; i < COEF_COUNT; i++) {
		coefbuf[i] = filter_taps[i];
		//printf("%d\n", coefbuf[i]);
	}

	memset(rxrun->buf, 0, (BUFSIZE * BUFCOUNT) / 4);
	memset(txrun->buf, 0, (BUFSIZE * BUFCOUNT) / 4);

	if (mode == MODE_NONE) goto finish;

	if (mode == MODE_RECORD || mode == MODE_REALTIME)
		record_loop(rxrun, txrun);
	else if (mode == MODE_PLAY || mode == MODE_PLAY_FIX)
		play_loop(txrun);
	else if (mode == MODE_PLAY_RECORD || 
				mode ==  MODE_PLAY_FIX_RECORD || 
				mode == MODE_LMS_LEARN) {

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
		coeff = open_file("float_coef.txt", "w");
		for (i = 0; i < COEF_COUNT; i++) {
			//fprintf(coeff, "%d\n", fl2fix26(target->h[i]));
			fprintf(coeff, "%.7g\n", target->h[i]);
		}
		FILE * icoeff = open_file("coef.txt", "w");
		for (i = 0; i < COEF_COUNT; i++) {
			fprintf(icoeff, "%d\n", fl2fix26(target->h[i]));
			//fprintf(coeff, "%.7g\n", target->h[i]);
		}
		fclose(icoeff);
		fclose(buffer1); fclose(buffer2);
	}

finish:
	free(rxrun); free(txrun);
	fclose(coeff);
	return 0;
}
