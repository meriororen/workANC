#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <asm/ioctl.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <getopt.h>
#include "konami.h"

#define IMP_INTERVAL 40
#define IMP_DATA 0x7fffff
#define CALIBUFSIZE 256
#define LMS_MAX_DATA 40000

static int sinmult;
static int filter_taps[COEF_COUNT];
static int fix_samples[BUFSIZE/4]; 
static int fixtype = 0;
static int delaysize = 0;
static int interval = 0;
static int samplecount = 0;
static int stop_all_threads = 0;
static int signal_received;
static int do_calibration = 0;
static int modnum = 1;
static int verbose = 0;
static int calibc0 = 0; 
static int calibc1 = 0;
static int calibc2 = 0;
static long calibuf0[CALIBUFSIZE];
static long calibuf1[CALIBUFSIZE];
static long calibuf2[CALIBUFSIZE];
static lms_t *target;
static int unstarted = 0;
static FILE *reff;
static FILE *resf;
static FILE *resf2;
pthread_mutex_t start_mutex = PTHREAD_MUTEX_INITIALIZER;
extern char *optarg;

static 
void fix_data_generate(int type)
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

static 
int map_buffers(struct runtime *rx, struct runtime *tx)
{
	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open memory\n");
		return -1;
	}
	
	rx->buf = mmap(NULL, BUFSIZE * BUFCOUNT, PROT_WRITE | PROT_READ, 
							MAP_SHARED, fd, rx->base);
	if (rx->buf == MAP_FAILED) {
		printf("Failed to map rx buffer\n");
		return -1;
	} 

	tx->buf = mmap(NULL, BUFSIZE * BUFCOUNT, PROT_WRITE | PROT_READ, 
							MAP_SHARED, fd, tx->base);
	if (tx->buf == MAP_FAILED) {
		printf("Failed to map tx buffer\n");
		return -1;
	} 

	close(fd);

	memset(rx->buf, 0, (BUFSIZE * BUFCOUNT));
	memset(tx->buf, 0, (BUFSIZE * BUFCOUNT));
	return 0;
}

static 
int map_coef_buffer(unsigned long **addr, int modnum) 
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

static 
int enable_i2s(int mnum) 
{
	unsigned long base;
	
	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open /dev/mem\n");
		return -1;
	}

	if (mnum == 1) base = 0xff240000;
	else base = 0xff250000;
	
	unsigned long *addr = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
					MAP_SHARED, fd, base);

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

static 
void sigint_handler(int sig)
{
	signal_received = sig;
}

static inline 
void next_addr(struct runtime *run, unsigned long addr)
{
	run->current = addr + run->count * BUFSIZE;
	if (run->count == BUFCOUNT) { 
		run->count = 0; 
		run->current = addr;
	}
}

static 
int play_next_period(struct runtime *run, int fix)
{
	int ret;
	struct descriptor desc;
	
	if (!run->enable) return 0;
	if (!fix) next_addr(run, run->base);

	desc.read_address = run->current;
	desc.length = BUFSIZE;
	desc.control = DESC_CTL_GO | DESC_CTL_CMPL_IRQ;

	ret = ioctl(run->fd, KONAMI_PREPARE, &desc);
	if (ret) {
		printf("PLAY: failed ioctl: %d, fd: %d\n", ret * errno, run->fd);
	}

	if (verbose) printf("Start Playing.. %08lx count %d\n", run->current, run->count);
	ret = ioctl(run->fd, KONAMI_START_PLAY);
	if (ret) {
		printf("Failed ioctl: %d\n", ret * errno);
	}

	return 1;
}

static 
int record_next_period(struct runtime *run)
{
	int ret;
	struct descriptor desc;

	if (!run->enable) return 0;
	next_addr(run, run->base);

	desc.write_address = run->current;
	desc.length = BUFSIZE;
	desc.control = DESC_CTL_GO | DESC_CTL_CMPL_IRQ;

	ret = ioctl(run->fd, KONAMI_PREPARE, &desc);
	if (ret) {
		printf("REC: failed ioctl: %d\n", ret * errno);
	}

	if (verbose) printf("Start Recording.. %08lx count: %d\n", run->current, run->count);
	ret = ioctl(run->fd, KONAMI_START_RECORD);
	if (ret) {
		printf("Failed ioctl: %d\n", ret * errno);
	}

	return 1;
}

static 
int init_devices(struct runtime *rx, struct runtime *tx, int modulenum)
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

	return 0;
}

#define DTHRSHOLD 0x599999 //(0.7) * (2^23)
#define DTHRSHOLD2 0x66666 //(0.05) * (2^23)

static
int calibrate_delay(void)
{
	int j;
	int delay = 0;

	for(j = 0; j < CALIBUFSIZE; j++) {
		//	int e = calibuf1[j] - calibuf1[j-1];
		//printf("%d : %d [%d] %d\n", calibuf0[j], calibuf1[j], j, e);
		if (calibuf0[j] == IMP_DATA) delay = 0;
		//if (e > DTHRSHOLD) { 
		if (calibuf1[j] > DTHRSHOLD) {
			printf("delay = %d\n", delay);
			return delay;
		}
		delay++;
	}

	return 0;
}


static 
int calibrate_delay2(void)
{
	int j;
	int delay1 = 0, delay2 = 0;
	float dist = 0.;
	
	for (j = 0; j < CALIBUFSIZE; j++) {
		//int d1 = calibuf1[j] - calibuf1[j-1];
		if (calibuf0[j] == IMP_DATA) delay1 = 0;
		//if (d1 > DTHRSHOLD) {
		if (calibuf1[j] > DTHRSHOLD) {
			printf("delay1 = %d\n", delay1);
			break;
		}
		delay1++;
	}

	for (j = 0; j < CALIBUFSIZE; j++) {
		//int d2 = calibuf2[j] - calibuf2[j-1];
		if (calibuf0[j] == IMP_DATA) delay2 = 0;
		//if (d2 > DTHRSHOLD2) {
		if (calibuf2[j] > DTHRSHOLD2) {
			printf("delay2 = %d\n", delay2);
			break;
		}
		delay2++;
	}

	dist = ((float)((delay2 - delay1)/441.0)) * 340.29;

	printf("delay2 - delay1 = %d (%.1f cm)\n", delay2 - delay1, dist);

	return delay2 - delay1;
}

/* Receive one period (1 full buffer) */
static 
void receive_period(struct runtime *rx)
{
	int offset, i;
	char buf[3];
	offset = i = rx->count * BUFSIZE/4;

	while (i < offset + BUFSIZE/4) {
		long x = rx->buf[i];
		int left = !(i % 2);
		bool waitfordelay = (samplecount <= delaysize);
		buf[0] = x & 0xff;
		buf[1] = (x >> 8) & 0xff;
		buf[2] = (x >> 16) & 0xff;

		if (stop_all_threads == 1) break;
		switch (mode) {
			case MODE_LMS_LEARN:
				if ((!waitfordelay && left) && target->xcount < target->max)
					target->x[target->xcount++] = fix2fl(sext(x));
				break;
			case MODE_PLAY_RECORD:
			case MODE_PLAY_FIX_RECORD:
				if (calibc1 < CALIBUFSIZE) {
					if (left) calibuf1[calibc1++] = sext(x);
				} else {
					if ((delaysize = calibrate_delay()) > 0) {
						stop_all_threads = 1;
						return;
					}
				}
				if (!waitfordelay)
					rx->filesize += fwrite(buf, 1, sizeof(buf), rx->wav_file);
				break;
			case MODE_EQUAL_FILTER:
				switch (eq_stat) {
					case EQ_CALIBRATE_DELAY:
						if (calibc1 < CALIBUFSIZE) {
							if (left) calibuf1[calibc1++] = sext(x);
						} else {
							if ((delaysize = calibrate_delay()) > 0) {
								stop_all_threads = 1;
								return;
							}
						}
						break;
					case EQ_LMS_LEARN:
					case EQ_TEST:
						if (!waitfordelay && left && (target->xcount < target->max)) 
							target->x[target->xcount++] = fix2fl(sext(x));
						break;
					default:
						printf("Unknown state for mode EQUALIZER\n");
						break;
				}
				break;
			case MODE_ANC:
				switch (anc_stat) {
					case ANC_CALIBRATE_DELAY:
						if (calibc1 < CALIBUFSIZE && rx->modnum == 2)
							if (left) calibuf1[calibc1++] = sext(x);
						if (calibc2 < CALIBUFSIZE && rx->modnum == 1)
							if (left) calibuf2[calibc2++] = sext(x);

						if (calibc1 >= CALIBUFSIZE && calibc2 >= CALIBUFSIZE &&
							 rx->modnum == 2) {
							delaysize = calibrate_delay2();
							stop_all_threads = 1;
							return;
						}
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
		if (left) samplecount++;
		i++;
	}
}

/* Set one period (1 full buffer) */
static 
int set_period(struct runtime *tx)
{
	unsigned long d, w;
	int i, offset, ret; 

	if (tx->count == BUFCOUNT) i = offset = 0;
	else i = offset = tx->count * BUFSIZE/4;

	ret = 1; d = 0;
	while (i < offset + BUFSIZE/4) {
		int left = !(i % 2);

		if (stop_all_threads == 1) return 0;
		switch (mode) {
			case MODE_PLAY:
			case MODE_PLAY_RECORD:
			case MODE_LMS_LEARN:
				if (feof(tx->wav_file)) return 0;

				fread(&d, 3, 1, tx->wav_file);
				tx->buf[i] = d;   			

				if (calibc0 < CALIBUFSIZE && left && do_calibration) 
					calibuf0[calibc0++] = d;

				if (left && mode == MODE_LMS_LEARN) {
					if (target->vcount < target->max && !target->ready) { 
						target->v[target->vcount++] = fix2fl(sext(d));
					} 
				}
				break;
			case MODE_PLAY_FIX:
				tx->buf[i] = fix_samples[i - offset];
				break;
			case MODE_PLAY_FIX_RECORD:
				w = (interval == 0 || fixtype == FIX_SINE) ? fix_samples[i - offset] : 0; 

				if (calibc0 < CALIBUFSIZE && left && do_calibration) 
					calibuf0[calibc0++] = w; 

				tx->buf[i] = w;
				break;
			case MODE_ANC:
			case MODE_EQUAL_FILTER:
					if (feof(tx->wav_file)) return 0;

					fread(&d, 3, 1, tx->wav_file);

					if (anc_stat == ANC_CALIBRATE_DELAY || 
						eq_stat == EQ_CALIBRATE_DELAY) {
						if (calibc0 < CALIBUFSIZE && left)
							calibuf0[calibc0++] = d;
					}

					if (left && eq_stat == EQ_LMS_LEARN) {
						if (target->vcount < target->max && !target->ready) { 
							target->v[target->vcount++] = fix2fl(sext(d));
						} 
					}

					tx->buf[i] = d;   			
				break;
			default:
				break;
		}

		i++; 
	}

	return ret;
}

static 
void *record_loop(void *runtime)
{
	struct pollfd pfd;
	int ret = 0;
	struct runtime *rx = (struct runtime *)runtime;

	if ((!rx->enable || rx->modnum != modnum) && mode != MODE_ANC) 
		goto _exit;

	/* synchronize with other threads */
	pthread_mutex_lock(&start_mutex);
	rx->filesize = 0;
	rx->count = 0;
	samplecount = 0;
	unstarted--;
	while (unstarted > 0) {
		pthread_mutex_unlock(&start_mutex);
		pthread_mutex_lock(&start_mutex);
	}
	pthread_mutex_unlock(&start_mutex);

	while (1) {
		if (!record_next_period(rx)) continue;

		pfd.fd = rx->fd;
		pfd.events = POLLIN | POLLRDNORM;

		ret = poll(&pfd, 1, 12); 

		if (ret > 0) {
			if (verbose) printf("RX polled\n");
			if (pfd.revents & POLLRDNORM) {
				receive_period(rx);
				rx->count++;
			}
		} else {
			printf("rx timeout\n");
			goto _exit;
		}

		switch (mode) {
			case MODE_LMS_LEARN:
				if (target->xcount >= target->max) {
					int j;
					lms_learn(target);

					for (j = 0; j < target->max; j++) {
						fwrite(&target->v[j], sizeof(double), 1, reff);
						fwrite(&target->x[j], sizeof(double), 1, resf);
					}

					lms_complete(target);
					printf("Learn Complete\n");
					stop_all_threads = 1;
				}
				break;
			case MODE_ANC:
				break;
			case MODE_EQUAL_FILTER:
				if (eq_stat == EQ_LMS_LEARN) {
					if (target->xcount >= target->max) {
						int j;
						lms_learn(target);

						for (j = 0; j < target->max; j++) {
							fwrite(&target->v[j], sizeof(double), 1, reff);
							fwrite(&target->x[j], sizeof(double), 1, resf);
						}

						lms_complete(target);
						printf("Learn Complete\n");
						stop_all_threads = 1;
					}
				}
				break;
			default:
				break;
		}

		if (stop_all_threads == 1) break;
	}

_exit:
	//printf("RX Thread #%d exited\n", rx->modnum);
	pthread_exit(NULL);
}


static 
void *play_loop(void *runtime)
{
	struct pollfd pfd;
	int ret = 0; 
	struct runtime *tx = (struct runtime *)runtime;
	
	if ((!tx->enable || tx->modnum != modnum) && mode != MODE_ANC) 
		goto _exit;

	/* synchronize with other threads */
	pthread_mutex_lock(&start_mutex);
	tx->count = 0;
	set_period(tx);
	unstarted--;
	while (unstarted > 0) {
		pthread_mutex_unlock(&start_mutex);
		pthread_mutex_lock(&start_mutex);
	}
	pthread_mutex_unlock(&start_mutex);

	while (1) {
		/* set_period() and play_next_period() must stay
			at the same tx->count */
		if (!play_next_period(tx, 0)) continue;
	
		pfd.fd = tx->fd;
		pfd.events = POLLOUT | POLLWRNORM;
		
		ret = poll(&pfd, 1, 12); 

		if (ret > 0) {
			if (verbose) printf("TX polled\n");
			if (pfd.revents & POLLWRNORM) {
				tx->count++;
				if (set_period(tx) == 0) break;
			}
		} else {
			printf("tx timeout\n"); break;
		}

		if (mode == MODE_PLAY_FIX_RECORD)
			interval = (interval == IMP_INTERVAL ? 0 : interval + 1);

		if (stop_all_threads == 1) break;
	} 

_exit:
	//printf("TX Thread #%d exited\n", tx->modnum);
	pthread_exit(NULL);
}


static 
int init_files(struct runtime *txrun, struct runtime *rxrun,
					const char *playback_filename, const char *record_filename)
{
	if (txrun->enable && !fixtype) {
		if((txrun->wav_file = open_file(playback_filename, "r")) == NULL) return -1;
		fseek(txrun->wav_file, 0x2C, SEEK_SET); 
	}

	if (rxrun->enable) {
		if((rxrun->wav_file = open_file(record_filename, "wb+")) == NULL) return -1;
		init_wav(rxrun->wav_file, 1);
	}

	return 0;
}

static
void close_files(struct runtime *txrun, struct runtime *rxrun) {
	if (txrun->wav_file) fclose(txrun->wav_file);
	if (rxrun->wav_file) fclose(rxrun->wav_file);
}

int main(int argc, char **argv)
{
	struct runtime *rxrun, *txrun;
	struct runtime *rxrun2, *txrun2;
	int i, j, coef;
	unsigned long *coefbuf;
	unsigned long *coefbuf2;
	FILE *coeff = NULL;
	pthread_t playback_thread[2];
	pthread_t record_thread[2];
	char playback_filename[32] = "";
	char record_filename[32] = "recorded.wav";
	char response_data[16] = "res";
	char response_data2[16] = "res2";
	char reference_data[16] = "ref"; 
	int nofilter = 0;
	struct sigaction sa;

	int option_index;
	int c;
	static const char short_options[] = "er::p:c:n:f:vd:m:NCaq";
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
		{"nofilter", 0, 0, 'N'},
		{"calibration", 0, 0, 'C'},
		{"anc", 0, 0, 'a'},
		{"equal", 0, 0, 'q'},
		{0, 0, 0, 0},
	};

	if (argc < 2) {
		printf("usage: %s <mode>\n", argv[0]);	
		return 1;
	}

	rxrun = malloc(sizeof(struct runtime));
	txrun = malloc(sizeof(struct runtime));
	rxrun2 = malloc(sizeof(struct runtime));
	txrun2 = malloc(sizeof(struct runtime));

	init_module(rxrun, txrun, 1);
	init_module(rxrun2, txrun2, 2);
	fixtype = 0;

	delaysize = 0;
	while ((c = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1) {
		switch (c) {
			case 'C':
				do_calibration = 1;
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
			case 'a':
				mode = MODE_ANC;
				strcpy(playback_filename, "imp.wav");
				anc_stat = ANC_CALIBRATE_DELAY;
				break;
			case 'q':
				mode = MODE_EQUAL_FILTER;
				strcpy(playback_filename, "imp.wav");
				eq_stat = EQ_CALIBRATE_DELAY;
				break;
			case 'e':
				mode = MODE_REALTIME;
				break;
			case 'r':
				mode = MODE_RECORD;
				break;
			case 'p':
				mode = MODE_PLAY;
				strcpy(playback_filename, optarg);
				break;
			case 'c':
				mode = MODE_PLAY_RECORD;
				if (!strcmp(optarg, "imp")) {
					mode = MODE_PLAY_FIX_RECORD;
					fixtype = FIX_IMPULSE;
					fix_data_generate(fixtype);
				} else {
					strcpy(playback_filename, optarg);
				}
				break;
			case 'n':
				mode = MODE_LMS_LEARN;
				strcpy(playback_filename, optarg);
				/* lms_init(tap, max, mu) */
				target = lms_init(COEF_COUNT, LMS_MAX_DATA, MU); 
				break;
			case 'N':
				nofilter = 1;
				break;
			case 'f':
				if (!strcmp(optarg, "imp")) {
					fixtype = FIX_IMPULSE;	
				} else {
					sinmult = atoi(optarg);
					fixtype = FIX_SINE;
				}
				mode = MODE_PLAY_FIX;
				fix_data_generate(fixtype);
				break;
			default:
				break;
		}
	}

	/* Open coefficient file */
	if((coeff = open_file("coef.txt", "r")) == NULL) return -1;

	//printf("Reading Coefficients.. \n");
	i = 0;
	while (!feof(coeff)) {
		fscanf(coeff, "%d\n", &coef);
		filter_taps[i] = coef;
		i++;
	}

	if (nofilter) {
		//printf("No Filter Mode\n");
		for(j = 0; j < COEF_COUNT; j++) {
			filter_taps[j] = (j == 1) ? 0x800000 : 0;
		} 
	}

	/* Allocate buffers */
	if(map_buffers(rxrun, txrun) < 0) return -1;
	if(map_buffers(rxrun2, txrun2) < 0) return -1;

	/* Map coefficient buffer */
	if (map_coef_buffer(&coefbuf, 1) < 0) return -1;
	if (map_coef_buffer(&coefbuf2, 2) < 0) return -1;

	/* Open devices */
	if(init_devices(rxrun, txrun, 1) < 0) return -1;
	if(init_devices(rxrun2, txrun2, 2) < 0) return -1;

	//printf("Setting Coefficients..\n");
	for (i = 0; i < COEF_COUNT; i++) {
		coefbuf[i] = filter_taps[i];
		coefbuf2[i] = filter_taps[i];
		//printf("%d\n", coefbuf2[i]);
	}

	/* Begin */
	init_mode(rxrun, txrun, modnum);
	init_mode(rxrun2, txrun2, modnum);

	if (reff == NULL || resf == NULL) {
		reff = open_file(reference_data, "wb+");
		resf = open_file(response_data, "wb+");
		resf2 = open_file(response_data2, "wb+");
	}

	if (mode == MODE_NONE) goto finish;

	enable_i2s(1);
	enable_i2s(2);

	/* Flush the last remaining RX & TX buffer */
	record_next_period(rxrun);
	record_next_period(rxrun2);
	play_next_period(txrun, 0);
	play_next_period(txrun2, 0);

	/* Main Loop */
	int exit_loop = 0; 
	while (1) {
		if (init_files(txrun, rxrun, playback_filename, record_filename) < 0) break;
		if (init_files(txrun2, rxrun2, playback_filename, record_filename) < 0) break;
	
		stop_all_threads = 0;
		pthread_mutex_lock(&start_mutex);
		unstarted = 0;

		launch_thread_sync(record_thread[0], record_loop, rxrun); 
		launch_thread_sync(record_thread[1], record_loop, rxrun2);
		launch_thread_sync(playback_thread[0], play_loop, txrun);
		launch_thread_sync(playback_thread[1], play_loop, txrun2);

		pthread_mutex_unlock(&start_mutex);

		memset(&sa, 0, sizeof(sa));
		sa.sa_flags = SA_NOCLDSTOP;
		sa.sa_handler = sigint_handler;
		sigaction(SIGINT, &sa, NULL);

		while (1) {
			/* just idle loop on main thread */
			usleep(1);
			if (stop_all_threads == 1) break;
			if (signal_received == SIGINT) 
				stop_all_threads = 1;
		}

		join_thread(record_thread[0], rxrun);
		join_thread(record_thread[1], rxrun2);
		join_thread(playback_thread[0], txrun);
		join_thread(playback_thread[1], txrun2);

		if (mode == MODE_ANC) {
			switch (anc_stat) {
				case ANC_CALIBRATE_DELAY:
					printf("ANC delay finished\n");
					delaysize = 0;
					txrun2->enable = 0;
					rxrun2->enable = 1;
					txrun->enable = 1;
					rxrun->enable = 0;
					anc_stat = ANC_CANCELLATION;
					exit_loop = 1;
					break;
				case ANC_CANCELLATION:
					break;
				case ANC_FINISH:
					break;
				default:
					break;
			}
		} else if (mode == MODE_EQUAL_FILTER) {
			switch (eq_stat) {
				case EQ_CALIBRATE_DELAY:
					printf("Equalizer LMS Learn started..\n");
					free(target);
					target = lms_init(COEF_COUNT, LMS_MAX_DATA, MU);
					strcpy(playback_filename, "imp.wav");
					eq_stat = EQ_LMS_LEARN;
					if (txrun->enable) close_files(txrun, rxrun);
					if (txrun2->enable) close_files(txrun2, rxrun2);
					delaysize -= 6; // a little calibration
					sleep(1);
					break;
				case EQ_LMS_LEARN:
					exit_loop = 1;
					break;
				default:
					break;	
			}
		} else exit_loop = 1;

		if (exit_loop) break;
	}

	if (mode == MODE_LMS_LEARN || mode == MODE_EQUAL_FILTER) {
		double *result;
		FILE * icoeff;

		result = malloc(sizeof(double) * COEF_COUNT);
		calc_filter_lsq(target->h, result);

		fclose(coeff);
		coeff = open_file("float_coef.txt", "w");
		for (i = 0; i < COEF_COUNT; i++) {
			fprintf(coeff, "%.7g\n", target->h[i]);
		}

		icoeff = open_file("coef.txt", "w");
		for (i = 0; i < COEF_COUNT; i++) {
			fprintf(icoeff, "%d\n", -1 * fl2fix24(result[i]));
		}

		printf("Learning finished.. \n");

		fclose(icoeff);
		free(result);
	}

	//if (mode == MODE_ANC) {
		for(i = 0; i < CALIBUFSIZE; i++) {
			fwrite(&calibuf0[i], sizeof(long), 1, reff);
			fwrite(&calibuf1[i], sizeof(long), 1, resf);
			fwrite(&calibuf2[i], sizeof(long), 1, resf2);
		}
	//}

	if (mode == MODE_RECORD || mode == MODE_PLAY_RECORD) {
		finalize_wav(rxrun->wav_file, rxrun->filesize);
		finalize_wav(rxrun2->wav_file, rxrun2->filesize);
	}

	close_files(txrun, rxrun);
	close_files(txrun2, rxrun2);
	if (reff) fclose(reff); 
	if (resf) fclose(resf);

finish:
	free(rxrun); free(txrun);
	free(rxrun2); free(txrun2);
	fclose(coeff);
	return 0;
}
