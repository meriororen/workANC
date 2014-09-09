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
#define CALIBUFSIZE 512
#define LMS_MAX_DATA 60000

static int sinmult;
static int filter_taps[COEF_COUNT];
static int fix_samples[BUFSIZE/4]; 
static int fixtype = 0;
static int delaysize = 0;
static int interval = 0;
static int samplecount = 0;
static int stop_all_threads = 0;
static int signal_received;
static int do_calibration;
static int modnum = 1;
static int verbose = 0;
static int calibc0 = 0; 
static int calibc1 = 0;
static int calibc2 = 0;
static int calibuf0[CALIBUFSIZE];
static int calibuf1[CALIBUFSIZE];
static int calibuf2[CALIBUFSIZE];
static int nofilter = 0;
static lms_t *target;
static int unstarted = 0;
static FILE *reff;
static FILE *resf;
static FILE *resf2;
static unsigned extradelay = 0;
static int noanc = 0;
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
			if (i == 0) fix_samples[i] = IMP_DATA;
			else fix_samples[i] = 0;
		}
	}
}

static
int configure_i2s(struct module *m, int rx, int tx, unsigned delay, int mix)
{
#define CODEC_RX_SHORT	0x10
#define CODEC_TX_SHORT 0x20

	unsigned long i2s_base;
	unsigned long val = 0;

	/* All defaults to MEM (0) if not changed 
		otherwise */

	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open /dev/mem\n");
		return -1;
	}

	if (m->modnum == 1) i2s_base = CODEC1_I2S_BASE;
	else i2s_base = CODEC2_I2S_BASE;
	
	m->i2sconfig = mmap(NULL, PAGE_SIZE, PROT_WRITE | PROT_READ,
					MAP_SHARED, fd, i2s_base);

	if (m->i2sconfig == MAP_FAILED) {
		printf("Failed to map I2S config\n");
		return -1;
	}
	
	val = ((delay & 0xFF) << 6)| (tx << 5) | (tx << 4) | (mix << 3) | 1;

	m->i2sconfig[4] = val;

	close(fd);
	return 0;
}

static 
void init_mode(struct module *m)
{
	struct runtime *rx = m->rx;
	struct runtime *tx = m->tx;
	
	configure_i2s(m, 0, 0, 0, 0);
	
	switch (mode) {
		case MODE_PLAY:
		case MODE_PLAY_FIX:
			if (modnum == m->modnum)
				 tx->enable = 1;
			break;
		case MODE_RECORD:
		case MODE_REALTIME:
			if (modnum == m->modnum)
				rx->enable = 1;
			break;
		case MODE_PLAY_RECORD:
		case MODE_PLAY_FIX_RECORD:
		case MODE_LMS_LEARN:
		case MODE_EQUAL_FILTER:
			if (modnum == m->modnum) {
				tx->enable = 1;
				rx->enable = 1;
			}
			break;
		case MODE_ANC:
			if (anc_stat == ANC_CALIBRATE_DELAY) {
				if (m->modnum == 2) {
					tx->enable = 1; 
					configure_i2s(m, 1, 0, 0, 0); /* rx short, tx mem */
				} 
				if (m->modnum == 1) {	 /* codec1 */
					rx->enable = 1;
					configure_i2s(m, 0, 1, 0, 0); /* rx mem, tx short */
				}
			} else if (anc_stat == ANC_CANCELLATION) {
				printf("Not implemented (yet)\n");
			}
		default:
			break;
	}

	if (mode != MODE_EQUAL_FILTER && mode != MODE_RECORD) {
		if (m->modnum == 2) configure_i2s(m, 1, 0, 0, 0);
		if (m->modnum == 1) configure_i2s(m, 0, 1, extradelay, 0); /* rx mem, tx short */
	}
}

static 
int map_buffers(struct module *m)
{
	struct runtime *rx = m->rx;
	struct runtime *tx = m->tx;

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
int map_coef_buffer(struct module *m)
{
	unsigned long coef_base;

	int fd = open("/dev/mem", O_RDWR);
	if (!fd) {
		printf("Can't open /dev/mem\n");
		return -1;
	}

	if (m->modnum == 1) coef_base = CODEC1_COEF_BASE;
	else coef_base = CODEC2_COEF_BASE;
	
	m->coefbuf = mmap(NULL, PAGE_SIZE, PROT_WRITE | PROT_READ,
					MAP_SHARED, fd, coef_base);

	if (m->coefbuf == MAP_FAILED) {
		printf("Failed to map FIR coefficient buffer\n");
		return -1;
	}

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
int init_devices(struct module *m) 
{
	char devpath[16];
	struct runtime *rx = m->rx;
	struct runtime *tx = m->tx;
	
	sprintf(devpath, "/dev/c%dcapt", m->modnum);

	rx->fd = open(devpath, O_RDWR);
	if (rx->fd < 0) {
		printf("Can't open %s\n", devpath);
		return -1;
	} 

	sprintf(devpath, "/dev/c%dplay", m->modnum);

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
int lms_calibrate_delay(double *res, int max) {
	int j;

	for (j = 0; j < max; j++) {
		if (res[j] > 0.1) break;
	}

	printf("LMS delay: %d\n", j - 2);

	return j -= 2; // a little calibration
}

static 
int anc_calibrate_delay(void) {
	int i;
	int m1d = 0, m2d = 0;

	for (i = 0; i < CALIBUFSIZE; i++) {
		double r = fix2fl(calibuf2[i]);
		if (r > 0.10 && m2d == 0) m2d = i;
		if (r < -0.30) { m1d = i; break; }
	}

	printf("extra delay: %d\n", 100 - (2 * abs(m2d - m1d)));

	return 100 - (2 * (m2d - m1d));
}

static 
int calibrate_delay(void)
{
	int j;
	int delay = 0;
	
	for (j = 0; j < CALIBUFSIZE; j++) {
		if (calibuf0[j] == IMP_DATA) delay = 0;
		//int d1 = calibuf1[j] - calibuf1[j-1];
		//if (d1 > DTHRSHOLD) {
		if (calibuf1[j] > DTHRSHOLD) {
			printf("delay = %d\n", delay);
			break;
		}
		delay++;
	}

	return delay;
}

static 
int calibrate_delay2(void)
{
	int j;
	int delay1 = 0, delay2 = 0;
	float dist = 0.;
	
	for (j = 0; j < CALIBUFSIZE; j++) {
		//if (calibuf0[j] == IMP_DATA) delay1 = 0;
		int d1 = calibuf1[j] - calibuf1[j-1];
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
void receive_period(struct module *m)
{
	int offset, i;
	char buf[3];
	struct runtime *rx = m->rx;
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
			case MODE_RECORD:
			case MODE_PLAY_RECORD:
			case MODE_PLAY_FIX_RECORD:
				if (do_calibration) {
					if (calibc1 < CALIBUFSIZE) {
						if (left) calibuf1[calibc1++] = sext(x);
					} else {
						if ((delaysize = calibrate_delay()) > 0) {
							stop_all_threads = 1;
							return;
						}
					}
				}
				if (!waitfordelay)
					rx->filesize += fwrite(buf, 1, sizeof(buf), rx->wav_file);
				break;
			case MODE_EQUAL_FILTER:
				switch (eq_stat) {
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
						if (calibc2 < CALIBUFSIZE && m->modnum == 1) {
							if (left) calibuf2[calibc2++] = sext(x);
						} else {
							extradelay = anc_calibrate_delay();
							stop_all_threads = 1;
							return;
						}

						rx->filesize += fwrite(buf, 1, sizeof(buf), rx->wav_file);
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
int set_period(struct module *m)
{
	long l, r, w;
	int i, offset, ret; 
	struct runtime *tx = m->tx;

	if (tx->count == BUFCOUNT) i = offset = 0;
	else i = offset = tx->count * BUFSIZE/4;

	ret = 1; l = 0; r = 0;
	while (i < offset + BUFSIZE/4) {
		if (stop_all_threads == 1) return 0;

		if (tx->wav_file) {
			if (feof(tx->wav_file)) return 0;
			if (tx->stereo) {
				fread(&r, 3, 1, tx->wav_file);
				fread(&l, 3, 1, tx->wav_file);
			} else {
				fread(&l, 3, 1, tx->wav_file);
			}
		}

		switch (mode) {
			case MODE_PLAY:
			case MODE_PLAY_RECORD:
			case MODE_LMS_LEARN:
				tx->buf[i] = l;   			

				if (calibc0 < CALIBUFSIZE && do_calibration) 
					calibuf0[calibc0++] = l;

				if (mode == MODE_LMS_LEARN) {
					if (target->vcount < target->max && !target->ready) { 
						target->v[target->vcount++] = fix2fl(sext(l));
					} 
				}
				break;
			case MODE_PLAY_FIX:
				tx->buf[i] = fix_samples[i - offset];
				break;
			case MODE_PLAY_FIX_RECORD:
				w = (interval == 0 || fixtype == FIX_SINE) ? fix_samples[i - offset] : 0; 

				if (calibc0 < CALIBUFSIZE && do_calibration) 
					calibuf0[calibc0++] = w; 

				tx->buf[i] = w;
				break;
			case MODE_ANC:
			case MODE_EQUAL_FILTER:
					if (anc_stat == ANC_CALIBRATE_DELAY)
						if (calibc0 < CALIBUFSIZE)
							calibuf0[calibc0++] = l;

					if (eq_stat == EQ_LMS_LEARN) {
						if (target->vcount < target->max && !target->ready) { 
							target->v[target->vcount++] = fix2fl(sext(l));
						} 
					}

					tx->buf[i] = l;   			
				break;
			default:
				break;
		}

		i++; 
	}

	return ret;
}

static 
void *record_loop(void *module)
{
	struct pollfd pfd;
	int ret = 0;
	struct module *m = (struct module *)module;
	struct runtime *rx = m->rx;

	if ((!rx->enable || m->modnum != modnum) && mode != MODE_ANC) 
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
		rx->count++;
		if (!record_next_period(rx)) continue;

		pfd.fd = rx->fd;
		pfd.events = POLLIN | POLLRDNORM;

		ret = poll(&pfd, 1, 12); 

		if (ret > 0) {
			if (verbose) printf("RX polled\n");
			if (pfd.revents & POLLRDNORM) receive_period(m);
		} else {
			printf("rx timeout\n");
			goto _exit;
		}

		switch (mode) {
			case MODE_LMS_LEARN:
				if (target->xcount >= target->max) {
					int j, del;
					double *temp; 
					
					del = lms_calibrate_delay(target->x, target->max);
					target->max -= del;
					temp = target->x;
					target->x = &target->x[del];

					for (j = 0; j < target->max; j++) {
						fwrite(&target->v[j], sizeof(double), 1, reff);
						fwrite(&target->x[j], sizeof(double), 1, resf);
					}

					printf("LMS Learning..\n");

					lms_learn(target);

					target->x = temp;

					lms_complete(target);

					printf("Learn Complete\n");
					stop_all_threads = 1;
				}
				break;
			case MODE_ANC:
				if (anc_stat == ANC_CALIBRATE_DELAY)
				
				if (anc_stat == ANC_CANCELLATION) {
					m->m->tx->current = rx->current;
					m->m->tx->enable = 1;
					play_next_period(m->m->tx, 1);
				}
				break;
			case MODE_EQUAL_FILTER:
				if (eq_stat == EQ_LMS_LEARN) {
					if (target->xcount >= target->max) {
						int j, del;
						double *temp; 

						for (j = 0; j < target->max; j++) {
							fwrite(&target->v[j], sizeof(double), 1, reff);
							fwrite(&target->x[j], sizeof(double), 1, resf);
						}

						del = lms_calibrate_delay(target->x, target->max);
						target->max -= del;
						temp = target->x;
						target->x = &target->x[del];
							
						printf("LMS Learning..\n");
						lms_learn(target);

						target->x = temp;
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
	printf("RX Thread #%d exited\n", m->modnum);
	pthread_exit(NULL);
}

static 
void *play_loop(void *module)
{
	struct pollfd pfd;
	int ret = 0; 
	struct module *m = (struct module *)module;
	struct runtime *tx = m->tx;
	
	if ((!tx->enable || m->modnum != modnum) && mode != MODE_ANC) 
		goto _exit;

	/* synchronize with other threads */
	pthread_mutex_lock(&start_mutex);
	tx->count = 0;
	set_period(m);
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
				if (set_period(m) == 0) break;
			}
		} else {
			printf("tx timeout\n"); break;
		}

		if (mode == MODE_PLAY_FIX_RECORD)
			interval = (interval == IMP_INTERVAL ? 0 : interval + 1);

		if (stop_all_threads == 1) break;
	} 

_exit:
	printf("TX Thread #%d exited\n", m->modnum);
	pthread_exit(NULL);
}

static
int set_filter_coef(struct module *m)
{
	int i;

	for(i = 0; i < COEF_COUNT; i++) {
		if (nofilter) {
			m->coefbuf[i] = (i == 0) ? (0x800000) : 0;
		} else {
			m->coefbuf[i] = filter_taps[i];
		}
	}
}

int monitor_button(int num, struct module *m1, struct module *m2)
{
	static char previous_value = 49;
	char value;

	lseek(button_fd, 0, SEEK_SET);

	if (read(button_fd, &value, 1) < 0) {
		printf("Can't read button file\n");
		return -1;
	}

	if (previous_value == 48 && value == 49) {
		//nofilter = nofilter ? 0 : 1;
		//set_filter_coef(m2);
		noanc = noanc ? 0 : 1;
		if (noanc) {
			configure_i2s(m1, 0, 0, 0, 0);
		} else {
			configure_i2s(m1, 0, 1, extradelay, 0); /* rx mem, tx short */
		}
	}

	previous_value = value;

	return 0;
}

static 
int init_files(struct module *m, const char *playback_filename, const char *record_filename)
{
	char stereo;
	struct runtime *rx = m->rx;
	struct runtime *tx = m->tx;

	if (tx->enable && !fixtype) {
		if((tx->wav_file = open_file(playback_filename, "r")) == NULL) return -1;

		/* check stereo */
		fseek(tx->wav_file, 22, SEEK_SET);
		fread(&stereo, sizeof(char), 1, tx->wav_file);
		
		tx->stereo = stereo == 0x02 ? 1 : 0;
		printf("%s\n", tx->stereo ? "stereo" : "mono");
	
		/* to data */
		fseek(tx->wav_file, 0x2C, SEEK_SET); 
	}

	if (rx->enable) {
		if((rx->wav_file = open_file(record_filename, "wb+")) == NULL) return -1;
		init_wav(rx->wav_file, 1);
	}

	return 0;
}

static
void close_files(struct module *m) 
{
	if (m->tx->wav_file) fclose(m->tx->wav_file);
	if (m->rx->wav_file) fclose(m->rx->wav_file);
}

int main(int argc, char **argv)
{
	struct module *m1, *m2;
	int i, j, coef;
	FILE *coeff = NULL;
	pthread_t playback_thread[2];
	pthread_t record_thread[2];
	char playback_filename[32] = "";
	char record_filename[32] = "recorded.wav";
	char reference_data[16] = "ref"; 
	char response_data[16] = "res";
	char response_data2[16] = "res2";
	char btn_path[50];
	struct sigaction sa;

	int option_index;
	int c;
	static const char short_options[] = "er::p:c:n:f:vd:m:NCa:qx:";
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
		{"anc", 1, 0, 'a'},
		{"equal", 0, 0, 'q'},
		{"delaym", 1, 0, 'x'},
		{0, 0, 0, 0},
	};

	if (argc < 2) {
		printf("usage: %s <mode>\n", argv[0]);	
		return 1;
	}

	m1 = malloc(sizeof(struct module));
	m2 = malloc(sizeof(struct module));

	m1->modnum = 1; m2->modnum = 2;
	m1->m = m2; m2->m = m1;

	m1->rx = malloc(sizeof(struct runtime));
	m1->tx = malloc(sizeof(struct runtime));
	m2->rx = malloc(sizeof(struct runtime));
	m2->tx = malloc(sizeof(struct runtime));

	init_module(m1);
	init_module(m2);
	fixtype = 0;

	do_calibration = 0;
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
				strcpy(playback_filename, optarg);
				anc_stat = ANC_CALIBRATE_DELAY;
				extradelay = 100;
				//anc_stat = ANC_CANCELLATION;
				break;
			case 'q':
				mode = MODE_EQUAL_FILTER;
				strcpy(playback_filename, "imp.wav");
				target = lms_init(COEF_COUNT, LMS_MAX_DATA, MU);
				eq_stat = EQ_LMS_LEARN;
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
			case 'x':
				if (extradelay == 0) 
					extradelay = atoi(optarg);
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


	/* Allocate buffers */
	if(map_buffers(m1) < 0) return -1;
	if(map_buffers(m2) < 0) return -1;

	/* Map coefficient buffer */
	//if (map_coef_buffer(m1) < 0) return -1;
	if (map_coef_buffer(m2) < 0) return -1;

	/* Open devices */
	if(init_devices(m1) < 0) return -1;
	if(init_devices(m2) < 0) return -1;

	/* Open coefficient file */
	if((coeff = open_file("coef.txt", "r")) == NULL) return -1;

	//printf("Reading Coefficients.. \n");
	i = 0;
	while (!feof(coeff)) {
		fscanf(coeff, "%d\n", &coef);
		filter_taps[i] = coef;
		i++;
	}

	set_filter_coef(m2);

	/* Begin */
	init_mode(m1);
	init_mode(m2);

	if (reff == NULL || resf == NULL) {
		reff = open_file(reference_data, "wb+");
		resf = open_file(response_data, "wb+");
		resf2 = open_file(response_data2, "wb+");
	}

	if (mode == MODE_NONE) goto finish;

	/* Flush the last remaining RX & TX buffer */
	record_next_period(m1->rx);
	record_next_period(m2->rx);
	play_next_period(m1->tx, 0);
	play_next_period(m2->tx, 0);

	/* Button initialization */
	sprintf(btn_path, "%s/gpio%d/value", GPIO_PATH, BUTTON0);
	if (access(btn_path, F_OK) < 0) btn_gpio_init();

	sprintf(btn_path, "%s/gpio%d/value", GPIO_PATH, BUTTON0);

	button_fd = open(btn_path, O_RDONLY);
	if (button_fd < 0) { 
		printf("Cannot open BUTTON0\n"); 
		exit(EXIT_FAILURE);
	}

	/* I2C Config */
	int i2c_fd = open("/dev/i2c-0", O_RDWR);
	if (i2c_fd < 0) { fprintf(stderr, "Can't open i2c\n"); exit(1); }

	if (mode == MODE_EQUAL_FILTER) adjust_mic_vol(i2c_fd, 1, 0x25);

	/* Main Loop */
	int exit_loop = 0; 
	int u;
	while (1) {
		if (init_files(m1, playback_filename, record_filename) < 0) break;
		if (init_files(m2, playback_filename, record_filename) < 0) break;
	
		stop_all_threads = 0;
		pthread_mutex_lock(&start_mutex);
		unstarted = 0;

		launch_thread_sync(record_thread[0], record_loop, rx, m1); 
		launch_thread_sync(record_thread[1], record_loop, rx, m2);
		launch_thread_sync(playback_thread[0], play_loop, tx, m1);
		launch_thread_sync(playback_thread[1], play_loop, tx, m2);

		pthread_mutex_unlock(&start_mutex);

		memset(&sa, 0, sizeof(sa));
		sa.sa_flags = SA_NOCLDSTOP;
		sa.sa_handler = sigint_handler;
		sigaction(SIGINT, &sa, NULL);

		while (1) {
			/* just idle loop on main thread */
			if (stop_all_threads == 1) break;
			if (signal_received == SIGINT) 
				stop_all_threads = 1;
			monitor_button(BUTTON0, m1, m2);
		}

		join_thread(record_thread[0], rx, m1);
		join_thread(record_thread[1], rx, m2);
		join_thread(playback_thread[0], tx, m1);
		join_thread(playback_thread[1], tx, m2);

		if (mode == MODE_ANC) {
			switch (anc_stat) {
				case ANC_CALIBRATE_DELAY:
					printf("ANC delay finished\n");
					delaysize = 0;
					m2->tx->enable = 0;
					m2->rx->enable = 1;
					m1->tx->enable = 0;
					m1->rx->enable = 0;
					anc_stat = ANC_CANCELLATION;
					if (modnum == m1->modnum) finalize_wav(m1->rx);
					exit_loop = 1;
					break;
				case ANC_CANCELLATION:
					exit_loop = 1;
					break;
				case ANC_FINISH:
					break;
				default:
					break;
			}
		} else if (mode == MODE_EQUAL_FILTER) {
			switch (eq_stat) {
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

	if (mode != MODE_EQUAL_FILTER) {
		double a0, a1, a2;
		for(i = 0; i < CALIBUFSIZE; i++) {
			a0 = fix2fl(calibuf0[i]); fwrite(&a0, sizeof(double), 1, reff);
			a1 = fix2fl(calibuf1[i]); fwrite(&a1, sizeof(double), 1, resf);
			a2 = fix2fl(calibuf2[i]); fwrite(&a2, sizeof(double), 1, resf2);
		}
	}

	if (mode == MODE_RECORD || mode == MODE_PLAY_RECORD) {
		if (modnum == m1->modnum) finalize_wav(m1->rx);
		if (modnum == m2->modnum) finalize_wav(m2->rx);
	}

	close_files(m1);
	close_files(m2);
	if (reff) fclose(reff); 
	if (resf) fclose(resf);
	if (resf2) fclose(resf2);

finish:
	free(m1->rx); free(m1->tx);
	free(m2->rx); free(m2->tx);
	free(m1); free(m2);
	fclose(coeff);
	return 0;
}
