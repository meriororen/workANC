#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "orz_lms.h"
#include "fop.h"

#define BUFSIZE 1024
#define TAPSIZE 63

#define HEADER_SIZE

static int identify_wav(FILE *fp) {
	char buf;
	int i;
	
	fseek(fp, 0x14, SEEK_SET);
	fread(&buf, 1, 1, fp);

	return buf == 0x03; // is float
}

int main(int argc, char **argv) 
{
	FILE *noisyfp, *coeffp, *resultfp;
	int i; int skip = 0x2C;
	unsigned int filesize = 0;
	lms_t *target;

	if (argc < 2) {
		fprintf(stderr, "Usage: %s <noisy file> <coeff filename> <result file>\n", 
				 argv[0]);
		exit(1);
	}

	if ((noisyfp = fopen(argv[1], "r")) == NULL) {
		fprintf(stderr, "Can't open %s", argv[1]);
		exit(1);
	}

	if ((coeffp = fopen(argv[2], "r")) == NULL) {
		fprintf(stderr, "Can't open %s", argv[2]);
		exit(1);
	}

	if ((resultfp = fopen(argv[3], "wb+")) == NULL) {
		fprintf(stderr, "Can't open %s", argv[3]);
		exit(1);
	}

	if (identify_wav(noisyfp)) {
		skip = 0x50;
		float_init_wav(resultfp);
		printf("Is float\n");
	} else {
		int_init_wav(resultfp, 0);
		printf("Is int\n");
	}

	/* cut to the cheese */
	fseek(noisyfp, skip, SEEK_SET);

	target = lms_init(TAPSIZE, BUFSIZE, MU);

	target->ready = true;
	lms_complete(target);

	i = 0;
	double x;
	while(!feof(coeffp) && i < TAPSIZE) {
		fscanf(coeffp, "%lf\n", &x);
	//	printf("%.20g\n", x);
		target->h[i] = x;
		i++;
	}

	double d, b;
	while(!feof(noisyfp)) {
		fread(&d, sizeof(double), 1, noisyfp);	
		b = lms_run(target, d);
		filesize += fwrite(&b, 1, sizeof(double), resultfp);
	}

	if (skip == 0x50) {
		printf("Finalize float\n");
		float_finalize_wav(resultfp, filesize);
	} else {
		printf("Finalize int\n");
		int_finalize_wav(resultfp, filesize);
	}

	fclose(noisyfp); 
	fclose(coeffp); 
	fclose(resultfp);
}
