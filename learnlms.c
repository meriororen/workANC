#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <math.h>
#include <arpa/inet.h>
#include "orz_lms.h"

#include "konami.h"
#include "fop.h"

static lms_t *target;

#undef BUFSIZE
#define BUFSIZE 1024
#define TAPSIZE 37

#define FLOAT 0

static double ref[BUFSIZE];
static double res[BUFSIZE];

int main(int argc, char **argv) {
	int ret, i;
	FILE *fp1, *fp2;
	FILE *coeff = NULL;
	int b, x; int in_int = 0;
	unsigned int filesize;
	
	fp1 = NULL; fp2 = NULL; 

	if (argc < 4) {
		fprintf(stderr, "Usage: %s <ref file> <res file> <coef out file>\n",
					argv[0]);
		exit(1);
	}

	if ((fp1 = fopen(argv[1], "r")) == NULL) {
		fprintf(stderr, "Can't open \"%s\"\n", argv[1]);
		return 1;
	}

	if ((fp2 = fopen(argv[2], "r")) == NULL) {
		fprintf(stderr, "Can't open \"%s\"\n", argv[2]);
		return 1;
	}

	if ((coeff = fopen(argv[3], "w+")) == NULL) {
		fprintf(stderr, "Can't open \"%s\"\n", argv[3]);
		return 1;
	}


	for(i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-int")) {
			in_int = 1;
		}
	}

#if !FLOAT 
		/* skip to content */
		fseek(fp1, 0x2C, SEEK_SET);
		fseek(fp2, 0x2C, SEEK_SET);

		int r, s;
		for (i = 0; i < BUFSIZE; i++) {
			fread(&r, 3, 1, fp1);
			ref[i] = fix2fl(sext(r));
			fread(&s, 3, 1, fp2);
			res[i] = fix2fl(sext(s));
		}
#else
		/* skip to content */
		fseek(fp1, 0x50, SEEK_SET);
		fseek(fp2, 0x50, SEEK_SET);
	
		for (i = 0; i < BUFSIZE; i++) {
			fread(&ref[i], sizeof(double), 1, fp1);
			//printf("%.7g\n", ref[i]);
			fread(&res[i], sizeof(double), 1, fp2);
		}
#endif

	/* initialize */
	target = lms_init(TAPSIZE, BUFSIZE, MU);

	/* input reference */
	for (i = 0; i < BUFSIZE; i++) {
		target->x[target->xcount++] = ref[i];
	}

	/* input response */
	for (i = 0; i < BUFSIZE; i++) {
		target->v[target->vcount++] = res[i];
	}

	/* learn */
	lms_learn(target);
	lms_complete(target);

	if (coeff != NULL) printf("printing coefficient to file..\n");
	for (i = 0; i < TAPSIZE; i++) {
		//printf("%.7g -> %d\n", target->h[i], coef[i]);
		if (coeff != NULL) {
			if (in_int) fprintf(coeff, "%d\n", fl2fix26(target->h[i]));
			else fprintf(coeff, "%.20g\n", target->h[i]);
		}
	}

	fclose(coeff);
	fclose(fp1);
	fclose(fp2);
	return 0;
}
