#define MU 100
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

typedef struct {
	bool ready;				// learning complete flag
	int tap;					// tap number
	int max; 				// learning data size
	int count; 				// learning data counter
	
	int norm_ref;			// normalized reference
	int norm_res;			// normalized response
	
	int mu;
	int *x; 					// reference
	int *v;					// response
	int *y;					// desired
	int *e;					// learned error
	int *h; 					// coefficient
   int *buf;				// for scratch
} lms_t;

// initialization
lms_t *lms_init(int taps, int max, int mu);

// learning data
bool lms_learn(lms_t *target);

void lms_complete(lms_t *target);

int lms_run(lms_t * target, int input);
