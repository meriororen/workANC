#include "lms.h"

static void normalize(int *target, int num);
static void lms(lms_t *target);

#define abs(x) \
	(x = x < 0 ? -1 * x : x)

void normalize(int *target, int num)
{
	int i;
	int max = 0;
	for (i = 0; i < num; i++) {
		if (max < abs(target[i])) {
			 max = abs(target[i]);
		}
	}
	if (max == 0) {
		printf("Can't normalize!\n");
	}
	for (i = 0; i < num; i++) {
		target[i] /= max;
	}
}

lms_t *lms_init(int taps, int max, int mu)
{
	lms_t *ret = NULL;

	ret = (lms_t *)malloc(sizeof(lms_t));
	if (!ret) goto exit;
	
	memset(ret, 0, sizeof(lms_t));

	ret->tap = taps;

	ret->h = (int *)malloc(sizeof(int) * taps);
	if (!ret->h) goto error;

	memset(ret->h, 0, sizeof(int) * taps);

	ret->x =(int *)malloc(sizeof(int) * max);
	if (!ret->x) goto error;

	memset(ret->x, 0, sizeof(int) * max);

	ret->v = (int *)malloc(sizeof(int) * max);
	if (!ret->v) goto error;

	memset(ret->v, 0, sizeof(int) * max);

	ret->y = (int *)malloc(sizeof(int) * max);
	if (!ret->y) goto error;

	memset(ret->y, 0, sizeof(int) * max);
	
	ret->e = (int *)malloc(sizeof(int) * max);
	if (!ret->e) goto error;

	memset(ret->e, 0, sizeof(int) * max);

	if (mu != -100) {
		ret->mu = mu;
	} else {
		ret->mu = MU;
	}

	ret->max = max;
	ret->count = 0;
	ret->ready = false;
	goto exit;

error:
	if (ret->y != NULL) free(ret->y);
	if (ret->x != NULL) free(ret->x);
	if (ret->v != NULL) free(ret->v);
	if (ret->h != NULL) free(ret->h);
	if (ret->e != NULL) free(ret->e);
	if (ret) free(ret);

exit:
	return ret;
}

bool lms_learn(lms_t *target)
{
	normalize(target->x, target->max);
	normalize(target->v, target->max);

	lms(target);
	target->ready = true;
	target->count = 0;

	return true;
}

void lms_complete(lms_t *target)
{
	if (target->ready == true) {
		free(target->x);
		free(target->v);
		free(target->y);
		free(target->e);
	}
	target->buf = (int *)malloc(sizeof(int) * target->tap * 2);
	target->count = 0;
}

void lms(lms_t *target)
{
	int i, k;
	long long acc;
		
	for (i = target->tap; i < target->max; i++) {
		for (k = 0; k < target->tap; k++) {
			acc += target->h[k] * target->v[i - k];
			target->y[i] = acc >> 32;
		}
		target->e[i] = target->x[i] - target->y[i];
		for (k = 0; k < target->tap; k++) {
			acc += target->mu * target->e[i] * target->v[i - k];
			target->h[k] = acc >> 32;
		}
	}
}

int lms_run(lms_t *target, int input)
{
	int ret = 0;
	int i;

	if (target->ready == false) return ret;
	target->buf[target->count] = input;
	target->buf[target->count + target->tap] = input;

	target->count++;
	if (target->count == target->tap) {
		target->count = 0;
	}

	for (i = 0; i < target->tap; i++) {
		ret += (target->h[i] * target->buf[target->count + i]);
	}

	return ret;
}
