#include "orz_lms.h"

static double normalize (double * target,int num);
static void lms (lms_t * target);

#define abs(x) (x < 0 ? -1 * x : x)

double normalize (double * target,int num)
{
	int i;
	double max=0.0;
	for (i=0;i<num;i++) {
//		printf("%.7g\n", target[i]);
		if (max<abs(target[i])) {
			max=abs(target[i]);
		}
	}
	for (i=0;i<num;i++) {
		target[i] /= max;
		
	}
	return max;
}


lms_t * lms_init(int taps, int max, double mu) 
{
	lms_t * ret=NULL;
	
	ret = (lms_t * )malloc(sizeof(lms_t));
	if (ret ==NULL) goto exit;
	memset(ret,0,sizeof(lms_t));
	
	ret->tap = taps;

	ret->h = (double*)malloc(sizeof(double)*taps);
	if (ret->h==NULL) {
		goto error;
	}
	memset(ret->h,0,sizeof(double)*taps);

	ret->x = (double*)malloc(sizeof(double)*max);
	if (ret->x==NULL) {
		goto error;
	}
	memset(ret->x,0,sizeof(double)*taps);

	ret->v = (double*)malloc(sizeof(double)*max);
	if (ret->v==NULL) {
		goto error;
	}
	memset(ret->v,0,sizeof(double)*taps);

	ret->y = (double*)malloc(sizeof(double)*max);
	if (ret->y==NULL) {
		goto error;
	}
	memset(ret->y,0,sizeof(double)*taps);

	ret->e = (double*)malloc(sizeof(double)*max);
	if (ret->e==NULL) {
		goto error;
	}
	memset(ret->e,0,sizeof(double)*taps);

	if (mu!=-1.0) {
		ret->mu = mu;
	} else {
		ret->mu = MU;
	}
	ret->max = max;
	ret->xcount = 0;
	ret->vcount = 0;
	ret->ready=false;
	goto exit;

error:	
	if (ret->y!=NULL) free(ret->y);
	if (ret->x!=NULL) free(ret->x);
	if (ret->v!=NULL) free(ret->v);
	if (ret->h!=NULL) free(ret->h);
	if (ret->e!=NULL) free(ret->e);
	if (ret) free(ret);

exit:
	return ret;
}

bool lms_learn(lms_t * target)
{
	bool ret = false;

	normalize (target->x,target->max);
	normalize (target->v,target->max);

	lms(target);
	target->ready = true;
	target->xcount=0;
	target->vcount=0;
	ret=true;

	return ret;
}

void lms_complete(lms_t * target)
{
	if (target->ready==true) {
		free(target->x);
		free(target->v);
		free(target->y);
		free(target->e);
	}
	target->buf = (double*)malloc(sizeof(double)*target->tap*2);
	target->xcount=0;
	target->vcount=0;
	return;
}

void lms (lms_t * target)
{
	int i, k;

	for (i = target->tap; i < target->max; i++) {
		for (k = 0; k < target->tap; k++) {
			target->y[i] += target->h[k] * target->v[i - k];
		}
		target->e[i] = target->x[i] - target->y[i];
		for (k = 0; k < target->tap; k++) {
			target->h[k] += target->mu * target->e[i] * target->v[i - k];
		}
	}
}

double lms_run(lms_t * target,double input)
{
	double ret=0.0;
	int i;

	if (target->ready==false) {
		goto exit;
	}
	target->buf[target->xcount] = input;
	target->buf[target->xcount + target->tap] = input;

	target->xcount++;
	if (target->xcount == target->tap) {
		target->xcount = 0;
	}

	for (i=0; i< target->tap; i++) {
		ret = ret + ( target->h[i] * target->buf[target->xcount + i]);
	}


exit:
	return ret;
}



