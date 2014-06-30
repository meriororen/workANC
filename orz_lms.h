#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#ifndef _ORZ_lms_H_
#define _ORZ_lms_H_

#define MU           0.01


typedef struct {
	bool	ready;	
	int		tap;	
	int		max;	
	int		xcount;
	int		vcount;
	
	double	mu;
   double	*x;	 
   double	*v;	
   double	*y;
   double	*e;	
	double	*h;	
	double	*buf;	
} lms_t;

lms_t * lms_init(int taps,int max,double mu);

bool lms_learn(lms_t * target);

void lms_complete(lms_t * target);

double lms_run(lms_t * target,double input);


#endif /* _ORZ_lms_H_ */
