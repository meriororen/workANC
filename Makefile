all:
	gcc -g -o k konami.c orz_lms.c least_square.c -lm -lpthread -O3 -mcpu=cortex-a9 -mfpu=vfpv3
	gcc -g -o i2c i2c.c

lms:
	gcc -g -o lms learnlms.c orz_lms.c -lm
	gcc -g -o run runlms.c orz_lms.c -lm

conv:
	gcc -g -o tofl tofl.c -lm

rel:
	gcc -g -o kn k.c orz_lms.c -lm -lpthread

.PHONY: lms conv run
