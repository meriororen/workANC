all:
	gcc -g -o k konami.c orz_lms.c -lm -lpthread
	gcc -g -o i2c i2c.c

lms:
	gcc -g -o lms learnlms.c orz_lms.c -lm
	gcc -g -o run runlms.c orz_lms.c -lm

conv:
	gcc -g -o tofl tofl.c -lm


.PHONY: lms conv run
