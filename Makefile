all:
	gcc -g -o k konami.c lms.c -lm -lpthread
	gcc -g -o i2c i2c.c
