CC = gcc
LD = ld
CFLAGS = -g -O3
CFLAGS2 = -mcpu=cortex-a9 -mfpu=vfpv3
LDFLAGS = -lm -lpthread

all: i2c konami

konami.o: konami.c konami.h orz_lms.h
	gcc -c $(CFLAGS) $^
orz_lms.o: orz_lms.c orz_lms.h
	gcc -c $(CFLAGS) $^
least_square.o: least_square.c konami.h
	gcc -c $(CFLAGS) $(CFLAGS2) $^
button.o: button.c
	gcc -c $(CFLAGS) $^

konami: orz_lms.o least_square.o button.o konami.o
	$(CC) -o k $^ $(LDFLAGS)

i2c: i2c.o
	$(CC) -o i2c $^


clean:
	rm -rf *.o

.PHONY: lms conv run konami i2c
