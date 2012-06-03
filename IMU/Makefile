#CC=arm-linux-gnueabi-g++
CC=g++
CFLAGS=-I. -Wall -O2
LFLAGS=-lrt

OBJSTATIC = iduec.o daisysette.o 


all: static  

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

static: $(OBJSTATIC)
	ar rcs libiduecdev.a $^

clean:
	rm -f *.o
	rm -f libiduecdev.a
	rm -f libiduecdev.so
