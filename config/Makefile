#CC=arm-linux-gnueabi-g++
CC=g++
CFLAGS=-I. -Wall -O2
LFLAGS=-lrt

OBJSTATIC = loadconf.o


all: static  

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

static: $(OBJSTATIC)
	ar rcs libloadconf.a $^

clean:
	rm -f *.o
	rm -f libloadconf.a
	rm -f libloadconf.so
