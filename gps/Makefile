#CC=arm-linux-gnueabi-g++
CC=g++
CFLAGS=-I. -Wall -O2
LFLAGS=-lrt

OBJSTATIC = gps.o nmeap01.o


all: static  

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $< 

static: $(OBJSTATIC)
	ar rcs libgps.a $^

clean:
	rm -f *.o
	rm -f libgps.a
	rm -f libgps.so
