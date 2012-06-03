#CC=arm-linux-gnueabi-g++
CC=g++
CFLAGS=-I. -Wall -O2
LFLAGS=-lrt

OBJSTATIC = calibrazione_acc_mag0_data.o calibrazione_acc_mag0.o rtGetInf.o rtGetNaN.o rt_nonfinite.o


all: static 

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

static: $(OBJSTATIC)
	ar rcs libdcalmag.a $^

clean:
	rm -f *.o
	rm -f libdcalmag.a
	rm -f libdcalmag.so
