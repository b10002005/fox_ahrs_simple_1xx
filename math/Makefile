
CC = g++
LFLAGS=-lrt
OBJSTATIC =simple_matrix.o 



CFLAGS = -I. -Wall -Wunused -static	


all: static

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<  

static: $(OBJSTATIC)
	ar rcs libosm-matrix.a $^
	

clean: 
	rm -f *.o 
	rm -f *.o
	rm -f libosm-matrix.a
	rm -f libosm-matrix.so
