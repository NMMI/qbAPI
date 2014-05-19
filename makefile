# compiler
COMPILER = g++

# flags
CFLAGS = -c -Wall

all:objs/qbmove_communications.o objs/qbmovelibrary.o objs lib
	ar rcs lib/libqbmove.a objs/qbmove_communications.o objs/qbmovelibrary.o
	ar rcs lib/libqbmove_comm.a objs/qbmove_communications.o
	cp qbmove_communications.h lib

objs/qbmove_communications.o:qbmove_communications.c objs
	$(COMPILER) $(CFLAGS) qbmove_communications.c -o objs/qbmove_communications.o

objs/qbmovelibrary.o:qbmovelibrary.cpp objs
	$(COMPILER) $(CFLAGS) qbmovelibrary.cpp -o objs/qbmovelibrary.o

clean:
	rm -rf *.o bin objs lib

objs:
	mkdir objs

lib:
	mkdir lib