# compiler
COMPILER = gcc

# flags
CFLAGS = -c -Wall

all:objs/qbmove_packages.o objs/qbmove_communications.o objs lib
	ar rcs lib/libqbmove.a objs/*.o
	cp qbmove_communications.h qbmove_packages.h lib
    
objs/qbmove_packages.o:qbmove_packages.c objs
	$(COMPILER) $(CFLAGS) qbmove_packages.c -o objs/qbmove_packages.o

objs/qbmove_communications.o:qbmove_communications.c objs
	$(COMPILER) $(CFLAGS) qbmove_communications.c -o objs/qbmove_communications.o

clean:
	rm -rf *.o bin objs lib

objs:
	mkdir objs

lib:
	mkdir lib