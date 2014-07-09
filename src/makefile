# compiler
COMPILER = g++

# flags
CFLAGS = -c -Wall

# folders
OBJS_FOLDER = ../objs
LIB_FOLDER = ../lib

all:$(OBJS_FOLDER)/qbmove_communications.o $(OBJS_FOLDER)/qbmovelibrary.o $(OBJS_FOLDER) $(LIB_FOLDER)
	ar rcs $(LIB_FOLDER)/libqbmove.a $(OBJS_FOLDER)/qbmove_communications.o $(OBJS_FOLDER)/qbmovelibrary.o
	ar rcs $(LIB_FOLDER)/libqbmove_comm.a $(OBJS_FOLDER)/qbmove_communications.o

$(OBJS_FOLDER)/qbmove_communications.o:qbmove_communications.cpp $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbmove_communications.cpp -o $(OBJS_FOLDER)/qbmove_communications.o

$(OBJS_FOLDER)/qbmovelibrary.o:qbmovelibrary.cpp $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbmovelibrary.cpp -o $(OBJS_FOLDER)/qbmovelibrary.o

clean:
ifeq "$(OS)"  "Windows_NT"
	erase /Q /S ..\objs
	erase /Q /S ..\lib
else
	rm -rf $(OBJS_FOLDER) $(LIB_FOLDER)
endif

$(OBJS_FOLDER):
	mkdir $(OBJS_FOLDER)

$(LIB_FOLDER):
	mkdir $(LIB_FOLDER)