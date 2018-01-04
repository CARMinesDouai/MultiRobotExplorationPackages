CC = gcc -I/usr/include/libusb-1.0
GXX = g++
CXXFLAGS      = -pipe -O2 -Wall -W -D_REENTRANT -lusb-1.0 -lm
INCPATH = -I. -I/usr/include/libusb-1.0

all:  Palettes.o flir8k.o flir8k


Palettes.o: Palettes.cpp Palettes.h
	${CXX} -c ${CXXFLAGS} ${INCPATH} -o Palettes.o Palettes.cpp

flir8k.o: flir8k.c

flir8k: flir8k.o
	${CC} -o flir8k Palettes.o flir8k.o -lusb-1.0 -ljpeg -lm -Wall


clean:
	rm -f  Palettes.o flir8k.o flir8k
