# Makefile
#
# holozone GmbH Marcus Werner 2018 
CC = g++  
# Includefiles und lib-Pfade Zusatz -std=c++0x für ältere Compiler
OPT  = -O3  -Wall -std=c++11 
# Libraries
LIB = -lpthread 
# make all
all:  mmd

mmd: mmd.cpp mmd.h drive.h motordriver.h file_parameter.h Makefile
	$(CC) -o mmd mmd.cpp $(LIB) $(OPT)
