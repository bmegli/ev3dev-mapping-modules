OBJS = misc.o net_udp.o

CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -DEV3 -c 
CXX_FLAGS = -O2 -std=c++11 -Wall -DEV3 -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

all : $(OBJS)
	
misc.o : misc.h misc.cpp
	$(CXX) $(CXX_FLAGS) misc.cpp
	
net_udp.o : net_udp.h net_udp.cpp misc.h
	$(CXX) $(CXX_FLAGS) net_udp.cpp

clean:
	\rm -f *.o 
