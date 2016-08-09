TARGET = ev3control
OBJS = main.o net_udp.o misc.o modules.o

SHARED = ../lib/shared
INCLUDE = ../lib

CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -DEV3 -c 
CXX_FLAGS = -O2 -std=c++11 -Wall -DEV3 -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) -I $(INCLUDE)
LFLAGS = -Wall $(DEBUG)

$(TARGET) : $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) -o $(TARGET)

main.o : main.cpp $(SHARED)/misc.h $(SHARED)/net_udp.h 
	$(CXX) $(CXX_FLAGS) main.cpp

modules.o: modules.h modules.cpp $(SHARED)/misc.h
	$(CXX) $(CXX_FLAGS) modules.cpp
	
misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(CXX) $(CXX_FLAGS) $(SHARED)/misc.cpp
	
net_udp.o: $(SHARED)/net_udp.h $(SHARED)/net_udp.cpp $(SHARED)/misc.h
	$(CXX) $(CXX_FLAGS) $(SHARED)/net_udp.cpp

clean:
	\rm -f *.o $(TARGET)
