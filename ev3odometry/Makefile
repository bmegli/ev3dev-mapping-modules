TARGET = ev3odometry
SHARED = ../lib/shared
EV3DEV = ../lib/ev3dev-lang-cpp
OBJS = main.o $(EV3DEV)/ev3dev.o $(SHARED)/net_udp.o $(SHARED)/misc.o

INCLUDE = ../lib

CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -DEV3 -c 
CXX_FLAGS = -O2 -std=c++11 -Wall -DEV3 -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) -I $(INCLUDE)
LFLAGS = -Wall $(DEBUG)

$(TARGET) : $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) -o $(TARGET)

main.o : main.cpp $(EV3DEV)/ev3dev.h $(SHARED)/misc.h $(SHARED)/net_udp.h 
	$(CXX) $(CXX_FLAGS) main.cpp

$(EV3DEV)/ev3dev.o : $(EV3DEV)/ev3dev.h $(EV3DEV)/ev3dev.cpp 
	$(MAKE) -C $(EV3DEV)

$(SHARED)/misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(MAKE) -C $(SHARED)
	
$(SHARED)/net_udp.o: $(SHARED)/net_udp.h $(SHARED)/net_udp.cpp $(SHARED)/misc.h
	$(MAKE) -C $(SHARED)

clean:
	\rm -f *.o $(TARGET)
	$(MAKE) -C $(EV3DEV) clean
	$(MAKE) -C $(SHARED) clean
