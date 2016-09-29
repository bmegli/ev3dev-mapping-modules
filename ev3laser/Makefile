TARGET = ev3laser
EV3DEV = ../lib/ev3dev-lang-cpp
SHARED = ../lib/shared
XV11LIDAR = ../lib/xv11lidar

OBJS = main.o $(EV3DEV)/ev3dev.o $(SHARED)/net_udp.o $(SHARED)/misc.o xv11lidar.o

INCLUDE = ../lib

CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -DEV3 -c -I $(INCLUDE)
CXX_FLAGS = -O2 -std=c++11 -Wall -DEV3 -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) -I $(INCLUDE)
LFLAGS = -Wall $(DEBUG)

$(TARGET) : $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) -o $(TARGET)

main.o : main.cpp $(EV3DEV)/ev3dev.h $(SHARED)/misc.h $(SHARED)/net_udp.h  $(XV11LIDAR)/xv11lidar.h 
	$(CXX) $(CXX_FLAGS) main.cpp

$(EV3DEV)/ev3dev.o : $(EV3DEV)/ev3dev.h $(EV3DEV)/ev3dev.cpp 
	$(MAKE) -C $(EV3DEV)

$(SHARED)/misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(MAKE) -C $(SHARED)
	
$(SHARED)/net_udp.o: $(SHARED)/net_udp.h $(SHARED)/net_udp.cpp $(SHARED)/misc.h
	$(MAKE) -C $(SHARED)

xv11lidar.o: $(XV11LIDAR)/xv11lidar.h $(XV11LIDAR)/xv11lidar.c
	$(CC) $(CFLAGS) $(XV11LIDAR)/xv11lidar.c

clean:
	\rm -f *.o $(TARGET)
	$(MAKE) -C $(EV3DEV) clean
	$(MAKE) -C $(SHARED) clean
