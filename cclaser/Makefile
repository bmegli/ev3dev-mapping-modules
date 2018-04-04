TARGET = cclaser
PWM = .
SHARED = ../lib/shared
XV11LIDAR = ../lib/xv11lidar

OBJS = main.o $(PWM)/pwm.o $(SHARED)/net_udp.o $(SHARED)/misc.o xv11lidar.o

INCLUDE = ../lib

CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -c -I $(INCLUDE)
CXX_FLAGS = -O2 -Wall -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) -I $(INCLUDE)
LFLAGS = -Wall $(DEBUG)

$(TARGET) : $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) -o $(TARGET)

main.o : main.cpp $(PWM)/pwm.h $(SHARED)/misc.h $(SHARED)/net_udp.h  $(XV11LIDAR)/xv11lidar.h 
	$(CXX) $(CXX_FLAGS) main.cpp

$(PWM)/pwm.o : $(PWM)/pwm.h $(PWM)/pwm.cpp 
	$(CXX) $(CXX_FLAGS) $(PWM)/pwm.cpp

$(SHARED)/misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(MAKE) -C $(SHARED)
	
$(SHARED)/net_udp.o: $(SHARED)/net_udp.h $(SHARED)/net_udp.cpp $(SHARED)/misc.h
	$(MAKE) -C $(SHARED)

xv11lidar.o: $(XV11LIDAR)/xv11lidar.h $(XV11LIDAR)/xv11lidar.c
	$(CC) $(CFLAGS) $(XV11LIDAR)/xv11lidar.c

clean:
	\rm -f *.o $(TARGET)
	$(MAKE) -C $(SHARED) clean
