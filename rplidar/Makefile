TARGET = rplidar
RPLIDAR_SDK = ../lib/rplidar_sdk/sdk/sdk
RPLIDAR_LIB = ../lib/rplidar_sdk/sdk/output/Linux/Release/librplidar_sdk.a

SHARED = ../lib/shared
OBJS = main.o $(RPLIDAR_LIB) $(SHARED)/net_udp.o $(SHARED)/misc.o

INCLUDES = -I ../lib -I $(RPLIDAR_SDK)/include -I $(RPLIDAR_SDK)/src

CC = gcc
CXX = g++
DEBUG =
CFLAGS = -O2 -Wall -c 
CXX_FLAGS = -O2 -std=c++11 -Wall -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) $(INCLUDES)
LFLAGS = -lpthread

$(TARGET) : $(OBJS)
	$(CXX) $(OBJS) $(LFLAGS) -o $(TARGET)

main.o : main.cpp $(RPLIDAR_SDK)/include/rplidar.h $(SHARED)/misc.h $(SHARED)/net_udp.h 
	$(CXX) $(CXX_FLAGS) main.cpp

$(RPLIDAR_LIB) : $(RPLIDAR_SDK)/src/rplidar_driver.cpp
	$(MAKE) -C $(RPLIDAR_SDK)

$(SHARED)/misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(MAKE) -C $(SHARED)
	
$(SHARED)/net_udp.o: $(SHARED)/net_udp.h $(SHARED)/net_udp.cpp $(SHARED)/misc.h
	$(MAKE) -C $(SHARED)

clean:
	\rm -f *.o $(TARGET)
	$(MAKE) -C $(RPLIDAR_SDK) clean
	$(MAKE) -C $(SHARED) clean
