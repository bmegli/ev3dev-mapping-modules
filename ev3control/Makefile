TARGET = ev3control
SHARED = ../lib/shared
OBJS = main.o control.o control_protocol.o net_tcp.o $(SHARED)/misc.o

INCLUDE = ../lib

CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -DEV3 -c 
CXX_FLAGS = -O2 -std=c++11 -Wall -DEV3 -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) -I $(INCLUDE)
LFLAGS = -Wall $(DEBUG)

$(TARGET) : $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) -o $(TARGET)

main.o : main.cpp $(SHARED)/misc.h net_tcp.h 
	$(CXX) $(CXX_FLAGS) main.cpp

control.o: control.h control.cpp $(SHARED)/misc.h
	$(CXX) $(CXX_FLAGS) control.cpp

control_protocol.o: control_protocol.h control_protocol.cpp
	$(CXX) $(CXX_FLAGS) control_protocol.cpp
		
net_tcp.o: net_tcp.h net_tcp.cpp $(SHARED)/misc.h
	$(CXX) $(CXX_FLAGS) net_tcp.cpp
	
$(SHARED)/misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(MAKE) -C $(SHARED)
	
clean:
	\rm -f *.o $(TARGET)
	$(MAKE) -C $(SHARED) clean
