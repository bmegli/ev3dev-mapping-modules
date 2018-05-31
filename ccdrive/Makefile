TARGET = ccdrive
ROBOCLAW = ../lib/roboclaw
VMU931 = ../lib/vmu931
SHARED = ../lib/shared
OBJS = main.o $(ROBOCLAW)/roboclaw.o $(VMU931)/vmu931.o $(SHARED)/net_udp.o $(SHARED)/misc.o

INCLUDE = ../lib

CC = gcc
CXX = g++
DEBUG =
CFLAGS = -O2 -Wall -c 
CXX_FLAGS = -O2 -std=c++11 -Wall -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) -I $(INCLUDE)
LFLAGS = -Wall $(DEBUG)

$(TARGET) : $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) -o $(TARGET)

main.o : main.cpp $(ROBOCLAW)/roboclaw.h $(VMU931)/vmu931.h $(SHARED)/misc.h $(SHARED)/net_udp.h 
	$(CXX) $(CXX_FLAGS) main.cpp

$(ROBOCLAW)/roboclaw.o : $(ROBOCLAW)/roboclaw.h $(ROBOCLAW)/roboclaw.c 
	$(MAKE) -C $(ROBOCLAW)

$(VMU931)/vmu931.o : $(VMU931)/vmu931.h $(VMU931)/vmu931.c 
	$(MAKE) -C $(VMU931)

$(SHARED)/misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(MAKE) -C $(SHARED)
	
$(SHARED)/net_udp.o: $(SHARED)/net_udp.h $(SHARED)/net_udp.cpp $(SHARED)/misc.h
	$(MAKE) -C $(SHARED)

clean:
	\rm -f *.o $(TARGET)
	$(MAKE) -C $(ROBOCLAW) clean
	$(MAKE) -C $(VMU931) clean
	$(MAKE) -C $(SHARED) clean
