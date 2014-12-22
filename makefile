CC=g++
CFLAGS=-c -Wall
LDFLAGS=
LIBS=-lboost_system -lboost_thread -pthread

SRCS=src/main.cpp src/robotSerial.cpp
OBJS=$(SRCS:.cpp=.o)
EXEC=piRobot


all: $(SRCS) $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean::
	rm -f $(OBJS) $(EXEC)
