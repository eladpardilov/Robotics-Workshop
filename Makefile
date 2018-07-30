# basic variables:
CC = g++
CFLAGS = -std=c++11 -g 
LDFLAGS = -L/usr/local/lib -I/usr/include/eigen3 -lompl -pthread -lboost_system

# source files:
SOURCES = PathCalculator.cpp MyPRM.cpp SimpleBatchPRM.cpp main.cpp PostProcessor.cpp AngleDiffOptimizationObjective.cpp Utils.cpp
# output executable:
OBJECTS = PathCalculator.o

LDFLAGS += `pkg-config --cflags --libs opencv`

# if we want to take care of warnings (off by default since PRM and SimpleBatchPRM have warnings and we don't change those parts of the files)
#CFLAGS += -pedantic-errors -Wall -Wextra -Werror

all:
	$(CC) $(CFLAGS) $(SOURCES) -o $(OBJECTS) $(LDFLAGS)

clean:
	rm -rf $(OBJECTS)