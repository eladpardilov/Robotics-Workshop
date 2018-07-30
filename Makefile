all:
	g++ -L/usr/local/lib -std=c++11 -I/usr/include/eigen3 -g PathCalculator.cpp MyPRM.cpp SimpleBatchPRM.cpp main.cpp PostProcessor.cpp -o output.o -lompl -pthread -lboost_system `pkg-config --cflags --libs opencv`

clean:
	rm -rf output.o
