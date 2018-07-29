all:
	g++ -L/usr/local/lib -std=c++11 -I/usr/include/eigen3 -g PathCalculator.cpp SimpleBatchPRM.cpp main.cpp -o output.o -lompl -lboost_system `pkg-config --cflags --libs opencv`
clean:
	rm -rf output.o
