all:
	g++ -L/usr/local/lib -L/usr/include/glibmm-2.4/ -std=c++11 -I/usr/include/eigen3 -g PRM_Gui.cpp PRM_Gui_img.cpp main.cpp -o output.o `pkg-config --cflags --libs gtkmm-3.0`
clean:
	rm -rf output.o
