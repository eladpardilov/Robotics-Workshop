/*********** Includes ***********/
#include <iostream>
#include <string>
#include "PathCalculator.h"
#include "Defs.h"
#include <fstream>
/*
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
*/
#include <fstream>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc.hpp>

using namespace std;

int main(int argc, char **argv)
{
	int rows;
	int cols;
	int * global_end_coordinates = new int[2];
	int * tile_coordinates = new int[2];
	int * end_coordinates = new int[2];
	int max_turn_rate;
	int radius;
	int velocity;
	cv::Mat src_image, image, image2;


	if (argc < 2) {
		printf("Ignoring input params...\n");
		global_end_coordinates[0] = 260;
		global_end_coordinates[1] = 350;
		max_turn_rate = 45;
		radius = 5000/30;
		velocity = 3;
	} else {
		// Parse Arguments
		global_end_coordinates[0] = atoi(argv[1]);
		global_end_coordinates[1] = atoi(argv[2]);
		max_turn_rate = atoi(argv[3]);
		radius = 5000/30;
		velocity = 3;
	}
	
	tile_coordinates[0] = global_end_coordinates[0] / MAP_SIZE;
	tile_coordinates[1] = global_end_coordinates[1] / MAP_SIZE;
	end_coordinates[0] = global_end_coordinates[0] % MAP_SIZE;
	end_coordinates[1] = global_end_coordinates[1] % MAP_SIZE;

	printf("Taking (%d,%d) as destination coordinates.\n", global_end_coordinates[0], global_end_coordinates[1]);
	printf("-> (%d,%d) in tile (%d,%d)\n", end_coordinates[0], end_coordinates[1], tile_coordinates[0], tile_coordinates[1]);


	// Check the number of parameters
	/*
	if (argc < MAIN_NUM_OF_ARGS) {
		// Tell the user how to run the program
		string arg_string = "tif_file man_coordinates_x man_coordinates_y initial_radius helicopter_velocity";
		std::cerr << "Usage: " << argv[0] << arg_string << std::endl;
		return RETURN_CODE_ERROR;
	}
	*/
	
	src_image = cv::imread(MAP_TIF , CV_LOAD_IMAGE_UNCHANGED);
	
	cv::Rect region_of_interest = cv::Rect(tile_coordinates[0] * MAP_SIZE, tile_coordinates[1] * MAP_SIZE, MAP_SIZE, MAP_SIZE);

	image2 = src_image(region_of_interest);

	image = image2 / Z_AXIS_DIV_FACTOR;
	
	printf("Map size is %d X %d, map overview: (point every 10 coordinates):\n", MAP_SIZE, MAP_SIZE);
	for(int y=0; y < MAP_SIZE; y+=10){
		for(int x=0; x < MAP_SIZE; x+=10){
			printf("%4.1f ", image.at<float>(y, x));
		}
		printf("\n");
	}

	// Create object for the PathCalculator
	PathCalculator path(image, end_coordinates, max_turn_rate, radius, velocity);

	path.PlanRoute();
	
	path.Show();
	

    // for(int i = 0; i < MAP_SIZE; ++i) {
    //	 delete [] map[i];
    // }
	//delete [] map;

}


