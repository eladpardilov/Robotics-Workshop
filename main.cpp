/*********** Includes ***********/
#include <iostream>
#include <string>
#include "Defs.h"
#include "PathCalculator.h"
#include "PostProcessor.h"
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
	int * global_end_coordinates = new int[2];
	int * end_coordinates = new int[2];
	int max_turn_rate;
	double max_up_down_rate;
	int radius;
	int num_states;
	cv::Mat src_image, image, image2;
	char gm_tiff[19];

	// Check the number of parameters
	if (argc >= 2 && argc < MAIN_NUM_OF_ARGS) {
		// Tell the user how to run the program
		string arg_string = "./PathCalculator.o man_coordinates_x man_coordinates_y max_turn_rate max_up_down_rate num_states";
		std::cerr << "Usage: " << arg_string << std::endl;

		return RETURN_CODE_ERROR;
	}

	// Input parameters - default sets when none given:
	if (argc < 2) {
		printf("Using default input params...\n");
		global_end_coordinates[0] = 230;
		global_end_coordinates[1] = 350;
		max_turn_rate = 45; // angle per second
		max_up_down_rate = 100 / Z_AXIS_DIV_FACTOR; // meters per second
		num_states = 10000;
	} else {
		// Parse Arguments if given
		global_end_coordinates[0] = atoi(argv[1]);
		global_end_coordinates[1] = atoi(argv[2]);
		max_turn_rate = atoi(argv[3]);
		max_up_down_rate = atof(argv[4]) / Z_AXIS_DIV_FACTOR;
		num_states = atoi(argv[5]);		
	}
	
	radius = 5000/30;

	end_coordinates[0] = 180;
	end_coordinates[1] = 180;

	printf("Taking (%d,%d) as destination coordinates.\n", global_end_coordinates[0], global_end_coordinates[1]);
	printf("-> (%d,%d) in the displayed tile\n", end_coordinates[0], end_coordinates[1]);

	// Reading the tiff image
	src_image = cv::imread(MAP_TIF , CV_LOAD_IMAGE_UNCHANGED);
	
	cv::Rect region_of_interest = cv::Rect(global_end_coordinates[0] - MAP_SIZE/2, global_end_coordinates[1] - MAP_SIZE/2, MAP_SIZE, MAP_SIZE);

	image2 = src_image(region_of_interest);

	image = image2 / Z_AXIS_DIV_FACTOR;
	sprintf(gm_tiff, "roi_%d_%d.tif", global_end_coordinates[0], global_end_coordinates[1]);
	cv::imwrite(gm_tiff,  image2);

	// Security space
	image += (30 / Z_AXIS_DIV_FACTOR);

	// Create object for the PathCalculator
	PathCalculator path(image, end_coordinates, max_turn_rate, max_up_down_rate, radius, num_states);

	path.PlanRoute();
	
	path.finish();

	PostProcessor post(end_coordinates);
	
	int best_index = post.FindBestPath();
	printf("Best path index: %d\n",best_index);
	post.PrepareOnePath(best_index, "best");

	int safe_index = post.FindSafestPath();
	printf("Safest path index: %d\n",safe_index);
	post.PrepareOnePath(safe_index, "safe");

	int fast_index = post.FindFastestPath();
	printf("Fastest path index: %d\n",fast_index);
	post.PrepareOnePath(fast_index, "fast");
	
	post.Show();
	
	/* TODO: check which releases are to be made */	
    // for(int i = 0; i < MAP_SIZE; ++i) {
    //	 delete [] map[i];
    // }
	//delete [] map;

}


