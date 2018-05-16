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

//using namespace cv;
using namespace std;

//cv::Mat global_mat;

int main(int argc, char **argv)
{
//	TIFF* tif;
	float** map;
    int rows;
    int cols;
    int * end_coordinates;
    int radius;
    int velocity;
    cv::Mat src_image, image;

    printf("Ignoring input params...\n");
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
	
	cout << "done reading " << endl;
	
	cv::Rect region_of_interest = cv::Rect(0, 0, MAP_SIZE, MAP_SIZE);

	image = src_image(region_of_interest);
	
    printf("Map size is %d X %d, map overview: (point every 5 coordinates):\n", MAP_SIZE, MAP_SIZE);
    for(int i=0; i < MAP_SIZE; i+=10){
        for(int j=0; j < MAP_SIZE; j+=10){
        	printf("%4.0f ", image.at<float>(i, j));
        }
        printf("\n");
    }
    end_coordinates = new int[2];

    // Parse Arguments
    // Open the TIFF file using libtiff
    /*
    tif = TIFFOpen(argv[0].c_str(), "r");
    coordinates[0] = atoi(argv[1]);
    coordinates[1] = atoi(argv[2]);
    radius = atoi(argv[3]);
    velocity = atoi(argv[4]);
    */
    end_coordinates[0] = MAP_SIZE / 2;
    end_coordinates[1] = MAP_SIZE / 2;
    radius = 3;
    velocity = 3;

    // Create object for the PathCalculator
    PathCalculator path(image, end_coordinates, radius, velocity);

    path.PlanRoute();
    
    path.Show();
    

   // for(int i = 0; i < MAP_SIZE; ++i) {
   //     delete [] map[i];
   // }
    //delete [] map;

}


