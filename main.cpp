/*
 * main.cpp
 *
 *  Created on: May 9, 2018
 *     Author: Dana
 */
/*********** Includes ***********/
#include <iostream>
#include <string>
#include "PathCalculator.h"
#include "Defs.h"
#include <fstream>
#include <iostream>
#include <string>

//using namespace cv;
using namespace std;

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

	src_image = cv::imread(MAP_TIF , CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);

	cv::Rect region_of_interest = cv::Rect(0, 0, 360, 360);

	image = src_image(region_of_interest);

    map = new float*[MAP_SIZE];
    for(int i = 0; i < MAP_SIZE; i++)
        map[i] = new float[MAP_SIZE];

    printf("Map size is %d X %d, map overview: (point every 5 coordinates):\n", MAP_SIZE, MAP_SIZE);

    for(int i=0; i < MAP_SIZE; i++){
        for(int j=0; j < MAP_SIZE; j++){
        	if (25 <= i && i < 75 && 25 <= j && j < 75)
        		map[i][j] = 20;
        	else
                	if (15 <= i && i < 85 && 15 <= j && j < 85)
                		map[i][j] = 15;
                	else
                        	if (10 <= i && i < 90 && 10 <= j && j < 90)
                        		map[i][j] = 10;
                        	else
                        		map[i][j] = 0;
        }
    }
/*
    for(int i=0; i < MAP_SIZE; i+=5){
        for(int j=0; j < MAP_SIZE; j+=5){
        	printf("%4.1f ", map[i][j]);
        }
        printf("\n");
    }
*/
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
    PathCalculator path(rows, cols, image, end_coordinates, radius, velocity);

    path.PlanRoute();
    
    path.Show();
    

    for(int i = 0; i < MAP_SIZE; ++i) {
        delete [] map[i];
    }
    delete [] map;

}


