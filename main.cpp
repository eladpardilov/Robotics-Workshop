/*********** Includes ***********/
#include <iostream>
#include <string>
#include "Defs.h"
#include "PathCalculator.h"
#include "PostProcessor.h"
#include "PRM_Gui.h"
#include "PRM_Gui_img.h"

#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc.hpp>

#include <gtk/gtk.h>
#include <cstring>
#include <glib.h>
#include <gtkmm/drawingarea.h>
#include <gdkmm/pixbuf.h>
#include <gtkmm/image.h>
#include <gtkmm.h>
#include <gtkmm/application.h>

using namespace std;

int main(int argc, char **argv)
{
	int * global_end_coordinates = new int[2];
	int * end_coordinates = new int[2];
	int max_turn_rate;
	double max_up_down_rate;
	int radius = DEFAULT_RADIUS;
	int num_states = DEFAULT_NUM_STATES;
	cv::Mat src_image, image, image2;
	char gm_tiff[19];
	bool safe, best, fast;
	
	end_coordinates[0] = 180;
	end_coordinates[1] = 180;
	
	auto app = Gtk::Application::create(argc, argv, "app_gui.part_1", Gio::APPLICATION_NON_UNIQUE);
	PRM_Gui gui_inputWindow;
	//Shows the window and returns when it is closed.
	app->run(gui_inputWindow);
	
	
	global_end_coordinates[0]  = gui_inputWindow.m_x;
	global_end_coordinates[1]  = gui_inputWindow.m_y;
	max_turn_rate 			   = gui_inputWindow.m_angle;
	max_up_down_rate		   = gui_inputWindow.m_rate;
	best					   = gui_inputWindow.m_best;		
	safe					   = gui_inputWindow.m_safe;
	fast				       = gui_inputWindow.m_fast;
	
	printf("Taking (%d,%d) as destination coordinates.\n", global_end_coordinates[0], global_end_coordinates[1]);
	printf("-> (%d,%d) in the displayed tile\n", end_coordinates[0], end_coordinates[1]);

	// Reading the tiff image
	src_image = cv::imread(gui_inputWindow.m_file , CV_LOAD_IMAGE_UNCHANGED);
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
	
	PostProcessor post(end_coordinates);
	
	if(best)
	{
		int best_index = post.FindBestPath();
		printf("Best path index: %d\n",best_index);
		post.PrepareOnePath(best_index, "best");
    }
    
    if(safe)
    {
		int safe_index = post.FindSafestPath();
		printf("Safest path index: %d\n",safe_index);
		post.PrepareOnePath(safe_index, "safe");
	}
	
	if(fast)
	{
		int fast_index = post.FindFastestPath();
		printf("Fastest path index: %d\n",fast_index);
		post.PrepareOnePath(fast_index, "fast");
	}	
	
	auto app2 = Gtk::Application::create(argc, argv, "app_gui.part_2", Gio::APPLICATION_NON_UNIQUE);
	PRM_Gui_img gui_imgWindow(safe, best, fast); 
	app2->run(gui_imgWindow);
	
	/* TODO: check which releases are to be made */	
    // for(int i = 0; i < MAP_SIZE; ++i) {
    //	 delete [] map[i];
    // }
	//delete [] map;

	delete[] global_end_coordinates;
	delete[] end_coordinates;
	
}


