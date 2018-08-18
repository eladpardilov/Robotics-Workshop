/*********** Includes ***********/
#include <iostream>
#include <string>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc.hpp>


#include  "Prm_Gui.h"
#include "PRM_Gui_img.h"

using namespace std; 
//#include <gtkmm-3.0/gtkmm/application.h>
/*

using namespace std;

int main(int argc, char **argv)
{
	//~ int rows;
	//~ int cols;
	//~ int * global_end_coordinates = new int[2];
	//~ int * end_coordinates = new int[2];
	//~ int max_turn_rate;
	//~ int max_up_down_rate;
	//~ int radius;
	//~ cv::Mat src_image, image, image2;


	//~ if (argc < 2) {
		//~ printf("Ignoring input params...\n");
		//~ global_end_coordinates[0] = 230;
		//~ global_end_coordinates[1] = 350;
		//~ max_turn_rate = 45; // angle per second
		//~ max_up_down_rate = 100; // meters per minute
		//~ radius = 5000/30;
	//~ } else {
		//~ // Parse Arguments
		//~ global_end_coordinates[0] = atoi(argv[1]);
		//~ global_end_coordinates[1] = atoi(argv[2]);
		//~ max_turn_rate = atoi(argv[3]);
		//~ max_up_down_rate = atoi(argv[4]);
		//~ radius = 5000/30;
	//~ }
	
	//~ end_coordinates[0] = 180;
	//~ end_coordinates[1] = 180;

	//~ printf("Taking (%d,%d) as destination coordinates.\n", global_end_coordinates[0], global_end_coordinates[1]);
	//~ printf("-> (%d,%d) in the displayed tile\n", end_coordinates[0], end_coordinates[1]);


	//~ // Check the number of parameters
	//~ 
	//~ if (argc < MAIN_NUM_OF_ARGS) {
		//~ // Tell the user how to run the program
		//~ string arg_string = "tif_file man_coordinates_x man_coordinates_y initial_radius helicopter_velocity";
		//~ std::cerr << "Usage: " << argv[0] << arg_string << std::endl;
		//~ return RETURN_CODE_ERROR;
	//~ }
	//~ 
	
	//~ src_image = cv::imread(MAP_TIF , CV_LOAD_IMAGE_UNCHANGED);
	
	//~ cv::Rect region_of_interest = cv::Rect(global_end_coordinates[0] - MAP_SIZE/2, global_end_coordinates[1] - MAP_SIZE/2, MAP_SIZE, MAP_SIZE);

	//~ image2 = src_image(region_of_interest);

	//~ image = image2 / Z_AXIS_DIV_FACTOR;
//~ 
	//~ printf("Map size is %d X %d, map overview: (point every 10 coordinates):\n", MAP_SIZE, MAP_SIZE);
	//~ for(int y=0; y < MAP_SIZE; y+=10){
		//~ for(int x=0; x < MAP_SIZE; x+=10){
			//~ printf("%4.1f ", image.at<float>(y, x));
		//~ }
		//~ printf("\n");
	//~ }

	//~ // Create object for the PathCalculator
	//~ PathCalculator path(image, end_coordinates, max_turn_rate, max_up_down_rate, radius);

	//~ path.PlanRoute();
	
	//~ path.Show();
	
	
	 auto app = Gtk::Application::create(argc, argv, "org.gtkmm.example");

	 PRM_Gui* window;
	 
	 std::cout << atof(window->m_Entry.get_text().c_str()) << endl;
     //Shows the window and returns when it is closed.
     app->run(*window);
	
	

    // for(int i = 0; i < MAP_SIZE; ++i) {
    //	 delete [] map[i];
    // }
	//delete [] map;

}*/
#include <iostream>
#include <gtk/gtk.h>
#include <cstring>
#include <glib.h>
#include <gtkmm/drawingarea.h>
#include <gdkmm/pixbuf.h>
#include <gtkmm/image.h>
#include <gtkmm.h>
#include <gtkmm/application.h>

 
int main(int argc, char* argv[])
{
	//gtk_init(&argc, &argv);
   
  auto app = Gtk::Application::create(argc, argv, "app_1.h_1", Gio::APPLICATION_NON_UNIQUE);

  PRM_Gui gui;
  
  //Shows the window and returns when it is closed.
  app->run(gui);
  
  if(gui.m_closedByX)
	{
		printf("close!!!!!\n");
		return 0;
	}
  
  cout << "Main: first finish, gui angle = " << gui.m_angle << endl;
  
  auto app2 = Gtk::Application::create(argc, argv, "app_2.h_2", Gio::APPLICATION_NON_UNIQUE);
  
  printf("here\n");
  
  PRM_Gui_img gui2; 
  
    printf("here2\n");


  app2->run(gui2);
  
  return 0;
}


