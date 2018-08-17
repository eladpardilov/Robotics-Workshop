#include <gtkmm-3.0/gtkmm.h>
#include "PRM_Gui.h"
#include <iostream>
#include <string>
#include <fstream>
#include <gtk/gtk.h>
#include "PRM_Gui_img.h"


using namespace std; 

int x;
int y;
int angle; 
double rate; 
Glib::ustring file; 
bool fast = false;
bool safe = false;
bool best = false;


PRM_Gui::PRM_Gui() : 
  m_VBox(Gtk::ORIENTATION_VERTICAL),
  m_Label_explain("Hello! this is our app for saving a traveler! You can change the default values\n if you wish. Press the Start button to get the results"),
  m_ButtonStart("START"),
  m_ButtonFast("Fastest route"), m_ButtonBest("Best route"), m_ButtonSafe("Safest route"),
  m_EntryX(), m_EntryY(), m_EntryAngle(), m_EntryRate(), m_EntryFile(),
  m_Label_xy("Where is the lost man? (X,Y bigger than 180)"), 
  m_Label_rotate_angle("What is the maximum turn rate? ([degree/sec] between 5 to 90)"), 
  m_Label_upDown_rate("What is the maximum up-down velocity? ([meter/sec] between 5 to 100)"),
  m_Label_route("Which route would you like to check?"),
  m_Label_file("Please insert a .tif fileName of the map"),
  m_lable_empty(""), m_lable_empty1(""), m_lable_empty2(""), m_lable_empty3(""), m_lable_empty4(""), m_lable_empty5("")
{
  set_title("Save the man!");
  set_border_width(12);

  add(m_VBox);

  //m_VBox.pack_start(m_Label_explain, Gtk::PACK_SHRINK);
  
  m_EntryX.set_placeholder_text(std::to_string(DEFAULT_X));
  m_EntryY.set_placeholder_text(std::to_string(DEFAULT_Y));
  m_EntryAngle.set_placeholder_text(std::to_string(DEFAULT_ANGLE));
  m_EntryRate.set_placeholder_text(std::to_string(DEFAULT_RATE));
  m_EntryFile.set_placeholder_text(DEFAULTF_FILE);

  //Fill Grid:
  m_VBox.pack_start(m_Grid);
  m_Grid.set_row_homogeneous(false);
  m_Grid.set_column_homogeneous(false);
  m_Grid.set_row_spacing(4);
  
  m_Grid.attach(m_Label_explain, 	0, 0, 2, 2);
  
  m_Grid.attach(m_lable_empty, 	0, 2, 1, 1);
  
  m_Grid.attach(m_Label_xy, 0, 3, 2, 1);
  m_Grid.attach(m_EntryX,   0, 4, 1, 1);
  m_Grid.attach(m_EntryY,   1, 4, 1, 1);
 
  m_Grid.attach(m_lable_empty1, 	0, 5, 1, 1);

  m_Grid.attach(m_Label_rotate_angle, 	0, 6, 2, 1);
  m_Grid.attach(m_EntryAngle,   		0, 7, 2, 1);
  
  m_Grid.attach(m_lable_empty2, 	0, 8, 1, 1);
  
  m_Grid.attach(m_Label_upDown_rate, 	0, 9, 2, 1);
  m_Grid.attach(m_EntryRate,   			0, 10, 2, 1);
  
  m_Grid.attach(m_lable_empty3, 	0, 11, 1, 1);

  m_Grid.attach(m_Label_file, 			0, 12, 2, 1);
  m_Grid.attach(m_EntryFile,   			0, 13, 2, 1);
  
  m_Grid.attach(m_lable_empty4, 	0, 14, 1, 1);

  m_Grid.attach(m_Label_route, 			0, 15, 2, 1);
  m_Grid.attach(m_ButtonFast,   		0, 16, 1, 1);
  m_Grid.attach(m_ButtonSafe,   		1, 16, 1, 1);
  m_Grid.attach(m_ButtonBest,   		2, 16, 1, 1);
  
  m_Grid.attach(m_lable_empty5, 	0, 17, 1, 1);
  
  m_Grid.attach(m_ButtonStart,   		1, 18, 1, 1);


  //Add ButtonBox to bottom:
  //m_VBox.pack_start(m_ButtonBox, Gtk::PACK_SHRINK);
  
  //m_VBox.set_spacing(7);
  m_ButtonStart.signal_clicked().connect(sigc::mem_fun(*this,
              &PRM_Gui::on_button_start) );
              
  m_ButtonFast.signal_clicked().connect(sigc::mem_fun(*this,
		      &PRM_Gui::on_button_fast) );
  
  m_ButtonSafe.signal_clicked().connect(sigc::mem_fun(*this,
              &PRM_Gui::on_button_safe) );
              
  m_ButtonBest.signal_clicked().connect(sigc::mem_fun(*this,
              &PRM_Gui::on_button_best) );

  show_all_children();
}

PRM_Gui::~PRM_Gui()
{
}

void PRM_Gui::on_button_start()
{
	//m_Entry.get_text()
	std::string x_str = m_EntryX.get_text();
    x = atoi(x_str.c_str()) >= 180? atoi(x_str.c_str()) : DEFAULT_X;
    
    std::string y_str = m_EntryY.get_text();
    y = atoi(y_str.c_str()) >= 180? atoi(y_str.c_str()) : DEFAULT_Y;
    
    std::string angle_str = m_EntryAngle.get_text();
    angle = atoi(angle_str.c_str());
    if(angle<5 || angle>90)
		angle = DEFAULT_ANGLE;
	
	std::string rate_str = m_EntryRate.get_text();	
	rate = atof(rate_str.c_str());
	if(rate<5 || rate>100)
		rate = DEFAULT_RATE;
	 
    file = m_EntryFile.get_text();
    if(file == "")
		file = DEFAULTF_FILE;
		
	
	cout << "x: " << x << endl; 
	cout << "y: " << y << endl; 
	cout << "file: " << file << endl;
	cout << "angle: " << angle << endl; 
	cout << "rate: " << rate << endl; 
	
	m_angle = angle; 
	
	hide();

	//***PRM_Gui_img* win = new PRM_Gui_img;
    //win->signal_hide().connect(sigc::mem_fun(*this, &PRM_Gui::aboutWinClose));
    //***win->show();
    
  //  property_application().release();
	//Gtk::Window win;
	Gtk::Image *im;

	//im = Gtk::manage(new Gtk::Image());
	//im->set("test.jpeg");
	//grid->attach(*im, i, j, 1, 1);
	//add(im);
	
	//show_all_children();

}

void PRM_Gui::on_button_fast()
{
	fast = m_ButtonFast.get_active() ? true : false;
	
	cout << "fast : " << fast << endl;
	
}

void PRM_Gui::on_button_best()
{
	best = m_ButtonBest.get_active() ? true : false;;
	
	cout << "best : " << best << endl;
	
}

void PRM_Gui::on_button_safe()
{
	safe = m_ButtonSafe.get_active() ? true : false;;
	
	cout << "safe : " << safe << endl;
	
}

