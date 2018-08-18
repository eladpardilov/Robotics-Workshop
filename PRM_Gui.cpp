#include <gtkmm-3.0/gtkmm.h>
#include "PRM_Gui.h"
#include <iostream>
#include <string>
#include <fstream>
#include <gtk/gtk.h>
#include "Defs.h"


using namespace std; 

PRM_Gui::PRM_Gui() : 
  m_VBox(Gtk::ORIENTATION_VERTICAL),
  m_Label_explain("Hello! this is our app for saving a traveler! You can change the default values\n if you wish. Press the Start button to get the results"),
  m_ButtonStart("START"),
  m_ButtonFast("Fastest route"), m_ButtonBest("Best route"), m_ButtonSafe("Safest route"),
  m_EntryX(), m_EntryY(), m_EntryAngle(), m_EntryRate(), m_EntryFile(),
  m_Label_xy("Provide x,y coordinates (pixels, 180-3420)"), 
  m_Label_rotate_angle("Enter maximum turn rate (degree/sec, 5-90)"), 
  m_Label_upDown_rate("Enter maximum rate of climb/decent (meter/sec, 5-100)"),
  m_Label_route("Choose the desirable routes"),
  m_Label_file("Insert a .tif file name of the map"),
  m_lable_empty(""), m_lable_empty1(""), m_lable_empty2(""), m_lable_empty3(""), m_lable_empty4(""), m_lable_empty5("")
{
  m_closedByX = false;
  
              
  set_title("Save The Man!");
  set_border_width(12);

  add(m_VBox);

  //m_VBox.pack_start(m_Label_explain, Gtk::PACK_SHRINK);
  
  m_EntryX.set_placeholder_text(std::to_string(DEFAULT_X));
  m_EntryY.set_placeholder_text(std::to_string(DEFAULT_Y));
  m_EntryAngle.set_placeholder_text(std::to_string(DEFAULT_ANGLE));
  m_EntryRate.set_placeholder_text(std::to_string(DEFAULT_RATE));
  m_EntryFile.set_placeholder_text(MAP_TIF);

  //Fill Grid:
  m_VBox.pack_start(m_Grid);
  m_Grid.set_row_homogeneous(false);
  m_Grid.set_column_homogeneous(false);
  m_Grid.set_row_spacing(4);
  
  m_ButtonBest.set_halign(Gtk::ALIGN_CENTER);
  m_ButtonFast.set_halign(Gtk::ALIGN_CENTER);
  m_ButtonSafe.set_halign(Gtk::ALIGN_CENTER);
  
  m_Grid.attach(m_Label_explain, 	0, 0, 6, 2);
  
  m_Grid.attach(m_lable_empty, 	0, 2, 1, 1);
  
  m_Grid.attach(m_Label_xy, 0, 3, 6, 1);
  m_Grid.attach(m_EntryX,   0, 4, 3, 1);
  m_Grid.attach(m_EntryY,   3, 4, 3, 1);
 
  m_Grid.attach(m_lable_empty1, 	0, 5, 1, 1);

  m_Grid.attach(m_Label_rotate_angle, 	0, 6, 6, 1);
  m_Grid.attach(m_EntryAngle,   		0, 7, 6, 1);
  
  m_Grid.attach(m_lable_empty2, 	0, 8, 1, 1);
  
  m_Grid.attach(m_Label_upDown_rate, 	0, 9, 6, 1);
  m_Grid.attach(m_EntryRate,   			0, 10, 6, 1);
  
  m_Grid.attach(m_lable_empty3, 	0, 11, 1, 1);

  m_Grid.attach(m_Label_file, 			0, 12, 6, 1);
  m_Grid.attach(m_EntryFile,   			0, 13, 6, 1);
  
  m_Grid.attach(m_lable_empty4, 	0, 14, 1, 1);

  m_Grid.attach(m_Label_route, 			0, 15, 6, 1);
  m_Grid.attach(m_ButtonFast,   		0, 16, 2, 1);
  m_Grid.attach(m_ButtonSafe,   		2, 16, 2, 1);
  m_Grid.attach(m_ButtonBest,   		4, 16, 2, 1);
  
  m_Grid.attach(m_lable_empty5, 	0, 17, 1, 1);
  
  m_Grid.attach(m_ButtonStart,   		2, 18, 2, 1);

  //m_VBox.set_spacing(7);
  m_ButtonStart.signal_clicked().connect(sigc::mem_fun(*this,
              &PRM_Gui::on_button_start) );
              
  m_ButtonFast.signal_clicked().connect(sigc::mem_fun(*this,
		      &PRM_Gui::on_button_fast) );
  
  m_ButtonSafe.signal_clicked().connect(sigc::mem_fun(*this,
              &PRM_Gui::on_button_safe) );
              
  m_ButtonBest.signal_clicked().connect(sigc::mem_fun(*this,
              &PRM_Gui::on_button_best) );
              
  signal_delete_event().connect(sigc::mem_fun(*this,
			  &PRM_Gui::on_button_x) );

  show_all_children();
}

PRM_Gui::~PRM_Gui()
{
}

void PRM_Gui::on_button_start()
{
	//m_Entry.get_text()
	std::string x_str = m_EntryX.get_text();
    m_x = atoi(x_str.c_str()) >= 180? atoi(x_str.c_str()) : DEFAULT_X;
    
    std::string y_str = m_EntryY.get_text();
    m_y = atoi(y_str.c_str()) >= 180? atoi(y_str.c_str()) : DEFAULT_Y;
    
    std::string angle_str = m_EntryAngle.get_text();
    m_angle = atoi(angle_str.c_str());
    if(m_angle<5 || m_angle>90)
		m_angle = DEFAULT_ANGLE;
	
	std::string rate_str = m_EntryRate.get_text();	
	m_rate = atof(rate_str.c_str());
	if(m_rate<5 || m_rate>100)
		m_rate = DEFAULT_RATE;
	 
    m_file = m_EntryFile.get_text();
    if(m_file == "")
		m_file = MAP_TIF;
		
	/*
	cout << "x: " << m_x << endl; 
	cout << "y: " << m_y << endl; 
	cout << "file: " << m_file << endl;
	cout << "angle: " << m_angle << endl; 
	cout << "rate: " << m_rate << endl; 
	*/
		
	hide();
}

void PRM_Gui::on_button_fast()
{
	m_fast = m_ButtonFast.get_active() ? true : false;
	
	//cout << "fast : " << m_fast << endl;
	
}

void PRM_Gui::on_button_best()
{
	m_best = m_ButtonBest.get_active() ? true : false;;
	
	//cout << "best : " << m_best << endl;
	
}

void PRM_Gui::on_button_safe()
{
	m_safe = m_ButtonSafe.get_active() ? true : false;;
	
	//cout << "safe : " << m_safe << endl;
	
}

bool PRM_Gui::on_button_x(GdkEventAny*)
{
	m_closedByX = true;
	hide();
	return true;
}


