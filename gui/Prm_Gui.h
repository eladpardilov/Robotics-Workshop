#ifndef GTKMM_PRM_Gui_H
#define GTKMM_PRM_Gui_H

#include <gtkmm-3.0/gtkmm.h>
#include <gtk/gtk.h>


#define DEFAULT_ANGLE 30
#define DEFAULT_X 180
#define DEFAULT_Y 180
#define DEFAULT_RATE 50 
#define DEFAULTF_FILE "map.tif"



class PRM_Gui : public Gtk::Window
{
public:
  PRM_Gui();
  virtual ~PRM_Gui();
  
  int m_angle;
  
  bool m_closedByX;
  
  bool test2(GdkEventAny*);

protected:
  //Signal handlers:
  void on_button_start();
  void on_button_fast();
  void on_button_best();
  void on_button_safe();

  

  //Child widgets:
  Gtk::Box m_VBox;

  Gtk::Label m_Label_explain, m_Label_xy, m_Label_rotate_angle, m_Label_upDown_rate, m_Label_route, m_Label_file, 
			m_lable_empty, m_lable_empty1, m_lable_empty2, m_lable_empty3, m_lable_empty4, m_lable_empty5;

  Gtk::Grid m_Grid;
  
  Gtk::Button m_ButtonStart; //m_ButtonXY, m_ButtonAngle, m_ButtonRate, m_ButtonFile, 
  
  Gtk::CheckButton m_ButtonFast, m_ButtonBest, m_ButtonSafe;
  
  Gtk::Entry m_EntryX, m_EntryY, m_EntryAngle, m_EntryRate, m_EntryFile;

  Gtk::ButtonBox m_ButtonBox;
};

#endif //GTKMM_EXAMPLEWINDOW_H
