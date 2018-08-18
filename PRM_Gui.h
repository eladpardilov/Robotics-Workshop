#ifndef GTKMM_PRM_Gui_H
#define GTKMM_PRM_Gui_H

#include <gtkmm-3.0/gtkmm.h>
#include <gtk/gtk.h>
#include "Defs.h"

class PRM_Gui : public Gtk::Window
{
public:
  PRM_Gui();
  virtual ~PRM_Gui();
    
  int m_x;
  int m_y;
  int m_angle; 
  double m_rate; 
  std::string m_file; 
  bool m_fast = false;
  bool m_safe = false;
  bool m_best = false;
  
  bool m_closedByX;  

protected:
  //Signal handlers:
  void on_button_start();
  void on_button_fast();
  void on_button_best();
  void on_button_safe();
  bool on_button_x(GdkEventAny*);


  

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
