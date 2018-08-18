//#include "PRM_Gui.h"
#ifndef GTKMM_PRM_Gui_img_H
#define GTKMM_PRM_Gui_img_H

#include <gtkmm-3.0/gtkmm.h>
#include <gtk/gtk.h>
#include <iostream>
#include <string>
#include <fstream>

#define BEST_IMG_FILE "best.jpg" 
#define SAFE_IMG_FILE "safe.jpg" 
#define FAST_IMG_FILE "fast.jpg" 




static bool b_safe = true; 
static bool b_best = true; 
static bool b_fast = true; 

class PRM_Gui_img : public Gtk::Window
{
public:
    PRM_Gui_img();
    ~PRM_Gui_img();

protected:
  Gtk::Box m_VBox;

    Gtk::Grid m_Grid;
    
    Gtk::Label m_Label_explain, m_Label_best, m_Label_safe, m_Label_fast;
    
    Gtk::Image* m_best;
    Gtk::Image* m_safe;
    Gtk::Image* m_fast; 
};

#endif
