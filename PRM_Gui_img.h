//#include "PRM_Gui.h"
#ifndef GTKMM_PRM_Gui_img_H
#define GTKMM_PRM_Gui_img_H

#include <gtkmm-3.0/gtkmm.h>
#include <gtk/gtk.h>
#include <iostream>
#include <string>
#include <fstream>
#include "Defs.h"

/*
 * A window displaying the 3 routes chosen by the user.  
 * (The chosen routes are given to the constructor- 
 * bool safe, best, fast) 
 */
class PRM_Gui_img : public Gtk::Window
{
public:
    PRM_Gui_img(bool safe, bool best, bool fast);
    ~PRM_Gui_img();

protected:
    //GTK container for the grid 
    Gtk::Box m_VBox;

    Gtk::Grid m_Grid;
    
    Gtk::Label m_Label_explain, m_Label_best, m_Label_safe, m_Label_fast;
    
    Gtk::Image* m_best;
    Gtk::Image* m_safe;
    Gtk::Image* m_fast; 
    
    //Routes chosen by the user to see    
    bool m_bBest, m_bSafe, m_bFast;
};

#endif
