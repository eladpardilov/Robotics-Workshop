#include "PRM_Gui_img.h"


PRM_Gui_img::PRM_Gui_img():
  m_VBox(Gtk::ORIENTATION_VERTICAL),
  m_Label_best("Best route: "),
  m_Label_safe("Safe route: "),
  m_Label_fast("Fast route: "),
  m_Label_explain("And the results are...")
{
	set_title("Save the man - The results!");
	set_border_width(12);
	 
	add(m_VBox);
	 
	m_best = Gtk::manage(new Gtk::Image());
	m_fast = Gtk::manage(new Gtk::Image());
	m_safe = Gtk::manage(new Gtk::Image());
	 
	m_VBox.pack_start(m_Grid);
    m_Grid.set_row_homogeneous(false);
    m_Grid.set_column_homogeneous(false);
    m_Grid.set_row_spacing(5);
  
    m_Grid.attach(m_Label_explain, 	0, 0, 2, 2);
    
    int i = 2; 
   
	if(b_best)
	{
		m_best->set(BEST_IMG_FILE);
		m_Grid.attach(m_Label_best, 0, i, 1, 1);
		m_Grid.attach(*m_best, 	0, i+1, 1, 1);
		i+=2;
	}

	if(b_safe)
	{
		m_safe->set(SAFE_IMG_FILE);
		m_Grid.attach(m_Label_safe, 0, i, 1, 1);
		m_Grid.attach(*m_safe, 	0, i+1, 1, 1);
		i+=2;
	}
	
	if(b_fast)
	{
		m_fast->set(FAST_IMG_FILE);
		m_Grid.attach(m_Label_fast, 0, i, 1, 1);
		m_Grid.attach(*m_fast, 	0, i+1, 1, 1);
		i+=2;
	}
	
	//By default show the best route (if, from any reason, the user didn't choose routes to display)
	if(!b_safe && !b_fast && !b_best)
	{
		m_best->set(BEST_IMG_FILE);
		m_Grid.attach(m_Label_best, 0, i, 1, 1);
		m_Grid.attach(*m_best, 	0, i+1, 1, 1);
	}
		
		
    show_all_children();
}

PRM_Gui_img::~PRM_Gui_img()
{

}
