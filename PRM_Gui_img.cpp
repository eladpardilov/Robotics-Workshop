#include "PRM_Gui_img.h"


PRM_Gui_img::PRM_Gui_img(bool safe, bool best, bool fast):
  m_VBox(Gtk::ORIENTATION_VERTICAL),
  m_Label_best("Best route: "),
  m_Label_safe("Safest route: "),
  m_Label_fast("Fastest route: ")
{
	set_title("Save The Man - The Selected Routes");
	set_border_width(12);
	
	m_bBest = best;
	m_bSafe = safe;
	m_bFast = fast;
	 
	add(m_VBox);
	 
	m_best = Gtk::manage(new Gtk::Image());
	m_fast = Gtk::manage(new Gtk::Image());
	m_safe = Gtk::manage(new Gtk::Image());
	 
	m_VBox.pack_start(m_Grid);
    m_Grid.set_row_homogeneous(false);
    m_Grid.set_column_homogeneous(false);
    m_Grid.set_row_spacing(5);
      
    int i = 0; 
   
	if(m_bBest)
	{
		m_best->set(BEST_IMG_FILE);
		//m_Grid.attach(m_Label_best, i, 0, 1, 1);
		//m_Label_best.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_best, 	i, 1, 1, 1);
		m_best->set_halign(Gtk::ALIGN_CENTER);
		m_best->set_margin_right(20);
		i+=2;
	}

	if(m_bSafe)
	{
		m_safe->set(SAFE_IMG_FILE);
		m_Grid.attach(m_Label_safe, i, 0, 1, 1);
		m_Label_safe.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_safe, 	i, 1, 1, 1);
		m_safe->set_halign(Gtk::ALIGN_CENTER);
		m_safe->set_margin_right(20);
		i+=2;
	}
	
	if(m_bFast)
	{
		m_fast->set(FAST_IMG_FILE);
		m_Grid.attach(m_Label_fast, i, 0, 1, 1);
		m_Label_fast.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_fast, 	i, 1, 1, 1);
		m_fast->set_halign(Gtk::ALIGN_CENTER);
		i++;
	}
	
	//By default show the best route 
	//(if, from any reason, the user didn't choose routes to display)
	if(!m_bSafe && !m_bFast && !m_bBest)
	{
		m_best->set(BEST_IMG_FILE);
		m_Grid.attach(m_Label_best, i, 0, 1, 1);
		m_Label_best.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_best, 	i, 1, 1, 1);
		m_best->set_halign(Gtk::ALIGN_CENTER);
	}
		
		
    show_all_children();
}

PRM_Gui_img::~PRM_Gui_img()
{

}
