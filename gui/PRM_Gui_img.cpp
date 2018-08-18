#include "PRM_Gui_img.h"


PRM_Gui_img::PRM_Gui_img():
  m_VBox(Gtk::ORIENTATION_HORIZONTAL),
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
  
   // m_Grid.attach(m_Label_explain, 	0, 0, 2, 2);
    
    int i = 0; 
   
	if(b_best)
	{
		m_best->set(BEST_IMG_FILE);
		m_Grid.attach(m_Label_best, i, 0, 2, 1);
		m_Label_best.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_best, 	i, 1, 1, 1);
		m_best->set_halign(Gtk::ALIGN_CENTER);
	//	m_Grid.attach(m_lable_empty1, i+1, 1, 1, 1);
		m_best->set_margin_right(20);
		i+=2;
	}

	if(b_safe)
	{
		m_safe->set(SAFE_IMG_FILE);
		m_Grid.attach(m_Label_safe, i, 0, 2, 1);
		m_Label_safe.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_safe, 	i, 1, 1, 1);
		m_safe->set_halign(Gtk::ALIGN_CENTER);
		m_safe->set_margin_right(20);

	//	m_Grid.attach(m_lable_empty2, i+1, 1, 1, 1);
		i+=2;
	}
	
	if(b_fast)
	{
		m_fast->set(FAST_IMG_FILE);
		m_Grid.attach(m_Label_fast, i, 0, 2, 1);
		m_Label_fast.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_fast, 	i, 1, 1, 1);
		m_fast->set_halign(Gtk::ALIGN_CENTER);

		i++;
	}
	
	//By default show the best route (if, from any reason, the user didn't choose routes to display)
	if(!b_safe && !b_fast && !b_best)
	{
		m_best->set(BEST_IMG_FILE);
		m_Grid.attach(m_Label_best, i, 0, 2, 1);
		m_Label_best.set_halign(Gtk::ALIGN_CENTER);
		m_Grid.attach(*m_best, 	i, 1, 1, 1);
		m_best->set_halign(Gtk::ALIGN_CENTER);
	}
		
		
    show_all_children();
}

PRM_Gui_img::~PRM_Gui_img()
{

}
