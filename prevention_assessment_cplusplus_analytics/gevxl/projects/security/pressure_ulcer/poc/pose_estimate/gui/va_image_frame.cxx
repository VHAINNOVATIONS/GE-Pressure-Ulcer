//GE

/// \file
/// \author Peter Tu


#include "va_image_frame.h"
#include "va_image_client.h"



#include <vcl_cstdlib.h>

#include <wx/menu.h>
#include <gui/wx/wxid.h>

using namespace gevxl;
using namespace gevxl::gui;
using gevxl::gui::wx::wxid;
using namespace gevxl::img;
using namespace gesec::detectors;


// My frame constructor
va_image_frame::va_image_frame(wxFrame *f, const wxString& title, const wxPoint& pos,
                          const wxSize& size, long style)
  : wxFrame(f, wxID_ANY, title, pos, size, style)
  , double_buffer_(false)
{
  canvas_[0] = NULL;
  config_=0;
  Connect(wxid("TimerID"), wxEVT_TIMER, 
         wxTimerEventHandler(va_image_frame::on_tick),
         NULL,this);
}

void va_image_frame::init(bool double_buffer)
{
  double_buffer_=double_buffer;
    
  setup_canvas();
  setup_layout();
  setup_visualization();
  setup_clients();
  setup_menus();
}

void va_image_frame::setup_canvas(void)
{
  /////////////////////////////////////////////////////////////////////////////////
  // Set up OpenGL drawing area. Represented by a canvas object.
  /////////////////////////////////////////////////////////////////////////////////

  // Setting these attributes leads to high performance on Windows.
  int gl_attrib[10] = { WX_GL_RGBA,WX_GL_BUFFER_SIZE,32,0*WX_GL_DOUBLEBUFFER, 0 };

  panel_ = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize);

  canvas_[0] = new wx::canvas(panel_, wxid(), wxDefaultPosition, 
                           wxSize(500,500), 0*wxRAISED_BORDER,
                           ("Camera Image"), gl_attrib );



}

void va_image_frame::setup_layout(void)
{
  /////////////////////////////////////////////////////////////////////////////////
  // Set up object that manages layout and resizing.
  /////////////////////////////////////////////////////////////////////////////////

  // Container for a stack of controls.
  wxBoxSizer *canvasSizer = new wxBoxSizer( wxHORIZONTAL );

  canvasSizer->Add(canvas_[0],1,wxALL|wxEXPAND,1);
 
  // Container for a stack of controls.
  wxBoxSizer *topsizer = new wxBoxSizer( wxVERTICAL );

  // Canvas on top.
  topsizer->Add(canvasSizer,1,wxBOTTOM|wxEXPAND,2);

  // Attach sizer to this Frame.
 
  panel_->SetAutoLayout( true );
  panel_->SetSizer( topsizer );

  topsizer->SetSizeHints( this );

}

void va_image_frame::setup_visualization(void)
{
  /////////////////////////////////////////////////////////////////////////////////
  // Tell video process (set up above) where to visualize into.
  /////////////////////////////////////////////////////////////////////////////////

  // Visualizer visualizes into canvas.
  viz_[0].set_output_canvas(canvas_[0]);


  
  // Tell canvas about visualizer (needed for resizing when proc is paused).
  canvas_[0]->set_proc_visualizer(&viz_[0]);


  // Tell OpenGL visualizer owned by proc about a secondary visualizer 
  // used by clients for overlays.
  viz_[0].add_visualizer(&overlay_viz_[0]);


  // Tell canvas about this overlay visualizer. This visualizer can be 
  // used by clients during events such as on_mouse().
  canvas_[0]->set_visualizer(&overlay_viz_[0]);


  // Use swap buffer if we double buffer.
  viz_[0].use_swap_buffer_on_flush(double_buffer_);

}

void va_image_frame::setup_clients(void)
{
  /////////////////////////////////////////////////////////////////////////////////
  // Allocate clients. One va and one attached to canvas.
  /////////////////////////////////////////////////////////////////////////////////

  view_client_[0]=new wx::canvas_view_client;


  // Connect view client to canvas.
  canvas_[0]->add_client(view_client_[0]);


  //: put in an image io client
  io_client_[0] = new img::image_io_client;
  canvas_[0]->add_client(io_client_[0]);
  io_client_[0]->set_canvas(canvas_[0]);

  // set up an image filter client
  va_image_client_ = new va_image_client;
  va_image_client_->set_canvas(canvas_[0]);
    
  canvas_[0]->get_view().set_stretch_request();
  // set up an image filter client
  ifm_client_ = new gevxl::img::image_filter_client;
  ifm_client_->set_input_canvas(canvas_[0]);
  ifm_client_->set_output_canvas(canvas_[0]);
 
  


    

  // NOTE this should be configured by the app
  if(config_){
      va_image_client_->configure(*config_);
  }
  else{
    vcl_cout << "Warning no configuration file specified" << vcl_endl;
  }
  // Attach handler of client to previous client.
  canvas_[0]->add_client(va_image_client_);
  canvas_[0]->add_client(ifm_client_);
}

void va_image_frame::setup_menus(void)
{
  /////////////////////////////////////////////////////////////////////////////////
  // Setup some menus.
  /////////////////////////////////////////////////////////////////////////////////

  // Menubar
  wxMenuBar* menuBar = new wxMenuBar( wxMB_DOCKABLE );

  io_client_[0]->update_menu_as_submenu(menuBar,"File");
  va_image_client_->update_menu_as_submenu(menuBar,"VA Processing");
  ifm_client_->update_menu_as_submenu(menuBar,"Image Filters");
  SetMenuBar(menuBar);
}

va_image_frame::~va_image_frame()
{
  // Stop process.

  delete canvas_[0];
  delete panel_;
}

void va_image_frame::on_tick(wxTimerEvent& event)
{
  //vcl_cout << "va_image_frame::on_tick()\n";
  
  event.Skip();
}


