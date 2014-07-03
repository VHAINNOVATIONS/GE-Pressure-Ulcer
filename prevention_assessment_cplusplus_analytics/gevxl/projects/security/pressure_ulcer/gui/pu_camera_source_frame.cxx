// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_camera_source_frame.h"

// Our namespace.
using namespace gevxl::gui;
using namespace gevxl::gui::wx;
using namespace gevxl::pressure_ulcer;

void options_handler_client::update_menu( wxMenu *menu )
{
  append(menu,"&Configure",(wxCommandEventFunction)&options_handler_client::on_configure, "Change Process Prameters.");
}

void options_handler_client::on_configure(wxCommandEvent &evt)
{
  gevxl::gui::wx::settings_dialog dlg(parent_);
  dlg.add_page("Settings");  
}


pu_camera_source_frame::~pu_camera_source_frame(void) 
{
  if(!get_process_control()->is_terminated()) {
    get_process_control()->terminate_execution();
    get_process_control()->wait_until_terminated();
  }

  viz_.set_output_canvas(NULL);

  proc_->set_visualizer(NULL);
  
  canvas_->set_proc_visualizer(NULL);

  canvas_->remove_client(options_handler_client_);
  canvas_->remove_client(pixel_picking_client_);

  if(NULL != record_client_) {
    RemoveEventHandler(record_client_);
    delete record_client_;
    record_client_ = NULL;
  }

  if(NULL != options_handler_client_) {
    RemoveEventHandler(options_handler_client_);
    delete options_handler_client_;
    options_handler_client_ = NULL;
  }

  if(NULL != pixel_picking_client_) {
    RemoveEventHandler(pixel_picking_client_);
    delete pixel_picking_client_;
    pixel_picking_client_ = NULL;
  }
  
  // the proc_ is owned by the MyApp class, so the frame shouldn't do anything to clean up this pointer
  if(NULL != canvas_) {
    delete canvas_;
    canvas_ = NULL;
  }
}

bool pu_camera_source_frame::initialize(void)
{
  int gl_attrib[10] = { WX_GL_RGBA,WX_GL_BUFFER_SIZE,32,WX_GL_DOUBLEBUFFER, 0 };

  // Allocate canvas
  canvas_ = new wx::canvas(get_panel(), wxid("Canvas"), wxDefaultPosition, wxSize(proc_->cur_frame().ni(), proc_->cur_frame().nj()), 0, ("PUDepthCanvas"), gl_attrib );
  canvas_->set_process_control(get_process_control());
 
  // Canvas on top.
  top_sizer_->Add(canvas_, 1, wxGROW, 0);

  // Visualizer visualizes into canvas.
  viz_.set_output_canvas(canvas_);

  // Tell canvas about visualizer (needed for resizing when proc is paused).
  canvas_->set_proc_visualizer(&viz_);

  // Tell OpenGL visualizer owned by proc about a secondary visualizer 
  // used by clients for overlays.
  viz_.add_visualizer(&overlay_viz_);

  // Tell canvas about this overlay visualizer. This visualizer can be 
  // used by clients during events such as on_mouse().
  canvas_->set_visualizer(&overlay_viz_);

  // Don't use swap buffer if we don't double buffer.
  viz_.use_swap_buffer_on_flush(true);

  // Add a canvas view client.
  // This is actually a handler client.
  canvas_->add_canvas_view_client();
  
  record_client_ = new gevxl::gui::wx::frame_record_client;
  record_client_->set_visualizer(&viz_);
  add_client(record_client_);   

  options_handler_client_ = new options_handler_client();
  options_handler_client_->set_proc(proc_);
  add_client(options_handler_client_);

  pixel_picking_client_ = new pu_pixel_picking_client();
  pixel_picking_client_->set_proc(proc_);
  add_client(pixel_picking_client_);
  canvas_->add_client(pixel_picking_client_);

  if(get_menu_bar()) {
    record_client_->update_menu_as_submenu(get_menu_bar(), "&Record");
    options_handler_client_->update_menu_as_submenu(get_menu_bar(), "&Modify Params");
    pixel_picking_client_->update_menu_as_submenu(get_menu_bar(), "&Pixel Picking");
  }

  // Tell process about visualizer.
  proc_->set_visualizer(&viz_);

  return true;
}
