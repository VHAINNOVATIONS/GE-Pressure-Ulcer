// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_prevention_frame.h"

// Our namespace.
using namespace gevxl;
using namespace gevxl::gui;
using namespace gevxl::gui::wx;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::prevention;

pu_prevention_frame::~pu_prevention_frame(void) 
{
  if(!get_process_control()->is_terminated()) {
    get_process_control()->terminate_execution();
    get_process_control()->wait_until_terminated();
  }

  viz_.set_output_canvas(NULL);

	prevention_chaining_proc_->set_visualizer(NULL);

  canvas_->set_proc_visualizer(NULL);
  
  canvas_->remove_client(pose_estimate_client_);

	if(NULL != record_client_) {
    RemoveEventHandler(record_client_);
    delete record_client_;
    record_client_ = NULL;
  }

  if(NULL != pose_estimate_client_) {
    RemoveEventHandler(pose_estimate_client_);
    delete pose_estimate_client_;
    pose_estimate_client_ = NULL;
  }

	if(NULL != canvas_) {
    delete canvas_;
    canvas_ = NULL;
  }
}

/// Configure the prevention frame from the config_file
bool pu_prevention_frame::configure(gevxl::util::config_file &config)
{
	config_ = config;
	return true;
}

bool pu_prevention_frame::initialize(void)
{
  int gl_attrib[10] = { WX_GL_RGBA,WX_GL_BUFFER_SIZE,32,WX_GL_DOUBLEBUFFER, 0 };

  // Allocate canvas  
	canvas_ = new wx::canvas(get_panel(), wxid("Canvas"), wxDefaultPosition, wxSize(1280, 720), 0, ("PUAssessmentSystemCanvas"), gl_attrib );

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

  pose_estimate_client_ = new pu_pose_estimate_client();
	pose_estimate_client_->set_prevention_chaining_proc(prevention_chaining_proc_);	
  add_client(pose_estimate_client_);
  canvas_->add_client(pose_estimate_client_);

	if(get_menu_bar()) {
    record_client_->update_menu_as_submenu(get_menu_bar(), "&Record");    
    pose_estimate_client_->update_menu_as_submenu(get_menu_bar(), "&Pose Estimate");    
  }

  // Tell process about visualizer.
	prevention_chaining_proc_->set_visualizer(&viz_);

  return true;
}

