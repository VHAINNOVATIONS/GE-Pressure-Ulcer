//
// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
//

/// \file
/// \author Yi Yao
/// \date 6/5/2012
/// \par Modifications:
/// - Original version

#include <wx/menu.h>
#include "pu_kinectsdk_frame.h"

using namespace gevxl;
using namespace gevxl::gui;
using namespace gesec::smartroom;

/// frame constructor
sec_skeleton_frame::sec_skeleton_frame()
  : gui::wx::frame( NULL, "Skeleton", wxDefaultPosition, wxDefaultSize ),
    canvas_(NULL),
    record_client_(NULL)
{      
}

sec_skeleton_frame::~sec_skeleton_frame()
{   
  //viz_.set_output_canvas(NULL);
  //control_proc_->set_visualizer(NULL);
  //canvas_->set_proc_visualizer(NULL);
  canvas_->remove_client(record_client_);
  //canvas_->remove_client(bed_detection_client_);
  //canvas_->remove_client(kinect_camera_client_);
  
  if(NULL != record_client_) {
    delete record_client_;
    record_client_ = NULL;
  }
  
  if(NULL != canvas_) {
    delete canvas_;
    canvas_ = NULL;
  }
}

bool sec_skeleton_frame::initialize()
{   
  setup_canvas();
  setup_layout();
  setup_visualization();
  setup_clients();
  setup_menus();

  return true;
}

/// Set up OpenGL drawing area. Represented by a canvas object.
void sec_skeleton_frame::setup_canvas(void)
{ 
    // Setting these attributes leads to high performance on Windows.
    int gl_attrib[10] = { WX_GL_RGBA,WX_GL_BUFFER_SIZE,32, WX_GL_DOUBLEBUFFER, 0 };

    canvas_ = new wx::canvas(get_panel(), wxID_ANY, wxDefaultPosition, 
                              wxSize(640,480), 0*wxRAISED_BORDER,
                            ("Smartroom depth"), gl_attrib  );
   
    canvas_->set_process_control( get_process_control() );    
}


/// Set up object that manages layout and resizing.
void sec_skeleton_frame::setup_layout(void)
{    
    // Canvas on top.
    top_sizer_->Add(canvas_,1,wxGROW,0);
    //top_sizer_->SetSizeHints(this);
}

/// Tell video process (set up above) where to visualize into.
void sec_skeleton_frame::setup_visualization(void)
{
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
    // Use swap buffer if we double buffer.
    viz_.use_swap_buffer_on_flush(true);  
}

// Allocate clients.
void sec_skeleton_frame::setup_clients(void)
{  
    // canvas_view_client allows us to zoom in/out and move the images attached to the canvas
    canvas_->add_canvas_view_client();   
    canvas_->get_view().set_stretch_request();

    //bed_detection_client_ = new gesec::smartroom::bed_detection_client;
    //bed_detection_client_->set_process(control_proc_->bed_detection_proc());
    //canvas_->add_client(bed_detection_client_);
    //bed_detection_client_->add_event();


    //kinect_camera_client_ = new gesec::smartroom::kinect_camera_client;
    //kinect_camera_client_->set_process(control_proc_->frame_process());
    //canvas_->add_client(kinect_camera_client_);

    // Attach the recording client to the canvas
    record_client_ = new gui::wx::canvas_record_client;
    canvas_->add_client(record_client_);
//    add_client(record_client_);   
}

void sec_skeleton_frame::setup_menus(void)
{    
  if( get_menu_bar() )
  {
    //kinect_camera_client_->update_menu_as_submenu( get_menu_bar(), ("&Kinect camera") );
    //bed_detection_client_->update_menu_as_submenu( get_menu_bar(), ("&Bed detection") );
    record_client_->update_menu_as_submenu( get_menu_bar(), ("&Record") );        
  }
}
