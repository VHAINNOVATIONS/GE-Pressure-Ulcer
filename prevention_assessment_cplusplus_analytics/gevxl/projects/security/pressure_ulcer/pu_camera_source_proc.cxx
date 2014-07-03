// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/25/2013
/// \par Modifications:


#include "pu_camera_source_proc.h"

#include <vcl_iostream.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>

#ifdef GEVXL_VID_HAS_OPENNI2
#include <vid/openni2_frame_process.h>
#endif

#include <vid/micro_epsilon_socket_frame_process.h>

#ifdef GEVXL_VID_HAS_PCSDK
#include <vid/pxc_frame_process.h>
#endif

using namespace gevxl;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::util::time;

pu_camera_source_proc::pu_camera_source_proc(char const *name)
: gevxl::framework::process(name), 	
  source_proc_("vid::source_process"), 
  viz_(NULL),
	frame_nr_(0),
  source_process_type_("openni2"),
  source_output_type_("depth")
{
	highres_timer_.reset();
}

pu_camera_source_proc::~pu_camera_source_proc(void)
{

}

bool pu_camera_source_proc::configure(util::config_file &config)
{
	config_ = config;

  source_process_type_ = "openni2";
  //source_process_type_ = "micro_epsilon_socket";
  //source_process_type_ = "pcsdk";
  config_.get_string(name()+"::source_process_type", source_process_type_);

  source_output_type_ = "depth";
  //source_output_type_ = "rgb" // for 3D depth camera, output type = "rgb" and "depth" and "rgb_in_depth_view"
  //source_output_type_ = "grayscale" // for thermal camera, output type = "grayscale" and "color"
  config_.get_string(name()+"::source_output_type", source_output_type_);
  
  // ---- source configuration 
	if( !source_proc_.configure(config_) ) {
		vcl_cerr << "pu_camera_source_proc::configure, Error configuring source_proc_." << vcl_endl;
		return false;
	}

  if(source_process_type_ == "openni2") {
    if(source_output_type_ != "rgb" && source_output_type_ != "depth") {
      // ensure the output is the depth view
      source_proc_.set_output_type("depth");
    }
    else {
      source_proc_.set_output_type(source_output_type_);
    }
  }
  else if(source_process_type_ == "micro_epsilon_socket") {
    if(source_output_type_ != "color" && source_output_type_ != "grayscale") {
      // ensure the output is the grayscale
      source_proc_.set_output_type("grayscale");
    }
    else {
      source_proc_.set_output_type(source_output_type_);
    }
  }
  else if(source_process_type_ == "pcsdk") {
    if(source_output_type_ != "rgb" && source_output_type_ != "depth" && source_output_type_ != "rgb_in_depth_view") {
      // ensure the output is the depth view
      source_proc_.set_output_type("depth");
    }
    else {
      source_proc_.set_output_type(source_output_type_);
    }
  }
  else {
    vcl_cerr << "pu_camera_source_proc::configure, Error unsupported source process type." << vcl_endl;
    return false;
  }

  // configure the frame tag
  if(!frame_tag_proc_.configure(config_)) {
    vcl_cerr << "pu_camera_source_proc::configure, Error configuring frame_tag_proc_." << vcl_endl;
    return false;
  }
  
  return true;
}

bool pu_camera_source_proc::initialize(void)
{
  if(!source_proc_.initialize()) {
    vcl_cerr << "pu_camera_source_proc::initialize, Error initializing source_proc_." << vcl_endl;		
    return false;
  }

  if(source_process_type_ == "openni2") {
#ifdef GEVXL_VID_HAS_OPENNI2
    const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
    frame_tag_proc_.set_source_tagged_frame_process(openni2_source);
    if(!frame_tag_proc_.initialize()) {
      vcl_cerr << "pu_camera_source_proc::initialize, Error initializing frame_tag_proc_." << vcl_endl;		
      return false;
    }
#endif
  }
  else if(source_process_type_ == "micro_epsilon_socket") {
    const gevxl::vid::micro_epsilon_socket_frame_process *micro_epsilon_socket_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(source_proc_.get_frame_process());
    frame_tag_proc_.set_source_tagged_frame_process(micro_epsilon_socket_source);
    if(!frame_tag_proc_.initialize()) {
      vcl_cerr << "pu_camera_source_proc::initialize, Error initializing frame_tag_proc_." << vcl_endl;		
      return false;
    }
  }
  else if(source_process_type_ == "pcsdk") {
#ifdef GEVXL_VID_HAS_PCSDK
    const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(source_proc_.get_frame_process());
    frame_tag_proc_.set_source_tagged_frame_process(pxc_source);
    if(!frame_tag_proc_.initialize()) {
      vcl_cerr << "pu_camera_source_proc::initialize, Error initializing frame_tag_proc_." << vcl_endl;		
      return false;
    }
#endif
  }
  else {
    vcl_cerr << "pu_camera_source_proc::initialize, Error unsupported source process type." << vcl_endl;
    return false;
  }

 	return true;
}

void pu_camera_source_proc::uninitialize(void)
{
  
}

bool pu_camera_source_proc::step(vil_image_view<vxl_byte> img)
{
  //Note the viz image size must be a 4 multiplier.
  int ni = vcl_ceil (double(img.ni())/4)*4;
  int nj = vcl_ceil (double(img.nj())/4)*4;
  curr_frame_ = vil_image_view<vxl_byte> (ni, nj, 1, img.nplanes());
  curr_frame_.fill (0);

  for (int j=0; j<img.nj(); j++) {
    for (int i=0; i<img.ni(); i++) {
      for (int k=0; k<img.nplanes(); k++) {
        curr_frame_(i,j,k) = img(i,j,k);
      }
    }
  }

  //curr_frame_.deep_copy(img);

  if(viz_) {
		gevxl::threading::scoped_lock lock( viz_ );
		viz_->initialize();
		if (viz_->is_initialized()) { 
			viz_->set_image(curr_frame_);
		}
	}

  // deal with the frame rate issue.
	double frame_rate = 1000/highres_timer_.elapsed();	
	highres_timer_.reset();

	gevxl::util::time::highres_timer state_timer;
	state_timer.reset();

  ////////////////////////////////////////////////////////////////////////
	// Visualization of the process
  visualize();

  prev_frame_.deep_copy(curr_frame_);

#if 0
  vil_save (curr_frame_, "curr_frame.png");
#endif
	
  return true;
}


bool pu_camera_source_proc::step(void)
{
  if(!source_proc_.step()) {
    vcl_cerr << "pu_camera_source_proc::step, stepping source_proc_ error." << vcl_endl;
		return false;
  }

  if(!frame_tag_proc_.step()) {
    vcl_cerr << "pu_camera_source_proc::step, stepping frame_tag_proc_ error." << vcl_endl;
		return false;
  }
  
  frame_tag_proc_.set_frame_id(frame_nr_);
  frame_tag_proc_.set_frame_nr(frame_nr_);
  frame_tag_proc_.set_time_code((1000.0/30.0)*frame_nr_);
  frame_nr_++;

  vil_image_view<vxl_byte> img = source_proc_.cur_frame();

	return step(img);	
}

void pu_camera_source_proc::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;
}

void pu_camera_source_proc::visualize(void)
{
	// Visualization stuff of each processes    	
	
	IF_CAN_VISUALIZE( viz_ ) {

	  viz_->flush();
	}
}

