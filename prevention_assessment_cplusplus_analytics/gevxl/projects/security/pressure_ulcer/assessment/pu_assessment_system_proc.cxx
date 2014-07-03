// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/09/2013
/// \par Modifications:

#include "pu_assessment_system_proc.h"

#include <vcl_iostream.h>

#include <vil/vil_convert.h>

#include <vil/vil_load.h>
#include <vil/vil_save.h>

#include <vid/pxc_frame_process.h>
#include <vid/micro_epsilon_socket_frame_process.h>
#include <vid/bayspec_socket_frame_process.h>

#include <vcl_algorithm.h>
#include <vul/vul_sprintf.h>

using namespace gevxl;
using namespace gevxl::util;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::assessment;

pu_assessment_system_proc::pu_assessment_system_proc(char const *name)
: gevxl::framework::process(name), 	
  viz_(NULL),
	rgb_viz_(NULL),
	depth_viz_(NULL),
	thermal_viz_(NULL),
  rgb_copy_viz_(NULL),
	hyperspectral_viz_(NULL),
  rgb_touch_viz_(NULL),
  thermal_touch_viz_(NULL),
  hyperspectral_touch_viz_(NULL),
	frame_nr_(0),
	rgbd_cam_source_proc_("vid::pu_assessment_rgbd_cam_source_process"), 
	rgbd_cam_source_output_type_("depth"), 
	thermal_cam_source_proc_("vid::pu_assessment_thermal_cam_source_process"),
	thermal_cam_source_output_type_("color"),
  hyperspectral_cam_source_proc_("vid::pu_assessment_hyperspectral_cam_source_process"),
  hyperspectral_cam_source_output_type_("grayscale"),
	async_rgbd_cam_source_proc_(NULL),
	async_thermal_cam_source_proc_(NULL),
	verbose_(0)
{
	highres_timer_.reset();
}

pu_assessment_system_proc::~pu_assessment_system_proc(void)
{

}

bool pu_assessment_system_proc::configure(util::config_file &config)
{
	config_ = config;

	verbose_ = 0;
	config_.get_integer(name()+"::verbose", verbose_);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// configure the rgbd_cam_source_proc_
	// rgb-d camera source configuration 
	if( !rgbd_cam_source_proc_.configure(config_) ) {
		vcl_cerr << "pu_assessment_system_proc::configure, Error configuring rgbd_cam_source_proc_." << vcl_endl;
		return false;
	}

	// rgb-d camera source output frame type configuration	
	rgbd_cam_source_output_type_ = "depth";	// for 3D depth camera, output type = "rgb" and "depth" and "rgb_in_depth_view"  
  config_.get_string(name()+"::rgbd_cam_source_output_type", rgbd_cam_source_output_type_);
    
	if(rgbd_cam_source_output_type_ != "rgb" && rgbd_cam_source_output_type_ != "depth" && rgbd_cam_source_output_type_ != "rgb_in_depth_view") {
		// ensure the output is the depth view
		rgbd_cam_source_proc_.set_output_type("depth");
  }
	else {
		rgbd_cam_source_proc_.set_output_type(rgbd_cam_source_output_type_);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// configure the thermal_cam_source_proc_
	// thermal camera source configuration
	if( !thermal_cam_source_proc_.configure(config_) ) {
		vcl_cerr << "pu_assessment_system_proc::configure, Error configuring thermal_cam_source_proc_." << vcl_endl;
		return false;
	}

	// thermal camera source output frame type configuration
	thermal_cam_source_output_type_ = "color";	// for thermal camera, output type = "color" and "grayscale"
	config_.get_string(name()+"::thermal_cam_source_output_type", thermal_cam_source_output_type_);

  if(thermal_cam_source_output_type_ != "color" && thermal_cam_source_output_type_ != "grayscale") {
		// ensure the output is the color view
		thermal_cam_source_proc_.set_output_type("color");
  }
	else {
		thermal_cam_source_proc_.set_output_type(thermal_cam_source_output_type_);
	}

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// configure the hyperspectral_cam_source_proc_
	// hyperspectral camera source configuration
	if( !hyperspectral_cam_source_proc_.configure(config_) ) {
		vcl_cerr << "pu_assessment_system_proc::configure, Error configuring hyperspectral_cam_source_proc_." << vcl_endl;
		return false;
	}

	// hyperspectral camera source output frame type configuration
	hyperspectral_cam_source_output_type_ = "grayscale";	// for hyperspectral camera, output type = "color" and "grayscale"
	config_.get_string(name()+"::hyperspectral_cam_source_output_type", hyperspectral_cam_source_output_type_);

  if(hyperspectral_cam_source_output_type_ != "color" && hyperspectral_cam_source_output_type_ != "grayscale") {
		// ensure the output is the grayscale view
    hyperspectral_cam_source_proc_.set_output_type("grayscale");
  }
	else {
		hyperspectral_cam_source_proc_.set_output_type(hyperspectral_cam_source_output_type_);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // configure the GUI visualization properties
  viz_depth_or_color_in_depth_ = "depth";
  config_.get_string(name()+"::viz_depth_or_color_in_depth", viz_depth_or_color_in_depth_);

  optimal_snapshot_distance_ = 0.4;
  config_.get_double(name()+"::optimal_snapshot_distance", optimal_snapshot_distance_);

  optimal_snapshot_distance_tolerance_ = 0.05;
  config_.get_double(name()+"::optimal_snapshot_distance_tolerance", optimal_snapshot_distance_tolerance_);

  max_color_offset_ = 150;
  config_.get_integer(name()+"::max_color_offset", max_color_offset_);

  int viz_outer_rect_to_image_border = 60;
  config_.get_integer(name()+"::viz_outer_rect_to_image_border", viz_outer_rect_to_image_border);
  viz_outer_rect_.x0 = viz_outer_rect_to_image_border;
  viz_outer_rect_.y0 = viz_outer_rect_to_image_border;
  viz_outer_rect_.x1 = 320 - viz_outer_rect_to_image_border + 1;
  viz_outer_rect_.y1 = 240 - viz_outer_rect_to_image_border + 1;

  int viz_inner_rect_to_image_border = 80;
  config_.get_integer(name()+"::viz_inner_rect_to_image_border", viz_inner_rect_to_image_border);
  viz_inner_rect_.x0 = viz_inner_rect_to_image_border;
  viz_inner_rect_.y0 = viz_inner_rect_to_image_border;
  viz_inner_rect_.x1 = 320 - viz_inner_rect_to_image_border + 1;
  viz_inner_rect_.y1 = 240 - viz_inner_rect_to_image_border + 1;

  ////////////////////////////////////////////////////////////////////////////////////////////
  // visualization boxes inside the rgb view
  // depth in rgb
  vcl_vector<int> depth_in_rgb_viz_inner_rect_x0y0x1y1;
  depth_in_rgb_viz_inner_rect_x0y0x1y1.clear();
  config_.get_vcl_vector_int(name()+"::depth_in_rgb_viz_inner_rect_x0y0x1y1", depth_in_rgb_viz_inner_rect_x0y0x1y1);
  if(depth_in_rgb_viz_inner_rect_x0y0x1y1.size() != 4) {
    depth_in_rgb_viz_inner_rect_x0y0x1y1.resize(4);
    depth_in_rgb_viz_inner_rect_x0y0x1y1[0] = 160;  //x0
    depth_in_rgb_viz_inner_rect_x0y0x1y1[1] = 60;   //y0
    depth_in_rgb_viz_inner_rect_x0y0x1y1[2] = 640 - depth_in_rgb_viz_inner_rect_x0y0x1y1[0];  //x1
    depth_in_rgb_viz_inner_rect_x0y0x1y1[3] = 360 - depth_in_rgb_viz_inner_rect_x0y0x1y1[1];  //y1  
  }
  depth_in_rgb_viz_inner_rect_.x0 = depth_in_rgb_viz_inner_rect_x0y0x1y1[0];
  depth_in_rgb_viz_inner_rect_.y0 = depth_in_rgb_viz_inner_rect_x0y0x1y1[1];
  depth_in_rgb_viz_inner_rect_.x1 = depth_in_rgb_viz_inner_rect_x0y0x1y1[2];
  depth_in_rgb_viz_inner_rect_.y1 = depth_in_rgb_viz_inner_rect_x0y0x1y1[3];

  int depth_in_rgb_viz_rect_border_width = 20;
  config_.get_integer(name()+"::depth_in_rgb_viz_rect_border_width", depth_in_rgb_viz_rect_border_width);
  depth_in_rgb_viz_outer_rect_.x0 = depth_in_rgb_viz_inner_rect_.x0 - depth_in_rgb_viz_rect_border_width;
  depth_in_rgb_viz_outer_rect_.y0 = depth_in_rgb_viz_inner_rect_.y0 - depth_in_rgb_viz_rect_border_width;
  depth_in_rgb_viz_outer_rect_.x1 = depth_in_rgb_viz_inner_rect_.x1 + depth_in_rgb_viz_rect_border_width;
  depth_in_rgb_viz_outer_rect_.y1 = depth_in_rgb_viz_inner_rect_.y1 + depth_in_rgb_viz_rect_border_width;

  // hyperspectral in rgb  
  vcl_vector<int> hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1;
  hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1.clear();
  config_.get_vcl_vector_int(name()+"::hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1", hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1);
  if(hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1.size() != 4) {
    hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1.resize(4);
    hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[0] = 300;  //x0
    hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[1] = 100;  //y0
    hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[2] = 500;  //x1
    hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[3] = 300;  //y1
  }
  hyperspectral_in_rgb_viz_inner_rect_.x0 = hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[0];
  hyperspectral_in_rgb_viz_inner_rect_.y0 = hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[1];
  hyperspectral_in_rgb_viz_inner_rect_.x1 = hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[2];
  hyperspectral_in_rgb_viz_inner_rect_.y1 = hyperspectral_in_rgb_viz_inner_rect_x0y0x1y1[3];

  int hyperspectral_in_rgb_viz_rect_border_width = 10;
  config_.get_integer(name()+"::hyperspectral_in_rgb_viz_rect_border_width", hyperspectral_in_rgb_viz_rect_border_width);
  hyperspectral_in_rgb_viz_outer_rect_.x0 = hyperspectral_in_rgb_viz_inner_rect_.x0 - hyperspectral_in_rgb_viz_rect_border_width;
  hyperspectral_in_rgb_viz_outer_rect_.y0 = hyperspectral_in_rgb_viz_inner_rect_.y0 - hyperspectral_in_rgb_viz_rect_border_width;
  hyperspectral_in_rgb_viz_outer_rect_.x1 = hyperspectral_in_rgb_viz_inner_rect_.x1 + hyperspectral_in_rgb_viz_rect_border_width;
  hyperspectral_in_rgb_viz_outer_rect_.y1 = hyperspectral_in_rgb_viz_inner_rect_.y1 + hyperspectral_in_rgb_viz_rect_border_width;
  ////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  viz_canvas_width_ = 1280;
	config_.get_integer(name()+"::viz_canvas_width", viz_canvas_width_);

	viz_canvas_height_ = 720;
	config_.get_integer(name()+"::viz_canvas_height", viz_canvas_height_);

	viz_canvas_frame_ = vil_image_view<vxl_byte>(viz_canvas_width_, viz_canvas_height_, 1, 3);
	viz_canvas_frame_.fill(0);

	tissue_analysis_viz_offset_i_ = 0;
	config_.get_integer(name()+"::tissue_analysis_viz_offset_i", tissue_analysis_viz_offset_i_);
	tissue_analysis_viz_offset_j_ = 0;
	config_.get_integer(name()+"::tissue_analysis_viz_offset_j", tissue_analysis_viz_offset_j_);

	threed_recon_viz_offset_i_ = 640;
	config_.get_integer(name()+"::threed_recon_viz_offset_i", threed_recon_viz_offset_i_);
	threed_recon_viz_offset_j_ = 0;
	config_.get_integer(name()+"::threed_recon_viz_offset_j", threed_recon_viz_offset_j_);
				
	thermal_analysis_viz_offset_i_ = 640;
	config_.get_integer(name()+"::thermal_analysis_viz_offset_i", thermal_analysis_viz_offset_i_);
	thermal_analysis_viz_offset_j_ = 360;
	config_.get_integer(name()+"::thermal_analysis_viz_offset_j", thermal_analysis_viz_offset_j_);

	// tissue analysis process configuration
	tissue_analysis_proc_.configure(config);

	// 3d reconstruction process configuration
	threed_recon_proc_.configure(config);

  // thermal analysis process configuration
	thermal_analysis_proc_.configure(config);

  return true;
}

bool pu_assessment_system_proc::initialize(void)
{
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// initialize the rgbd_cam_source_proc_ and check whether it is truly the gevxl::vid::pxc_frame_process
	if(!rgbd_cam_source_proc_.initialize()) {
    vcl_cerr << "pu_assessment_system_proc::initialize, Error initializing rgbd_cam_source_proc_." << vcl_endl;		
    return false;
  }

	// check out whether the input video source process is the pxc_frame_process
	const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
	if(pxc_source == NULL) {
		vcl_cerr << "pu_assessment_system_proc::initialize, Error the video source process is not the pxc_frame_process." << vcl_endl;
		return false;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// initialize the thermal_cam_source_proc_ and check whether it is truly the gevxl::vid::micro_epsilon_socket_frame_process
	if(!thermal_cam_source_proc_.initialize()) {
    vcl_cerr << "pu_assessment_system_proc::initialize, Error initializing thermal_cam_source_proc_." << vcl_endl;		
    return false;
  }

  const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());
  if(thermal_source == NULL) {
		vcl_cerr << "pu_assessment_system_proc::initialize, Error the video source process is not the micro_epsilon_socket_frame_process." << vcl_endl;
		return false;
	}

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// initialize the hyperspectral_cam_source_proc_ and check whether it is truly the gevxl::vid::bayspec_socket_frame_process
	if(!hyperspectral_cam_source_proc_.initialize()) {
    vcl_cerr << "pu_assessment_system_proc::initialize, Error initializing hyperspectral_cam_source_proc_." << vcl_endl;		
    return false;
  }

  const gevxl::vid::bayspec_socket_frame_process *hyperspectral_source = dynamic_cast<const gevxl::vid::bayspec_socket_frame_process *>(hyperspectral_cam_source_proc_.get_frame_process());
  if(hyperspectral_source == NULL) {
		vcl_cerr << "pu_assessment_system_proc::initialize, Error the video source process is not the bayspec_socket_frame_process." << vcl_endl;
		return false;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// configure the asynchronous process to wrap around the camera source processes
	vcl_string process_name = "";
	
	process_name = rgbd_cam_source_proc_.name() + "_async";
	async_rgbd_cam_source_proc_ = new gevxl::framework::async_wrapping_process(process_name.c_str());
	async_rgbd_cam_source_proc_->set_async_stepping(true);
	async_rgbd_cam_source_proc_->set_process(&rgbd_cam_source_proc_);

	process_name = thermal_cam_source_proc_.name() + "_async";
	async_thermal_cam_source_proc_ = new gevxl::framework::async_wrapping_process(process_name.c_str());
	async_thermal_cam_source_proc_->set_async_stepping(true);
	async_thermal_cam_source_proc_->set_process(&thermal_cam_source_proc_);

  process_name = hyperspectral_cam_source_proc_.name() + "_async";
  async_hyperspectral_cam_source_proc_ = new gevxl::framework::async_wrapping_process(process_name.c_str());
	async_hyperspectral_cam_source_proc_->set_async_stepping(true);
	async_hyperspectral_cam_source_proc_->set_process(&hyperspectral_cam_source_proc_);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// tissue analysis process initialization
	tissue_analysis_proc_.set_source_process(&rgbd_cam_source_proc_);
  if(!tissue_analysis_proc_.initialize()) {
		vcl_cerr << "pu_assessment_system_proc::initialize, Error initializing tissue_analysis_proc_." << vcl_endl;		
    return false;
	}

	// 3d reconstruction process initialization
	threed_recon_proc_.set_source_process(&rgbd_cam_source_proc_);
	if(!threed_recon_proc_.initialize()) {
		vcl_cerr << "pu_assessment_system_proc::initialize, Error initializing threed_recon_proc_." << vcl_endl;		
    return false;
	}

	// thermal analysis process initialization
	thermal_analysis_proc_.set_source_process(&thermal_cam_source_proc_);
  if(!thermal_analysis_proc_.initialize()) {
		vcl_cerr << "pu_assessment_system_proc::initialize, Error initializing thermal_analysis_proc_." << vcl_endl;		
    return false;
	}

 	return true;
}

void pu_assessment_system_proc::uninitialize(void)
{
  
}

bool pu_assessment_system_proc::step(void)
{
	// perform parallel threading on two indepedent camera source processes
	bool ret = true;

	ret = ret & async_rgbd_cam_source_proc_->step();
	if(!ret) {
		vcl_cerr << "Error, rgbd_cam_source_proc_ process stepping failed." << vcl_endl;
		return false;
	}

	ret = ret & async_thermal_cam_source_proc_->step();
	if(!ret) {
		vcl_cerr << "Error, thermal_cam_source_proc_ process stepping failed.\n" << vcl_endl;
		return false;
	}

  // Let's not step the hyperspectral camera continuously as its SDK is not well supporting this mode.
  //ret = ret & async_hyperspectral_cam_source_proc_->step();
	//if(!ret) {
	//	vcl_cerr << "Error, hyperspectral_cam_source_proc_ process stepping failed.\n" << vcl_endl;
	//	return false;
	//}

	// now wait for both threads to finish before proceeding to the next process
	async_rgbd_cam_source_proc_->wait_until_ready();
	async_thermal_cam_source_proc_->wait_until_ready();
  // Let's not step the hyperspectral camera continuously as its SDK is not well supporting this mode.
  //async_hyperspectral_cam_source_proc_->wait_until_ready();

	ret = ret & async_rgbd_cam_source_proc_->get_step_result();
	if(!ret) {
		vcl_cerr << "Error, rgbd_cam_source_proc_ process step result failed.\n";
		return false;
	}

	ret = ret & async_thermal_cam_source_proc_->get_step_result();
	if(!ret) {
		vcl_cerr << "Error, thermal_cam_source_proc_ process step result failed.\n";
		return false;
	}

  // Let's not step the hyperspectral camera continuously as its SDK is not well supporting this mode.
  //ret = ret & async_hyperspectral_cam_source_proc_->get_step_result();
	//if(!ret) {
	//	vcl_cerr << "Error, hyperspectral_cam_source_proc_ process step result failed.\n";
	//	return false;
	//}

	// debug
	//if(frame_nr_ == 100) {
	//	gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
	//	pxc_source->set_save_out_flag(true);

	//	gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());
	//	thermal_source->set_save_out_flag(true);
	//}

  // deal with the frame rate issue.
	double frame_rate = 1000/highres_timer_.elapsed();	
	highres_timer_.reset();

	gevxl::util::time::highres_timer state_timer;
	state_timer.reset();

	bool succeeded = false;

	// tissue analysis process stepping
	succeeded = tissue_analysis_proc_.step();
	if(succeeded == false) {
		vcl_cerr << "pu_assessment_system_proc::step, tissue_analysis_proc_.step() error." << vcl_endl;
		return false;
	}

	// 3d reconstruction process stepping
	succeeded = threed_recon_proc_.step();
	if(succeeded == false) {
		vcl_cerr << "pu_assessment_system_proc::step, threed_recon_proc_.step() error." << vcl_endl;
		return false;
	}

	// make sure we transfer the point cloud knowledge from the depth camera to the thermal camera
	const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
	thermal_analysis_proc_.set_point_cloud_xyzrgb_frame( pxc_source->cur_xyz_rgb_frame() );

	// thermal analysis process stepping
	succeeded = thermal_analysis_proc_.step();
	if(succeeded == false) {
		vcl_cerr << "pu_assessment_system_proc::step, thermal_analysis_proc_.step() error." << vcl_endl;
		return false;
	}

  ////////////////////////////////////////////////////////////////////////
	// Visualization of the process
  visualize();

  frame_nr_++;

	return true;
}

bool pu_assessment_system_proc::take_snapshot(int view_pt_id)
{
  gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
  gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());

  vcl_vector<vcl_string> viewpoint_descriptions;
  viewpoint_descriptions.resize(9);

  viewpoint_descriptions[0] = "northwest";
  viewpoint_descriptions[1] = "north";
  viewpoint_descriptions[2] = "northeast";
  viewpoint_descriptions[3] = "west";
  viewpoint_descriptions[4] = "center";
  viewpoint_descriptions[5] = "east";
  viewpoint_descriptions[6] = "southwest";
  viewpoint_descriptions[7] = "south";
  viewpoint_descriptions[8] = "southeast";  

  if(view_pt_id < 0 || view_pt_id >= 9) return false;

  bool result = true;

  result = pxc_source->take_snapshot(rgb_file_directory_, depth_file_directory_, viewpoint_descriptions[view_pt_id]);
  if(result == false) return false;

  result = thermal_source->take_snapshot(thermal_file_directory_, viewpoint_descriptions[view_pt_id]);
  if(result == false) return false;

  return true;
}

bool pu_assessment_system_proc::take_hyperspectral_snapshot(void)
{
  gevxl::vid::bayspec_socket_frame_process *hyperspectral_source = dynamic_cast<gevxl::vid::bayspec_socket_frame_process *>(hyperspectral_cam_source_proc_.get_frame_process());

  bool result = true;

  result = hyperspectral_source->take_snapshot(hyperspectral_file_directory_);
  if(result == false) return false;

  // augment the hyperspectral snapshot taken with all other imaging modalities
  gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
  gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());

  result = pxc_source->take_snapshot(hyperspectral_file_directory_, hyperspectral_file_directory_, "hyperspectral_vp");
  if(result == false) return false;

  result = thermal_source->take_snapshot(hyperspectral_file_directory_, "hyperspectral_vp");
  if(result == false) return false;

  return true;
}

// python GUI to control the imaging video data capture
bool pu_assessment_system_proc::start_recording(int view_pt_id)
{
  gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
  gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());

  vcl_vector<vcl_string> viewpoint_descriptions;
  viewpoint_descriptions.resize(9);

  viewpoint_descriptions[0] = "northwest";
  viewpoint_descriptions[1] = "north";
  viewpoint_descriptions[2] = "northeast";
  viewpoint_descriptions[3] = "west";
  viewpoint_descriptions[4] = "center";
  viewpoint_descriptions[5] = "east";
  viewpoint_descriptions[6] = "southwest";
  viewpoint_descriptions[7] = "south";
  viewpoint_descriptions[8] = "southeast";

  if(view_pt_id < 0 || view_pt_id >= 9) return false;

  bool result = true;

  result = pxc_source->start_recording(rgb_file_directory_, depth_file_directory_, viewpoint_descriptions[view_pt_id]);
  if(result == false) return false;

  result = thermal_source->start_recording(thermal_file_directory_, viewpoint_descriptions[view_pt_id]);
  if(result == false) return false;

  return true;
}
      
bool pu_assessment_system_proc::stop_recording()
{
  gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
  gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());

  bool result = true;

  result = pxc_source->stop_recording();
  if(result == false) return false;

  result = thermal_source->stop_recording();
  if(result == false) return false;

  return true;
}

bool pu_assessment_system_proc::playback_recording(int view_pt_id)
{
  gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
  gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());

  vcl_vector<vcl_string> viewpoint_descriptions;
  viewpoint_descriptions.resize(9);

  viewpoint_descriptions[0] = "northwest";
  viewpoint_descriptions[1] = "north";
  viewpoint_descriptions[2] = "northeast";
  viewpoint_descriptions[3] = "west";
  viewpoint_descriptions[4] = "center";
  viewpoint_descriptions[5] = "east";
  viewpoint_descriptions[6] = "southwest";
  viewpoint_descriptions[7] = "south";
  viewpoint_descriptions[8] = "southeast";

  if(view_pt_id < 0 || view_pt_id >= 9) return false;

  bool result = true;

  result = pxc_source->playback_recording(rgb_file_directory_, depth_file_directory_, viewpoint_descriptions[view_pt_id]);
  if(result == false) return false;

  result = thermal_source->playback_recording(thermal_file_directory_, viewpoint_descriptions[view_pt_id]);
  if(result == false) return false;

  return true;
}

void pu_assessment_system_proc::get_tissue_analysis_viz_offset(int &viz_offset_i, int &viz_offset_j)
{
	viz_offset_i = tissue_analysis_viz_offset_i_;
	viz_offset_j = tissue_analysis_viz_offset_j_;
}

void pu_assessment_system_proc::get_3d_recon_viz_offset(int &viz_offset_i, int &viz_offset_j)
{
	viz_offset_i = threed_recon_viz_offset_i_;
	viz_offset_j = threed_recon_viz_offset_j_;
}

void pu_assessment_system_proc::get_thermal_analysis_viz_offset(int &viz_offset_i, int &viz_offset_j)
{
	viz_offset_i = thermal_analysis_viz_offset_i_;
	viz_offset_j = thermal_analysis_viz_offset_j_;
}

void pu_assessment_system_proc::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;

	// tissue analysis process
	tissue_analysis_proc_.set_visualizer(viz);
	tissue_analysis_proc_.set_viz_offset(tissue_analysis_viz_offset_i_, tissue_analysis_viz_offset_j_);

	// 3d reconstruction process
	threed_recon_proc_.set_visualizer(viz);
	threed_recon_proc_.set_viz_offset(threed_recon_viz_offset_i_, threed_recon_viz_offset_j_);

	// thermal analysis process
	thermal_analysis_proc_.set_visualizer(viz);
	thermal_analysis_proc_.set_viz_offset(thermal_analysis_viz_offset_i_, thermal_analysis_viz_offset_j_);
}

void pu_assessment_system_proc::visualize(void)
{
	if(verbose_ > 0) {
		vcl_cout << "pu_assessment_system_proc::visualize, enter the visualization routine." << vcl_endl;
	}

	// Visualization stuff of each processes    	
	if(viz_) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the cpp camera canvas." << vcl_endl;
		}

		viz_->lock();
		if( !viz_->is_initialized() ) {
			viz_->initialize();
		}
		viz_->unlock();
	}

	IF_CAN_VISUALIZE( viz_ ) {

		if(viz_) {
			if(viz_->is_initialized()) { 
				
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				viz_canvas_frame_.fill(0);
				
				tissue_analysis_proc_.visualize_canvas(viz_canvas_frame_);
				threed_recon_proc_.visualize_canvas(viz_canvas_frame_);
				thermal_analysis_proc_.visualize_canvas(viz_canvas_frame_);
				viz_->set_image(viz_canvas_frame_);

				tissue_analysis_proc_.visualize_overlay();
				threed_recon_proc_.visualize_overlay();
				thermal_analysis_proc_.visualize_overlay();
			}
			
			viz_->flush();			
		}	  
	}

	////////////////////////////////////////////////////////////////////////////////
  // perform the necessary image rescaling to fit into the externally set visualizers
  prepare_viz_images();

  vcl_string distance_viz_str = vul_sprintf( "Dist=%1.2f, ", current_distance_ );
  if(current_distance_ < optimal_snapshot_distance_ - optimal_snapshot_distance_tolerance_) {
    distance_viz_str = distance_viz_str + "move further away";
  }
  else if(current_distance_ > optimal_snapshot_distance_ + optimal_snapshot_distance_tolerance_) {
    distance_viz_str = distance_viz_str + "move closer";
  }
  else {
    distance_viz_str = distance_viz_str + "perfect";
  }

	// individual camera data visualization
	const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
	const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());
  const gevxl::vid::bayspec_socket_frame_process *hyperspectral_source = dynamic_cast<const gevxl::vid::bayspec_socket_frame_process *>(hyperspectral_cam_source_proc_.get_frame_process());
	
  // rgb frame visualizer
	if(verbose_ > 0) {
		vcl_cout << "pu_assessment_system_proc::visualize, rgb_viz_ = " << rgb_viz_ << ", with the frame size = " << pxc_source->cur_rgb_frame().size() << vcl_endl;
	}
	if( rgb_viz_ && pxc_source->cur_rgb_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python RGB camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( rgb_viz_ );
		rgb_viz_->initialize();
		if (rgb_viz_->is_initialized() ) { 
			
			//rgb_viz_->set_image( pxc_source->cur_rgb_frame() );
      rgb_viz_->set_image( rgb_viz_image_ );
      
      rgb_viz_->set_foreground(0,1,1);
      rgb_viz_->add_text(0, 0, distance_viz_str, true, false, "below");

			rgb_viz_->flush();
		}
	}
	
  // depth frame visualizer
	if(verbose_ > 0) {
		vcl_cout << "pu_assessment_system_proc::visualize, depth_viz_ = " << depth_viz_ << ", with the frame size = " << pxc_source->cur_depth_byte_frame().size() << vcl_endl;
	}
	if( depth_viz_ && pxc_source->cur_depth_byte_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python Depth camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( depth_viz_ );
		depth_viz_->initialize();
		if (depth_viz_->is_initialized() ) { 
			
			//depth_viz_->set_image( pxc_source->cur_rgb_frame_in_depth_view() );
      depth_viz_->set_image( depth_viz_image_ );
      
      /*
      vcl_string depth_viz_str = vul_sprintf( "Dist=%1.2f, ", current_distance_ );
      if(current_distance_ < optimal_snapshot_distance_ - optimal_snapshot_distance_tolerance_) {
        depth_viz_str = depth_viz_str + "move further away";
      }
      else if(current_distance_ > optimal_snapshot_distance_ + optimal_snapshot_distance_tolerance_) {
        depth_viz_str = depth_viz_str + "move closer";
      }
      else {
        depth_viz_str = depth_viz_str + "perfect";
      }
      depth_viz_->set_foreground(0,1,1);
      depth_viz_->add_text(0, 0, depth_viz_str, true, false, "below");
			*/

      depth_viz_->flush();
		}
	}
	
  // thermal frame visualizer
	if(verbose_ > 0) {
		vcl_cout << "pu_assessment_system_proc::visualize, thermal_viz_ = " << thermal_viz_ << ", with the frame size = " << thermal_source->cur_thermal_byte_frame().size() << vcl_endl;
	}
	if( thermal_viz_ && thermal_source->cur_thermal_byte_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python Thermal camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( thermal_viz_ );
		thermal_viz_->initialize();
		if (thermal_viz_->is_initialized() ) { 
			
			//thermal_viz_->set_image( thermal_source->cur_thermal_color_palette_frame() );
      thermal_viz_->set_image( thermal_viz_image_ );
			thermal_viz_->flush();			
		}
	}

  // rgb frame copy visualizer
	if(verbose_ > 0) {
		vcl_cout << "pu_assessment_system_proc::visualize, rgb_copy_viz_ = " << rgb_copy_viz_ << ", with the frame size = " << pxc_source->cur_rgb_frame().size() << vcl_endl;
	}
	if( rgb_copy_viz_ && pxc_source->cur_rgb_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python RGB camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( rgb_copy_viz_ );
		rgb_copy_viz_->initialize();
		if (rgb_copy_viz_->is_initialized() ) { 
			
			//rgb_copy_viz_->set_image( pxc_source->cur_rgb_frame() );
      rgb_copy_viz_->set_image( rgb_viz_image_ );

      rgb_copy_viz_->set_foreground(0,1,1);
      rgb_copy_viz_->add_text(0, 0, distance_viz_str, true, false, "below");

			rgb_copy_viz_->flush();
		}
	}
	
  // hyperspectral frame visualizer
	if(verbose_ > 0) {
    vcl_cout << "pu_assessment_system_proc::visualize, hyperspectral_viz_ = " << hyperspectral_viz_ << ", with the frame size = " << hyperspectral_source->cur_frame().size() << vcl_endl;
	}
  if( hyperspectral_viz_ && hyperspectral_source->cur_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python Hyperspectral camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( hyperspectral_viz_ );
		hyperspectral_viz_->initialize();
		if (hyperspectral_viz_->is_initialized() ) { 
			
      //hyperspectral_viz_->set_image( hyperspectral_source->cur_frame() );
      hyperspectral_viz_->set_image( hyperspectral_viz_image_ );
      hyperspectral_viz_->flush();
		}
	}

  // the newly added rgb, thermal and hyperspectral frame visualizers for the touch screen.
  // rgb frame touch visualizer
  if(verbose_ > 0) {
    vcl_cout << "pu_assessment_system_proc::visualize, rgb_touch_viz_ = " << rgb_touch_viz_ << ", with the frame size = " << pxc_source->cur_rgb_frame().size() << vcl_endl;
	}
  if( rgb_touch_viz_ && pxc_source->cur_rgb_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python RGB camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( rgb_touch_viz_ );
		rgb_touch_viz_->initialize();
		if (rgb_touch_viz_->is_initialized() ) { 
			      
      rgb_touch_viz_->set_image( rgb_viz_image_ );

      rgb_touch_viz_->set_foreground(0,1,1);
      rgb_touch_viz_->add_text(0, 0, distance_viz_str, true, false, "below");

      rgb_touch_viz_->flush();
		}
	}

  // thermal frame touch visualizer
  if(verbose_ > 0) {
		vcl_cout << "pu_assessment_system_proc::visualize, thermal_touch_viz_ = " << thermal_touch_viz_ << ", with the frame size = " << thermal_source->cur_thermal_byte_frame().size() << vcl_endl;
	}
	if( thermal_touch_viz_ && thermal_source->cur_thermal_byte_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python Thermal camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( thermal_touch_viz_ );
		thermal_touch_viz_->initialize();
		if (thermal_touch_viz_->is_initialized() ) { 
			
			thermal_touch_viz_->set_image( thermal_touch_viz_image_ );
			thermal_touch_viz_->flush();			
		}
	}

  // hyperspectral frame touch visualizer
  if(verbose_ > 0) {
    vcl_cout << "pu_assessment_system_proc::visualize, hyperspectral_touch_viz_ = " << hyperspectral_touch_viz_ << ", with the frame size = " << hyperspectral_source->cur_frame().size() << vcl_endl;
	}
  if( hyperspectral_touch_viz_ && hyperspectral_source->cur_frame().size() > 0 ) {
		if(verbose_ > 0) {
			vcl_cout << "pu_assessment_system_proc::visualize, start to visualize the python Hyperspectral camera." << vcl_endl;
		}

		gevxl::threading::scoped_lock lock( hyperspectral_touch_viz_ );
		hyperspectral_touch_viz_->initialize();
		if (hyperspectral_touch_viz_->is_initialized() ) { 
			      
      hyperspectral_touch_viz_->set_image( hyperspectral_touch_viz_image_ );
      hyperspectral_touch_viz_->flush();
		}
	}
	////////////////////////////////////////////////////////////////////////////////
}

// rescale the source image to the target image size and store it to the target image.
void pu_assessment_system_proc::rescale_image(const vil_image_view<vxl_byte> &source, vil_image_view<vxl_byte> &target)
{
  if(source.ni() == target.ni() && source.nj() == target.nj()) {
    target.deep_copy(source);
    return;
  }

  // not the same size, start the rescaling
  if(target.nplanes() != source.nplanes()) {
    target = vil_image_view<vxl_byte>(target.ni(), target.nj(), 1, source.nplanes());
  }

  int sw, sh, tw, th;
  sw = source.ni();
  sh = source.nj();
  tw = target.ni();
  th = target.nj();

  double scaling_w = ((double)sw/(double)tw);
  double scaling_h = ((double)sh/(double)th);

  int si, sj, ti, tj, p;

  for(tj = 0; tj < target.nj(); tj++) {
    for(ti = 0; ti < target.ni(); ti++) {
      si = (int)(scaling_w*ti + 0.5);
      if(si < 0) si = 0;
      if(si >= sw) si = sw - 1;

      sj = (int)(scaling_h*tj + 0.5);
      if(sj < 0) sj = 0;
      if(sj >= sh) sj = sh - 1;

      for(p = 0; p < target.nplanes(); p++) {
        target(ti, tj, p) = source(si, sj, p);
      }
    }
  }
}

void pu_assessment_system_proc::prepare_viz_images(void)
{
  // the visualization images
  rgb_viz_width_ = 640;
  rgb_viz_height_ = 360;
  if(rgb_viz_image_.ni() != rgb_viz_width_ || rgb_viz_image_.nj() != rgb_viz_height_) {
    rgb_viz_image_ = vil_image_view<vxl_byte>(rgb_viz_width_, rgb_viz_height_, 1, 3);
    rgb_viz_image_.fill(0);
  }

  depth_viz_width_ = 320;
  depth_viz_height_ = 240;  
  if(depth_viz_image_.ni() != depth_viz_width_ || depth_viz_image_.nj() != depth_viz_height_) {
    depth_viz_image_ = vil_image_view<vxl_byte>(depth_viz_width_, depth_viz_height_, 1, 3);
    depth_viz_image_.fill(0);
  }
  
  thermal_viz_width_ = 320;
  thermal_viz_height_ = 240;  
  if(thermal_viz_image_.ni() != thermal_viz_width_ || thermal_viz_image_.nj() != thermal_viz_height_) {
    thermal_viz_image_ = vil_image_view<vxl_byte>(thermal_viz_width_, thermal_viz_height_, 1, 3);
    thermal_viz_image_.fill(0);
  }
      
  hyperspectral_viz_width_ = 256;
  hyperspectral_viz_height_ = 256;  
  if(hyperspectral_viz_image_.ni() != hyperspectral_viz_width_ || hyperspectral_viz_image_.nj() != hyperspectral_viz_height_) {
    hyperspectral_viz_image_ = vil_image_view<vxl_byte>(hyperspectral_viz_width_, hyperspectral_viz_height_, 1, 3);
    hyperspectral_viz_image_.fill(0);
  }

  thermal_touch_viz_width_ = 160;
  thermal_touch_viz_height_ = 120;  
  if(thermal_touch_viz_image_.ni() != thermal_touch_viz_width_ || thermal_touch_viz_image_.nj() != thermal_touch_viz_height_) {
    thermal_touch_viz_image_ = vil_image_view<vxl_byte>(thermal_touch_viz_width_, thermal_touch_viz_height_, 1, 3);
    thermal_touch_viz_image_.fill(0);
  }
      
  hyperspectral_touch_viz_width_ = 160;
  hyperspectral_touch_viz_height_ = 160;  
  if(hyperspectral_touch_viz_image_.ni() != hyperspectral_touch_viz_width_ || hyperspectral_touch_viz_image_.nj() != hyperspectral_touch_viz_height_) {
    hyperspectral_touch_viz_image_ = vil_image_view<vxl_byte>(hyperspectral_touch_viz_width_, hyperspectral_touch_viz_height_, 1, 3);
    hyperspectral_touch_viz_image_.fill(0);
  }

  const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
	const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(thermal_cam_source_proc_.get_frame_process());
  const gevxl::vid::bayspec_socket_frame_process *hyperspectral_source = dynamic_cast<const gevxl::vid::bayspec_socket_frame_process *>(hyperspectral_cam_source_proc_.get_frame_process());

  if(pxc_source->cur_rgb_frame().size() > 0) {
    rescale_image(pxc_source->cur_rgb_frame(), rgb_viz_image_);  
  }
  
  if(pxc_source->cur_depth_byte_frame().size() > 0) {
    //rescale_image(pxc_source->cur_depth_byte_frame(), depth_viz_image_);
    generate_depth_viz_image();
    update_rgb_viz_image();
  }
  
  if(thermal_source->cur_thermal_color_palette_frame().size() > 0) {
    rescale_image(thermal_source->cur_thermal_color_palette_frame(), thermal_viz_image_);
  }
  
  if(hyperspectral_source->cur_three_wavelength_bands_frame().size() > 0) {
    rescale_image(hyperspectral_source->cur_three_wavelength_bands_frame(), hyperspectral_viz_image_);
  }

  if(thermal_source->cur_thermal_color_palette_frame().size() > 0) {
    rescale_image(thermal_source->cur_thermal_color_palette_frame(), thermal_touch_viz_image_);
  }
  
  if(hyperspectral_source->cur_three_wavelength_bands_frame().size() > 0) {
    rescale_image(hyperspectral_source->cur_three_wavelength_bands_frame(), hyperspectral_touch_viz_image_);
  }
}

// generate the depth visualization image
void pu_assessment_system_proc::generate_depth_viz_image(void)
{
  const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(rgbd_cam_source_proc_.get_frame_process());
  const vil_image_view<vxl_uint_16> &depth_frame_16bit = pxc_source->cur_depth_frame();
  const vil_image_view<vxl_byte> &depth_frame = pxc_source->cur_depth_byte_frame();
  const vil_image_view<vxl_byte> &color_in_depth_frame = pxc_source->cur_rgb_frame_in_depth_view();

  ///////////////////////////////////////////////////////////////////////////////////////////////
  int i, j;
  if(viz_depth_or_color_in_depth_ == "depth") {
    for(j = 0; j < depth_frame.nj(); j++) {
      for(i = 0; i < depth_frame.ni(); i++) {
        depth_viz_image_(i, j, 0) = depth_frame(i, j);
        depth_viz_image_(i, j, 1) = depth_frame(i, j);
        depth_viz_image_(i, j, 2) = depth_frame(i, j);
      }
    }
  }
  else {
    for(j = 0; j < color_in_depth_frame.nj(); j++) {
      for(i = 0; i < color_in_depth_frame.ni(); i++) {
        depth_viz_image_(i, j, 0) = color_in_depth_frame(i, j, 0);
        depth_viz_image_(i, j, 1) = color_in_depth_frame(i, j, 1);
        depth_viz_image_(i, j, 2) = color_in_depth_frame(i, j, 2);
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // based on the viz inner rectangle, compute the median distance value, also occlude all the noisy depth pixel values, i.e., the val = 0
  vcl_vector<int> distance_vec;
  distance_vec.reserve(depth_frame_16bit.nj()*depth_frame_16bit.ni());
  for(j = viz_inner_rect_.y0; j < viz_inner_rect_.y1; j++) {
    for(i = viz_inner_rect_.x0; i < viz_inner_rect_.x1; i++) {
      if( depth_frame_16bit(i, j) == 0 ) continue;

      distance_vec.push_back( (int)(depth_frame_16bit(i, j)) );
    }
  }
  // sorting the distance
  vcl_sort(distance_vec.begin(), distance_vec.end());
  // obtain the median temperature value	
	int size = distance_vec.size();
  if(size == 0) {
    current_distance_ = optimal_snapshot_distance_;
  }
  else {
	  if(size % 2 == 0) {
	    // even number of elements
		  current_distance_ = (double)(distance_vec[(size/2)-1] + distance_vec[size/2])/2.0;
	  }
	  else {
	    // odd number of elements
		  current_distance_ = (double)(distance_vec[(size-1)/2]);
    }
    current_distance_ = current_distance_/1000; // to make it as meter
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // calculate the visualization color of the inner and outer rectangles
  double max_distance = 2*optimal_snapshot_distance_;
  double min_distance = 0.0;
  double diff_distance = fabs(current_distance_ - optimal_snapshot_distance_);

  if(current_distance_ > max_distance) {
    distance_rgb_viz_[0] = max_color_offset_;
    distance_rgb_viz_[1] = 0;
    distance_rgb_viz_[2] = 0;
  }
  else if(current_distance_ < min_distance) {
    distance_rgb_viz_[0] = max_color_offset_;
    distance_rgb_viz_[1] = 0;
    distance_rgb_viz_[2] = 0;
  }
  else {
    int red_offset = (int)( max_color_offset_*(diff_distance/optimal_snapshot_distance_) + 0.5 );
    distance_rgb_viz_[0] = red_offset;
    int green_offset = (int)( max_color_offset_*(1.0 - diff_distance/optimal_snapshot_distance_) + 0.5 );
    distance_rgb_viz_[1] = green_offset;
    distance_rgb_viz_[2] = 0;
  }

  /*
  int r, g;
  for(j = viz_outer_rect_.y0; j < viz_outer_rect_.y1; j++) {
    for(i = viz_outer_rect_.x0; i < viz_outer_rect_.x1; i++) {
      if(viz_inner_rect_.inside(i, j) == true) continue;

      r = depth_viz_image_(i, j, 0) + distance_rgb_viz_[0];
      g = depth_viz_image_(i, j, 1) + distance_rgb_viz_[1];

      if(r > 255) r = 255;
      if(g > 255) g = 255;

      depth_viz_image_(i, j, 0) = r;
      depth_viz_image_(i, j, 1) = g;
    }
  }
  */
}

// update the rgb visualization image image based on the distance measure
void pu_assessment_system_proc::update_rgb_viz_image(void)
{
  int r, g, b;
  unsigned i, j;
  for(j = depth_in_rgb_viz_outer_rect_.y0; j < depth_in_rgb_viz_outer_rect_.y1; j++) {
    for(i = depth_in_rgb_viz_outer_rect_.x0; i < depth_in_rgb_viz_outer_rect_.x1; i++) {
      if(depth_in_rgb_viz_inner_rect_.inside(i, j) == true) continue;

      r = rgb_viz_image_(i, j, 0) + distance_rgb_viz_[0];
      g = rgb_viz_image_(i, j, 1) + distance_rgb_viz_[1];

      if(r > 255) r = 255;
      if(g > 255) g = 255;

      rgb_viz_image_(i, j, 0) = r;
      rgb_viz_image_(i, j, 1) = g;
    }
  }

  for(j = hyperspectral_in_rgb_viz_outer_rect_.y0; j < hyperspectral_in_rgb_viz_outer_rect_.y1; j++) {
    for(i = hyperspectral_in_rgb_viz_outer_rect_.x0; i < hyperspectral_in_rgb_viz_outer_rect_.x1; i++) {
      if(hyperspectral_in_rgb_viz_inner_rect_.inside(i, j) == true) continue;

      r = rgb_viz_image_(i, j, 0) - 150;
      g = rgb_viz_image_(i, j, 1) - 150;
      b = rgb_viz_image_(i, j, 2) - 150;

      if(r < 0) r = 0;
      if(g < 0) g = 0;
      if(b < 0) b = 0;
      
      rgb_viz_image_(i, j, 0) = r;
      rgb_viz_image_(i, j, 1) = g;
      rgb_viz_image_(i, j, 2) = b;
    }
  }
}

