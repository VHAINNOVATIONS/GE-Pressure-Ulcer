// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_prevention_chaining_process.h"

#include <vcl_iostream.h>

#include <vil/vil_save.h>
#include <vil/vil_load.h>

#include <vil/vil_convert.h>
#include <vil/vil_flip.h>

#include <vid/openni2_frame_process.h>

#include <sstream> // for converting float to string

#include <img/image_as_string.h>
#include <vid/frame_tag.h>

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer::prevention;

pu_prevention_chaining_process::pu_prevention_chaining_process(char const *name)
: gevxl::framework::process(name), 
source_proc_("vid::pu_prevention_source_process"),
viz_(NULL), 
viz_img_(NULL), 
avi_fps_(30),
rgb_video_out_openned_(false), 
depth_video_out_openned_(false), 
height_filtered_video_out_openned_(false), 
depth_filtered_video_out_openned_(false), 
frame_nr_(0),
cur_frame_output_type_("depth"),
database_writer_(NULL),
verbose_(0)
{
	highres_timer_.reset();
}

pu_prevention_chaining_process::~pu_prevention_chaining_process(void)
{
	if(rgb_video_out_openned_) {
		rgb_video_out_.close();
		rgb_video_out_openned_ = false;
	}

	if(depth_video_out_openned_) {
		depth_video_out_.close();
		depth_video_out_openned_ = false;
	}

  if(height_filtered_video_out_openned_) {
		height_filtered_video_out_.close();
		height_filtered_video_out_openned_ = false;
	}

	if(depth_filtered_video_out_openned_) {
		depth_filtered_video_out_.close();
		depth_filtered_video_out_openned_ = false;
	}
}

bool pu_prevention_chaining_process::configure(util::config_file &config)
{
	config_ = config;

	verbose_ = 0;
	config.get_integer(name()+"::verbose", verbose_);

	avi_fps_ = 30;
	config.get_integer(name()+"::avi_fps", avi_fps_);

	rgb_video_out_filename_ = "";
	config.get_string(name()+"::rgb_video_out_filename", rgb_video_out_filename_);

	depth_video_out_filename_ = "";
	config.get_string(name()+"::depth_video_out_filename", depth_video_out_filename_);

  height_filtered_video_out_filename_ = "";
  config.get_string(name()+"::height_filtered_video_out_filename", height_filtered_video_out_filename_);

	depth_filtered_video_out_filename_ = "";
  config.get_string(name()+"::depth_filtered_video_out_filename", depth_filtered_video_out_filename_);

	// ---- Configure source process 
	if( !source_proc_.configure(config) ) {
		vcl_cerr << "pu_prevention_chaining_process::configure, Error configuring source_proc_." << vcl_endl;
		return false;
	}
	source_proc_.set_output_type("depth"); // ensure the output is the depth view

	// ---  Configure the frame tag process
  if( !frame_tag_proc_.configure(config) ) {
		vcl_cerr << "pu_prevention_chaining_process::configure, Error configuring frame_tag_proc_." << vcl_endl;
		return false;
	}

	// -- Configure the rectify_kinect process, and the rectify_kinect_process is always enabled.
	if( !rectify_kinect_proc_.configure(config) ) {
		vcl_cerr << "pu_prevention_chaining_process::configure, Error configuring rectify_kinect_proc_." << vcl_endl;
		return false;
	}

	// -- Configure the motion_estimate_process
	if( !motion_estimate_proc_.configure(config) ) {
		vcl_cerr << "pu_prevention_chaining_process::configure, Error configuring motion_estimate_proc_." << vcl_endl;
		return false;
	}

  // -- Configure the turning_protocol_proc_
  if( !turning_protocol_proc_.configure(config) ) {
    vcl_cerr << "pu_prevention_chaining_process::configure, Error configuring turning_protocol_proc_." << vcl_endl;
		return false;
  }

	// cur output frame type for visualization
  cur_frame_output_type_ = "depth";
  config.get_string(name()+"::cur_frame_output_type", cur_frame_output_type_);

	// gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_writer_process
	videoarchive_writer_proc_.set_source_frame_process(&source_proc_);
	videoarchive_writer_proc_.set_tag_source(&frame_tag_proc_);
	if( !videoarchive_writer_proc_.configure(config) ) {
		vcl_cerr << "pu_prevention_chaining_process::configure, Error configuring videoarchive_writer_proc_." << vcl_endl;
		return false;
	}
  //videoarchive_writer_proc_.enable(false);
	
	return true;
}

bool pu_prevention_chaining_process::initialize(void)
{
	if( !source_proc_.initialize() ) {
		vcl_cerr << "pu_prevention_chaining_process::initialize, Error initializing source_proc_." << vcl_endl;
		return false;
	}

	const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
	frame_tag_proc_.set_source_tagged_frame_process(openni2_source);
	if(!frame_tag_proc_.initialize()) {
		vcl_cerr << "pu_prevention_chaining_process::initialize, Error initializing frame_tag_proc_." << vcl_endl;        
		return false;
	}
	frame_tag_proc_.use_tagged_frame_process_tag_information_if_available(true);

	// kinect's depth value rectification process based on the known ground plane points
  rectify_kinect_proc_.set_source_process(&source_proc_);
  if( !rectify_kinect_proc_.initialize() ) {
		vcl_cerr << "pu_prevention_chaining_process::initialize, Error initializing rectify_kinect_proc_." << vcl_endl;
		return false;
	}

  // turning protocol process's initialization method
  if( !turning_protocol_proc_.initialize() ) {
    vcl_cerr << "pu_prevention_chaining_process::initialize, Error initializing turning_protocol_proc_." << vcl_endl;
		return false;
  }

	if( !videoarchive_writer_proc_.initialize() ) {
		vcl_cerr << "pu_prevention_chaining_process::initialize, Error initializing videoarchive_writer_proc_." << vcl_endl;
		return false;
	}

	return true;
}

void pu_prevention_chaining_process::uninitialize(void)
{
	if(rgb_video_out_openned_) {
		rgb_video_out_.close();
		rgb_video_out_openned_ = false;
	}

	if(depth_video_out_openned_) {
		depth_video_out_.close();
		depth_video_out_openned_ = false;
	}

  if(height_filtered_video_out_openned_) {
		height_filtered_video_out_.close();
		height_filtered_video_out_openned_ = false;
	}

	if(depth_filtered_video_out_openned_) {
		depth_filtered_video_out_.close();
		depth_filtered_video_out_openned_ = false;
	}

	videoarchive_writer_proc_.uninitialize();
}

// ensure the avi files have been openned for the ffmpeg writing out
void pu_prevention_chaining_process::ensure_avi_files_openned(const vil_image_view<vxl_byte> &rgb_img,
                                                              const vil_image_view<vxl_byte> &depth_img)
{
  // make sure the avi file writers are openned.
	if( !rgb_video_out_openned_ && rgb_video_out_filename_ != "" ) {
			vid::io::ffmpeg_writer_param p;
			rgb_video_out_.open( rgb_video_out_filename_.c_str(), 
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(avi_fps_).bit_rate(2000).size(rgb_img.ni(), rgb_img.nj()) );
									 //lossless, p.encoder(vid::io::ffmpeg_writer_param::HUFFYUV).frame_rate(avi_fps_).bit_rate(2000).size(rgb_img.ni(), rgb_img.nj()) );
			rgb_video_out_openned_ = true;
	}

	if( !depth_video_out_openned_ && depth_video_out_filename_ != "" ) {
			vid::io::ffmpeg_writer_param p;
			depth_video_out_.open( depth_video_out_filename_.c_str(), 
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(avi_fps_).bit_rate(2000).size(depth_img.ni(), depth_img.nj()) );
									 //lossless, p.encoder(vid::io::ffmpeg_writer_param::HUFFYUV).frame_rate(avi_fps_).bit_rate(2000).size(depth_img.ni(), depth_img.nj()) );
			depth_video_out_openned_ = true;
	}

  if( !height_filtered_video_out_openned_ && height_filtered_video_out_filename_ != "" ) {
			vid::io::ffmpeg_writer_param p;
			height_filtered_video_out_.open( height_filtered_video_out_filename_.c_str(), 
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(avi_fps_).bit_rate(2000).size(depth_img.ni(), depth_img.nj()) );
									 //lossless, p.encoder(vid::io::ffmpeg_writer_param::HUFFYUV).frame_rate(avi_fps_).bit_rate(2000).size(depth_img.ni(), depth_img.nj()) );
			height_filtered_video_out_openned_ = true;
	}

	if( !depth_filtered_video_out_openned_ && depth_filtered_video_out_filename_ != "" ) {
			vid::io::ffmpeg_writer_param p;
			depth_filtered_video_out_.open( depth_filtered_video_out_filename_.c_str(), 
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(avi_fps_).bit_rate(2000).size(depth_img.ni(), depth_img.nj()) );
									 //lossless, p.encoder(vid::io::ffmpeg_writer_param::HUFFYUV).frame_rate(avi_fps_).bit_rate(2000).size(depth_img.ni(), depth_img.nj()) );
			depth_filtered_video_out_openned_ = true;
	}
}

// ffmpeg write out 
void pu_prevention_chaining_process::write_out_frames_to_avi_files(const vil_image_view<vxl_byte> &rgb_img,
                                                                   const vil_image_view<vxl_byte> &depth_img)
{
  // write out the frames to avi files.
	if(rgb_video_out_openned_) {
		rgb_video_out_.write_frame(rgb_img);		
	}

	if(depth_video_out_openned_) {
		depth_video_out_.write_frame(depth_img);		
	}

  if(height_filtered_video_out_openned_) {
    height_filtered_video_out_.write_frame(rectify_kinect_proc_.height_filtered_frame());		
	}

	if(depth_filtered_video_out_openned_) {
    depth_filtered_video_out_.write_frame(rectify_kinect_proc_.depth_filtered_frame());		
	}
}

bool pu_prevention_chaining_process::step(void)
{
	if( verbose_ > 0 ) {
		vcl_cout << "pu_prevention_chaining_process::step, frame_nr_ = " << frame_nr_ << vcl_endl;
	}

	// Stepping through individual process in the chain
	if( !source_proc_.step() ) {
		vcl_cerr << "pu_prevention_chaining_process::step, stepping openni2 source error." << vcl_endl;
		return false;
	}

	if( !frame_tag_proc_.step() ) {
		vcl_cerr << "pu_prevention_chaining_process::step, stepping frame_tag_proc_ error." << vcl_endl;
		return false;
	}

  const gevxl::vid::frame_tag &tag = frame_tag_proc_.cur_frame_tag();

	// --- Rectify
	const vid::openni2_frame_process *openni2_proc = dynamic_cast<const vid::openni2_frame_process *>(source_proc_.get_frame_process());
	if( !openni2_proc ) {
		vcl_cerr << "pu_prevention_chaining_process::step, the generic_frame_process is not openni2_frame_process." << vcl_endl;
		return false;
	}

	vil_image_view<vxl_byte> rgb_img = openni2_proc->cur_rgb_frame();
	vil_image_view<vxl_byte> depth_img = openni2_proc->cur_depth_byte_frame();

  // ensure the avi files are openned
  ensure_avi_files_openned(rgb_img, depth_img);
	
	// rectification process
	if( !rectify_kinect_proc_.step( openni2_proc->cur_xyz_rgb_frame() ) ) {
		vcl_cerr << "pu_prevention_chaining_process::step(), Error executing rectify_kinect_proc_.step()." << vcl_endl;
		return false;
	}

	// motion estimate process
	//motion_estimate_proc_.step( openni2_proc->cur_depth_byte_frame() );
	if( !motion_estimate_proc_.step( tag, rectify_kinect_proc_.depth_filtered_frame() ) ) {
		vcl_cerr << "pu_prevention_chaining_process::step(), Error executing motion_estimate_proc_.step()." << vcl_endl;
		return false;
	}

  // patient turning protocol monitoring process
  const vil_image_view<vxl_uint_16> &raw_depth_frame = openni2_proc->cur_depth_frame();
  const vil_image_view<vxl_byte> &filtered_depth_frame = rectify_kinect_proc_.depth_filtered_frame();
  const vil_image_view<float> &rectified_xyz_frame = rectify_kinect_proc_.rectified_xyz_frame();
  
  vgl_point_3d<float>  origin;
  vgl_vector_3d<float> vx, vy, vz;
  rectify_kinect_proc_.get_transformed_coordinate_system(origin, vx, vy, vz);

  if( !turning_protocol_proc_.step(tag, raw_depth_frame, filtered_depth_frame, rectified_xyz_frame, origin, vx, vy, vz) ) {
    vcl_cerr << "pu_prevention_chaining_process::step(), Error executing turning_protocol_proc_.step()." << vcl_endl;
		return false;
  }

	//-- Deal with the frame rate issue.
	double frame_rate = 1000/highres_timer_.elapsed();    
	highres_timer_.reset();

	gevxl::util::time::highres_timer state_timer;
	state_timer.reset();

	//-- Visualize the process
	visualize();

	// stepping the videoarchive_writer_proc_
	if( !videoarchive_writer_proc_.step() ) {
		vcl_cerr << "pu_prevention_chaining_process::step(), Error executing videoarchive_writer_proc_.step()." << vcl_endl;
		return false;
	}

  // write out the frames to the various avi files
  write_out_frames_to_avi_files(rgb_img, depth_img);

  // debugging the database_writer_
  /*
  if(database_writer_) {
    if(frame_nr_ % 50 == 0) {
      database_writer_->write(true, "Right side");
    }
    else if(frame_nr_ % 70 == 0) {
      database_writer_->write(false, "Left side");
    }
    else if(frame_nr_ % 30 == 0) {
      database_writer_->write(true, "Back");
    }
  }
  */
  

	frame_nr_++;

	return true;
}

const vil_image_view<vxl_byte> &pu_prevention_chaining_process::cur_frame(void) const
{
	const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
	
	if(cur_frame_output_type_ == "depth") {
		openni2_source->cur_depth_byte_frame();
  }
	else if(cur_frame_output_type_ == "rgb") {
		openni2_source->cur_rgb_frame();
	}
  else if(cur_frame_output_type_ == "motion") {
    return motion_estimate_proc_.get_visualization_image();
  }
  else if(cur_frame_output_type_ == "rectified_xyz") {    
    return rectify_kinect_proc_.xyz2rgb(rectify_kinect_proc_.rectified_xyz_frame());    
  }
  else if(cur_frame_output_type_ == "height_filtered") {
    return rectify_kinect_proc_.height_filtered_frame();
  }
  else if(cur_frame_output_type_ == "depth_filtered") {
    return rectify_kinect_proc_.depth_filtered_frame();
  }
}

void pu_prevention_chaining_process::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;

  turning_protocol_proc_.set_visualizer(viz);
}

void pu_prevention_chaining_process::visualize(void)
{		
	vil_image_view<vxl_byte> viz_img;
	viz_img = cur_frame();
	
	if( viz_ && viz_img.size() > 0 ) {
		gevxl::threading::scoped_lock lock( viz_ );
		viz_->initialize();
		if (viz_->is_initialized() ) { 
			
			viz_->set_image( viz_img );

			// visualize the overlay content
			turning_protocol_proc_.visualize_overlay();
      
			// flush out the content
			viz_->flush();
		}
	}
}

bool pu_prevention_chaining_process::start_recording(const vcl_string folder)
{
  return videoarchive_writer_proc_.start_recording(folder);
}

bool pu_prevention_chaining_process::stop_recording(void)
{
  return videoarchive_writer_proc_.stop_recording();
}
