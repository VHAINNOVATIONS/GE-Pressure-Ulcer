// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_prevention_pose_estimate_chaining_process.h"

#include <vcl_iostream.h>

#include <vil/vil_save.h>
#include <vil/vil_load.h>

#include <vil/vil_convert.h>
#include <vil/vil_flip.h>

#include <vid/openni2_frame_process.h>

#include <img/image_as_string.h>
#include <vid/frame_tag.h>

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer::prevention;

pu_prevention_pose_estimate_chaining_process::pu_prevention_pose_estimate_chaining_process(char const *name)
: gevxl::framework::process(name), 
  source_proc_("vid::pu_prevention_pose_estimate_source_process"),
  viz_(NULL), 
  viz_img_(NULL), 
  avi_fps_(30),
  rgb_video_out_openned_(false), 
  depth_video_out_openned_(false), 
  height_filtered_video_out_openned_(false), 
  depth_filtered_video_out_openned_(false), 
  frame_nr_(0),
  cur_frame_output_type_("depth"),
  verbose_(0)
{
	highres_timer_.reset();
}

pu_prevention_pose_estimate_chaining_process::~pu_prevention_pose_estimate_chaining_process(void)
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

bool pu_prevention_pose_estimate_chaining_process::configure(util::config_file &config)
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
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::configure, Error configuring source_proc_." << vcl_endl;
		return false;
	}
	source_proc_.set_output_type("depth"); // ensure the output is the depth view

	// ---  Configure the frame tag process
  if( !frame_tag_proc_.configure(config) ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::configure, Error configuring frame_tag_proc_." << vcl_endl;
		return false;
	}

	// -- Configure the rectify_kinect process, and the rectify_kinect_process is always enabled.
	if( !rectify_kinect_proc_.configure(config) ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::configure, Error configuring rectify_kinect_proc_." << vcl_endl;
		return false;
	}

	// cur output frame type for visualization
  cur_frame_output_type_ = "depth";
  config.get_string(name()+"::cur_frame_output_type", cur_frame_output_type_);

	return true;
}

bool pu_prevention_pose_estimate_chaining_process::initialize(void)
{
	if( !source_proc_.initialize() ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::initialize, Error initializing source_proc_." << vcl_endl;
		return false;
	}

	const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
	frame_tag_proc_.set_source_tagged_frame_process(openni2_source);
	if(!frame_tag_proc_.initialize()) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::initialize, Error initializing frame_tag_proc_." << vcl_endl;        
		return false;
	}
	frame_tag_proc_.use_tagged_frame_process_tag_information_if_available(true);

	// kinect's depth value rectification process based on the known ground plane points
  rectify_kinect_proc_.set_source_process(&source_proc_);
  if( !rectify_kinect_proc_.initialize() ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::initialize, Error initializing rectify_kinect_proc_." << vcl_endl;
		return false;
	}

	return true;
}

void pu_prevention_pose_estimate_chaining_process::uninitialize(void)
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

// ensure the avi files have been openned for the ffmpeg writing out
void pu_prevention_pose_estimate_chaining_process::ensure_avi_files_openned(const vil_image_view<vxl_byte> &rgb_img,
                                                              const vil_image_view<vxl_byte> &depth_byte_img)
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
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(avi_fps_).bit_rate(2000).size(depth_byte_img.ni(), depth_byte_img.nj()) );
									 //lossless, p.encoder(vid::io::ffmpeg_writer_param::HUFFYUV).frame_rate(avi_fps_).bit_rate(2000).size(depth_byte_img.ni(), depth_byte_img.nj()) );
			depth_video_out_openned_ = true;
	}

  if( !height_filtered_video_out_openned_ && height_filtered_video_out_filename_ != "" ) {
			vid::io::ffmpeg_writer_param p;
			height_filtered_video_out_.open( height_filtered_video_out_filename_.c_str(), 
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(avi_fps_).bit_rate(2000).size(depth_byte_img.ni(), depth_byte_img.nj()) );
									 //lossless, p.encoder(vid::io::ffmpeg_writer_param::HUFFYUV).frame_rate(avi_fps_).bit_rate(2000).size(depth_byte_img.ni(), depth_byte_img.nj()) );
			height_filtered_video_out_openned_ = true;
	}

	if( !depth_filtered_video_out_openned_ && depth_filtered_video_out_filename_ != "" ) {
			vid::io::ffmpeg_writer_param p;
			depth_filtered_video_out_.open( depth_filtered_video_out_filename_.c_str(), 
									 p.encoder(vid::io::ffmpeg_writer_param::MSMPEG4V2).frame_rate(avi_fps_).bit_rate(2000).size(depth_byte_img.ni(), depth_byte_img.nj()) );
									 //lossless, p.encoder(vid::io::ffmpeg_writer_param::HUFFYUV).frame_rate(avi_fps_).bit_rate(2000).size(depth_byte_img.ni(), depth_byte_img.nj()) );
			depth_filtered_video_out_openned_ = true;
	}
}

// ffmpeg write out 
void pu_prevention_pose_estimate_chaining_process::write_out_frames_to_avi_files(const vil_image_view<vxl_byte> &rgb_img,
                                                                   const vil_image_view<vxl_byte> &depth_byte_img)
{
  // write out the frames to avi files.
	if(rgb_video_out_openned_) {
		rgb_video_out_.write_frame(rgb_img);		
	}

	if(depth_video_out_openned_) {
		depth_video_out_.write_frame(depth_byte_img);		
	}

  if(height_filtered_video_out_openned_) {
    height_filtered_video_out_.write_frame(rectify_kinect_proc_.height_filtered_frame());		
	}

	if(depth_filtered_video_out_openned_) {
    depth_filtered_video_out_.write_frame(rectify_kinect_proc_.depth_filtered_frame());		
	}
}

bool pu_prevention_pose_estimate_chaining_process::step(void)
{
	// Stepping through individual process in the chain
	if( !source_proc_.step() ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::step, stepping openni2 source error." << vcl_endl;
		return false;
	}

	if( !frame_tag_proc_.step() ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::step, stepping frame_tag_proc_ error." << vcl_endl;
		return false;
	}

	const gevxl::vid::frame_tag &tag = frame_tag_proc_.cur_frame_tag();

	frame_nr_ = tag.get_frame_nr();
	if( verbose_ > 0 ) {
		vcl_cout << "pu_prevention_pose_estimate_chaining_process::step, frame_nr_ = " << frame_nr_ << vcl_endl;
	}

	// --- Rectify
	const vid::openni2_frame_process *openni2_proc = dynamic_cast<const vid::openni2_frame_process *>(source_proc_.get_frame_process());
	if( !openni2_proc ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::step, the generic_frame_process is not openni2_frame_process." << vcl_endl;
		return false;
	}

	const vil_image_view<vxl_byte> &rgb_img = openni2_proc->cur_rgb_frame();
	const vil_image_view<vxl_byte> &depth_byte_img = openni2_proc->cur_depth_byte_frame();
	const vil_image_view<vxl_uint_16> &raw_depth_frame = openni2_proc->cur_depth_frame();
  
  // ensure the avi files are openned
  ensure_avi_files_openned(rgb_img, depth_byte_img);
	
	// rectification process
	if( !rectify_kinect_proc_.step( openni2_proc->cur_xyz_rgb_frame() ) ) {
		vcl_cerr << "pu_prevention_pose_estimate_chaining_process::step(), Error executing rectify_kinect_proc_.step()." << vcl_endl;
		return false;
	}

	const vil_image_view<vxl_byte> &filtered_depth_frame = rectify_kinect_proc_.depth_filtered_frame();
  const vil_image_view<float> &rectified_xyz_frame = rectify_kinect_proc_.rectified_xyz_frame();
  
	vgl_point_3d<float>  origin;
  vgl_vector_3d<float> vx, vy, vz;
  rectify_kinect_proc_.get_transformed_coordinate_system(origin, vx, vy, vz);

  //-- Deal with the frame rate issue.
	double frame_rate = 1000/highres_timer_.elapsed();    
	highres_timer_.reset();

	//-- Visualize the process
	visualize();

	// write out the frames to the various avi files
  write_out_frames_to_avi_files(rgb_img, depth_byte_img);

  return true;
}

const vil_image_view<vxl_byte> &pu_prevention_pose_estimate_chaining_process::cur_frame(void) const
{
	const gevxl::vid::openni2_frame_process *openni2_source = dynamic_cast<const gevxl::vid::openni2_frame_process *>(source_proc_.get_frame_process());
	
	if(cur_frame_output_type_ == "depth") {
		openni2_source->cur_depth_byte_frame();
  }
	else if(cur_frame_output_type_ == "rgb") {
		openni2_source->cur_rgb_frame();
	}
  else if(cur_frame_output_type_ == "height_filtered") {
    return rectify_kinect_proc_.height_filtered_frame();
  }
  else if(cur_frame_output_type_ == "depth_filtered") {
    return rectify_kinect_proc_.depth_filtered_frame();
  }
}

void pu_prevention_pose_estimate_chaining_process::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;  
}

void pu_prevention_pose_estimate_chaining_process::visualize(void)
{		
	vil_image_view<vxl_byte> viz_img;
	viz_img = cur_frame();
	
	if( viz_ && viz_img.size() > 0 ) {
		gevxl::threading::scoped_lock lock( viz_ );
		viz_->initialize();
		if (viz_->is_initialized() ) { 
			
			viz_->set_image( viz_img );

			viz_->flush();
		}
	}
}
