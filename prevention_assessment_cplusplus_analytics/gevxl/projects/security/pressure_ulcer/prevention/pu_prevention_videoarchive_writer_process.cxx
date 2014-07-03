// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 02/27/2014
/// \par Modifications:

#include "pu_prevention_videoarchive_writer_process.h"

#include <vul/vul_file.h>
#include <vul/vul_sprintf.h>

#include <vid/openni2_frame_process.h>

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::prevention;
  
pu_prevention_videoarchive_writer_process::pu_prevention_videoarchive_writer_process(char const *name)
: gevxl::framework::process(name),
  source_(NULL),
	tag_source_(NULL),
	writer_openned_(false),
	subfolder_id_(0),
  stop_recording_flag_(false)
{
	
}

pu_prevention_videoarchive_writer_process::~pu_prevention_videoarchive_writer_process(void)
{
	// to be continued
}

bool pu_prevention_videoarchive_writer_process::configure(gevxl::util::config_file &config)
{
	config_ = config;

	bool enabled = false;
	config.get_bool(name()+"::enabled", enabled);
	enable(enabled);

	// depth image sequence 
	depth_img_seq_root_folder_ = "";
	config.get_string(name()+"::depth_img_seq_root_folder", depth_img_seq_root_folder_);

	// depth image sequence filename pattern
	depth_img_seq_filename_pattern_ = "depth_%04N_%04I_%6.1T.png";
	config.get_string(name()+"::depth_img_seq_filename_pattern", depth_img_seq_filename_pattern_);

	// time window in millisecond for the frames placed in one subfolder
	time_window_in_millisecond_in_subfolder_ = 1000*60;	// 1000 millisecond (1 sec) x 60 sec = 1 minutes time window
	config.get_double(name()+"::time_window_in_millisecond_in_subfolder", time_window_in_millisecond_in_subfolder_);
	
	return true;
}

bool pu_prevention_videoarchive_writer_process::initialize(void)
{
	if(!is_enabled()) return true;

	if(!source_) {
		vcl_cerr << "pu_prevention_videoarchive_writer_process::initialize, source_ is not set." << vcl_endl;
		return false;
	}
	
	if(!tag_source_) {
		vcl_cerr << "pu_prevention_videoarchive_writer_process::initialize, tag_source_ is not set." << vcl_endl;
		return false;	
	}

	const vid::openni2_frame_process *openni2_proc = dynamic_cast<const vid::openni2_frame_process *>(source_->get_frame_process());
	if(!openni2_proc) {
		vcl_cerr << "pu_prevention_videoarchive_writer_process::initialize, the source must be the openni2_frame_process." << vcl_endl;
		return false;	
	}

	return true;
}

void pu_prevention_videoarchive_writer_process::uninitialize(void)
{
	if(!is_enabled()) return;

	if(writer_openned_) {
		close();
		writer_openned_ = false;
	}
}

bool pu_prevention_videoarchive_writer_process::step(void)
{
	if(!is_enabled()) return true;

	// handle the subfolder openning.
	if(!writer_openned_) {
		vcl_string subfolder = "depth_recording_%04d";
		subfolder = vul_sprintf( subfolder.c_str(), subfolder_id_ );
		subfolder_id_++;
		
		if( !create_subfolder(subfolder) ) {
			vcl_cerr << "pu_prevention_videoarchive_writer_process::step, create_subfolder errors." << vcl_endl;
			return false;
		}

		if( !open(subfolder) ) {
			vcl_cerr << "pu_prevention_videoarchive_writer_process::step, open errors." << vcl_endl;
			return false;
		}

		time_code_of_the_first_frame_in_subfolder_ = tag_source_->get_time_code();
		writer_openned_ = true;	
	}

	if(!writer_openned_) {
		vcl_cerr << "pu_prevention_videoarchive_writer_process::step, writer_openned_ must be true for frame write out." << vcl_endl;
		return false;
	}
	
	// deal with the frame tag
	tag_.invalidate();
	tag_.set_frame_id(tag_source_->get_frame_id());
	tag_.set_frame_nr(tag_source_->get_frame_nr());
	tag_.set_time_code(tag_source_->get_time_code());

	// start to write the frame
	const vid::openni2_frame_process *openni2_proc = dynamic_cast<const vid::openni2_frame_process *>(source_->get_frame_process());
	const vil_image_view<vxl_uint_16> &depth_frame = openni2_proc->cur_depth_frame();

	// write out the frame
	depth_img_seq_writer_.write_frame(depth_frame, tag_);

	// write out the frame tag
	tag_ofstream_ << tag_ << vcl_endl;
	tag_ofstream_.flush();
	
	// handle the subfolder closing.
	if(writer_openned_) {
		double current_time_code = tag_source_->get_time_code();
		if(current_time_code - time_code_of_the_first_frame_in_subfolder_ >= time_window_in_millisecond_in_subfolder_) {
			close();
			writer_openned_ = false;
		}
	}

  if(stop_recording_flag_) {
    close();
	  writer_openned_ = false;
    stop_recording_flag_ = false;
    enable(false);
  }

	return true;
}

bool pu_prevention_videoarchive_writer_process::create_subfolder(const vcl_string &subfolder)
{
	if(!is_enabled()) return true;
	vcl_string fullpath = depth_img_seq_root_folder_ + "/" + subfolder;
	return vul_file::make_directory_path(fullpath);
}

bool pu_prevention_videoarchive_writer_process::open(const vcl_string &subfolder)
{
	if(!is_enabled()) return true;

	vcl_string full_pattern = depth_img_seq_root_folder_ + "/" + subfolder + "/" + depth_img_seq_filename_pattern_;
	
	// open the depth_img_seq_writer_
	bool success = true;
	depth_img_seq_writer_.close();
	success = depth_img_seq_writer_.open(full_pattern.c_str());
	
	// open the frame_tag writer
	vcl_string tag_filename = depth_img_seq_root_folder_ + "/" + subfolder + "/frame_tags.dat";
	if(tag_ofstream_.is_open()) {
		tag_ofstream_.close();
	}
	tag_ofstream_.open(tag_filename.c_str());

	return success;
}

void pu_prevention_videoarchive_writer_process::close(void)
{
	if(!is_enabled()) return;

	depth_img_seq_writer_.close();

	if(tag_ofstream_.is_open()) {
		tag_ofstream_.close();
	}
}

bool pu_prevention_videoarchive_writer_process::start_recording(const vcl_string &folder)
{
  depth_img_seq_root_folder_ = folder;
  subfolder_id_ = 0;

  if(writer_openned_) {
		close();
		writer_openned_ = false;
	}

  enable(true);

  return true;
}

bool pu_prevention_videoarchive_writer_process::stop_recording(void)
{
  stop_recording_flag_ = true;

  return true;
}

