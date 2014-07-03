// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "micro_epsilon_socket_frame_process.h"
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <util/time/global_time.h>
#include <vid/config.h>
#include <threading/thread.h>
#include <vcl_algorithm.h>
#include "micro_epsilon_socket_camera.h"

#include <vil/vil_save.h>
#include <img/image_raw_file_io.h>
#include <util/string.h>

#include <threading/sleep.h>

#include <vul/vul_sprintf.h>
#include <vul/vul_file.h>

#if defined(VCL_WIN32) && !defined(__CYGWIN__) && !defined(__MINGW32__)
#include <process.h>
#endif

using namespace gevxl::vid;
using namespace gevxl::threading;

micro_epsilon_socket_frame_process::micro_epsilon_socket_frame_process( char const * n )
    : gevxl::vid::tagged_frame_process<vxl_byte>(n),
      capture_mode_("camera"), 
      initialized_(false),
      verbose_(0),      
      frame_nr_(0),
			saved_out_frame_nr_(0),
			captured_frame_system_time_(0),
      camera_(NULL),
      gui_recording_flag_(false),
      gui_stop_recording_flag_(false),
      gui_playback_flag_(false),
      thermal_file_directory_(""),
      gui_thermal_frame_tag_ifstream_(NULL)
{

}

micro_epsilon_socket_frame_process::~micro_epsilon_socket_frame_process()
{
	if(capture_mode_ == "camera") {
		
		// terminate the thermal_cam_capture.exe process
		//vcl_string call = "taskkill /F /T /IM thermal_cam_capture.exe";
		//int retval = system(call.c_str());

		if(NULL != camera_) {
			delete camera_;
			camera_ = NULL;
		}

		set_save_out_flag(false);
	}
}


bool micro_epsilon_socket_frame_process::camera_frames_save(const int frame_nr)
{
	vcl_string filename;

	bool success = true;

	// thermal_frame_ 
	filename = filepath_ + "/thermal_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_save(thermal_frame_, filename);
	if(!success) return false;
	
	return true;
}

bool micro_epsilon_socket_frame_process::frame_tags_load(void)
{
	vcl_string frame_tag_filename = filepath_ + "/frame_tags.dat";

	vcl_ifstream frame_tag_ifstream;
	frame_tag_ifstream.open(frame_tag_filename.c_str());

	loaded_tags_.clear();
	gevxl::vid::frame_tag tag;
	while(frame_tag_ifstream.good() && frame_tag_ifstream >> tag) {
		loaded_tags_.push_back(tag);	
	}
	
	return true;
}

bool micro_epsilon_socket_frame_process::playback_frames_load(const int frame_nr)
{
	vcl_string filename;

	bool success = true;
	
	// thermal_frame_ 
	filename = filepath_ + "/thermal_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_load(thermal_frame_, filename);
	if(!success) return false;
	
	return true;
}

void micro_epsilon_socket_frame_process::set_save_out_flag(bool flag)
{	
	save_out_ = flag;
	
	if(save_out_) {
		// start the new save out session, let's open the frame_tag_ofstream_
		vcl_string frame_tag_filename = filepath_ + "/frame_tags.dat";
		if(frame_tag_ofstream_.is_open()) {
			frame_tag_ofstream_.close();
		}
		frame_tag_ofstream_.open(frame_tag_filename.c_str());

		saved_out_frame_nr_ = 0;
	}
	else {
		// stop the new save out session, let's close the frame_tag_ofstream_
		if(frame_tag_ofstream_.is_open()) {
			frame_tag_ofstream_.close();
		}
	}
}

bool micro_epsilon_socket_frame_process::configure( gevxl::util::config_file & config )
{
  config_ = config;

	img_width_ = 384;	// the original image width is 382, which can not be devided by 4. so (382/4)*4=384
  img_height_ = 288;
  
	port_ = 9080;
  config_.get_integer(name()+"::port", port_);

  upsidedown_leftright_flipping_ = true;
  config_.get_bool(name()+"::upsidedown_leftright_flipping", upsidedown_leftright_flipping_);
  
	filepath_ = "";
	config_.get_string(name()+"::file_storage_directory", filepath_);
	if(filepath_ == "") {
		vcl_cerr << "micro_epsilon_socket_frame_process::configure, file_storage_directory is not set." << vcl_endl;
		return false;
	}

	capture_mode_ = "camera";
	config_.get_string(name()+"::capture_mode", capture_mode_);

  verbose_ = 0;
  config_.get_integer(name()+"::verbose", verbose_);

  viz_mode_ = 0;
  config_.get_integer(name()+"::viz_mode", viz_mode_);

  supported_output_type_.insert(vcl_make_pair("color", 0));
  supported_output_type_.insert(vcl_make_pair("grayscale", 1));  

  set_num_outputs(supported_output_type_.size());
  
	vcl_string color_palette_filepath = "";
	config_.get_string(name()+"::color_palette_filepath", color_palette_filepath);
	if(!read_in_color_palette(color_palette_filepath)) {
		vcl_cerr << "micro_epsilon_socket_frame_process::configure, read_in_color_palette error.\n" << vcl_endl;
		return false;
	}

	max_frame_waiting_time_in_ms_ = 30;
	config_.get_integer(name()+"::max_frame_waiting_time_in_ms", max_frame_waiting_time_in_ms_);

  // initialize the frames
  thermal_frame_ = vil_image_view<float>(img_width_, img_height_, 1, 1);
  thermal_frame_.fill(0);

  thermal_frame_in_C_ = vil_image_view<float>(img_width_, img_height_, 1, 1);
  thermal_frame_in_C_.fill(0);

  thermal_frame_in_F_ = vil_image_view<float>(img_width_, img_height_, 1, 1);
  thermal_frame_in_F_.fill(0);
  
  thermal_byte_frame_ = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 1);
  thermal_byte_frame_.fill(0);
  
  thermal_color_palette_frame_ = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 3);
  thermal_color_palette_frame_.fill(0);

	if(capture_mode_ == "camera") {
		// initialize the camera
		camera_ = new micro_epsilon_socket_camera(img_height_, img_width_, port_, upsidedown_leftright_flipping_);
		camera_->initialize_socket_server();

		bool flag = false;
		config_.get_bool(name()+"::save_out", flag);
		set_save_out_flag(flag);

		//gevxl::threading::sleep(3000);
		//vcl_string thermal_cam_capture_executable_filepath;
		//config_.get_string(name()+"::thermal_cam_capture_executable_filepath", thermal_cam_capture_executable_filepath);
		//thermal_cam_capture_executable_filepath = "start call \"" + thermal_cam_capture_executable_filepath + "\"";
		//system(thermal_cam_capture_executable_filepath.c_str());
	}
	else if(capture_mode_ == "playback") {
		// this is the playback mode
		bool success = frame_tags_load();
		if(!success) {
			vcl_cerr << "micro_epsilon_socket_frame_process::configure, frame_tags_load is not successful." << vcl_endl;
			return false;
		}
	}
	else {
		vcl_cerr << "micro_epsilon_socket_frame_process::configure, capture_mode_ unknown." << vcl_endl;
		return false;
	}
  
  initialized_ = true;
  return initialized_;
}

bool micro_epsilon_socket_frame_process::read_in_color_palette(vcl_string filepath)
{
	vcl_ifstream infile;
	infile.open(filepath.c_str());

	color_palette_.resize(256);
	for(int i = 0; i < 256; i++) {
		color_palette_[i].resize(3);
	}

	int val = 0;
	for(int i = 0; i < 256; i++) {
		for(int j = 0; j < 3; j++) {
			if( (infile >> val) == false ) {
				return false;
			}
			color_palette_[i][j] = vxl_byte(val);
		}
	}
	return true;
}

bool micro_epsilon_socket_frame_process::step()
{
  if(!initialized_) return false;

	int frame_nr, frame_id;
	double frame_time_code;

  bool result = false;
  
	if(capture_mode_ == "camera") {
		if(max_frame_waiting_time_in_ms_ == -1) {
			while(true) {
				// infinite long time of waiting
				result = camera_->get_next_frame(thermal_frame_, captured_frame_system_time_);
				if(result) break;
				thread::sleep(1);
			}
		}
		else {
			for(int i = 0; i < max_frame_waiting_time_in_ms_; i++) {
				// block waiting to get the next frame, but only wait for max_frame_waiting_time_in_ms_ milliseconds
				result = camera_->get_next_frame(thermal_frame_, captured_frame_system_time_);
				if(result) break;
				thread::sleep(1);
			}
			if(!result) return true;
		}

		frame_nr = frame_nr_;
		frame_id = frame_nr_;
		frame_time_code = captured_frame_system_time_;
	}
	else {
		// capture frames from the playback file.
		if(frame_nr_ >= loaded_tags_.size()) {
			frame_nr = loaded_tags_.back().get_frame_nr();
			frame_id = loaded_tags_.back().get_frame_id();
			frame_time_code = loaded_tags_.back().get_time_code();
		}
		else {
			frame_nr = loaded_tags_[frame_nr_].get_frame_nr();
			frame_id = loaded_tags_[frame_nr_].get_frame_id();
			frame_time_code = loaded_tags_[frame_nr_].get_time_code();
		}

		result = playback_frames_load(frame_nr);
		if(!result) {
			vcl_cerr << "micro_epsilon_socket_frame_process::step, playback mode can not load the frames from the file." << vcl_endl;
			return false;
		}
	}

  // set the frame tag
  tag_.invalidate();
  tag_.set_frame_nr( frame_nr );
  tag_.set_frame_id( frame_id );
  tag_.set_time_code( frame_time_code );

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // the new implementation to support the GUI controlled recording and playback
  if(gui_recording_flag_) {
    result = recording_frame();
    if(!result) {
			vcl_cerr << "micro_epsilon_socket_frame_process::step, recording_frame error." << vcl_endl;
			return false;
		}
  }
  else if(gui_playback_flag_) {
    result = playback_frame();
    if(!result) {
			vcl_cerr << "micro_epsilon_socket_frame_process::step, playback_frame error." << vcl_endl;
			return false;
		}
  }

  if(gui_stop_recording_flag_) {
    // make sure the frame tag files are closed
    if(gui_thermal_frame_tag_ofstream_.is_open()) {
	    gui_thermal_frame_tag_ofstream_.close();
    }

    gui_stop_recording_flag_ = false;
    gui_recording_flag_ = false;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // create the thermal_byte_frame_ and thermal_color_palette_frame_
  int min_val = 1e+10, max_val = -(1e+10);
  double fact = 0.0;
  int frame_size = img_width_*img_height_;
  float *ptr = thermal_frame_.top_left_ptr();
  float *ptr_in_C = thermal_frame_in_C_.top_left_ptr();
  float *ptr_in_F = thermal_frame_in_F_.top_left_ptr();
  
  for(int i = 0; i < frame_size; i++, ptr++) {
    if((*ptr) < min_val) min_val = (*ptr);
    if((*ptr) > max_val) max_val = (*ptr);
  }
  
  if(max_val == 0 && min_val == 0) {
    thermal_frame_.fill(0);
    thermal_frame_in_C_.fill(0);
    thermal_frame_in_F_.fill(0);
    thermal_byte_frame_.fill(0);
    thermal_color_palette_frame_.fill(0);
  }
  else {

    fact = 255.0/(max_val - min_val);

    // assign the pixel value
    ptr = thermal_frame_.top_left_ptr();
    ptr_in_C = thermal_frame_in_C_.top_left_ptr();
    ptr_in_F = thermal_frame_in_F_.top_left_ptr();
    vxl_byte *byte_ptr = thermal_byte_frame_.top_left_ptr();
    vxl_byte *color_ptr = thermal_color_palette_frame_.top_left_ptr();
    
    for(int i = 0; i < frame_size; i++, ptr++, ptr_in_C++, ptr_in_F++, byte_ptr++, color_ptr=color_ptr+3) {
      
      // rescale the value reading to [0, 255]
      *byte_ptr = (vxl_byte)( vcl_min( vcl_max((int)(fact*((*ptr) - min_val)), 0), 255 ) );
      
      // will replace with the color palette later
      *color_ptr = color_palette_[(int)(*byte_ptr)][0];
      *(color_ptr+1) = color_palette_[(int)(*byte_ptr)][1];
      *(color_ptr+2) = color_palette_[(int)(*byte_ptr)][2];

      // now really calculate the temperature in celsius based on micro-epsilon's equation
      *ptr_in_C = (*ptr - 1000)/10.0;

      // now computes the temperature in Fahrenheit
      *ptr_in_F = (*ptr_in_C*9.0)/5.0 + 32.0;
    }
  }
  
	if(capture_mode_ == "camera" && save_out_) {
		result = camera_frames_save(saved_out_frame_nr_);
		if(!result) {
			vcl_cerr << "micro_epsilon_socket_frame_process::step, camera frames save out error." << vcl_endl;
			return false;
		}

		saved_out_tag_.invalidate();
		saved_out_tag_.set_frame_nr( saved_out_frame_nr_ );
		saved_out_tag_.set_frame_id( saved_out_frame_nr_ );
		saved_out_tag_.set_time_code( frame_time_code );

		frame_tag_ofstream_ << saved_out_tag_ << vcl_endl;
		frame_tag_ofstream_.flush();

		saved_out_frame_nr_++;
	}

	if(frame_nr_ % 100 == 0) {
		vcl_cout << "micro_epsilon_socket_frame_process::step(), captured_frame_system_time_ = " << (long long)(frame_time_code) << vcl_endl;
	}

	frame_nr_++;

  return true;
}

vil_image_view<vxl_byte> const &micro_epsilon_socket_frame_process::cur_frame() const
{
  if(output_id_ >= num_outputs_) {
    vcl_cout << "micro_epsilon_socket_frame_process::cur_frame()-- Warning, output_id > number of supported outputs!\n";
  }

  switch (output_id_) {
  case 0:
    {
      return thermal_color_palette_frame_;
      break;
    }
  case 1:
    {
      return thermal_byte_frame_;
      break;
    }
  default:
    {
      return thermal_color_palette_frame_;
      break;
    }
  }
}

//pixel unit in degree C
vil_image_view<float> const &micro_epsilon_socket_frame_process::cur_thermal_frame_in_C() const
{
  return thermal_frame_in_C_;
}

//pixel unit in degree F
vil_image_view<float> const &micro_epsilon_socket_frame_process::cur_thermal_frame_in_F() const
{
  return thermal_frame_in_F_;
}

//vxl_byte pixel of 0 to 255
vil_image_view<vxl_byte> const &micro_epsilon_socket_frame_process::cur_thermal_byte_frame() const
{
  return thermal_byte_frame_;
}

//pseudo color heat map
vil_image_view<vxl_byte> const &micro_epsilon_socket_frame_process::cur_thermal_color_palette_frame() const
{
  return thermal_color_palette_frame_;
}

bool micro_epsilon_socket_frame_process::take_snapshot(const vcl_string &thermal_file_directory, 
                                                       const vcl_string &viewpoint_description)
{
  vcl_string filename;

	bool success = true;

  vcl_string frame_nr_str = vul_sprintf( "_%04d", tag_.get_frame_nr() );
  vcl_string frame_time_code_str = vul_sprintf( "_%6.1f", tag_.get_time_code() );

  vcl_string frame_info_str = frame_nr_str + frame_time_code_str 
                              + "_" + viewpoint_description + ".raw";

  // thermal_frame_
	filename = thermal_file_directory + "/thermal" + frame_info_str;
	success = gevxl::img::image_raw_save(thermal_frame_, filename);
	if(!success) return false;
	
	return true; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// the new implementation to support the GUI controlled recording and playback
/// Control command to start/stop/playback recording
bool micro_epsilon_socket_frame_process::start_recording(const vcl_string &thermal_file_directory, 
                                                          const vcl_string &viewpoint_description)
{
  // ensure the folders are created.
  thermal_file_directory_ = thermal_file_directory + "/" + viewpoint_description;
  vul_file::make_directory_path(thermal_file_directory_);

  // make sure the frame tag files are openned for recording
  vcl_string filename;
  
  filename = thermal_file_directory_ + "/frame_tags.dat";
  if(gui_thermal_frame_tag_ofstream_.is_open()) {
    gui_thermal_frame_tag_ofstream_.close();
  }
	gui_thermal_frame_tag_ofstream_.open(filename.c_str());

  gui_recording_flag_ = true;
  gui_playback_flag_ = false;

  return true;
}

bool micro_epsilon_socket_frame_process::stop_recording(void)
{
  gui_stop_recording_flag_ = true;
  
  return true;
}

bool micro_epsilon_socket_frame_process::playback_recording(const vcl_string &thermal_file_directory, 
                                                              const vcl_string &viewpoint_description)
{
  thermal_file_directory_ = thermal_file_directory + "/" + viewpoint_description;

  // make sure the frame tag files are openned for playback
  vcl_string filename;
  
  filename = thermal_file_directory_ + "/frame_tags.dat";
  gui_thermal_frame_tag_ifstream_ = new vcl_ifstream(filename.c_str());

  gui_playback_flag_ = true;
  gui_recording_flag_ = false;

  return true;
}

bool micro_epsilon_socket_frame_process::recording_frame(void)
{
  vcl_string filename;

	bool success = true;

  vcl_string frame_nr_str = vul_sprintf( "_%04d", tag_.get_frame_nr() );
  vcl_string frame_time_code_str = vul_sprintf( "_%6.1f", tag_.get_time_code() );

  vcl_string frame_info_str = frame_nr_str + frame_time_code_str + ".raw";

  // thermal_frame_ 
	filename = thermal_file_directory_ + "/thermal" + frame_info_str;
	success = gevxl::img::image_raw_save(thermal_frame_, filename);
	if(!success) return false;
	
  // save out the frame tag
  gui_thermal_frame_tag_ofstream_ << tag_ << vcl_endl;
	gui_thermal_frame_tag_ofstream_.flush();

  return true; 
}

bool micro_epsilon_socket_frame_process::playback_frame(void)
{
  // the behavior is that once all the recorded frames have been played back, 
  // the process automatically switches back to the live camera capture mode.
  
  bool success = true;

  vcl_string filename;
  vcl_string frame_nr_str, frame_time_code_str, frame_info_str;

  bool playback_end_flag = false;

  // load back one frame tag
  gevxl::vid::frame_tag thermal_tag;
	if(gui_thermal_frame_tag_ifstream_ != NULL && 
      gui_thermal_frame_tag_ifstream_->is_open() && 
      //gui_thermal_frame_tag_ifstream_->good() && 
      *gui_thermal_frame_tag_ifstream_ >> thermal_tag) {

		// load one thermal_tag;
    frame_nr_str = vul_sprintf( "_%04d", thermal_tag.get_frame_nr() );
    frame_time_code_str = vul_sprintf( "_%6.1f", thermal_tag.get_time_code() );

    frame_info_str = frame_nr_str + frame_time_code_str + ".raw";
    
    // thermal_frame_
    filename = thermal_file_directory_ + "/thermal" + frame_info_str;
	  success = gevxl::img::image_raw_load(thermal_frame_, filename);
	  if(!success) return false;
	}
  else {
    playback_end_flag = true;
  }

  if(playback_end_flag) {

    if(gui_thermal_frame_tag_ifstream_ != NULL && 
        gui_thermal_frame_tag_ifstream_->is_open()) {
      
      gui_thermal_frame_tag_ifstream_->close();
      delete gui_thermal_frame_tag_ifstream_;
      gui_thermal_frame_tag_ifstream_ = NULL;
    }
    
    gui_playback_flag_ = false;
  }

  return true;
}
