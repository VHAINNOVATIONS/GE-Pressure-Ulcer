// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "bayspec_socket_frame_process.h"

#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_algorithm.h>

#include <util/time/global_time.h>
#include <util/string.h>

#include <threading/thread.h>
#include <threading/sleep.h>

#include <vul/vul_sprintf.h>
#include <vul/vul_file.h>
#include <vul/vul_file_iterator.h>

#include <vil/vil_save.h>
#include <vil/vil_load.h>

#include "bayspec_socket_camera.h"

#if defined(VCL_WIN32) && defined(VCL_VC)
  #include <Windows.h>
#else
  #include <sys/time.h>
  #include <sys/stat.h>
#endif

using namespace gevxl::vid;
using namespace gevxl::threading;

bayspec_socket_frame_process::bayspec_socket_frame_process( char const * n )
: gevxl::vid::tagged_frame_process<vxl_byte>(n),
  file_storage_directory_(""),  
  capture_mode_("camera"),
  save_out_(false),
  initialized_(false),
  frame_initialized_(false),
  verbose_(0),
  viz_mode_(0),
  frame_nr_(0),
  saved_out_frame_nr_(0),
  captured_frame_system_time_(0),
  server_host_or_ip_("localhost"),
  port_(""),
  max_frame_waiting_time_in_ms_(30),
  camera_com_msg_interval_elapse_in_ms_(1000),
  camera_(NULL),
  white_ref_folder_(""),
  dark_bkg_folder_(""),
  rotation_info_("clockwise_90")
{
 
}

bayspec_socket_frame_process::~bayspec_socket_frame_process()
{
	if(capture_mode_ == "camera") {
		
    if(NULL != camera_) {
      camera_->uninitialize_socket_client();
			delete camera_;
			camera_ = NULL;
		}

		set_save_out_flag(false);
	}
}

bool bayspec_socket_frame_process::frame_tags_load(void)
{
	vcl_string frame_tag_filename = file_storage_directory_ + "\\frame_tags.dat";

	vcl_ifstream frame_tag_ifstream;
	frame_tag_ifstream.open(frame_tag_filename.c_str());

	loaded_tags_.clear();
	gevxl::vid::frame_tag tag;
	while(frame_tag_ifstream.good() && frame_tag_ifstream >> tag) {
		loaded_tags_.push_back(tag);	
	}
	
	return true;
}

bool bayspec_socket_frame_process::playback_frames_load(const int frame_nr)
{
	vcl_string filename;

	bool success = true;
	
  // to be continued later.

	return true;
}

void bayspec_socket_frame_process::set_save_out_flag(bool flag)
{	
	save_out_ = flag;
	
	if(save_out_) {
		// start the new save out session, let's open the frame_tag_ofstream_
		vcl_string frame_tag_filename = file_storage_directory_ + "\\frame_tags.dat";
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

bool bayspec_socket_frame_process::configure( gevxl::util::config_file & config )
{
  config_ = config;

  // configure the server machine's hostname or ip address
  server_host_or_ip_ = "localhost";
  config_.get_string(name()+"::server_host_or_ip", server_host_or_ip_);

  // configure the server machine's port number
	port_ = "";
  config_.get_string(name()+"::port", port_);
  if(port_ == "") {
		vcl_cerr << "bayspec_socket_frame_process::configure, port is not set." << vcl_endl;
		return false;
	}
  
  // configure the image size and allocate frame memory space
  img_height_ = 256;
  img_width_ = 256;

  frame_ = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 1);
  frame_.fill(0);
  
  three_wavelength_bands_frame_ = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 3);
  three_wavelength_bands_frame_.fill(0);

  three_one_wavelength_band_frames_[0] = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 1);
  three_one_wavelength_band_frames_[0].fill(0);

  three_one_wavelength_band_frames_[1] = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 1);
  three_one_wavelength_band_frames_[1].fill(0);

  three_one_wavelength_band_frames_[2] = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 1);
  three_one_wavelength_band_frames_[2].fill(0);

  // configure to specify which wavelength band we are visualizing for the single wavelength band
  wavelength_band_idx_ = 0;
  config_.get_integer(name()+"::wavelength_band_idx", wavelength_band_idx_);

  // configure to specify which three wavelength bands we are visualizing for the colorized wavelength bands image
  three_wavelength_bands_idx_.resize(3);
  three_wavelength_bands_idx_[0] = 0;
  three_wavelength_bands_idx_[1] = 10;
  three_wavelength_bands_idx_[2] = 20;

  vcl_vector<int> bands;
  bands.resize(3);
  config_.get_vcl_vector_int(name()+"::three_wavelength_bands_idx", bands);
  if( (bands.size() == 3) && (bands[0] != 0 || bands[1] != 0 || bands[2] != 0) ) {
    three_wavelength_bands_idx_[0] = bands[0];
    three_wavelength_bands_idx_[1] = bands[1];
    three_wavelength_bands_idx_[2] = bands[2];
  }

  // rotating the captured frames 90 degree clockwise or counter-clockwise or clockwise_180 or upside_down or none
  rotation_info_ = "clockwise_90";  // clockwise_90, counter_clockwise_90, clockwise_180, upside_down, none
  config_.get_string(name()+"::rotation_info", rotation_info_);

  // configure the file storage directory
	file_storage_directory_ = "";
	config_.get_string(name()+"::file_storage_directory", file_storage_directory_);
	if(file_storage_directory_ == "") {
		vcl_cerr << "bayspec_socket_frame_process::configure, file_storage_directory is not set." << vcl_endl;
		return false;
	}
  vcl_replace(file_storage_directory_.begin(), file_storage_directory_.end(), '/', '\\');

  // configure the directory where the white reference is stored
  white_ref_folder_ = "";
  config_.get_string(name()+"::white_ref_folder", white_ref_folder_);
	if(white_ref_folder_ == "") {
		vcl_cerr << "bayspec_socket_frame_process::configure, white_ref_folder is not set." << vcl_endl;
		return false;
	}
  vcl_replace(white_ref_folder_.begin(), white_ref_folder_.end(), '/', '\\');

  // configure the directory where the dark background is stored
  dark_bkg_folder_ = "";
  config_.get_string(name()+"::dark_bkg_folder", dark_bkg_folder_);
	if(dark_bkg_folder_ == "") {
		vcl_cerr << "bayspec_socket_frame_process::configure, dark_bkg_folder is not set." << vcl_endl;
		return false;
	}
  vcl_replace(dark_bkg_folder_.begin(), dark_bkg_folder_.end(), '/', '\\');

  // configure the operation mode, camera vs playback
	capture_mode_ = "camera";
	config_.get_string(name()+"::capture_mode", capture_mode_);

  verbose_ = 0;
  config_.get_integer(name()+"::verbose", verbose_);

  // configure the visualization mode
  viz_mode_ = 0;
  config_.get_integer(name()+"::viz_mode", viz_mode_);

  supported_output_type_.insert(vcl_make_pair("color", 0));
  supported_output_type_.insert(vcl_make_pair("grayscale", 1));  

  set_num_outputs(supported_output_type_.size());

  // configure the waiting time for per frame, affecting the frame rate.
  max_frame_waiting_time_in_ms_ = 30;
	config_.get_integer(name()+"::max_frame_waiting_time_in_ms", max_frame_waiting_time_in_ms_);

  // configure how often we send the camera control command to the server.
  camera_com_msg_interval_elapse_in_ms_ = 1000;
  config_.get_integer(name()+"::camera_com_msg_interval_elapse_in_ms", camera_com_msg_interval_elapse_in_ms_);

  if(capture_mode_ == "camera") {
		// initialize the camera.
		camera_ = new bayspec_socket_camera(server_host_or_ip_, port_);
    camera_->initialize_socket_client();
    threading::sleep(1000);

    // setup the white_ref_folder_ and dark_bkg_folder_ on the camera server.
    vcl_string com_msg;
    
    com_msg = "set_white_ref_folder=" + white_ref_folder_;
    camera_->send_command_message(com_msg);
    threading::sleep(1000);

    com_msg = "set_dark_bkg_folder=" + dark_bkg_folder_;
    camera_->send_command_message(com_msg);
    threading::sleep(1000);

    // as this is the live capture, we can save out the image data.
    bool flag = false;
		config_.get_bool(name()+"::save_out", flag);
		set_save_out_flag(flag);

    frame_nr_of_last_camera_com_msg_ = -1;

    FILETIME ft;
    // use the captured_time at the local system time, which may not reflect the true capturing time 
	  // because of hyper-spectral imaging raw data file saving and cube image generation on the server side
    // as well as the socket transmission, but we can't do anything about it.
	  GetSystemTimeAsFileTime( &ft );
	
    // ft values are in 100 nanoseconds = 10^-7 seconds, so divide by 10^-4 to get milliseconds.
	  timestamp_of_last_camera_com_msg_ = (ft.dwHighDateTime * 4294967296.0 + ft.dwLowDateTime) / 10000.0; 
	}
	else if(capture_mode_ == "playback") {
    // this is the playback mode
		bool success = frame_tags_load();
		if(!success) {
			vcl_cerr << "bayspec_socket_frame_process::configure, frame_tags_load is not successful." << vcl_endl;
			return false;
		}
	}
	else {
		vcl_cerr << "bayspec_socket_frame_process::configure, capture_mode_ unknown." << vcl_endl;
		return false;
	}
  
  initialized_ = true;
  return initialized_;
}

void bayspec_socket_frame_process::retrieve_last_captured_frame(const vcl_string &file_system_root_folder)
{
  frame_.fill(0);
  three_wavelength_bands_frame_.fill(0);

  three_one_wavelength_band_frames_[0].fill(0);
  three_one_wavelength_band_frames_[1].fill(0);
  three_one_wavelength_band_frames_[2].fill(0);

  vcl_string file_system_root_folder1 = file_system_root_folder + "\\*";

  vcl_cout << "file_system_root_folder1 = " << file_system_root_folder1 << vcl_endl;
  for( vul_file_iterator fn0 = file_system_root_folder1; fn0; ++fn0 ) {
    
    vcl_string level_0_folder(fn0());
    if(level_0_folder.substr(level_0_folder.size()-1) == ".") continue;
    if( vul_file::is_directory( level_0_folder ) ) {      
      // enter the level 0 of the directory path with "YYYY-MM-DD"
      vcl_cout << "level_0_folder = " << level_0_folder << vcl_endl;
      level_0_folder = level_0_folder + "\\*";
      for( vul_file_iterator fn1 = level_0_folder; fn1; ++fn1 ) {

        vcl_string level_1_folder(fn1());
        if(level_1_folder.substr(level_1_folder.size()-1) == ".") continue;        
        if( vul_file::is_directory( level_1_folder ) ) {
          // enter the level 1 of the directory path with "16-54-05RES2048_EXP0030_BIT08"          
          vcl_cout << "level_1_folder = " << level_1_folder << vcl_endl;
          level_1_folder = level_1_folder + "\\*";
          for( vul_file_iterator fn2 = level_1_folder; fn2; ++fn2 ) {
            
            vcl_string level_2_folder(fn2());
            if(level_2_folder.substr(level_2_folder.size()-1) == ".") continue;
            if( vul_file::is_directory( level_2_folder ) && level_2_folder.find("Processed") != level_2_folder.npos ) {
              // we are truly entering the folder where the wavelength bands images are saved.
              
              // load the single wavelength band frame
              vcl_string frame_nr_str = vul_sprintf( "\\band%03d.bmp", wavelength_band_idx_ );
              vcl_string filename = level_2_folder + frame_nr_str;
              frame_ = vil_load(filename.c_str());

              // load three wavelength band images to form the colorized frame
              frame_nr_str = vul_sprintf( "\\band%03d.bmp", three_wavelength_bands_idx_[0] );
              filename = level_2_folder + frame_nr_str;
              three_one_wavelength_band_frames_[0] = vil_load(filename.c_str());

              frame_nr_str = vul_sprintf( "\\band%03d.bmp", three_wavelength_bands_idx_[1] );
              filename = level_2_folder + frame_nr_str;
              three_one_wavelength_band_frames_[1] = vil_load(filename.c_str());

              frame_nr_str = vul_sprintf( "\\band%03d.bmp", three_wavelength_bands_idx_[2] );
              filename = level_2_folder + frame_nr_str;
              three_one_wavelength_band_frames_[2] = vil_load(filename.c_str());

              for(unsigned j = 0; j < three_wavelength_bands_frame_.nj(); j++) {
                for(unsigned i = 0; i < three_wavelength_bands_frame_.ni(); i++) {
                  three_wavelength_bands_frame_(i, j, 0) = three_one_wavelength_band_frames_[0](i, j);
                  three_wavelength_bands_frame_(i, j, 1) = three_one_wavelength_band_frames_[1](i, j);
                  three_wavelength_bands_frame_(i, j, 2) = three_one_wavelength_band_frames_[2](i, j);
                }
              }

              // rotating the captured frame according to the rotation_info_ configuration description
              rotate_captured_frame();
            }
          }
        }
      }

    }
  }
}

void bayspec_socket_frame_process::rotate_captured_frame(void)
{
  vil_image_view<vxl_byte> temp_frame, temp_three_wavelength_bands_frame;
  temp_frame.deep_copy(frame_);
  temp_three_wavelength_bands_frame.deep_copy(three_wavelength_bands_frame_);

  // rotating the captured frame according to the rotation_info_ configuration description
  if(rotation_info_ == "clockwise_90") {
    
    // old (i, j) -> new (i^, j^) = (w-1-j, i)
    // given new (i^, j^) -> it corresponds to the old image location at (j^, w-1-i^)
    for(unsigned j = 0; j < img_height_; j++) {
      for(unsigned i = 0; i < img_width_; i++) {
        three_wavelength_bands_frame_(i, j, 0) = temp_three_wavelength_bands_frame(j, img_width_-1-i, 0);
        three_wavelength_bands_frame_(i, j, 1) = temp_three_wavelength_bands_frame(j, img_width_-1-i, 1);
        three_wavelength_bands_frame_(i, j, 2) = temp_three_wavelength_bands_frame(j, img_width_-1-i, 2);

        frame_(i, j) = temp_frame(j, img_width_-1-i);
      }
    }
  }
  else if(rotation_info_ == "counter_clockwise_90") {
    
    // the opposite rotation as clockwise_90
    for(unsigned j = 0; j < img_height_; j++) {
      for(unsigned i = 0; i < img_width_; i++) {
        three_wavelength_bands_frame_(i, j, 0) = temp_three_wavelength_bands_frame(img_width_-1-j, i, 0);
        three_wavelength_bands_frame_(i, j, 1) = temp_three_wavelength_bands_frame(img_width_-1-j, i, 1);
        three_wavelength_bands_frame_(i, j, 2) = temp_three_wavelength_bands_frame(img_width_-1-j, i, 2);

        frame_(i, j) = temp_frame(img_width_-1-j, i);
      }
    }      
  }
  else if(rotation_info_ == "clockwise_180") {
    
    for(unsigned j = 0; j < img_height_; j++) {
      for(unsigned i = 0; i < img_width_; i++) {
			  
     	  three_wavelength_bands_frame_(i, j, 0) = temp_three_wavelength_bands_frame(img_width_-i-1, img_height_-j-1, 0);
        three_wavelength_bands_frame_(i, j, 1) = temp_three_wavelength_bands_frame(img_width_-i-1, img_height_-j-1, 1);
        three_wavelength_bands_frame_(i, j, 2) = temp_three_wavelength_bands_frame(img_width_-i-1, img_height_-j-1, 2);

        frame_(i, j) = temp_frame(img_width_-i-1, img_height_-j-1);
		  }
	  }
  }
  else if(rotation_info_ == "upside_down") {

    for(unsigned j = 0; j < img_height_; j++) {
      for(unsigned i = 0; i < img_width_; i++) {
        three_wavelength_bands_frame_(i, j, 0) = temp_three_wavelength_bands_frame(i, img_height_-1-j, 0);
        three_wavelength_bands_frame_(i, j, 1) = temp_three_wavelength_bands_frame(i, img_height_-1-j, 1);
        three_wavelength_bands_frame_(i, j, 2) = temp_three_wavelength_bands_frame(i, img_height_-1-j, 2);

        frame_(i, j) = temp_frame(i, img_height_-1-j);
      }
    }
  }
  else if(rotation_info_ == "none") {
    return;
  }
}

bool bayspec_socket_frame_process::step()
{
  if(!initialized_) return false;

  int frame_nr, frame_id;
	double frame_time_code;

  bool result = false;

  if(capture_mode_ == "camera") {
    // live camera capture.
    FILETIME ft;
    // use the captured_time at the local system time, which may not reflect the true capturing time 
	  // because of hyper-spectral imaging raw data file saving and cube image generation on the server side
    // as well as the socket transmission, but we can't do anything about it.
	  GetSystemTimeAsFileTime( &ft );
	
	  // ft values are in 100 nanoseconds = 10^-7 seconds, so divide by 10^-4 to get milliseconds.
	  captured_frame_system_time_ = (ft.dwHighDateTime * 4294967296.0 + ft.dwLowDateTime) / 10000.0;

    if(captured_frame_system_time_ - timestamp_of_last_camera_com_msg_ > camera_com_msg_interval_elapse_in_ms_) {
      frame_nr = frame_nr_;
		  frame_id = frame_nr_;
		  frame_time_code = captured_frame_system_time_;

      vcl_string frame_nr_str = vul_sprintf( "_%04d", frame_nr );
      vcl_string frame_time_code_str = vul_sprintf( "_%6.1f", frame_time_code );

      vcl_string file_system_root_folder = file_storage_directory_ + "\\hyperspectral"
                                            + frame_nr_str + frame_time_code_str;
      vcl_replace(file_system_root_folder.begin(), file_system_root_folder.end(), '/', '\\');

      // make this directory
      vul_file::make_directory_path(file_system_root_folder);
  
      vcl_string com_msg;
      com_msg = "set_file_system_root_folder=" + file_system_root_folder;
      camera_->send_command_message(com_msg);
      gevxl::threading::sleep(1000);

      com_msg = "capture_images=0";
      camera_->send_command_message(com_msg);
      gevxl::threading::sleep(1000);

      // now do something to load the image back for display.
      if(frame_nr_of_last_camera_com_msg_ >= 0) {
        frame_nr_str = vul_sprintf( "_%04d", frame_nr_of_last_camera_com_msg_ );
        frame_time_code_str = vul_sprintf( "_%6.1f", timestamp_of_last_camera_com_msg_ );

        file_system_root_folder = file_storage_directory_ + "\\hyperspectral"
                                              + frame_nr_str + frame_time_code_str;
        vcl_replace(file_system_root_folder.begin(), file_system_root_folder.end(), '/', '\\');
      
        retrieve_last_captured_frame(file_system_root_folder);
      }
    
      frame_nr_of_last_camera_com_msg_ = frame_nr_;
      timestamp_of_last_camera_com_msg_ = captured_frame_system_time_;

      // take care of the frame tag info
      tag_.invalidate();
      tag_.set_frame_nr( frame_nr );
      tag_.set_frame_id( frame_id );
      tag_.set_time_code( frame_time_code );

	    if(save_out_) {
		    
        saved_out_tag_.invalidate();
		    saved_out_tag_.set_frame_nr( saved_out_frame_nr_ );
		    saved_out_tag_.set_frame_id( saved_out_frame_nr_ );
		    saved_out_tag_.set_time_code( frame_time_code );

		    frame_tag_ofstream_ << saved_out_tag_ << vcl_endl;
		    frame_tag_ofstream_.flush();

		    saved_out_frame_nr_++;
	    }

      vcl_cout << "bayspec_socket_frame_process::step(), captured_frame_system_time_ = " << (long long)(frame_time_code) << vcl_endl;

	    frame_nr_++;
    }
    else {
      // the time elapse is too short, we will not poke the camera server at this time, 
      // and simply return the last captured frame
      
    }

    gevxl::threading::sleep(max_frame_waiting_time_in_ms_);
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

    // to be updated of this playback_frames_load() function
		result = playback_frames_load(frame_nr);
		if(!result) {
			vcl_cerr << "micro_epsilon_socket_frame_process::step, playback mode can not load the frames from the file." << vcl_endl;
			return false;
		}
	}

  return true;
}

vil_image_view<vxl_byte> const &bayspec_socket_frame_process::cur_frame() const
{
  // "color", 0
  // "grayscale", 1

  if(output_id_ >= num_outputs_) {
    vcl_cout << "bayspec_socket_frame_process::cur_frame()-- Warning, output_id > number of supported outputs!\n";
  }
  switch (output_id_) {
  case 0:
    {
      return three_wavelength_bands_frame_;
      break;
    }
  case 1:
    {
      return frame_;
      break;
    }
  default:
    {
      return three_wavelength_bands_frame_;
      break;
    }
  }
  return three_wavelength_bands_frame_;
}

vil_image_view<vxl_byte> const &bayspec_socket_frame_process::cur_three_wavelength_bands_frame() const
{
  return three_wavelength_bands_frame_;
}

bool bayspec_socket_frame_process::take_snapshot(const vcl_string &hyperspectral_file_directory)
{
  int frame_nr = 0;

  FILETIME ft;
  // use the captured_time at the local system time, which may not reflect the true capturing time 
	// because of hyper-spectral imaging raw data file saving and cube image generation on the server side
  // as well as the socket transmission, but we can't do anything about it.
	GetSystemTimeAsFileTime( &ft );
	
	// ft values are in 100 nanoseconds = 10^-7 seconds, so divide by 10^-4 to get milliseconds.
	double frame_time_code = (ft.dwHighDateTime * 4294967296.0 + ft.dwLowDateTime) / 10000.0;

  vcl_string frame_nr_str = vul_sprintf( "_%04d", frame_nr );
  vcl_string frame_time_code_str = vul_sprintf( "_%6.1f", frame_time_code );

  vcl_string file_system_root_folder = hyperspectral_file_directory + "\\hyperspectral"
                                      + frame_nr_str + frame_time_code_str;
  vcl_replace(file_system_root_folder.begin(), file_system_root_folder.end(), '/', '\\');
  
  // make this directory
  vul_file::make_directory_path(file_system_root_folder);
    
  vcl_string com_msg;
  com_msg = "set_file_system_root_folder=" + file_system_root_folder;
  camera_->send_command_message(com_msg);
  gevxl::threading::sleep(1000);

  com_msg = "capture_images=0";
  camera_->send_command_message(com_msg);
  gevxl::threading::sleep(5000);

  // now do something to load the image back for display.
  retrieve_last_captured_frame(file_system_root_folder);

  return true;
}

