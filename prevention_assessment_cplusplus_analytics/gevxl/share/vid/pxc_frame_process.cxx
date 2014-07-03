// Copyright (C) 2013 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pxc_frame_process.h"
#include <vcl_iostream.h>
#include <util/time/global_time.h>
#include <vid/config.h>
#include <threading/sleep.h>

#if defined(VCL_WIN32) && defined(VCL_VC)
  #include <Windows.h>
#else
  #include <sys/time.h>
  #include <sys/stat.h>
#endif

#include <vil/vil_save.h>
#include <img/image_raw_file_io.h>
#include <util/string.h>

#include <vul/vul_sprintf.h>
#include <vul/vul_file.h>

using namespace gevxl::vid;

pxc_frame_process::pxc_frame_process( char const * n )
    : gevxl::vid::tagged_frame_process<vxl_byte>(n),
			camera_(NULL),
      camera_initialized_(false),      
      verbose_(0),
      asynchronous_grabbing_(true),       
      frame_nr_(0),
			saved_out_frame_nr_(0),
      captured_frame_system_time_(0),
      gui_recording_flag_(false),
      gui_stop_recording_flag_(false),
      gui_playback_flag_(false),
      rgb_file_directory_(""),
      depth_file_directory_(""),
      gui_rgb_frame_tag_ifstream_(NULL),
      gui_depth_frame_tag_ifstream_(NULL)
{

}

pxc_frame_process::~pxc_frame_process()
{
	if(capture_mode_ == "camera") {
		camera_uninitialize();
	}
}

void pxc_frame_process::camera_initialize()
{
  if(camera_initialized_) return;

  camera_ = new pxc_camera(camera_live_, pxc_filename_, img_height_, img_width_, depth_height_, depth_width_, asynchronous_grabbing_);
  
	camera_initialized_ = camera_->is_pxc_initialized();
}

void pxc_frame_process::camera_uninitialize( void )
{
  if(!camera_initialized_) return;

  delete camera_;
	camera_ = NULL;
	
	camera_initialized_ = false;

	set_save_out_flag(false);
}

bool pxc_frame_process::camera_frames_save(const int frame_nr)
{
	vcl_string filename;

	bool success = true;

	// rgb_frame_ 
	filename = filepath_ + "/rgb_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_save(rgb_frame_, filename);
	if(!success) return false;
	
	// depth_frame_
	filename = filepath_ + "/depth_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_save(depth_frame_, filename);
	if(!success) return false;
	
	// xyz_rgb_frame_
	filename = filepath_ + "/xyz_rgb_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_save(xyz_rgb_frame_, filename);
	if(!success) return false;
	
	// depth_to_rgb_coordinate_map_
	filename = filepath_ + "/depth_to_rgb_coordinate_map_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_save(depth_to_rgb_coordinate_map_, filename);
	if(!success) return false;
	
	return true;
}

bool pxc_frame_process::frame_tags_load(void)
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

bool pxc_frame_process::playback_frames_load(const int frame_nr)
{
	vcl_string filename;

	bool success = true;
	
	// rgb_frame_ 
	filename = filepath_ + "/rgb_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_load(rgb_frame_, filename);
	if(!success) return false;
	
	// depth_frame_
	filename = filepath_ + "/depth_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_load(depth_frame_, filename);
	if(!success) return false;

	// xyz_rgb_frame_
	filename = filepath_ + "/xyz_rgb_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_load(xyz_rgb_frame_, filename);
	if(!success) return false;
	
	// depth_to_rgb_coordinate_map_
	filename = filepath_ + "/depth_to_rgb_coordinate_map_" + gevxl::util::to_str(frame_nr) + ".raw";
	success = gevxl::img::image_raw_load(depth_to_rgb_coordinate_map_, filename);
	if(!success) return false;
	
	return true;
}

void pxc_frame_process::set_save_out_flag(bool flag)
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

bool pxc_frame_process::configure( gevxl::util::config_file & config )
{
  config_ = config;

	img_height_ = 720;
  img_width_ = 1280;

	depth_height_ = 240;
  depth_width_ = 320;

	filepath_ = "";
	config_.get_string(name()+"::file_storage_directory", filepath_);
	if(filepath_ == "") {
		vcl_cerr << "pxc_frame_process::configure, file_storage_directory is not set." << vcl_endl;
		return false;
	}

	capture_mode_ = "camera";
	config_.get_string(name()+"::capture_mode", capture_mode_);
	
	if(capture_mode_ == "camera") {
		// only when the capture_mode_ is camera, we start the camera thread
		camera_live_ = true;
		config_.get_bool(name()+"::camera_live", camera_live_);
		
		pxc_filename_ = "";
		config_.get_string(name()+"::pxc_filename", pxc_filename_);

		if(!camera_live_) {
			// the camera is configured not to run live, i.e., playback mode
			if(pxc_filename_ == "") {
				vcl_cerr << "pxc_frame_process::configure, configured as the camera playback mode but the pxc_filename_ is not set." << vcl_endl;
				return false;
			}
		}

		camera_initialize();

		bool flag = false;
		config_.get_bool(name()+"::save_out", flag);
		set_save_out_flag(flag);
	}
	else if(capture_mode_ == "playback") {
		// this is the playback mode
		bool success = frame_tags_load();
		if(!success) {
			vcl_cerr << "pxc_frame_process::configure, frame_tags_load is not successful." << vcl_endl;
			return false;
		}
	}
	else {
		vcl_cerr << "pxc_frame_process::configure, capture_mode_ unknown." << vcl_endl;
		return false;
	}

  asynchronous_grabbing_ = false;
  config_.get_bool(name()+"::asynchronous_grabbing", asynchronous_grabbing_);
  
  verbose_ = 0;
  config_.get_integer(name()+"::verbose", verbose_);

  viz_mode_ = 0;
  config_.get_integer(name()+"::viz_mode", viz_mode_);

  supported_output_type_.insert(vcl_make_pair("rgb", 0));
  supported_output_type_.insert(vcl_make_pair("depth", 1));
  supported_output_type_.insert(vcl_make_pair("rgb_in_depth_view", 2));

  set_num_outputs(supported_output_type_.size());

  vcl_string output_type_;
  config_.get_string(name()+"::output_type", output_type_);
  vcl_cout << output_type_ << vcl_endl;
  set_output_type(output_type_);

  max_frame_waiting_time_in_ms_ = 30; // let's wait for a few milliseconds
	config_.get_integer(name()+"::max_frame_waiting_time_in_ms", max_frame_waiting_time_in_ms_);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// allocate the vxl_image_view memories if they haven't been allocated yet.
  if(depth_frame_.ni() != depth_width_ || depth_frame_.nj() != depth_height_ || depth_frame_.nplanes() != 1) {
    depth_frame_ = vil_image_view<vxl_uint_16>(depth_width_, depth_height_, 1, 1);    
  }
	depth_frame_.fill(0);

  if(rgb_frame_.ni() != img_width_ || rgb_frame_.nj() != img_height_ || rgb_frame_.nplanes() != 3) {
    rgb_frame_ = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 3);        
  }
	rgb_frame_.fill(0);

  if(xyz_rgb_frame_.ni() != depth_width_ || xyz_rgb_frame_.nj() != depth_height_ || xyz_rgb_frame_.nplanes() != 6) {
    xyz_rgb_frame_ = vil_image_view<float>(depth_width_, depth_height_, 1, 6);        
  }
	xyz_rgb_frame_.fill(0.0);

  if(depth_to_rgb_coordinate_map_.ni() != depth_width_ || depth_to_rgb_coordinate_map_.nj() != depth_height_ || depth_to_rgb_coordinate_map_.nplanes() != 2) {
    depth_to_rgb_coordinate_map_ = vil_image_view<int>(depth_width_, depth_height_, 1, 2);    
  }
	depth_to_rgb_coordinate_map_.fill(-1);

	if(depth_byte_frame_.ni() != depth_width_ || depth_byte_frame_.nj() != depth_height_ || depth_byte_frame_.nplanes() != 1) {
		depth_byte_frame_ = vil_image_view<vxl_byte>(depth_width_, depth_height_, 1, 1);
	}
	depth_byte_frame_.fill(0);

	if(rgb_frame_in_depth_view_.ni() != depth_width_ || rgb_frame_in_depth_view_.nj() != depth_height_ || rgb_frame_in_depth_view_.nplanes() != 3) {
		rgb_frame_in_depth_view_ = vil_image_view<vxl_byte>(depth_width_, depth_height_, 1, 3);
	}
	rgb_frame_in_depth_view_.fill(0);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	return true;
}

bool pxc_frame_process::step()
{
	if(capture_mode_ == "camera" && !camera_initialized_) {
    if(verbose_) {
      vcl_cout << "pxc_frame_process::step, capture_mode_ is camera but the camera is not initialized." << vcl_endl;
    }
		return true;
	}

	depth_frame_.fill(0);
	rgb_frame_.fill(0);
	xyz_rgb_frame_.fill(0.0);
	depth_to_rgb_coordinate_map_.fill(-1);
	depth_byte_frame_.fill(0);
	rgb_frame_in_depth_view_.fill(0);

	int frame_nr, frame_id;
	double frame_time_code;

	bool result = false;

	if(capture_mode_ == "camera") {
		// capture frames from the camera, either from live camera or camera specific file.
		if(max_frame_waiting_time_in_ms_ == -1) {
			while(true) {
				// infinite long time of waiting
				result = camera_->get_next_frames(depth_frame_, rgb_frame_, xyz_rgb_frame_, depth_to_rgb_coordinate_map_, captured_frame_system_time_);
				if(result) break;
				gevxl::threading::sleep(1);
			}
		}
		else {
			for(int i = 0; i < max_frame_waiting_time_in_ms_; i++) {
				// block waiting to get the next frame, but only wait for max_frame_waiting_time_in_ms_ milliseconds
				result = camera_->get_next_frames(depth_frame_, rgb_frame_, xyz_rgb_frame_, depth_to_rgb_coordinate_map_, captured_frame_system_time_);
				if(result) break;
				gevxl::threading::sleep(1);
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
			vcl_cerr << "pxc_frame_process::step, playback mode can not load the frames from the file." << vcl_endl;
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
			vcl_cerr << "pxc_frame_process::step, recording_frame error." << vcl_endl;
			return false;
		}
  }
  else if(gui_playback_flag_) {
    result = playback_frame();
    if(!result) {
			vcl_cerr << "pxc_frame_process::step, playback_frame error." << vcl_endl;
			return false;
		}
  }

  if(gui_stop_recording_flag_) {
    // make sure the frame tag files are closed
    if(gui_rgb_frame_tag_ofstream_.is_open()) {
	    gui_rgb_frame_tag_ofstream_.close();
    }

    if(gui_depth_frame_tag_ofstream_.is_open()) {
	    gui_depth_frame_tag_ofstream_.close();
    }

    gui_stop_recording_flag_ = false;
    gui_recording_flag_ = false;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // computable frames from the captured frames
  // copy the rgb to frame_
  frame_.deep_copy(rgb_frame_);
  
  // compress depth image to 8 bit for visualization purpose
  vxl_uint_16 *src_depth_ptr = depth_frame_.top_left_ptr();
  vxl_byte *dest_depth_ptr = depth_byte_frame_.top_left_ptr();

  int depth_width = depth_width_;
  int depth_height = depth_height_;

  float range_min = 0.0f;
  float range_max = 0.5f/13.0f;

  float val = 0.0;
  for(unsigned j = 0; j < depth_width*depth_height; j++) {
    if((*src_depth_ptr) == 0) {
      // saturated and low-confidence depth pixel, set it to the black pixel
      val = 0.0f;
    }
    else {
      val = (float)*src_depth_ptr / 32768;
      val = 1.0f - (val-range_min)/(range_max-range_min);
      if(val < 0.0f) val = 0.0f;
      if(val > 1.0f) val = 1.0f;
    }
    val *= val; // use square 
    *dest_depth_ptr = (unsigned char)(255.f*val);
    src_depth_ptr++;
    dest_depth_ptr++;
  }

	for(int j = 0; j < depth_height; j++) {
    for(int i = 0; i < depth_width; i++) {
			rgb_frame_in_depth_view_(i,j,0) = (vxl_byte)( xyz_rgb_frame_(i,j,3) );
      rgb_frame_in_depth_view_(i,j,1) = (vxl_byte)( xyz_rgb_frame_(i,j,4) );
      rgb_frame_in_depth_view_(i,j,2) = (vxl_byte)( xyz_rgb_frame_(i,j,5) );
		}
	}

	if(capture_mode_ == "camera" && save_out_) {
		result = camera_frames_save(saved_out_frame_nr_);
		if(!result) {
			vcl_cerr << "pxc_frame_process::step, camera frames save out error." << vcl_endl;
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
		vcl_cout << "pxc_frame_process::step(), captured_frame_system_time_ = " << (long long)frame_time_code << vcl_endl;
	}

	frame_nr_++;

	return true;
}

vil_image_view<vxl_byte> const &pxc_frame_process::cur_frame() const
{
  if(output_id_ >= num_outputs_) {
    vcl_cout << "pxc_frame_process::cur_frame()-- Warning, output_id > number of supported outputs!\n";
  }
  switch (output_id_) {
  case 0:
    {
      return frame_;
      break;
    }
  case 1:
    {
      return depth_byte_frame_;
      break;
    }
  case 2:
    {
      return rgb_frame_in_depth_view_;
      break;
    }
  default:
    {
      return frame_;
      break;
    }
  }
}

vil_image_view<vxl_byte> const &pxc_frame_process::cur_rgb_frame() const
{
  return rgb_frame_;
}

vil_image_view<vxl_uint_16> const &pxc_frame_process::cur_depth_frame() const
{
  return depth_frame_;
}

vil_image_view<vxl_byte> const &pxc_frame_process::cur_depth_byte_frame() const
{
  return depth_byte_frame_;
}

vil_image_view<float> const &pxc_frame_process::cur_xyz_rgb_frame() const
{
  return xyz_rgb_frame_;
}

vil_image_view<vxl_byte> const &pxc_frame_process::cur_rgb_frame_in_depth_view() const
{
  return rgb_frame_in_depth_view_;
}

vil_image_view<int> const &pxc_frame_process::cur_depth_to_rgb_coordinate_map() const
{
	return depth_to_rgb_coordinate_map_;
}

bool pxc_frame_process::take_snapshot(const vcl_string &rgb_file_directory, const vcl_string &depth_file_directory, 
                                      const vcl_string &viewpoint_description)
{
  vcl_string filename;

	bool success = true;

  vcl_string frame_nr_str = vul_sprintf( "_%04d", tag_.get_frame_nr() );
  vcl_string frame_time_code_str = vul_sprintf( "_%6.1f", tag_.get_time_code() );

  vcl_string frame_info_str = frame_nr_str + frame_time_code_str 
                              + "_" + viewpoint_description + ".raw";

  // rgb_frame_ 
	filename = rgb_file_directory + "/rgb" + frame_info_str;
	success = gevxl::img::image_raw_save(rgb_frame_, filename);
	if(!success) return false;
	
  // rgb_frame_ save as the png image
  filename = rgb_file_directory + "/rgb" + frame_nr_str + frame_time_code_str + "_" + viewpoint_description + ".png";
  vil_save(rgb_frame_, filename.c_str());

	// depth_frame_
	filename = depth_file_directory + "/depth" + frame_info_str;
	success = gevxl::img::image_raw_save(depth_frame_, filename);
	if(!success) return false;

  // depth_frame_ save as the png image
  filename = depth_file_directory + "/depth" + frame_nr_str + frame_time_code_str + "_" + viewpoint_description + ".png";
  vil_save(depth_frame_, filename.c_str());
	
	// xyz_rgb_frame_
	filename = depth_file_directory + "/xyz_rgb" + frame_info_str;
	success = gevxl::img::image_raw_save(xyz_rgb_frame_, filename);
	if(!success) return false;
	
	// depth_to_rgb_coordinate_map_
	filename = depth_file_directory + "/depth_to_rgb_coordinate_map" + frame_info_str;
	success = gevxl::img::image_raw_save(depth_to_rgb_coordinate_map_, filename);
	if(!success) return false;
	
	return true; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// the new implementation to support the GUI controlled recording and playback
/// Control command to start/stop/playback recording
bool pxc_frame_process::start_recording(const vcl_string &rgb_file_directory, const vcl_string &depth_file_directory, 
                                        const vcl_string &viewpoint_description)
{
  // ensure the folders are created.
  rgb_file_directory_ = rgb_file_directory + "/" + viewpoint_description;
  vul_file::make_directory_path(rgb_file_directory_);

  depth_file_directory_ = depth_file_directory + "/" + viewpoint_description;
  vul_file::make_directory_path(depth_file_directory_);

  // make sure the frame tag files are openned for recording
  vcl_string filename;
  
  filename = rgb_file_directory_ + "/frame_tags.dat";
  if(gui_rgb_frame_tag_ofstream_.is_open()) {
    gui_rgb_frame_tag_ofstream_.close();
  }
	gui_rgb_frame_tag_ofstream_.open(filename.c_str());

  filename = depth_file_directory_ + "/frame_tags.dat";
  if(gui_depth_frame_tag_ofstream_.is_open()) {
    gui_depth_frame_tag_ofstream_.close();
  }
	gui_depth_frame_tag_ofstream_.open(filename.c_str());

  gui_recording_flag_ = true;
  gui_playback_flag_ = false;

  return true;
}

bool pxc_frame_process::stop_recording(void)
{
  gui_stop_recording_flag_ = true;

  return true;
}

bool pxc_frame_process::playback_recording(const vcl_string &rgb_file_directory, const vcl_string &depth_file_directory, 
                                           const vcl_string &viewpoint_description)
{
  rgb_file_directory_ = rgb_file_directory + "/" + viewpoint_description;
  depth_file_directory_ = depth_file_directory + "/" + viewpoint_description;

  // make sure the frame tag files are openned for playback
  vcl_string filename;
  
  filename = rgb_file_directory_ + "/frame_tags.dat";
  gui_rgb_frame_tag_ifstream_ = new vcl_ifstream(filename.c_str());	

  filename = depth_file_directory_ + "/frame_tags.dat";
  gui_depth_frame_tag_ifstream_ = new vcl_ifstream(filename.c_str());	

  gui_playback_flag_ = true;
  gui_recording_flag_ = false;

  return true;
}

bool pxc_frame_process::recording_frame(void)
{
  vcl_string filename;

	bool success = true;

  vcl_string frame_nr_str = vul_sprintf( "_%04d", tag_.get_frame_nr() );
  vcl_string frame_time_code_str = vul_sprintf( "_%6.1f", tag_.get_time_code() );

  vcl_string frame_info_str = frame_nr_str + frame_time_code_str + ".raw";

  // rgb_frame_ 
	filename = rgb_file_directory_ + "/rgb" + frame_info_str;
	success = gevxl::img::image_raw_save(rgb_frame_, filename);
	if(!success) return false;

  // rgb_frame_ save as the png image
  filename = rgb_file_directory_ + "/rgb" + frame_nr_str + frame_time_code_str + ".png";
  vil_save(rgb_frame_, filename.c_str());
	
	// depth_frame_
	filename = depth_file_directory_ + "/depth" + frame_info_str;
	success = gevxl::img::image_raw_save(depth_frame_, filename);
	if(!success) return false;
	
  // depth_frame_ save as the png image
  filename = depth_file_directory_ + "/depth" + frame_nr_str + frame_time_code_str + ".png";
  vil_save(depth_frame_, filename.c_str());

	// xyz_rgb_frame_
	filename = depth_file_directory_ + "/xyz_rgb" + frame_info_str;
	success = gevxl::img::image_raw_save(xyz_rgb_frame_, filename);
	if(!success) return false;
	
	// depth_to_rgb_coordinate_map_
	filename = depth_file_directory_ + "/depth_to_rgb_coordinate_map" + frame_info_str;
	success = gevxl::img::image_raw_save(depth_to_rgb_coordinate_map_, filename);
	if(!success) return false;
	
  // save out the frame tag
  gui_rgb_frame_tag_ofstream_ << tag_ << vcl_endl;
	gui_rgb_frame_tag_ofstream_.flush();

  gui_depth_frame_tag_ofstream_ << tag_ << vcl_endl;
	gui_depth_frame_tag_ofstream_.flush();

	return true; 
}

bool pxc_frame_process::playback_frame(void)
{
  // the behavior is that once all the recorded frames have been played back, 
  // the process automatically switches back to the live camera capture mode.
  
  bool success = true;

  vcl_string filename;
  vcl_string frame_nr_str, frame_time_code_str, frame_info_str;

  bool playback_end_flag = false;

  // load back one frame tag
  gevxl::vid::frame_tag rgb_tag;
	if(gui_rgb_frame_tag_ifstream_ != NULL &&
      gui_rgb_frame_tag_ifstream_->is_open() && 
      //gui_rgb_frame_tag_ifstream_->good() && 
      *gui_rgb_frame_tag_ifstream_ >> rgb_tag) {

		// load one rgb_tag;
    frame_nr_str = vul_sprintf( "_%04d", rgb_tag.get_frame_nr() );
    frame_time_code_str = vul_sprintf( "_%6.1f", rgb_tag.get_time_code() );

    frame_info_str = frame_nr_str + frame_time_code_str + ".raw";
    
    // rgb_frame_
    filename = rgb_file_directory_ + "/rgb" + frame_info_str;
	  success = gevxl::img::image_raw_load(rgb_frame_, filename);
	  if(!success) return false;
	}
  else {
    playback_end_flag = true;
  }

  gevxl::vid::frame_tag depth_tag;
  if(gui_depth_frame_tag_ifstream_ != NULL && 
      gui_depth_frame_tag_ifstream_->is_open() && 
      //gui_depth_frame_tag_ifstream_->good() && 
      *gui_depth_frame_tag_ifstream_ >> depth_tag) {

		// load one depth_tag;
    frame_nr_str = vul_sprintf( "_%04d", depth_tag.get_frame_nr() );
    frame_time_code_str = vul_sprintf( "_%6.1f", depth_tag.get_time_code() );

    frame_info_str = frame_nr_str + frame_time_code_str + ".raw";
    
    // depth_frame_
	  filename = depth_file_directory_ + "/depth" + frame_info_str;
	  success = gevxl::img::image_raw_load(depth_frame_, filename);
	  if(!success) return false;
	
	  // xyz_rgb_frame_
	  filename = depth_file_directory_ + "/xyz_rgb" + frame_info_str;
	  success = gevxl::img::image_raw_load(xyz_rgb_frame_, filename);
	  if(!success) return false;
	
	  // depth_to_rgb_coordinate_map_
	  filename = depth_file_directory_ + "/depth_to_rgb_coordinate_map" + frame_info_str;
	  success = gevxl::img::image_raw_load(depth_to_rgb_coordinate_map_, filename);
	  if(!success) return false;
	}
  else {
    playback_end_flag = true;
  }

  if(playback_end_flag) {
    
    if(gui_rgb_frame_tag_ifstream_ != NULL &&
        gui_rgb_frame_tag_ifstream_->is_open()) {
      
      gui_rgb_frame_tag_ifstream_->close();
      delete gui_rgb_frame_tag_ifstream_;
      gui_rgb_frame_tag_ifstream_ = NULL;
    }
    
    if(gui_depth_frame_tag_ifstream_ != NULL && 
        gui_depth_frame_tag_ifstream_->is_open()) {

      gui_depth_frame_tag_ifstream_->close();
      delete gui_depth_frame_tag_ifstream_;
      gui_depth_frame_tag_ifstream_ = NULL;
    }
    
    gui_playback_flag_ = false;
  }

  return true;
}
