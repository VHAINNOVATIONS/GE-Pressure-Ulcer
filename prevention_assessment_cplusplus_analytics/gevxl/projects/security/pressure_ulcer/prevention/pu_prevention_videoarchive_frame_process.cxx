// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 02/27/2014
/// \par Modifications:

#include "pu_prevention_videoarchive_frame_process.h"

#include <vul/vul_sprintf.h>
#include <vul/vul_file.h>
#include <vul/vul_file_iterator.h>

#include <vcl_iostream.h>

#include <vil/vil_load.h>
#include <vil/vil_save.h>

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_inverse.h>


using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::prevention;
  
pu_prevention_videoarchive_frame_process::pu_prevention_videoarchive_frame_process(char const *name)
: gevxl::vid::tagged_frame_process<vxl_byte>(name),
	subfolder_id_(0),
  is_xy_frame_pre_computed_(false)
{
  enable(false);
  subfolder_frame_filenames_.clear();
}

pu_prevention_videoarchive_frame_process::~pu_prevention_videoarchive_frame_process(void)
{
	// to be continued
}

bool pu_prevention_videoarchive_frame_process::configure(gevxl::util::config_file &config)
{
  config_ = config;

  bool enabled = false;
	config.get_bool(name()+"::enabled", enabled);
	enable(enabled);

	// depth image sequence 
	depth_img_seq_root_folder_ = "";
	config.get_string(name()+"::depth_img_seq_root_folder", depth_img_seq_root_folder_);

  // depth image sequence filename pattern
	depth_img_seq_filename_pattern_ = "depth_%04d_%04d_%6.1f.png";
	config.get_string(name()+"::depth_img_seq_filename_pattern", depth_img_seq_filename_pattern_);

  // load the camera intrinsic parameters
	is_point_cloud_frame_available_ = false;	
	
	vcl_vector<double> camera_intrinsic_vector;

  if(config_.get_vcl_vector_double(name()+"::camera_intrinsic_vector", camera_intrinsic_vector) == false) {
    // default Kinect depth camera intrinsics
    // rgb intrinsics = [532.69041 0 328.73274 0 530.63846 254.07008 0 0 1]
    // depth intrinsics = [593.08673 0 319.15995 0 591.60550 246.86875 0 0 1]
    camera_intrinsic_vector.resize(9);
    camera_intrinsic_vector[0] = 532.69041;
    camera_intrinsic_vector[1] = 0;
    camera_intrinsic_vector[2] = 328.73274;
    camera_intrinsic_vector[3] = 0;
    camera_intrinsic_vector[4] = 530.63846;
    camera_intrinsic_vector[5] = 254.07008;
    camera_intrinsic_vector[6] = 0;
    camera_intrinsic_vector[7] = 0;
    camera_intrinsic_vector[8] = 1;
  }

	if(camera_intrinsic_vector.size() == 9) {
		
		unsigned idx = 0;
		for(unsigned i = 0; i <= 2; i++) {
      for(unsigned j = 0; j <= 2; j++, idx++) {
        cam_K_(i, j) = camera_intrinsic_vector[idx];
      }
    }
		
		vnl_matrix<double> K22(2, 2);
		for(unsigned i = 0; i <= 1; i++) {
      for(unsigned j = 0; j <= 1; j++, idx++) {
        K22(i, j) = cam_K_(i, j);
      }
    }
		cam_invK22_ = vnl_inverse(K22);

		is_point_cloud_frame_available_ = true;
	}

	return true;
}


bool pu_prevention_videoarchive_frame_process::initialize(void)
{
  if(!is_enabled()) return true;

  return true;
}

void pu_prevention_videoarchive_frame_process::uninitialize(void)
{
  if(!is_enabled()) return;
}

bool pu_prevention_videoarchive_frame_process::step(void)
{
  if(!is_enabled()) return true;

  if(depth_img_seq_root_folder_ == "") return true;

  if(subfolder_frame_filenames_.size() == 0) {
    // the filename list is empty, move to the next subfolder
    vcl_string subfolder = "depth_recording_%04d";
		subfolder = vul_sprintf( subfolder.c_str(), subfolder_id_ );
		subfolder_id_++;

    // check the subfolder
    vcl_string fullpath = depth_img_seq_root_folder_ + "/" + subfolder;
    if(vul_file::exists(fullpath) == false) {
      vcl_cout << "pu_prevention_videoarchive_frame_process::step, arrives the end of subfolder list, directly returns." << vcl_endl;
      return true;
    }

    // check the frame_tags.dat file
    vcl_string frame_tag_filename = fullpath + "/frame_tags.dat";
    if(vul_file::exists(frame_tag_filename) == false) {
      vcl_cout << "pu_prevention_videoarchive_frame_process::step, the current subfolder doesn't have the frame_tags.dat file, directly returns." << vcl_endl;
      return true;
    }
  
    // load back all the frame tags in the subfolder
    vcl_ifstream frame_tag_ifstream(frame_tag_filename.c_str());    
    subfolder_loaded_frame_tags_.clear();
    gevxl::vid::frame_tag tag;
    while(frame_tag_ifstream.is_open() && frame_tag_ifstream >> tag) {
		  subfolder_loaded_frame_tags_.push_back(tag);	
	  }
	
    // fill in the subfolder_frame_filenames_
    subfolder_frame_filenames_.clear();
    subfolder_frame_filenames_.resize(subfolder_loaded_frame_tags_.size());
    for(unsigned i = 0; i < subfolder_loaded_frame_tags_.size(); i++) {
      
      vcl_string filename = vul_sprintf( depth_img_seq_filename_pattern_.c_str(), 
                                          subfolder_loaded_frame_tags_[i].get_frame_nr(), 
                                          subfolder_loaded_frame_tags_[i].get_frame_id(), 
                                          subfolder_loaded_frame_tags_[i].get_time_code() );
      
      subfolder_frame_filenames_[i] = depth_img_seq_root_folder_ + "/" + subfolder + "/" + filename;
    }
  }

  // start to load back individual frame from the subfolder_frame_filenames_ deque for playback.
  if(subfolder_loaded_frame_tags_.size() == 0 || subfolder_frame_filenames_.size() == 0) return true;

  // get the filename
  vcl_string filename = subfolder_frame_filenames_.front();
  subfolder_frame_filenames_.pop_front();

  // load the depth frame
  depth_frame_ = vil_load(filename.c_str());

  // deal with the frame tag
  tag_ = subfolder_loaded_frame_tags_.front();
  subfolder_loaded_frame_tags_.pop_front();

  // now deal with other computable frames
  int img_width = depth_frame_.ni();
  int img_height = depth_frame_.nj();

  if(depth_byte_frame_.ni() != img_width || depth_byte_frame_.nj() != img_height) {
    // initialize the memory space of the computable frames
    depth_byte_frame_ = vil_image_view<vxl_byte>(img_width, img_height, 1, 1);
    rgb_frame_ = vil_image_view<vxl_byte>(img_width, img_height, 1, 3);
    rgb_frame_.fill(0);

    if(is_point_cloud_frame_available_) {
			xyz_rgb_frame_ = vil_image_view<float>(img_width, img_height, 1, 6);
			pre_computed_xy_frame_ = vil_image_view<float>(img_width, img_height, 1, 2);
		}
  }

  // now compute these frames
  vxl_uint_16 *src_depth_ptr = depth_frame_.top_left_ptr();
  vxl_byte *dest_depth_ptr = depth_byte_frame_.top_left_ptr();

  for(unsigned j = 0; j < img_height; j++) {
    for(unsigned i = 0; i < img_width; i++) {
      *dest_depth_ptr = (vxl_byte)((*src_depth_ptr) >> 5);
      src_depth_ptr++;
      dest_depth_ptr++;
    }
  }

	// compute the point cloud frame if the camera intrinsic parameters are available
	if(is_point_cloud_frame_available_) {

		// pre-compute the xy frame
		if(!is_xy_frame_pre_computed_) {

			vnl_matrix<double> x(2, 1);
			vnl_matrix<double> x_temp(2, 1);
			
			float *pre_computed_xy_ptr = pre_computed_xy_frame_.top_left_ptr();
			for(unsigned j = 0; j < img_height; j++) {
				for(unsigned i = 0; i < img_width; i++) {

					x(0, 0) = ((double)i) - cam_K_(0, 2);	// column
					x(1, 0) = ((double)j) - cam_K_(1, 2);	// row

					x_temp = cam_invK22_*x;

					*(pre_computed_xy_ptr + 0) = x_temp(0, 0);
					*(pre_computed_xy_ptr + 1) = x_temp(1, 0);

					pre_computed_xy_ptr = pre_computed_xy_ptr + 2;
				}
			}
		
			is_xy_frame_pre_computed_ = true;
		}

		// depth (16bit) and rgb frames (assume it holds the black image for our case)
		vxl_uint_16 *src_depth_ptr = depth_frame_.top_left_ptr();		

		// point cloud frame		
		float *dest_xyz_rgb_ptr = xyz_rgb_frame_.top_left_ptr();
		float *pre_computed_xy_ptr = pre_computed_xy_frame_.top_left_ptr();
		
		for(unsigned j = 0; j < img_height; j++) {
			for(unsigned i = 0; i < img_width; i++) {
				
				*(dest_xyz_rgb_ptr + 2) = double(*src_depth_ptr)/1000.0;	// convert to meter for z value
        *(dest_xyz_rgb_ptr + 0) = (*(pre_computed_xy_ptr + 0)) * (*(dest_xyz_rgb_ptr + 2));	// x value in meter
				*(dest_xyz_rgb_ptr + 1) = (*(pre_computed_xy_ptr + 1)) * (*(dest_xyz_rgb_ptr + 2));	// y value in meter

				// r,g,b
				*(dest_xyz_rgb_ptr + 3) = 0;
				*(dest_xyz_rgb_ptr + 4) = 0;
				*(dest_xyz_rgb_ptr + 5) = 0;

				src_depth_ptr++;				
				dest_xyz_rgb_ptr = dest_xyz_rgb_ptr + 6;
				pre_computed_xy_ptr = pre_computed_xy_ptr + 2;
			}
		}
	}

	return true;
}

// start playback
bool pu_prevention_videoarchive_frame_process::start_playback(const vcl_string &folder)
{
  depth_img_seq_root_folder_ = folder;
  enable(true);

  return true;
}

// stop playback
bool pu_prevention_videoarchive_frame_process::stop_playback(void)
{
  enable(false);

  return true;
}
