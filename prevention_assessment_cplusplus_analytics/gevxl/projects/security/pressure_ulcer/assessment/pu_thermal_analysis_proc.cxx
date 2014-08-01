// Copyright (C) 2013 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ming-Ching Chang and Ting Yu
/// \date 11/07/2013
/// \par Modifications:

#include "pu_thermal_analysis_proc.h"

#include <vcl_cstdio.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vcl_algorithm.h>

#include <vil/vil_save.h>
#include <vil/vil_convert.h>

#include <vid/micro_epsilon_socket_frame_process.h>
#include <vid/pxc_frame_process.h>

#include <shape/rectangle.h>

#include <util/rectangle.h>
#include <util/string.h>

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::assessment;
  
pu_thermal_analysis_proc::pu_thermal_analysis_proc(char const *name)
: gevxl::framework::process(name), 	
  source_proc_(NULL), 
  viz_(NULL),
  frame_nr_(0)
{
	roi_rect_handle_set_ = gevxl::shape::rectangle_handle_set_sptr(NULL);

	median_temperature_estimate_in_roi_ = 30.0;
	max_temperature_in_roi_ = 30.0;
	min_temperature_in_roi_ = 30.0;

	// parameters to be configured for the thermal analysis
	skin_temperature_lower_range_in_C_ = 30.0;
	skin_temperature_higher_range_in_C_ = 45.0;

	delta_temperature_in_heat_map_ = 0.5;

	viz_mode_selection_ = 0;
	viz_modes_.push_back("raw_temperature_frame");
	viz_modes_.push_back("skin_temperature_range_frame");
	viz_modes_.push_back("skin_temperature_layer_segmentation_frame");

	viz_hot_spot_flag_ = false;

	thermal_frames_in_depth_view_ = false;

	// other old parameters
	b_ratio_ = 0.10f;
  pixel_th_low_ = 25;
  pixel_th_high_ = 230;
  frame_pixel_max_ = 255;
  frame_pixel_min_ = 0;
	highres_timer_.reset();
}

pu_thermal_analysis_proc::~pu_thermal_analysis_proc(void)
{
	/*if(viz_video_out_openned_) {
		viz_video_out_.close();
		viz_video_out_openned_ = false;
	}*/
}

bool pu_thermal_analysis_proc::configure(util::config_file &config)
{
	config_ = config;

	bool enabled = false;
	config_.get_bool(name()+"::enabled", enabled);
	enable(enabled);

  // parameter configuration
	skin_temperature_lower_range_in_C_ = 30.0;
	config_.get_float(name()+"::skin_temperature_lower_range_in_C", skin_temperature_lower_range_in_C_);

	skin_temperature_higher_range_in_C_ = 45.0;
	config_.get_float(name()+"::skin_temperature_higher_range_in_C", skin_temperature_higher_range_in_C_);

	delta_temperature_in_heat_map_ = 0.5;
	config_.get_float(name()+"::delta_temperature_in_heat_map", delta_temperature_in_heat_map_);

	hot_spot_temperature_diff_thresh_ = 3.0;
	config_.get_float(name()+"::hot_spot_temperature_diff_thresh", hot_spot_temperature_diff_thresh_);

	// The thermal camera's intrinsic, rotation and translation knowledge
	// intrinsic parameters
	vcl_vector<double> thermal_camera_intrinsic_vector;
	config_.get_vcl_vector_double(name()+"::thermal_camera_intrinsic_vector", thermal_camera_intrinsic_vector);
	if(thermal_camera_intrinsic_vector.size() == 9) {
		
		unsigned idx = 0;
		for(unsigned i = 0; i <= 2; i++) {
      for(unsigned j = 0; j <= 2; j++, idx++) {
        cam_K_(i, j) = thermal_camera_intrinsic_vector[idx];
      }
    }
	}
	else {
		vcl_cerr << "pu_thermal_analysis_proc::configure, Error thermal camera's intrinsic parameters are not set." << vcl_endl;
    return false;
	}

	// rotation parameters
	vcl_vector<double> thermal_camera_rotation_vector;
	config_.get_vcl_vector_double(name()+"::thermal_camera_rotation_vector", thermal_camera_rotation_vector);
	if(thermal_camera_rotation_vector.size() == 9) {
		
		unsigned idx = 0;
		for(unsigned i = 0; i <= 2; i++) {
      for(unsigned j = 0; j <= 2; j++, idx++) {
        cam_rotation_(i, j) = thermal_camera_rotation_vector[idx];
				cam_extrinsic_mat_(i, j) = thermal_camera_rotation_vector[idx];
      }
    }
	}
	else {
		vcl_cerr << "pu_thermal_analysis_proc::configure, Error thermal camera's rotation parameters are not set." << vcl_endl;
    return false;
	}
		
	// translation parameters
	vcl_vector<double> thermal_camera_translation_vector;
	config_.get_vcl_vector_double(name()+"::thermal_camera_translation_vector", thermal_camera_translation_vector);
	if(thermal_camera_translation_vector.size() == 3) {
		
		for(unsigned i = 0; i <= 2; i++) {
      cam_translation_(i, 0) = thermal_camera_translation_vector[i];
			cam_extrinsic_mat_(i, 3) = thermal_camera_translation_vector[i];		
    }
	}
	else {
		vcl_cerr << "pu_thermal_analysis_proc::configure, Error thermal camera's translation parameters are not set." << vcl_endl;
    return false;
	}

	// compute the camera projection matrix
	cam_proj_mat_ = cam_K_ * cam_extrinsic_mat_;

	roi_rect_to_frame_border_x_margin_ = 50;
	config_.get_integer(name()+"::roi_rect_to_frame_border_x_margin", roi_rect_to_frame_border_x_margin_);

	roi_rect_to_frame_border_y_margin_ = 50;
	config_.get_integer(name()+"::roi_rect_to_frame_border_y_margin", roi_rect_to_frame_border_y_margin_);

	num_of_deviations_as_outlier_in_roi_ = 3;
	config_.get_integer(name()+"::num_of_deviations_as_outlier_in_roi", num_of_deviations_as_outlier_in_roi_);

	// other parameters
  config_.get_float (name()+"::b_ratio", b_ratio_);
  vcl_printf ("%s::b_ratio = %f\n", name().c_str(), b_ratio_);
	
	return true;
}


bool pu_thermal_analysis_proc::initialize(void)
{
	if(is_enabled() == false) return true;

 	return true;
}

void pu_thermal_analysis_proc::uninitialize(void)
{
	if(is_enabled() == false) return;
}

void pu_thermal_analysis_proc::set_point_cloud_xyzrgb_frame(const vil_image_view<float> &xyz_rgb_frame)
{
	xyz_rgb_frame_.deep_copy(xyz_rgb_frame);
}

bool pu_thermal_analysis_proc::step(void)
{
	if(is_enabled() == false) return true;

  const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(source_proc_->get_frame_process());

	// extract the current thermal byte frame
  const vil_image_view<vxl_byte> &thermal_byte_img = thermal_source->cur_frame();
	curr_frame_.deep_copy(thermal_byte_img);

	// allocate frame memory if there are not yet allocated.

	// thermal_byte_frame_in_depth_view_
	if( thermal_byte_frame_in_depth_view_.ni() != xyz_rgb_frame_.ni() || thermal_byte_frame_in_depth_view_.nj() != xyz_rgb_frame_.nj() ) {
		thermal_byte_frame_in_depth_view_ = vil_image_view<vxl_byte>(xyz_rgb_frame_.ni(), xyz_rgb_frame_.nj(), 1, 3);		
	}
	thermal_byte_frame_in_depth_view_.fill(0);

	// thermal_frame_in_C_in_depth_view_
	if( thermal_frame_in_C_in_depth_view_.ni() != xyz_rgb_frame_.ni() || thermal_frame_in_C_in_depth_view_.nj() != xyz_rgb_frame_.nj() ) {
		thermal_frame_in_C_in_depth_view_ = vil_image_view<float>(xyz_rgb_frame_.ni(), xyz_rgb_frame_.nj(), 1, 1);		
	}
	thermal_frame_in_C_in_depth_view_.fill(0);

	// thermal_valid_pixel_mask_in_depth_view_
	if( thermal_valid_pixel_mask_in_depth_view_.ni() != xyz_rgb_frame_.ni() || thermal_valid_pixel_mask_in_depth_view_.nj() != xyz_rgb_frame_.nj() ) {
		thermal_valid_pixel_mask_in_depth_view_ = vil_image_view<vxl_byte>(xyz_rgb_frame_.ni(), xyz_rgb_frame_.nj(), 1, 1);		
	}
	thermal_valid_pixel_mask_in_depth_view_.fill(255);

	// viz_skin_temperature_range_frame_
	if( viz_skin_temperature_range_frame_.ni() != thermal_byte_img.ni() || viz_skin_temperature_range_frame_.nj() != thermal_byte_img.nj() ) {
		viz_skin_temperature_range_frame_ = vil_image_view<vxl_byte>(thermal_byte_img.ni(), thermal_byte_img.nj(), 1, 3);		
	}
	viz_skin_temperature_range_frame_.fill(0);
	
	// viz_skin_temperature_layer_segmentation_frame_
	if( viz_skin_temperature_layer_segmentation_frame_.ni() != thermal_byte_img.ni() || viz_skin_temperature_layer_segmentation_frame_.nj() != thermal_byte_img.nj() ) {
		viz_skin_temperature_layer_segmentation_frame_ = vil_image_view<vxl_byte>(thermal_byte_img.ni(), thermal_byte_img.nj(), 1, 3);		
	}
	viz_skin_temperature_layer_segmentation_frame_.fill(0);

	// extract the color palette
	color_palette_.clear();
	color_palette_ = thermal_source->get_color_palette();

	// texture mapping to the depth view
	//thermal_texture_mapping_to_depth_view();
	
	if(!roi_rect_handle_set_) {
		roi_rect_.x0 = roi_rect_to_frame_border_x_margin_;
		roi_rect_.y0 = roi_rect_to_frame_border_y_margin_;
		roi_rect_.x1 = thermal_byte_img.ni() - roi_rect_to_frame_border_x_margin_;
		roi_rect_.y1 = thermal_byte_img.nj() - roi_rect_to_frame_border_y_margin_;
	}

	// perform the thermal reading scaling based on a pre-defined skin temperature range, anything that is outside of the temperature range should be set to zero.
	skin_temperature_range_scaling();

	// estimate the median, min and max temperature in the ROI
	perform_roi_thermal_analysis();
  
	frame_nr_++;

	return step(curr_frame_);	
}

bool pu_thermal_analysis_proc::step(const vil_image_view<vxl_byte> &thermal_byte_img)
{
	unsigned int i, j;
  const unsigned int ni = thermal_byte_img.ni();
  const unsigned int nj = thermal_byte_img.nj();

  //compute frame_pixel_max_ and frame_pixel_min_
  frame_pixel_max_ = 0;
  frame_pixel_min_ = 255;
  for (j=0; j<nj; j++) {
    for (i=0; i<ni; i++) {
      if (thermal_byte_img(i,j) < frame_pixel_min_)
        frame_pixel_min_ = thermal_byte_img(i,j);
      if (thermal_byte_img(i,j) > frame_pixel_max_)
        frame_pixel_max_ = thermal_byte_img(i,j);
    }
  }

  //compute pixel_th_low_ and pixel_th_high_
  pixel_th_high_ = (vxl_byte) ((1.0-b_ratio_)*frame_pixel_max_ + b_ratio_*frame_pixel_min_);
  pixel_th_low_ = (vxl_byte) ((1.0-b_ratio_)*frame_pixel_min_ + b_ratio_*frame_pixel_max_);

  //Visualize to viz_im_
  viz_im_ = vil_image_view<vxl_byte> (ni, nj, 1, 3);

  for (j=0; j<nj; j++) {
    for (i=0; i<ni; i++) {
      if (thermal_byte_img(i,j) > pixel_th_high_) {
        //Show the top n% brightest pixels in red 
        viz_im_(i,j,0) = 255;
        viz_im_(i,j,1) = 0;
        viz_im_(i,j,2) = 0;
      }        
      else if (thermal_byte_img(i,j) < pixel_th_low_) {
        //Show the top n% darkest pixels in red 
        viz_im_(i,j,0) = 0;
        viz_im_(i,j,1) = 0;
        viz_im_(i,j,2) = 255;
      }
      else {        
        //Pixel value from grayvalue of thermal_byte_img
        viz_im_(i,j,0) = thermal_byte_img(i,j);
        viz_im_(i,j,1) = thermal_byte_img(i,j);
        viz_im_(i,j,2) = thermal_byte_img(i,j);
      }        
    }
  }   

  //visualize();

  return true;
}

void pu_thermal_analysis_proc::set_visualizer(gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;
}


void pu_thermal_analysis_proc::set_viz_offset(int viz_offset_i, int viz_offset_j)
{
	// visualizer offset
	viz_offset_i_ = viz_offset_i;
	viz_offset_j_ = viz_offset_j;
}

void pu_thermal_analysis_proc::visualize(void)
{	
	if(is_enabled() == false) return;
}

void pu_thermal_analysis_proc::thermal_texture_mapping_to_depth_view(void)
{
	const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(source_proc_->get_frame_process());

	const vil_image_view<vxl_byte> &thermal_byte_img = thermal_source->cur_frame();
	const vil_image_view<float> &thermal_frame_in_C = thermal_source->cur_thermal_frame_in_C();

	int i = 0, j = 0;
	int depth_view_width = xyz_rgb_frame_.ni();
	int depth_view_height = xyz_rgb_frame_.nj();
	
	vnl_matrix_fixed<double, 4, 1> world_homo_coordinate;
	vnl_matrix_fixed<double, 3, 1> image_homo_coordinate;
	world_homo_coordinate(3, 0) = 1;
	
	int thermal_i, thermal_j;
	int thermal_width = thermal_byte_img.ni();
	int thermal_height = thermal_byte_img.nj();

	thermal_valid_pixel_mask_in_depth_view_.fill(255);
	float min_temp_in_C = 1e+10;

	for(j = 0; j < depth_view_height; j++) {
		for(i = 0; i < depth_view_width; i++) {
			world_homo_coordinate(0, 0) = xyz_rgb_frame_(i, j, 0);
			world_homo_coordinate(1, 0) = xyz_rgb_frame_(i, j, 1);
			world_homo_coordinate(2, 0) = xyz_rgb_frame_(i, j, 2);

			if( world_homo_coordinate(0, 0) == 0.0 && 
				  world_homo_coordinate(1, 0) == 0.0 && 
					world_homo_coordinate(2, 0) == 0.0 ) {
				// an unstable depth pixel, so set the thermal valid pixel mask at this point to be 0
				thermal_valid_pixel_mask_in_depth_view_(i, j) = 0;
				continue;
			}

			world_homo_coordinate(0, 0) = world_homo_coordinate(0, 0)*100;
			world_homo_coordinate(1, 0) = 0 - world_homo_coordinate(1, 0)*100;
			world_homo_coordinate(2, 0) = world_homo_coordinate(2, 0)*100;

			image_homo_coordinate = cam_proj_mat_ * world_homo_coordinate;

			thermal_i = (int)(image_homo_coordinate(0, 0) / image_homo_coordinate(2, 0) + 0.5);
			thermal_j = (int)(image_homo_coordinate(1, 0) / image_homo_coordinate(2, 0) + 0.5);

			if( thermal_i < 0 || thermal_i >= thermal_width ||
				  thermal_j < 0 || thermal_j >= thermal_height ) {
	
				// the mapped pixel is outside of thermal frame size, need to figure out what value to fill in
				thermal_valid_pixel_mask_in_depth_view_(i, j) = 0;
				continue;
			}

			// the thermal texture map at the thermal frame's location thermal_i and thermal_j does have value
			thermal_byte_frame_in_depth_view_(i, j, 0) = thermal_byte_img(thermal_i, thermal_j, 0);
			thermal_byte_frame_in_depth_view_(i, j, 1) = thermal_byte_img(thermal_i, thermal_j, 1);
			thermal_byte_frame_in_depth_view_(i, j, 2) = thermal_byte_img(thermal_i, thermal_j, 2);

			thermal_frame_in_C_in_depth_view_(i, j, 0) = thermal_frame_in_C(thermal_i, thermal_j, 0);

			if( min_temp_in_C > thermal_frame_in_C(thermal_i, thermal_j, 0) ) {
				min_temp_in_C = thermal_frame_in_C(thermal_i, thermal_j, 0);
			}
		}
	}

	for(j = 0; j < depth_view_height; j++) {
		for(i = 0; i < depth_view_width; i++) {
			
			if(thermal_valid_pixel_mask_in_depth_view_(i, j) == 0) {
				thermal_byte_frame_in_depth_view_(i, j, 0) = 0;
				thermal_byte_frame_in_depth_view_(i, j, 1) = 0;
				thermal_byte_frame_in_depth_view_(i, j, 2) = 0;

				thermal_frame_in_C_in_depth_view_(i, j, 0) = min_temp_in_C;
			}
		}
	}

	thermal_frames_in_depth_view_ = true;
}

void pu_thermal_analysis_proc::skin_temperature_range_scaling(void)
{	
	const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(source_proc_->get_frame_process());

	const vil_image_view<vxl_byte> &thermal_byte_img = thermal_source->cur_frame();
	
	// get the thermal frame in C
	const vil_image_view<float> &thermal_frame_in_C = thermal_source->cur_thermal_frame_in_C();

	const float *ptr_in_C = thermal_frame_in_C.top_left_ptr();
	int frame_size = thermal_frame_in_C.ni()*thermal_frame_in_C.nj();
	
	float min_val = 1e+10, max_val = -(1e+10);
	
	for(int i = 0; i < frame_size; i++, ptr_in_C++) {
		if(*ptr_in_C < skin_temperature_lower_range_in_C_ || *ptr_in_C >= skin_temperature_higher_range_in_C_) {
			continue;
		}

		if(*ptr_in_C > max_val) max_val = *ptr_in_C;
		if(*ptr_in_C < min_val) min_val = *ptr_in_C;
	}
	
	viz_skin_temperature_range_frame_.fill(0);

	double fact = 0.0;
	if(max_val == 0 && min_val == 0) {
		viz_skin_temperature_range_frame_.fill(0);
	}
	else {
		fact = 255.0/(max_val - min_val);

		ptr_in_C = thermal_frame_in_C.top_left_ptr();
		vxl_byte *color_ptr = viz_skin_temperature_range_frame_.top_left_ptr();

		int scaled_pix_val = 0;
		for(int i = 0; i < frame_size; i++, ptr_in_C++, color_ptr=color_ptr+3) {
			
			if(*ptr_in_C < skin_temperature_lower_range_in_C_ || *ptr_in_C >= skin_temperature_higher_range_in_C_) {
				continue;
			}

			// rescale the value reading to [0, 255]
      scaled_pix_val = vcl_min( vcl_max((int)(fact*((*ptr_in_C) - min_val)), 0), 255 );

			*color_ptr = color_palette_[scaled_pix_val][0];
      *(color_ptr+1) = color_palette_[scaled_pix_val][1];
      *(color_ptr+2) = color_palette_[scaled_pix_val][2];
		}
	}
}

// perform the roi region thermal analysis
void pu_thermal_analysis_proc::perform_roi_thermal_analysis(void)
{
	// get the thermal frame in C
	const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(source_proc_->get_frame_process());

	const vil_image_view<float> &thermal_frame_in_C = thermal_source->cur_thermal_frame_in_C();

	hot_spot_pixels_coordinate_.clear();
	cold_spot_pixels_coordinate_.clear();

	if(roi_rect_handle_set_) {
		util::rectangle<double> rect_d = roi_rect_handle_set_->get_rectangle()->get_rectangle();
	
		// remove the viz_canvas_frame's offset 
		roi_rect_.x0 = (int)(rect_d.x0 - viz_offset_i_ + 0.5);
		roi_rect_.y0 = (int)(rect_d.y0 - viz_offset_j_ + 0.5);
		roi_rect_.x1 = (int)(rect_d.x1 - viz_offset_i_ + 0.5);
		roi_rect_.y1 = (int)(rect_d.y1 - viz_offset_j_ + 0.5);
	}
	
	int roi_size = (roi_rect_.y1 - roi_rect_.y0 + 1)*(roi_rect_.x1 - roi_rect_.x0 + 1);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// obtain the vector of temperature reading from the pre-defined ROI
	median_temperature_estimate_in_roi_ = 30.0;
	max_temperature_in_roi_ = -(1e+10);
	min_temperature_in_roi_ = 1e+10;
	
	// compute the median temperature estimate from the pre-defined ROI
	vcl_vector<float> sorted_temperature;
	sorted_temperature.reserve(roi_size);

	for(unsigned j = roi_rect_.y0; j < roi_rect_.y1; j++) {
		for(unsigned i = roi_rect_.x0; i < roi_rect_.x1; i++) {
			
			sorted_temperature.push_back(thermal_frame_in_C(i, j));

			if(thermal_frame_in_C(i, j) > max_temperature_in_roi_) max_temperature_in_roi_ = thermal_frame_in_C(i, j);
			if(thermal_frame_in_C(i, j) < min_temperature_in_roi_) min_temperature_in_roi_ = thermal_frame_in_C(i, j);
		}
	}

	if(sorted_temperature.size() > 0) {
		// sorting the temperature
		vcl_sort(sorted_temperature.begin(), sorted_temperature.end());

		// obtain the median temperature value	
		int size = sorted_temperature.size();
		if(size % 2 == 0) {
			// even number of elements
			median_temperature_estimate_in_roi_ = (sorted_temperature[(size/2)-1] + sorted_temperature[size/2])/2.0;
		}
		else {
			// odd number of elements
			median_temperature_estimate_in_roi_ = sorted_temperature[(size-1)/2];
		}
	}
	else {
		median_temperature_estimate_in_roi_ = 30.0;
		max_temperature_in_roi_ = 30.0;
		min_temperature_in_roi_ = 30.0;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// compute the median absolute deviation (MAD), 
	// a robust measure of the variability of a univariate sample of quantitative data.
	vcl_vector<float> sorted_diff_temperature_2_median;
	sorted_diff_temperature_2_median.reserve(roi_size);
	float diff_temperature_2_median = 0.0;

	for(unsigned j = roi_rect_.y0; j < roi_rect_.y1; j++) {
		for(unsigned i = roi_rect_.x0; i < roi_rect_.x1; i++) {
			
			diff_temperature_2_median = fabs( thermal_frame_in_C(i, j) - median_temperature_estimate_in_roi_ );
			sorted_diff_temperature_2_median.push_back(diff_temperature_2_median);
		}
	}

	if(sorted_diff_temperature_2_median.size() > 0) {
		// sorting the temperature
		vcl_sort(sorted_diff_temperature_2_median.begin(), sorted_diff_temperature_2_median.end());

		// obtain the median temperature value	
		int size = sorted_diff_temperature_2_median.size();
		if(size % 2 == 0) {
			// even number of elements
			median_deviation_temperature_estimate_in_roi_ = (sorted_diff_temperature_2_median[(size/2)-1] + sorted_diff_temperature_2_median[size/2])/2.0;
		}
		else {
			// odd number of elements
			median_deviation_temperature_estimate_in_roi_ = sorted_diff_temperature_2_median[(size-1)/2];
		}
	}

	// create the layer-based color palette
	num_of_layers_in_heat_map_ = 0;
	color_palette_in_heat_map_.clear();

	num_of_layers_in_heat_map_ = 2*num_of_deviations_as_outlier_in_roi_;
	int delta_idx_in_color_palette = (int)((float)(256)/(float)(num_of_layers_in_heat_map_));

	color_palette_in_heat_map_.resize(num_of_layers_in_heat_map_);
	for(int i = 0; i < num_of_layers_in_heat_map_; i++) {
		color_palette_in_heat_map_[i].resize(3);

		color_palette_in_heat_map_[i][0] = color_palette_[i*delta_idx_in_color_palette][0];
		color_palette_in_heat_map_[i][1] = color_palette_[i*delta_idx_in_color_palette][1];
		color_palette_in_heat_map_[i][2] = color_palette_[i*delta_idx_in_color_palette][2];
	}

	// generate the layer-based temperature segmentation frame	
	const vil_image_view<vxl_byte> &thermal_byte_img = thermal_source->cur_frame();
	viz_skin_temperature_layer_segmentation_frame_.deep_copy(thermal_byte_img);
	
	int layer_idx = 0;

	// detect all the hot spot pixels
	roi_size = (roi_rect_.y1 - roi_rect_.y0 + 1)*(roi_rect_.x1 - roi_rect_.x0 + 1);
	
	hot_spot_pixels_coordinate_.clear();
	hot_spot_pixels_coordinate_.reserve(roi_size);

	cold_spot_pixels_coordinate_.clear();
	cold_spot_pixels_coordinate_.reserve(roi_size);

	float num_of_deviations = 0;
	for(unsigned j = roi_rect_.y0; j < roi_rect_.y1; j++) {
		for(unsigned i = roi_rect_.x0; i < roi_rect_.x1; i++) {
			
			num_of_deviations = (thermal_frame_in_C(i, j) - median_temperature_estimate_in_roi_) / median_deviation_temperature_estimate_in_roi_;
			
			if(num_of_deviations < 0-num_of_deviations_as_outlier_in_roi_) {
				// this is a cold spot
				cold_spot_pixels_coordinate_.push_back(vnl_int_2(i, j));
			}
			else if(num_of_deviations > num_of_deviations_as_outlier_in_roi_) {
				// this is a hot spot
				hot_spot_pixels_coordinate_.push_back(vnl_int_2(i, j));
			}

			layer_idx = (int)(num_of_deviations + num_of_deviations_as_outlier_in_roi_);
			
			if(layer_idx < 0) {
				layer_idx = 0;
			}
			
			if(layer_idx >= num_of_layers_in_heat_map_) {
				layer_idx = num_of_layers_in_heat_map_ - 1;
			}

			viz_skin_temperature_layer_segmentation_frame_(i, j, 0) = color_palette_in_heat_map_[layer_idx][0];
			viz_skin_temperature_layer_segmentation_frame_(i, j, 1) = color_palette_in_heat_map_[layer_idx][1];
			viz_skin_temperature_layer_segmentation_frame_(i, j, 2) = color_palette_in_heat_map_[layer_idx][2];		
		}
	}

	/*
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// create the layer-based color palette based on max and min temperature values
	int min_temp = int(min_temperature_in_roi_);
	int max_temp = int(max_temperature_in_roi_ + 0.5);
	
	if(min_temp == max_temp) {
		num_of_layers_in_heat_map_ = 0;
		color_palette_in_heat_map_.clear();
		return;
	}
	
	num_of_layers_in_heat_map_ = (int)(float(max_temp - min_temp)/delta_temperature_in_heat_map_);
	
	int delta_idx_in_color_palette = (int)((float)(256)/(float)(num_of_layers_in_heat_map_));
	if(delta_idx_in_color_palette == 0) {
		// the number of layers is larger than the number of color palette elements, skip this.
		num_of_layers_in_heat_map_ = 0;
		color_palette_in_heat_map_.clear();
		return;
	}

	color_palette_in_heat_map_.resize(num_of_layers_in_heat_map_);
	for(int i = 0; i < num_of_layers_in_heat_map_; i++) {
		color_palette_in_heat_map_[i].resize(3);

		color_palette_in_heat_map_[i][0] = color_palette_[i*delta_idx_in_color_palette][0];
		color_palette_in_heat_map_[i][1] = color_palette_[i*delta_idx_in_color_palette][1];
		color_palette_in_heat_map_[i][2] = color_palette_[i*delta_idx_in_color_palette][2];
	}

	// generate the layer-based temperature segmentation frame
	viz_skin_temperature_layer_segmentation_frame_.fill(0);
	int layer_idx = 0;

	// detect all the hot spot pixels
	roi_size = (roi_rect_.y1 - roi_rect_.y0 + 1)*(roi_rect_.x1 - roi_rect_.x0 + 1);
	hot_spot_pixels_coordinate_.clear();
	hot_spot_pixels_coordinate_.reserve(roi_size);

	for(unsigned j = roi_rect_.y0; j < roi_rect_.y1; j++) {
		for(unsigned i = roi_rect_.x0; i < roi_rect_.x1; i++) {
			
			layer_idx = (int)((thermal_frame_in_C(i, j) - min_temp)/delta_temperature_in_heat_map_);
			if(layer_idx >= num_of_layers_in_heat_map_) {
				layer_idx = num_of_layers_in_heat_map_ - 1;
			}

			viz_skin_temperature_layer_segmentation_frame_(i, j, 0) = color_palette_in_heat_map_[layer_idx][0];
			viz_skin_temperature_layer_segmentation_frame_(i, j, 1) = color_palette_in_heat_map_[layer_idx][1];
			viz_skin_temperature_layer_segmentation_frame_(i, j, 2) = color_palette_in_heat_map_[layer_idx][2];

			if(thermal_frame_in_C(i, j) - median_temperature_estimate_in_roi_ >= hot_spot_temperature_diff_thresh_) {
				hot_spot_pixels_coordinate_.push_back(vnl_int_2(i, j));
			}
		}
	}
	*/
}

// clear the roi region thermal analysis
void pu_thermal_analysis_proc::clear_roi_thermal_analysis(void)
{
	if(roi_rect_handle_set_) {
		roi_rect_handle_set_ = gevxl::shape::rectangle_handle_set_sptr(NULL);
	}
}

// get GUI configurable parameters
void pu_thermal_analysis_proc::get_gui_configurable_params(float &skin_temperature_lower_range_in_C,
																														float &skin_temperature_higher_range_in_C,
																														float &delta_temperature_in_heat_map,
																														float &hot_spot_temperature_diff_thresh,
																														int &num_of_deviations_as_outlier)
{
	// visualization range of body skin temperature 
	skin_temperature_lower_range_in_C = skin_temperature_lower_range_in_C_;
	skin_temperature_higher_range_in_C = skin_temperature_higher_range_in_C_;

	// delta temperature for the heat map generation
	delta_temperature_in_heat_map = delta_temperature_in_heat_map_;

	// hot spot temperature difference threshold, i.e., 
	// the pixel whose temperature reading is greater than the median_temperature_estimate_in_roi_ with a margin of this threshold is considered as hot spot pixel
	hot_spot_temperature_diff_thresh = hot_spot_temperature_diff_thresh_;

	// number of deviations are considered as outlier
	num_of_deviations_as_outlier = num_of_deviations_as_outlier_in_roi_;
}

// set GUI configurable parameters
void pu_thermal_analysis_proc::set_gui_configurable_params(float skin_temperature_lower_range_in_C,
																														float skin_temperature_higher_range_in_C,
																														float delta_temperature_in_heat_map,
																														float hot_spot_temperature_diff_thresh,
																														int num_of_deviations_as_outlier)
{
	// visualization range of body skin temperature 
	skin_temperature_lower_range_in_C_ = skin_temperature_lower_range_in_C;
	skin_temperature_higher_range_in_C_ = skin_temperature_higher_range_in_C;

	// delta temperature for the heat map generation
	delta_temperature_in_heat_map_ = delta_temperature_in_heat_map;

	// hot spot temperature difference threshold, i.e., 
	// the pixel whose temperature reading is greater than the median_temperature_estimate_in_roi_ with a margin of this threshold is considered as hot spot pixel
	hot_spot_temperature_diff_thresh_ = hot_spot_temperature_diff_thresh;

	// number of deviations are considered as outlier
	num_of_deviations_as_outlier_in_roi_ = num_of_deviations_as_outlier;
}

// visualizing to the viz_canvas_frame
void pu_thermal_analysis_proc::visualize_canvas(vil_image_view<vxl_byte> &viz_canvas_frame)
{
	if(is_enabled() == false) return;

	if(thermal_frames_in_depth_view_) {
		
		for(unsigned j = 0; j < thermal_byte_frame_in_depth_view_.nj(); j++) {
			for(unsigned i = 0; i < thermal_byte_frame_in_depth_view_.ni(); i++) {
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = thermal_byte_frame_in_depth_view_(i, j, 0);
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = thermal_byte_frame_in_depth_view_(i, j, 1);
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = thermal_byte_frame_in_depth_view_(i, j, 2);
			}
		}

		return;
	}
	
	const gevxl::vid::micro_epsilon_socket_frame_process *thermal_source = dynamic_cast<const gevxl::vid::micro_epsilon_socket_frame_process *>(source_proc_->get_frame_process());
	const vil_image_view<vxl_byte> &thermal_byte_img = thermal_source->cur_frame();

	vil_image_view<vxl_byte> viz_temperature_frame;
	if(viz_modes_[viz_mode_selection_] == "raw_temperature_frame") {
		viz_temperature_frame = thermal_byte_img;
	}
	else if(viz_modes_[viz_mode_selection_] == "skin_temperature_range_frame") {
		viz_temperature_frame = viz_skin_temperature_range_frame_;
	}
	else if(viz_modes_[viz_mode_selection_] == "skin_temperature_layer_segmentation_frame") {
		viz_temperature_frame = viz_skin_temperature_layer_segmentation_frame_;
	}

	for(unsigned j = 0; j < viz_temperature_frame.nj(); j++) {
		for(unsigned i = 0; i < viz_temperature_frame.ni(); i++) {
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = viz_temperature_frame(i, j, 0);
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = viz_temperature_frame(i, j, 1);
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = viz_temperature_frame(i, j, 2);
		}
	}

	if(viz_hot_spot_flag_) {
		int i, j;
		for(int idx = 0; idx < hot_spot_pixels_coordinate_.size(); idx++) {
			
			i = hot_spot_pixels_coordinate_[idx](0);
			j = hot_spot_pixels_coordinate_[idx](1);

			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = 255;
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = 255;
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = 255;
		}

		for(int idx = 0; idx < cold_spot_pixels_coordinate_.size(); idx++) {
			
			i = cold_spot_pixels_coordinate_[idx](0);
			j = cold_spot_pixels_coordinate_[idx](1);

			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = 0;
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = 0;
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = 0;
		}
	}
}

// visualizing to the overlay visualizer
void pu_thermal_analysis_proc::visualize_overlay()
{
	if(is_enabled() == false) return;

	if(viz_) {
		if(viz_->is_initialized()) { 

			int i_offset = 10, j_offset = 20;
			viz_->set_foreground(0,1,0);
			
			viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_-j_offset, "median = " + gevxl::util::to_str(median_temperature_estimate_in_roi_) + "C", true, false, "below");
			j_offset = j_offset + 20;

			viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_-j_offset, "median_dev = " + gevxl::util::to_str(median_deviation_temperature_estimate_in_roi_) + "C", true, false, "below");
			j_offset = j_offset + 20;
			
			viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_-j_offset, "max = " + gevxl::util::to_str(max_temperature_in_roi_) + "C", true, false, "below");
			j_offset = j_offset + 20;

			viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_-j_offset, "min = " + gevxl::util::to_str(min_temperature_in_roi_) + "C", true, false, "below");
			j_offset = j_offset + 20;

			viz_->set_foreground(0,1,1);
			viz_->add_box(viz_offset_i_+roi_rect_.x0, viz_offset_j_+roi_rect_.y0, viz_offset_i_+roi_rect_.x1, viz_offset_j_+roi_rect_.y1);

		}
	}
}
