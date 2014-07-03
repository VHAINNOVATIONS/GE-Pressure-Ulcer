// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/25/2013
/// \par Modifications:

#include "pu_tissue_analysis_proc.h"

#include <vcl_iostream.h>

#include <vil/vil_convert.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>

#include <vid/pxc_frame_process.h>

#include <boost/bind.hpp>

#include <shape/shape.h>
#include <shape/point.h>
#include <shape/line.h>
#include <shape/rectangle.h>
#include <shape/polygon.h>

#include <util/rectangle.h>

#include <vgl/algo/vgl_fit_plane_3d.h>
#include <vgl/vgl_distance.h>

#include <vnl/vnl_random.h>

#include <opencv_bridge/ocv_utils.h>
#include "opencv2/highgui/highgui.hpp"

using namespace gevxl;
using namespace gevxl::util;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer;
using namespace gevxl::pressure_ulcer::assessment;
using namespace gevxl::shape;
using namespace cv;

pu_tissue_analysis_proc::pu_tissue_analysis_proc(char const *name)
: gevxl::framework::process(name), 	
  source_proc_(NULL), 
  viz_(NULL),
	frame_nr_(0)
{
	highres_timer_.reset();

	wound_segment_rect_handle_set_ = gevxl::shape::rectangle_handle_set_sptr(NULL);
	wound_segment_foreground_polygon_handle_sets_.clear();
	wound_segment_background_polygon_handle_sets_.clear();

	wound_segment_started_ = false;
	wound_segment_initial_segment_done_ = false;

	tissue_segmentation_spatial_radius_ = 20;
	tissue_segmentation_color_radius_ = 20;
	tissue_segmentation_max_pyramid_level_ = 1;

	tissue_segmentation_started_ = false;

	tissue_segmentation_mouse_selection_enabled_ = false;

	tissue_types_classify_result_available_ = false;

	wound_3d_measurement_started_ = false;
	wound_3d_measurement_done_ = false;
}

pu_tissue_analysis_proc::~pu_tissue_analysis_proc(void)
{

}

bool pu_tissue_analysis_proc::configure(util::config_file &config)
{
	config_ = config;

  // visualization downsampling factor for the rgb image
	viz_downsampling_factor_ = 2;
	config_.get_integer(name()+"::viz_downsampling_factor", viz_downsampling_factor_);

	//////////////////////////////////////////////////////////////////////////////////
	// member variables associated with wound segmentation stuff
	wound_segment_stroke_line_thickness_ = 10;
	config_.get_integer(name()+"::wound_segment_stroke_line_thickness", wound_segment_stroke_line_thickness_);

	viz_wound_segment_darken_pixel_value_offset_ = 50;
	config_.get_integer(name()+"::viz_wound_segment_darken_pixel_value_offset", viz_wound_segment_darken_pixel_value_offset_);

	//////////////////////////////////////////////////////////////////////////////////
	// member variables associated with the tissue segmentation stuff
	tissue_segmentation_spatial_radius_ = 20;
	config_.get_integer(name()+"::tissue_segmentation_spatial_radius", tissue_segmentation_spatial_radius_);

	tissue_segmentation_color_radius_ = 20;
	config_.get_integer(name()+"::tissue_segmentation_color_radius", tissue_segmentation_color_radius_);

	tissue_segmentation_max_pyramid_level_ = 1;
	config_.get_integer(name()+"::tissue_segmentation_max_pyramid_level", tissue_segmentation_max_pyramid_level_);

	//////////////////////////////////////////////////////////////////////////////////
	// member variables associated with the tissue classification stuff
	tissue_type_sample_path_.clear();
	config_.get_string(name()+"::tissue_type_sample_path", tissue_type_sample_path_);

	tissue_type_sample_filenames_.clear();
	config_.get_vcl_vector_string(name()+"::tissue_type_sample_filenames", tissue_type_sample_filenames_);

	viz_horizontal_spacing_for_tissue_type_sample_ = 20;
	config_.get_integer(name()+"::viz_horizontal_spacing_for_tissue_type_sample", viz_horizontal_spacing_for_tissue_type_sample_);

	if(tissue_type_sample_path_.empty() == false && tissue_type_sample_filenames_.size() > 0) {
		
		tissue_type_samples_.resize(tissue_type_sample_filenames_.size());
		tissue_types_.resize(tissue_type_sample_filenames_.size());
		tissue_types_2_labels_map_.clear();
		tissue_labels_2_types_map_.clear();
		tissue_types_color_hist_.clear();

		for(unsigned i = 0; i < tissue_type_sample_filenames_.size(); i++) {

			vcl_string filename = tissue_type_sample_path_ + "/" + tissue_type_sample_filenames_[i];
			vil_image_view<vxl_byte> loaded_sample = vil_load(filename.c_str());
			
			tissue_type_samples_[i] = vil_image_view<vxl_byte>(loaded_sample.ni(), loaded_sample.nj(), 1, loaded_sample.nplanes());
			tissue_type_samples_[i].deep_copy(loaded_sample);

			tissue_types_[i] = tissue_type_sample_filenames_[i].substr(0, tissue_type_sample_filenames_[i].length()-12);
			tissue_types_2_labels_map_[tissue_types_[i]] = i+1;
			tissue_labels_2_types_map_[i+1] = tissue_types_[i];

			gevxl::img::histogram_fast_rgb<HISTOGRAM_FAST_RGB_N> hist;
			tissue_types_color_hist_[i+1] = hist;
		}
	}

	// load the tissue types color histogram
	tissue_types_color_hist_filename_.clear();
	config_.get_string(name()+"::tissue_types_color_hist_filename", tissue_types_color_hist_filename_);
	if(!tissue_types_color_hist_filename_.empty()) {
		vcl_ifstream ifs(tissue_types_color_hist_filename_.c_str());
		load_from_file_tissue_type_histogram_model(ifs);
	}

	tissue_types_colorized_patterns_.clear();
	vcl_vector<int> color_patterns;
	config_.get_vcl_vector_int(name()+"::tissue_type_colorized_patterns", color_patterns);
	if(color_patterns.size() == 3*tissue_type_sample_filenames_.size()) {
		for(unsigned i = 0; i < tissue_type_sample_filenames_.size(); i++) {
			tissue_types_colorized_patterns_[i+1] = vil_rgb<vxl_byte>(color_patterns[3*i+0], color_patterns[3*i+1], color_patterns[3*i+2]);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////
	// member variables associated with the wound 3D measurement stuff
	wound_3d_length_width_cos_angle_thresh_ = (0.1/180.0)*3.1415926;	// = 0.001745 = 0.1 degree threshold
	config_.get_double(name()+"::wound_3d_length_width_cos_angle_thresh", wound_3d_length_width_cos_angle_thresh_);

	wound_3d_plane_fit_error_margin_thresh_ = 0.01;	// 1 centimeter error margin
	config_.get_double(name()+"::wound_3d_plane_fit_error_margin_thresh", wound_3d_plane_fit_error_margin_thresh_);

	wound_3d_width_pts_computation_in_2d_ = false;
	config_.get_bool(name()+"::wound_3d_width_pts_computation_in_2d", wound_3d_width_pts_computation_in_2d_);

	// debugging info
	rgb_dump_filename_ = "";
	config_.get_string(name()+"::rgb_dump_filename", rgb_dump_filename_);

	rgb_in_depth_dump_filename_ = "";
	config_.get_string(name()+"::rgb_in_depth_dump_filename", rgb_in_depth_dump_filename_);

	return true;
}

bool pu_tissue_analysis_proc::initialize(void)
{
  return true;
}

void pu_tissue_analysis_proc::uninitialize(void)
{
  
}

bool pu_tissue_analysis_proc::step(vil_image_view<vxl_byte> img)
{
  // deal with the frame rate issue.
	double frame_rate = 1000/highres_timer_.elapsed();	
	highres_timer_.reset();

	gevxl::util::time::highres_timer state_timer;
	state_timer.reset();

  return true;
}


bool pu_tissue_analysis_proc::step(void)
{
  vil_image_view<vxl_byte> img = source_proc_->cur_frame();
	
	const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(source_proc_->get_frame_process());
	
	curr_rgb_frame_ = pxc_source->cur_rgb_frame();
	curr_depth_frame_ = pxc_source->cur_depth_byte_frame();
	curr_rgb_in_depth_frame_ = pxc_source->cur_rgb_frame_in_depth_view();
	curr_xyz_rgb_frame_ = pxc_source->cur_xyz_rgb_frame();

	bool succeeded = step(img);

	//prev_rgb_frame_.deep_copy(curr_rgb_frame_);
	//prev_depth_frame_.deep_copy(curr_depth_frame_);
	//prev_rgb_in_depth_frame_.deep_copy(curr_rgb_in_depth_frame_);
	//prev_xyz_rgb_frame_.deep_copy(curr_xyz_rgb_frame_);

	// debugging info	
	if(rgb_dump_filename_ != "") {
		//vil_save(curr_rgb_frame_, rgb_dump_filename_.c_str());
	}
	if(rgb_in_depth_dump_filename_ != "") {
		//vil_save(curr_rgb_in_depth_frame_, rgb_in_depth_dump_filename_.c_str());
	}
	
	frame_nr_++;

	return succeeded;
}

void pu_tissue_analysis_proc::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	viz_ = viz;
}

void pu_tissue_analysis_proc::set_viz_offset(int viz_offset_i, int viz_offset_j)
{
	// visualizer offset
	viz_offset_i_ = viz_offset_i;
	viz_offset_j_ = viz_offset_j;
}

void pu_tissue_analysis_proc::downsample_image_view(const vil_image_view<vxl_byte> ori_img, vil_image_view<vxl_byte> &scaled_img)
{
	int downsampled_width = (double)(ori_img.ni()) / (double)(viz_downsampling_factor_);
	int downsampled_height = (double)(ori_img.nj()) / (double)(viz_downsampling_factor_);

	if(scaled_img.ni() != downsampled_width || 
		 scaled_img.nj() != downsampled_height || 
		 scaled_img.nplanes() != ori_img.nplanes()) {

		scaled_img = vil_image_view<vxl_byte>(downsampled_width, downsampled_height, 1, ori_img.nplanes());
	}

	int nplanes = scaled_img.nplanes();	
	int ori_x, ori_y;
	for(int y = 0; y < downsampled_height; y++) {
		for(int x = 0; x < downsampled_width; x++) {
			for(int p = 0; p < nplanes; p++) {
				ori_x = x*viz_downsampling_factor_;
				ori_y = y*viz_downsampling_factor_;
				scaled_img(x, y, p) = ori_img(ori_x, ori_y, p);
			}
		}
	}
}

void pu_tissue_analysis_proc::start_wound_segmentation(void)
{
	if(wound_segment_started_ == false) {
		
		wound_segment_rgb_frame_.deep_copy(curr_rgb_frame_);
		wound_segment_bgr_frame_.deep_copy(wound_segment_rgb_frame_);
		wound_segment_bgr_mat_ = gevxl::ocv::convert_to_opencv_mat_safe(wound_segment_bgr_frame_, CV_8UC3);

		wound_segment_result_rgb_frame_.deep_copy(wound_segment_rgb_frame_);
		wound_segment_started_ = true;
	}
	
	// based on the curr_rgb_frame_, create the opencv version of wound_segment_bgr_mat_ and wound_segment_mask_mat_
	if(wound_segment_mask_mat_.empty()) {
		wound_segment_mask_mat_.create(wound_segment_bgr_mat_.size(), CV_8UC1);
		wound_segment_mask_mat_.setTo(cv::Scalar::all(cv::GC_BGD));
	}
	
	generate_wound_segment_mask();

	//vcl_string bgrName = "GrabCut BGR Image";
	//cv::imshow( bgrName, wound_segment_bgr_mat_ );

	if(wound_segment_initial_segment_done_) {
		// initial segment has already been done, we continue
		cv::grabCut( wound_segment_bgr_mat_, wound_segment_mask_mat_, wound_segment_rect_, 
									wound_segment_background_model_, wound_segment_foreground_model_, 1 );		
	}
	else {
		// let's start the initial segment
		if(wound_segment_foreground_polygon_handle_sets_.size() != 0 || 
			 wound_segment_background_polygon_handle_sets_.size() != 0) {
			
			// some foreground/background stroke points have been set, so let's use the mask based grabcut
			cv::grabCut( wound_segment_bgr_mat_, wound_segment_mask_mat_, wound_segment_rect_, 
										wound_segment_background_model_, wound_segment_foreground_model_, 1, cv::GC_INIT_WITH_MASK );
			
			wound_segment_initial_segment_done_ = true;
		}
		else {
			if(wound_segment_rect_handle_set_) {
				// only the rectangle roi is set
				cv::grabCut( wound_segment_bgr_mat_, wound_segment_mask_mat_, wound_segment_rect_, 
											wound_segment_background_model_, wound_segment_foreground_model_, 1, cv::GC_INIT_WITH_RECT );	
				
				wound_segment_initial_segment_done_ = true;
			}
		}
	}

	// post-processing the wound_segment_mask_mat_ for re-constructing the result bgr image	
	cv::Mat wound_segment_bin_mask_mat;
	if( wound_segment_bin_mask_mat.empty() || 
		wound_segment_bin_mask_mat.rows != wound_segment_mask_mat_.rows || 
		wound_segment_bin_mask_mat.cols != wound_segment_mask_mat_.cols ) {
		
		wound_segment_bin_mask_mat.create(wound_segment_mask_mat_.size(), CV_8UC1);
	}        
  
	wound_segment_bin_mask_mat = wound_segment_mask_mat_ & 1;

	cv::Mat wound_segment_result_bgr_mat;
	wound_segment_bgr_mat_.copyTo( wound_segment_result_bgr_mat, wound_segment_bin_mask_mat );

	//vcl_string resultBgrName = "GrabCut Result BGR Image";
	//cv::imshow( resultBgrName, wound_segment_result_bgr_mat );

	// extract the wound_segment_result_bgr_mat cv::Mat data to the wound_segment_result_rgb_frame_
	vil_image_view<vxl_byte> *result_bgr_frame = gevxl::ocv::convert_to_vil_image(wound_segment_result_bgr_mat);
	wound_segment_result_rgb_frame_.deep_copy(*result_bgr_frame);

	// convert the image from bgr to rgb again.
	unsigned char temp_val = 0;
	for(unsigned j = 0; j < wound_segment_result_rgb_frame_.nj(); j++) {
		for(unsigned i = 0; i < wound_segment_result_rgb_frame_.ni(); i++) {
			temp_val = wound_segment_result_rgb_frame_(i, j, 2);
			wound_segment_result_rgb_frame_(i, j, 2) = wound_segment_result_rgb_frame_(i, j, 0);
			wound_segment_result_rgb_frame_(i, j, 0) = temp_val;			
		}
	}	
}

void pu_tissue_analysis_proc::cancel_wound_segmentation(void)
{
	wound_segment_result_rgb_frame_.deep_copy(wound_segment_rgb_frame_);
	
	if(wound_segment_mask_mat_.empty() == false) {		
		wound_segment_mask_mat_.release();		
		wound_segment_background_model_.release();
		wound_segment_foreground_model_.release();
	}

	wound_segment_rect_handle_set_ = gevxl::shape::rectangle_handle_set_sptr(NULL);
	wound_segment_foreground_polygon_handle_sets_.clear();
	wound_segment_background_polygon_handle_sets_.clear();

	wound_segment_initial_segment_done_ = false;	
}

void pu_tissue_analysis_proc::end_wound_segmentation(void)
{
	// the wound segmentation is finished, now the workflow comes to the tissue segmentation stage by default.
	tissue_segmentation_rgb_frame_.deep_copy(wound_segment_result_rgb_frame_);
	tissue_segmentation_result_rgb_frame_.deep_copy(tissue_segmentation_rgb_frame_);
	tissue_segmentation_result_label_colorized_frame_.deep_copy(tissue_segmentation_rgb_frame_);
	tissue_segmentation_tissue_type_label_frame_.deep_copy(tissue_segmentation_rgb_frame_);
	tissue_segmentation_tissue_type_already_labeled_pixels_.clear();
	tissue_types_classify_frame_.deep_copy(tissue_segmentation_rgb_frame_);
	tissue_types_classify_colorized_frame_.deep_copy(tissue_segmentation_rgb_frame_);
	tissue_segmentation_started_ = true;

	// the wound segmentation is finished, it can also enable the wound 3D measurement stage.
	wound_3d_measurement_rgb_frame_.deep_copy(wound_segment_result_rgb_frame_);
	wound_3d_measurement_started_ = true;

	// clean up the wound segmentation variables.
	cancel_wound_segmentation();

	if(wound_segment_started_ == true) {
		wound_segment_rgb_frame_.clear();
		wound_segment_started_ = false;
	}
}

void pu_tissue_analysis_proc::generate_wound_segment_mask(void)
{
	if(wound_segment_initial_segment_done_ == false) {
		// extract the rectangle ROI from the rectangle handle set
		util::rectangle<double> rect_d = wound_segment_rect_handle_set_->get_rectangle()->get_rectangle();
		// upsampling to get the coordinates in the original image view
		rect_d.x0 = rect_d.x0*viz_downsampling_factor_;
		rect_d.y0 = rect_d.y0*viz_downsampling_factor_;
		rect_d.x1 = rect_d.x1*viz_downsampling_factor_;
		rect_d.y1 = rect_d.y1*viz_downsampling_factor_;

		// set the rectangle to the mask
		wound_segment_rect_.x = int(rect_d.x0+0.5);
		wound_segment_rect_.y = int(rect_d.y0+0.5);
		wound_segment_rect_.width = int(rect_d.width()+0.5);
		wound_segment_rect_.height = int(rect_d.height()+0.5);

		(wound_segment_mask_mat_(wound_segment_rect_)).setTo( cv::Scalar(cv::GC_PR_FGD) );
	}

	// extract the foreground points from the vector of polygon handle set
	for(unsigned i = 0; i < wound_segment_foreground_polygon_handle_sets_.size(); i++) {
		
		gevxl::shape::polygon_handle_set_sptr set = wound_segment_foreground_polygon_handle_sets_[i];
		vcl_vector<vnl_double_2> pts = set->get_polygon()->get_vnl_double_2_vector();
		cv::Point cv_pt1, cv_pt2;
		for(unsigned j = 0; j <= pts.size()-2; j++) {
			
			cv_pt1.x = int(pts[j](0)*viz_downsampling_factor_ + 0.5);
			cv_pt1.y = int(pts[j](1)*viz_downsampling_factor_ + 0.5);
			
			cv_pt2.x = int(pts[j+1](0)*viz_downsampling_factor_ + 0.5);
			cv_pt2.y = int(pts[j+1](1)*viz_downsampling_factor_ + 0.5);
			
			cv::line( wound_segment_mask_mat_, cv_pt1, cv_pt2, cv::GC_FGD, wound_segment_stroke_line_thickness_ ); 
		}
	}

	// extract the background points from the vector of polygon handle set
	for(unsigned i = 0; i < wound_segment_background_polygon_handle_sets_.size(); i++) {
		
		gevxl::shape::polygon_handle_set_sptr set = wound_segment_background_polygon_handle_sets_[i];
		vcl_vector<vnl_double_2> pts = set->get_polygon()->get_vnl_double_2_vector();
		cv::Point cv_pt1, cv_pt2;
		for(unsigned j = 0; j <= pts.size()-2; j++) {
						
			cv_pt1.x = int(pts[j](0)*viz_downsampling_factor_ + 0.5);
			cv_pt1.y = int(pts[j](1)*viz_downsampling_factor_ + 0.5);

			cv_pt2.x = int(pts[j+1](0)*viz_downsampling_factor_ + 0.5);
			cv_pt2.y = int(pts[j+1](1)*viz_downsampling_factor_ + 0.5);
			
			cv::line( wound_segment_mask_mat_, cv_pt1, cv_pt2, cv::GC_BGD, wound_segment_stroke_line_thickness_ );
		}
	}
}

void pu_tissue_analysis_proc::start_tissue_segmentation(void)
{
	tissue_segmentation_bgr_frame_.deep_copy(tissue_segmentation_rgb_frame_);

	// convert to the opencv cv::Mat for mean-shift segmentation.
	tissue_segmentation_bgr_mat_ = gevxl::ocv::convert_to_opencv_mat_safe(tissue_segmentation_bgr_frame_, CV_8UC3);
	
	// perform the mean-shift segmentation, where the tissue_segmentation_result_mat's pixel stores 
	// the mean color value of each mean-shift cluster after the segmentation.
	cv::Mat tissue_segmentation_result_mat;
	cv::pyrMeanShiftFiltering( tissue_segmentation_bgr_mat_, tissue_segmentation_result_mat, 
				tissue_segmentation_spatial_radius_, tissue_segmentation_color_radius_, tissue_segmentation_max_pyramid_level_ );

	//vcl_string tissue_segmentation_result_mat_name = "Mean-Shift Segmentation Result Image";
	//cv::imshow(tissue_segmentation_result_mat_name, tissue_segmentation_result_mat);

	// count how many segmentation labels that the mean-shift segmentation generates using the floodFill algorithm
	cv::Mat tissue_segmentation_result_color_labeled_mat;
	tissue_segmentation_result_color_labeled_mat = tissue_segmentation_result_mat.clone();

	cv::Mat mask( tissue_segmentation_result_color_labeled_mat.rows+2, tissue_segmentation_result_color_labeled_mat.cols+2, CV_8UC1, Scalar::all(0) );
	int num_of_segments = 0;
	
	cv::RNG rng = cv::theRNG();
  for( int y = 0; y < tissue_segmentation_result_color_labeled_mat.rows; y++ ) {
		for( int x = 0; x < tissue_segmentation_result_color_labeled_mat.cols; x++ ) {
			if( mask.at<uchar>(y+1, x+1) == 0 ) {
				
				Scalar newVal( rng(256), rng(256), rng(256) );

				floodFill( tissue_segmentation_result_color_labeled_mat, mask, cv::Point(x,y), newVal, 0, cv::Scalar::all(2), cv::Scalar::all(2) );
				num_of_segments++;
      }
    }
  }
	vcl_cout << "pu_tissue_analysis_proc::start_tissue_segmentation, num_of_segments = " << num_of_segments << "\n" << vcl_endl;

	//vcl_string tissue_segmentation_result_color_labeled_mat_name = "Mean-Shift Segmentation Result Color Labeled Image";
	//cv::imshow(tissue_segmentation_result_color_labeled_mat_name, tissue_segmentation_result_color_labeled_mat);

	//vil_image_view<vxl_byte> *result = gevxl::ocv::convert_to_vil_image(tissue_segmentation_result_mat);
	vil_image_view<vxl_byte> *result = gevxl::ocv::convert_to_vil_image(tissue_segmentation_result_color_labeled_mat);
	
	//vcl_cout << "pu_tissue_analysis_proc::start_tissue_segmentation, result's width = " << result->ni() << vcl_endl;
	//vcl_cout << "pu_tissue_analysis_proc::start_tissue_segmentation, result's height = " << result->nj() << vcl_endl;
	//vcl_cout << "pu_tissue_analysis_proc::start_tissue_segmentation, result's planes = " << result->nplanes() << vcl_endl;

	// convert the *result back to the rgb vil_image_view
	tissue_segmentation_result_rgb_frame_.deep_copy(*result);

	// convert the image from bgr to rgb again.
	unsigned char temp_val = 0;
	for(unsigned j = 0; j < tissue_segmentation_result_rgb_frame_.nj(); j++) {
		for(unsigned i = 0; i < tissue_segmentation_result_rgb_frame_.ni(); i++) {
			temp_val = tissue_segmentation_result_rgb_frame_(i, j, 2);
			tissue_segmentation_result_rgb_frame_(i, j, 2) = tissue_segmentation_result_rgb_frame_(i, j, 0);
			tissue_segmentation_result_rgb_frame_(i, j, 0) = temp_val;			
		}
	}

	// the tissue_segmentation_result_rgb_frame_ holds the mean color value of each mean-shift cluster after the segmentation.
	// count the number of wound pixels
	long num_of_wound_pixels = 0;
	for(unsigned j = 0; j < tissue_segmentation_result_rgb_frame_.nj(); j++) {
		for(unsigned i = 0; i < tissue_segmentation_result_rgb_frame_.ni(); i++) {
			if( tissue_segmentation_rgb_frame_(i,j,0) == 0 && 
				  tissue_segmentation_rgb_frame_(i,j,1) == 0 && 
					tissue_segmentation_rgb_frame_(i,j,2) == 0 ) {
				// this is a background pixel, so we shouldn't worry about it.
				continue;
			}
			num_of_wound_pixels++;
		}
	}
	vcl_cout << "pu_tissue_analysis_proc::start_tissue_segmentation, num_of_wound_pixels = " << num_of_wound_pixels << "\n" << vcl_endl;

	// relabel the wound pixels based on clustering label and randomized colorization
	tissue_segmentation_result_label_coordinates_.clear();	// for each different mean color, stores the pixel coordinates that fall into this color category.
	tissue_segmentation_result_random_colorized_pattern_.clear(); // for each different mean color, generates a new color pattern for the pixels that fall into this color category.
	
	tissue_segmentation_result_label_colorized_frame_.deep_copy(tissue_segmentation_rgb_frame_); // based on the mean-shift clustering label index, assign an random color to this label.
	tissue_segmentation_result_label_colorized_frame_.fill(0);

	//random color generator
	vnl_random vrand(9667566);

	long pixel_label = 0;
	for(unsigned j = 0; j < tissue_segmentation_result_rgb_frame_.nj(); j++) {
		for(unsigned i = 0; i < tissue_segmentation_result_rgb_frame_.ni(); i++) {
			
			if( tissue_segmentation_rgb_frame_(i,j,0) == 0 && 
				  tissue_segmentation_rgb_frame_(i,j,1) == 0 && 
					tissue_segmentation_rgb_frame_(i,j,2) == 0 ) {
				// this is a background pixel, so we shouldn't worry about it.
				continue;
			}

			pixel_label = tissue_segmentation_result_rgb_frame_(i,j,0)*256*256 + 
										tissue_segmentation_result_rgb_frame_(i,j,1)*256 +
										tissue_segmentation_result_rgb_frame_(i,j,2);

			if( tissue_segmentation_result_label_coordinates_.find(pixel_label) == tissue_segmentation_result_label_coordinates_.end() ) {
				// this is a new pixel label, let's add it to tissue_segmentation_result_label_coordinates_
				vcl_vector<vnl_int_2> coordinates;
				coordinates.clear();
				tissue_segmentation_result_label_coordinates_[pixel_label] = coordinates;

				tissue_segmentation_result_label_coordinates_[pixel_label].reserve(num_of_wound_pixels);
				tissue_segmentation_result_label_coordinates_[pixel_label].push_back(vnl_int_2(i, j));

				// generate a new random color and add it to tissue_segmentation_result_random_colorized_pattern_
				vcl_vector<vxl_byte> color;
				color.resize(3);
				tissue_segmentation_result_random_colorized_pattern_[pixel_label] = color;

				tissue_segmentation_result_random_colorized_pattern_[pixel_label].resize(3);
				tissue_segmentation_result_random_colorized_pattern_[pixel_label][0] = vrand(255);
				tissue_segmentation_result_random_colorized_pattern_[pixel_label][1] = vrand(255);
				tissue_segmentation_result_random_colorized_pattern_[pixel_label][2] = vrand(255);
			}
			else {
				// this is an existing pixel label
				tissue_segmentation_result_label_coordinates_[pixel_label].push_back(vnl_int_2(i, j));
			}

			// assign the randomized color to the tissue_segmentation_result_label_colorized_frame_
			tissue_segmentation_result_label_colorized_frame_(i,j,0) = tissue_segmentation_result_random_colorized_pattern_[pixel_label][0];
			tissue_segmentation_result_label_colorized_frame_(i,j,1) = tissue_segmentation_result_random_colorized_pattern_[pixel_label][1];
			tissue_segmentation_result_label_colorized_frame_(i,j,2) = tissue_segmentation_result_random_colorized_pattern_[pixel_label][2];
		}
	}
	vcl_cout << "pu_tissue_analysis_proc::start_tissue_segmentation, num of random colorized patterns = " << tissue_segmentation_result_random_colorized_pattern_.size() << "\n" << vcl_endl;
	
	// printing out for validation.
	int num_of_colorized_wound_pixels = 0;
	int colorized_index = 0;
	vcl_map<long, vcl_vector<vnl_int_2> >::iterator iter = tissue_segmentation_result_label_coordinates_.begin();
	for(; iter != tissue_segmentation_result_label_coordinates_.end(); iter++) {
		
		if(iter->second.size() > 100) {
			vcl_cout << "pu_tissue_analysis_proc::start_tissue_segmentation, colorized_index = " << colorized_index << " has " << iter->second.size() << " wound pixels." << vcl_endl;
		}
		
		num_of_colorized_wound_pixels += iter->second.size();
		colorized_index++;
	}
	vcl_cout << "\npu_tissue_analysis_proc::start_tissue_segmentation, total number of colorized wound pixels = " << num_of_colorized_wound_pixels << "\n" << vcl_endl;
}

bool pu_tissue_analysis_proc::is_tissue_segmentation_labeling_mouse_clicking_valid(int x, int y)
{
	x = x*viz_downsampling_factor_;
	y = y*viz_downsampling_factor_;
	
	if( x < 0 || x >= tissue_segmentation_rgb_frame_.ni() ||
		  y < 0 || y >= tissue_segmentation_rgb_frame_.nj() ) {
		
		// the mouse clicking falls outside of the image
		return false;
	}

	if( tissue_segmentation_rgb_frame_(x,y,0) == 0 && 
			tissue_segmentation_rgb_frame_(x,y,1) == 0 && 
			tissue_segmentation_rgb_frame_(x,y,2) == 0 ) {
		// this is a background pixel, so we shouldn't worry about it.
		return false;
	}

	// perform the highlighting of the selected tissue segment
	long pixel_label = tissue_segmentation_result_rgb_frame_(x,y,0)*256*256 + 
										 tissue_segmentation_result_rgb_frame_(x,y,1)*256 +
										 tissue_segmentation_result_rgb_frame_(x,y,2);

	if( tissue_segmentation_result_label_coordinates_.find(pixel_label) == tissue_segmentation_result_label_coordinates_.end() ) {
		// something is wrong here.
		return false;
	}

	vcl_vector<vnl_int_2> &coordinates = tissue_segmentation_result_label_coordinates_[pixel_label];
	
	tissue_segmentation_mouse_selection_pixels_.clear();
	tissue_segmentation_mouse_selection_pixels_.insert(tissue_segmentation_mouse_selection_pixels_.end(), 
																											coordinates.begin(), coordinates.end());
	
	tissue_segmentation_mouse_selection_enabled_ = true;
	
	return true;
}

void pu_tissue_analysis_proc::set_tissue_segmentation_label(int x, int y, vcl_string tissue_type)
{
	int tissue_type_label = tissue_types_2_labels_map_[tissue_type];

	int i = 0, j = 0;
	for(unsigned p = 0; p < tissue_segmentation_mouse_selection_pixels_.size(); p++) {
		i = tissue_segmentation_mouse_selection_pixels_[p](0);
		j = tissue_segmentation_mouse_selection_pixels_[p](1);

		tissue_segmentation_tissue_type_label_frame_(i,j,0) = tissue_type_label;
		tissue_segmentation_tissue_type_label_frame_(i,j,1) = tissue_type_label;
		tissue_segmentation_tissue_type_label_frame_(i,j,2) = tissue_type_label;
	}

	tissue_segmentation_tissue_type_already_labeled_pixels_.insert(tissue_segmentation_tissue_type_already_labeled_pixels_.end(),
																																	tissue_segmentation_mouse_selection_pixels_.begin(),
																																	tissue_segmentation_mouse_selection_pixels_.end());

	// accumulatively add the tissue type pixels into the histogram
	vcl_map<int, gevxl::img::histogram_fast_rgb<HISTOGRAM_FAST_RGB_N> >::iterator iter = tissue_types_color_hist_.begin();

	vil_rgb<vxl_byte> pix;
	for(; iter != tissue_types_color_hist_.end(); iter++) {
		// go over each tissue type
		if(iter->first != tissue_type_label) {
			continue;
		}

		for(unsigned p = 0; p < tissue_segmentation_mouse_selection_pixels_.size(); p++) {
			i = tissue_segmentation_mouse_selection_pixels_[p](0);
			j = tissue_segmentation_mouse_selection_pixels_[p](1);

			pix.r = tissue_segmentation_rgb_frame_(i,j,0);
			pix.g = tissue_segmentation_rgb_frame_(i,j,1);
			pix.b = tissue_segmentation_rgb_frame_(i,j,2);

			// add data point with spread
			//iter->second.add_data_point_with_spread(pix);
			// add data point without spread
			iter->second.add_data_point(pix);			
		}
	}
	
	tissue_segmentation_mouse_selection_enabled_ = false;
}

bool pu_tissue_analysis_proc::save_to_file_tissue_type_histogram_model(vcl_ofstream &ofs)
{
	vcl_map<int, gevxl::img::histogram_fast_rgb<HISTOGRAM_FAST_RGB_N> >::iterator iter = tissue_types_color_hist_.begin();
	
	for(; iter != tissue_types_color_hist_.end(); iter++) {
		// go over each tissue type
		ofs << "tissue_type=" << vcl_endl;
		ofs << iter->first << vcl_endl;
		ofs << "histogram=" << vcl_endl;
		iter->second.sout(ofs);
	}

	return true;
}

bool pu_tissue_analysis_proc::load_from_file_tissue_type_histogram_model(vcl_ifstream &ifs)
{
	vcl_string description;
	int tissue_type = 0;

	int histogram_count = tissue_types_color_hist_.size();
	for(unsigned i = 0; i < histogram_count; i++) {

		// go over each tissue type
		ifs >> description;
		ifs >> tissue_type;
		ifs >> description;
		tissue_types_color_hist_[tissue_type].sin(ifs);
	}

	return true;
}

void pu_tissue_analysis_proc::classify_wound_segment_into_tissue_types(vil_image_view<vxl_byte> &image_label_map, 
																																				double &granulation_percentage,
																																				double &slough_percentage,
																																				double &eschar_percentage,
																																				double &bone_percentage)
{
	// create a normalized tissue type color histogram model for each tissue type.
	vcl_map<int, gevxl::img::histogram_fast_rgb<HISTOGRAM_FAST_RGB_N> > tissue_types_color_hist_normalized;
	
	vcl_map<int, gevxl::img::histogram_fast_rgb<HISTOGRAM_FAST_RGB_N> >::iterator iter = tissue_types_color_hist_.begin();
	for(; iter != tissue_types_color_hist_.end(); iter++) {
		// go over each tissue type
		int type_label = iter->first;
		tissue_types_color_hist_normalized[type_label] = iter->second;
		tissue_types_color_hist_normalized[type_label].normalise();
	}

	// star to classify each pixel
	vil_image_view<vxl_byte> temp_classify_label_frame;
	temp_classify_label_frame.deep_copy(tissue_types_classify_frame_);

	int tissue_type_with_max_likelihood = 1;
	double max_likelihood = -1.0, likelihood = -1.0;

	vil_rgb<vxl_byte> pix;
	for(unsigned j = 0; j < temp_classify_label_frame.nj(); j++) {
		for(unsigned i = 0; i < temp_classify_label_frame.ni(); i++) {
			
			if( temp_classify_label_frame(i,j,0) == 0 && 
				  temp_classify_label_frame(i,j,1) == 0 &&
					temp_classify_label_frame(i,j,2) == 0 ) {
				continue;
			}

			// for the pixel to be classified, go over each different tissue type, and classify the pixel to the tissue type that has the largest likelihood
			likelihood = -1.0;
			max_likelihood = -1.0;
			tissue_type_with_max_likelihood = 1;

			iter = tissue_types_color_hist_normalized.begin();
			for(; iter != tissue_types_color_hist_normalized.end(); iter++) {
				
				pix.r = temp_classify_label_frame(i,j,0);
				pix.g = temp_classify_label_frame(i,j,1);
				pix.b = temp_classify_label_frame(i,j,2);
				
				//likelihood = iter->second.get_bin_count( pix );
				likelihood = iter->second.get_data_point( pix );
				
				if(likelihood > max_likelihood) {
					max_likelihood = likelihood;
					tissue_type_with_max_likelihood = iter->first;
				}
			}

			temp_classify_label_frame(i,j,0) = tissue_type_with_max_likelihood;
			temp_classify_label_frame(i,j,1) = tissue_type_with_max_likelihood;
			temp_classify_label_frame(i,j,2) = tissue_type_with_max_likelihood;
		}
	}
	
	// perform a neighborhood-based majority voting on the classify label
	vcl_map<int, int> classify_label_count;
	classify_label_count.clear();
	iter = tissue_types_color_hist_normalized.begin();
	for(; iter != tissue_types_color_hist_normalized.end(); iter++) {
		classify_label_count[iter->first] = 0;
	}

	vcl_map<int, int>::iterator iter1 = classify_label_count.begin();
	int max_count = -1;
	int max_count_label = 1;

	// also count the percentage of each tissue type
	tissue_types_classify_percentage_.clear();
	vcl_map<int, vcl_string>::iterator iter2 = tissue_labels_2_types_map_.begin();
	for(; iter2 != tissue_labels_2_types_map_.end(); iter2++) {
		tissue_types_classify_percentage_[iter2->first] = 0.0;
	}
	double total_tissue_pixels_count = 0.0;
	
	for(unsigned j = 1; j < temp_classify_label_frame.nj()-1; j++) {
		for(unsigned i = 1; i < temp_classify_label_frame.ni()-1; i++) {

			if( temp_classify_label_frame(i,j,0) == 0 && 
				  temp_classify_label_frame(i,j,1) == 0 &&
					temp_classify_label_frame(i,j,2) == 0 ) {
				continue;
			}
			
			// re-set the counting of each classify label to zero
			iter1 = classify_label_count.begin();
			for(; iter1 != classify_label_count.end(); iter1++) {
				classify_label_count[iter1->first] = 0;
			}

			for(unsigned neigh_j = j-1; neigh_j <= j+1; neigh_j++) {
				for(unsigned neigh_i = i-1; neigh_i <= i+1; neigh_i++) {
					
					if( temp_classify_label_frame(neigh_i,neigh_j,0) == 0 && 
						  temp_classify_label_frame(neigh_i,neigh_j,1) == 0 &&
						  temp_classify_label_frame(neigh_i,neigh_j,2) == 0 ) {
					
						continue;
					}

					classify_label_count[temp_classify_label_frame(neigh_i,neigh_j,0)] = classify_label_count[temp_classify_label_frame(neigh_i,neigh_j,0)] + 1;
				}
			}

			max_count = -1;
			max_count_label = 1;
			iter1 = classify_label_count.begin();
			for(; iter1 != classify_label_count.end(); iter1++) {
				if(classify_label_count[iter1->first] > max_count) {
					max_count = classify_label_count[iter1->first];
					max_count_label = iter1->first;
				}
			}

			tissue_types_classify_frame_(i,j,0) = max_count_label;
			tissue_types_classify_frame_(i,j,1) = max_count_label;
			tissue_types_classify_frame_(i,j,2) = max_count_label;

			tissue_types_classify_colorized_frame_(i,j,0) = tissue_types_colorized_patterns_[max_count_label].r;
			tissue_types_classify_colorized_frame_(i,j,1) = tissue_types_colorized_patterns_[max_count_label].g;
			tissue_types_classify_colorized_frame_(i,j,2) = tissue_types_colorized_patterns_[max_count_label].b;

			tissue_types_classify_percentage_[max_count_label] = tissue_types_classify_percentage_[max_count_label] + 1.0;
			total_tissue_pixels_count = total_tissue_pixels_count + 1.0;
		}
	}

	iter2 = tissue_labels_2_types_map_.begin();
	for(; iter2 != tissue_labels_2_types_map_.end(); iter2++) {
		tissue_types_classify_percentage_[iter2->first] = (int)( (tissue_types_classify_percentage_[iter2->first]/total_tissue_pixels_count)*100 + 0.5);
	}

	// return the results to the caller
	image_label_map.deep_copy(tissue_types_classify_colorized_frame_);
	iter2 = tissue_labels_2_types_map_.begin();
	for(; iter2 != tissue_labels_2_types_map_.end(); iter2++) {
		if(iter2->second == "1_granulation_tissue") {
			granulation_percentage = tissue_types_classify_percentage_[iter2->first];
		}
		else if(iter2->second == "2_slough") {
			slough_percentage = tissue_types_classify_percentage_[iter2->first];
		}
		else if(iter2->second == "3_eschar") {
			eschar_percentage = tissue_types_classify_percentage_[iter2->first];
		}
		else if(iter2->second == "4_bone") {
			bone_percentage = tissue_types_classify_percentage_[iter2->first];
		}
	}

	tissue_types_classify_result_available_ = true;
}

void pu_tissue_analysis_proc::end_tissue_segmentation(void)
{
	// to be continued
}

void pu_tissue_analysis_proc::start_wound_3D_length_width_depth_measurement(void)
{
	// the wound segmentation is finished, it can also enable the wound 3D measurement stage.
	if(wound_3d_measurement_started_ == false) return;

	// depth to rgb coordinate map and xyz point cloud in depth view
	const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(source_proc_->get_frame_process());

	const vil_image_view<int> &depth_to_rgb_coordinate_map = pxc_source->cur_depth_to_rgb_coordinate_map();
	const vil_image_view<float> &xyzrgb_frame = pxc_source->cur_xyz_rgb_frame();
	
	wound_3d_mask_in_depth_view_frame_ = vil_image_view<vxl_byte>(xyzrgb_frame.ni(), xyzrgb_frame.nj(), 1, 1);
	wound_3d_mask_in_depth_view_frame_.fill(255);

	int xx, yy;
	for(unsigned j = 0; j < xyzrgb_frame.nj(); j++) {
		for(unsigned i = 0; i < xyzrgb_frame.ni(); i++) {
			
			// wound segmentation result is ready, and we can use it to blacken all the non-wound pixels in the depth view.
			xx = depth_to_rgb_coordinate_map(i, j, 0);
			yy = depth_to_rgb_coordinate_map(i, j, 1);
			if( wound_3d_measurement_rgb_frame_(xx, yy, 0) == 0 &&
					wound_3d_measurement_rgb_frame_(xx, yy, 1) == 0 &&
					wound_3d_measurement_rgb_frame_(xx, yy, 2) == 0 ) {
				
				// this is a background pixel after wound segmentation
				wound_3d_mask_in_depth_view_frame_(i, j) = 0;
			}
		}
	}
	
	// go through the wound_3d_mask_in_depth_view_frame_, to find the wound contour points
	wound_3d_contour_pts_.clear();
	wound_3d_contour_pts_.reserve(wound_3d_mask_in_depth_view_frame_.ni()*wound_3d_mask_in_depth_view_frame_.nj());
	wound_3d_contour_pts_ij_.clear();
	wound_3d_contour_pts_ij_.reserve(wound_3d_mask_in_depth_view_frame_.ni()*wound_3d_mask_in_depth_view_frame_.nj());

	wound_3d_length_in_cm_ = 0.0;
	wound_3d_width_in_cm_ = 0.0;
	wound_3d_depth_in_cm_ = 0.0;
	for(unsigned j = 1; j < wound_3d_mask_in_depth_view_frame_.nj()-1; j++) {
		for(unsigned i = 1; i < wound_3d_mask_in_depth_view_frame_.ni()-1; i++) {
			
			if(wound_3d_mask_in_depth_view_frame_(i, j) == 0) continue;

			// this is a foreground pixel, i.e., 255, now check the 8-neighbors
			if(wound_3d_mask_in_depth_view_frame_(i-1, j-1) == 0 || 
				 wound_3d_mask_in_depth_view_frame_(i-1, j) == 0 || 
				 wound_3d_mask_in_depth_view_frame_(i-1, j+1) == 0 || 
				 wound_3d_mask_in_depth_view_frame_(i, j-1) == 0 || 
				 wound_3d_mask_in_depth_view_frame_(i, j+1) == 0 || 
				 wound_3d_mask_in_depth_view_frame_(i+1, j-1) == 0 || 
				 wound_3d_mask_in_depth_view_frame_(i+1, j) == 0 || 
				 wound_3d_mask_in_depth_view_frame_(i+1, j+1) == 0) {
			
				// one of the neighbor pixels is background pixel, i.e., 0, this is a contour point
				vnl_double_3 pt = vnl_double_3( xyzrgb_frame(i,j,0), xyzrgb_frame(i,j,1), xyzrgb_frame(i,j,2) );
				if(pt(0) == 0.0 && 
					 pt(1) == 0.0 && 
					 pt(2) == 0.0) {
					// this is a noisy depth pixel				
					continue;
				}

				wound_3d_contour_pts_.push_back(pt);

				vnl_int_2 ij = vnl_int_2(i, j);
				wound_3d_contour_pts_ij_.push_back(ij);
			}
		}
	}

	vcl_cout << "number of wound 3D contour points = " << wound_3d_contour_pts_.size() << "\n" << vcl_endl;

	// find the length by searching for two points on the contour that have the longest distance
	double length_max = -1.0;
	double length;
	vnl_double_3 pt1, pt2;
	vnl_double_3 length_pt1, length_pt2;

	for(unsigned m = 0; m < wound_3d_contour_pts_.size(); m++) {
		for(unsigned n = m+1; n < wound_3d_contour_pts_.size(); n++) {
			pt1 = wound_3d_contour_pts_[m];
			pt2 = wound_3d_contour_pts_[n];

			length = ( pt1(0)-pt2(0) )*( pt1(0)-pt2(0) ) + 
							 ( pt1(1)-pt2(1) )*( pt1(1)-pt2(1) ) + 
							 ( pt1(2)-pt2(2) )*( pt1(2)-pt2(2) );
			
			if(length > length_max) {
				length_max = length;
				length_pt1 = pt1;
				length_pt2 = pt2;

				wound_3d_length_pts_[0] = wound_3d_contour_pts_ij_[m];
				wound_3d_length_pts_[1] = wound_3d_contour_pts_ij_[n];
			}
		}
	}

	wound_3d_length_in_cm_ = vcl_sqrt(length_max)*100;
	vcl_cout << "wound_3d_length_in_cm_ = " << wound_3d_length_in_cm_ << vcl_endl;
	vcl_cout << "two length points are = " << length_pt1 << "; " << length_pt2 << "\n" << vcl_endl;

	if(wound_3d_width_pts_computation_in_2d_ == false) {
		// compute the wound 3D width in 3D space
		vnl_double_3 length_line( length_pt1(0)-length_pt2(0), length_pt1(1)-length_pt2(1), length_pt1(2)-length_pt2(2) );
		vnl_double_3 line, width_line;

		double cos_val_thresh = cos(3.1415926/2.0 - wound_3d_length_width_cos_angle_thresh_);
		double width_max = -1.0;
		double width;
		double cos_val;
		vnl_double_3 width_pt1, width_pt2;

		double cos_elm1, cos_elm2;
		for(unsigned m = 0; m < wound_3d_contour_pts_.size(); m++) {
			for(unsigned n = m+1; n < wound_3d_contour_pts_.size(); n++) {
				pt1 = wound_3d_contour_pts_[m];
				pt2 = wound_3d_contour_pts_[n];

				line = vnl_double_3( pt1(0)-pt2(0), pt1(1)-pt2(1), pt1(2)-pt2(2) );
				
				cos_elm1 = fabs( line(0)*length_line(0) + line(1)*length_line(1) + line(2)*length_line(2) );
				cos_elm2 = vcl_sqrt( line(0)*line(0)+line(1)*line(1)+line(2)*line(2) ) *
									 vcl_sqrt( length_line(0)*length_line(0)+length_line(1)*length_line(1)+length_line(2)*length_line(2) );
				cos_val = cos_elm1/cos_elm2;
				
				if(cos_val < cos_val_thresh) {
					// this is a candidate width_line
					width = line(0)*line(0)+line(1)*line(1)+line(2)*line(2);
					if(width > width_max) {
						width_max = width;
						width_pt1 = pt1;
						width_pt2 = pt2;

						wound_3d_width_pts_[0] = wound_3d_contour_pts_ij_[m];
						wound_3d_width_pts_[1] = wound_3d_contour_pts_ij_[n];
					}
				}

			}
		}

		wound_3d_width_in_cm_ = vcl_sqrt(width_max)*100;
		vcl_cout << "wound_3d_width_in_cm_ = " << wound_3d_width_in_cm_ << vcl_endl;
		vcl_cout << "two width points are = " << width_pt1 << "; " << width_pt2 << "\n" << vcl_endl;
	}
	else {
		// compute the wound 3D width in 2D space
		vnl_int_2 length_line = wound_3d_length_pts_[0] - wound_3d_length_pts_[1];
		vnl_int_2 line, width_line;

		double wound_3d_length_width_cos_angle_thresh = wound_3d_length_width_cos_angle_thresh_;
		double cos_val_thresh = cos(3.1415926/2.0 - wound_3d_length_width_cos_angle_thresh);
		
		double cos_elm1, cos_elm2;
		double cos_val;
		
		double width_max = -1.0;
		double width;
		
		vnl_int_2 pt1, pt2;
		int m_star, n_star;
		
		bool found = false;

		while(true) {
		
			width_max = -1.0;
			for(unsigned m = 0; m < wound_3d_contour_pts_ij_.size(); m++) {
				for(unsigned n = m+1; n < wound_3d_contour_pts_ij_.size(); n++) {
					pt1 = wound_3d_contour_pts_ij_[m];
					pt2 = wound_3d_contour_pts_ij_[n];

					line = pt1 - pt2;
					
					cos_elm1 = fabs( (double)(line(0)*length_line(0) + line(1)*length_line(1)) );
					cos_elm2 = vcl_sqrt( (double)(line(0)*line(0)+line(1)*line(1)) ) *
										 vcl_sqrt( (double)(length_line(0)*length_line(0)+length_line(1)*length_line(1)) );
					cos_val = cos_elm1/cos_elm2;
					
					if(cos_val < cos_val_thresh) {
						// this is a candidate width_line
						width = line(0)*line(0)+line(1)*line(1);
						if(width > width_max) {
							width_max = width;
							
							wound_3d_width_pts_[0] = pt1;
							wound_3d_width_pts_[1] = pt2;

							m_star = m;
							n_star = n;
							
							found = true;
						}
					}

				}
			}

			if(found) break;

			// incrementally increase the angle threshold by 1 degree
			wound_3d_length_width_cos_angle_thresh += (1.0/180.0)*3.1415926;
			cos_val_thresh = cos(3.1415926/2.0 - wound_3d_length_width_cos_angle_thresh);
		}
		
		vnl_double_3 width_pt1_3d = wound_3d_contour_pts_[m_star];
		vnl_double_3 width_pt2_3d = wound_3d_contour_pts_[n_star];		
		wound_3d_width_in_cm_ = vcl_sqrt( (width_pt1_3d(0)-width_pt2_3d(0))*(width_pt1_3d(0)-width_pt2_3d(0)) + 
																			(width_pt1_3d(1)-width_pt2_3d(1))*(width_pt1_3d(1)-width_pt2_3d(1)) + 
																			(width_pt1_3d(2)-width_pt2_3d(2))*(width_pt1_3d(2)-width_pt2_3d(2)) )*100;
		vcl_cout << "wound_3d_width_in_cm_ = " << wound_3d_width_in_cm_ << vcl_endl;
		vcl_cout << "two width points are = " << width_pt1_3d << "; " << width_pt2_3d << "\n" << vcl_endl;
	}

	// perform a planar fitting to the contour points to get the plane
	vgl_fit_plane_3d<double> plane_fit;
	for(unsigned m = 0; m < wound_3d_contour_pts_.size(); m++) {
		plane_fit.add_point( wound_3d_contour_pts_[m](0), wound_3d_contour_pts_[m](1), wound_3d_contour_pts_[m](2) );
	}
	bool succeeded = plane_fit.fit(wound_3d_plane_fit_error_margin_thresh_);
	vcl_cout << "plane fitting succeeded = " << succeeded << "\n\n" << vcl_endl;

	vgl_homg_plane_3d<double> plane = plane_fit.get_plane();
	
	// compute the wound point that has the largest distance to the plane, which is the depth of the wound
	vnl_double_3 depth_pt;
	double depth_max = -1.0;
	double depth;
	for(unsigned j = 1; j < wound_3d_mask_in_depth_view_frame_.nj()-1; j++) {
		for(unsigned i = 1; i < wound_3d_mask_in_depth_view_frame_.ni()-1; i++) {
			
			if(wound_3d_mask_in_depth_view_frame_(i, j) == 0) continue;

			vnl_double_3 p = vnl_double_3( xyzrgb_frame(i,j,0), xyzrgb_frame(i,j,1), xyzrgb_frame(i,j,2) );

			if(p(0) == 0.0 && 
				 p(1) == 0.0 && 
				 p(2) == 0.0) {
				// this is a noisy depth pixel				
				continue;
			}

			vgl_homg_point_3d<double> pt = vgl_homg_point_3d<double>( xyzrgb_frame(i,j,0), xyzrgb_frame(i,j,1), xyzrgb_frame(i,j,2), 1 );
			
			depth = vgl_distance(pt, plane);
			if(depth > depth_max) {
				depth_max = depth;

				depth_pt(0) = xyzrgb_frame(i,j,0);
				depth_pt(1) = xyzrgb_frame(i,j,1);
				depth_pt(2) = xyzrgb_frame(i,j,2);

				wound_3d_depth_pt_(0) = i;
				wound_3d_depth_pt_(1) = j;
			}
		}
	}

	wound_3d_depth_in_cm_ = depth_max*100;
	vcl_cout << "wound_3d_depth_in_cm_ = " << wound_3d_depth_in_cm_ << vcl_endl;
	vcl_cout << "one depth point is = " << depth_pt << "\n" << vcl_endl;

	wound_3d_measurement_done_ = true;
}

void pu_tissue_analysis_proc::end_wound_and_tissue_analysis_for_current_frame(void)
{
	// clear tissue segmentation stuff
	tissue_segmentation_started_ = false;
	tissue_segmentation_result_label_coordinates_.clear();
	tissue_segmentation_result_random_colorized_pattern_.clear();
	tissue_segmentation_mouse_selection_enabled_ = false;
	tissue_segmentation_mouse_selection_pixels_.clear();
	tissue_segmentation_tissue_type_already_labeled_pixels_.clear();

	// clear tissue classify stuff
	tissue_types_classify_result_available_ = false;

	// clear wound 3d stuff
	wound_3d_measurement_started_ = false;
	wound_3d_measurement_done_ = false;
	wound_3d_contour_pts_.clear();
	wound_3d_contour_pts_ij_.clear();

	// clear wound segmentation stuff
	cancel_wound_segmentation();

	wound_segment_rgb_frame_.clear();
	wound_segment_started_ = false;
}

void pu_tissue_analysis_proc::visualize(void)
{
	
}

// visualizing to the viz_canvas_frame
void pu_tissue_analysis_proc::visualize_canvas(vil_image_view<vxl_byte> &viz_canvas_frame)
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// layout the different frames into the viz_canvas_frame
	int i_offset = 0, j_offset = 0;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// first visualize the stuff that could come from the wound segmentation process
	// paste the viz_rgb_frame_ into the canvas at the left-top of the canvas
	if(wound_segment_started_) {
		downsample_image_view(wound_segment_rgb_frame_, viz_rgb_frame_);
	}
	else {
		downsample_image_view(curr_rgb_frame_, viz_rgb_frame_);
	}
	for(unsigned j = 0; j < viz_rgb_frame_.nj(); j++) {
		for(unsigned i = 0; i < viz_rgb_frame_.ni(); i++) {
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = viz_rgb_frame_(i, j, 0);
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = viz_rgb_frame_(i, j, 1);
			viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = viz_rgb_frame_(i, j, 2);
		}
	}

	if(wound_segment_started_) {
		// paste the viz_wound_segment_result_rgb_frame_ into the right-top of the canvas
		downsample_image_view(wound_segment_result_rgb_frame_, viz_wound_segment_result_rgb_frame_);
		i_offset = viz_rgb_frame_.ni();
		j_offset = 0;
		for(unsigned j = 0; j < viz_wound_segment_result_rgb_frame_.nj(); j++) {
			for(unsigned i = 0; i < viz_wound_segment_result_rgb_frame_.ni(); i++) {
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 0) = viz_wound_segment_result_rgb_frame_(i, j, 0);
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 1) = viz_wound_segment_result_rgb_frame_(i, j, 1);
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 2) = viz_wound_segment_result_rgb_frame_(i, j, 2);

				if(viz_wound_segment_result_rgb_frame_(i, j, 0) == 0 && 
					viz_wound_segment_result_rgb_frame_(i, j, 1) == 0 &&
					viz_wound_segment_result_rgb_frame_(i, j, 2) == 0) {

						// after the wound segmentation, this becomes a background pixel.
						viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = (unsigned char)(vcl_max(0, (int)(viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0)) - viz_wound_segment_darken_pixel_value_offset_));
						viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = (unsigned char)(vcl_max(0, (int)(viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1)) - viz_wound_segment_darken_pixel_value_offset_));
						viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = (unsigned char)(vcl_max(0, (int)(viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2)) - viz_wound_segment_darken_pixel_value_offset_));
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// second, visualize the stuff that could come from the tissue segmentation process
	if(tissue_segmentation_started_) {
		// paste the tissue_segmentation_rgb_frame_ into the left-top of the canvas
		downsample_image_view(tissue_segmentation_rgb_frame_, viz_rgb_frame_);
		for(unsigned j = 0; j < viz_rgb_frame_.nj(); j++) {
			for(unsigned i = 0; i < viz_rgb_frame_.ni(); i++) {
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = viz_rgb_frame_(i, j, 0);
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = viz_rgb_frame_(i, j, 1);
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = viz_rgb_frame_(i, j, 2);
			}
		}

		if(tissue_segmentation_mouse_selection_enabled_) {
			// paste the tissue_segmentation_mouse_selection_pixels_ into the left-top of the canvas
			int i = 0, j = 0;
			for(unsigned p = 0; p < tissue_segmentation_mouse_selection_pixels_.size(); p++) {
				i = (int)((double)(tissue_segmentation_mouse_selection_pixels_[p](0)) / (double)(viz_downsampling_factor_) + 0.5);
				j = (int)((double)(tissue_segmentation_mouse_selection_pixels_[p](1)) / (double)(viz_downsampling_factor_) + 0.5);
				// highlight the green channel.
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = (unsigned char)(vcl_min(255, viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) + 100));
			}
		}

		if(tissue_segmentation_tissue_type_already_labeled_pixels_.size() > 0) {
			// paste the tissue_segmentation_tissue_type_already_labeled_pixels_ into the left-top of the canvas
			int i = 0, j = 0;
			for(unsigned p = 0; p < tissue_segmentation_tissue_type_already_labeled_pixels_.size(); p++) {
				i = (int)((double)(tissue_segmentation_tissue_type_already_labeled_pixels_[p](0)) / (double)(viz_downsampling_factor_) + 0.5);
				j = (int)((double)(tissue_segmentation_tissue_type_already_labeled_pixels_[p](1)) / (double)(viz_downsampling_factor_) + 0.5);
				// blacken the pixel as it has been labeled..
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = 0;
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = 0;
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = 0;
			}
		}

		// paste the viz_tissue_segmentation_result_label_colorized_frame_ into the right-top of the canvas
		downsample_image_view(tissue_segmentation_result_label_colorized_frame_, viz_tissue_segmentation_result_label_colorized_frame_);
		i_offset = viz_rgb_frame_.ni();
		j_offset = 0;
		for(unsigned j = 0; j < viz_tissue_segmentation_result_label_colorized_frame_.nj(); j++) {
			for(unsigned i = 0; i < viz_tissue_segmentation_result_label_colorized_frame_.ni(); i++) {
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 0) = viz_tissue_segmentation_result_label_colorized_frame_(i, j, 0);
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 1) = viz_tissue_segmentation_result_label_colorized_frame_(i, j, 1);
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 2) = viz_tissue_segmentation_result_label_colorized_frame_(i, j, 2);
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// third, visualize the stuff that could come from the tissue classification process
	if(tissue_types_classify_result_available_) {
		// paste the tissue_types_classify_colorized_frame_ into the left-top of the canvas
		downsample_image_view(tissue_types_classify_colorized_frame_, viz_rgb_frame_);
		for(unsigned j = 0; j < viz_rgb_frame_.nj(); j++) {
			for(unsigned i = 0; i < viz_rgb_frame_.ni(); i++) {
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 0) = viz_rgb_frame_(i, j, 0);
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 1) = viz_rgb_frame_(i, j, 1);
				viz_canvas_frame(viz_offset_i_+i, viz_offset_j_+j, 2) = viz_rgb_frame_(i, j, 2);
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// fourth, visualize the curr_rgb_in_depth_frame_ into the canvas
	// paste the curr_rgb_in_depth_frame_ into the left-bottom of the canvas
	const gevxl::vid::pxc_frame_process *pxc_source = dynamic_cast<const gevxl::vid::pxc_frame_process *>(source_proc_->get_frame_process());

	const vil_image_view<int> &depth_to_rgb_coordinate_map = pxc_source->cur_depth_to_rgb_coordinate_map();
	int xx, yy;
	i_offset = 0;
	j_offset = viz_rgb_frame_.nj();
	for(unsigned j = 0; j < curr_rgb_in_depth_frame_.nj(); j++) {
		for(unsigned i = 0; i < curr_rgb_in_depth_frame_.ni(); i++) {
			viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 0) = curr_rgb_in_depth_frame_(i, j, 0);
			viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 1) = curr_rgb_in_depth_frame_(i, j, 1);
			viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 2) = curr_rgb_in_depth_frame_(i, j, 2);

			if(wound_3d_measurement_started_) {
				// paste the non-wound blackened pixels into the left-bottom of the canvas
				// wound segmentation result is ready, and we can use it to blacken all the non-wound pixels in the depth view.
				xx = depth_to_rgb_coordinate_map(i, j, 0);
				yy = depth_to_rgb_coordinate_map(i, j, 1);
				if( wound_3d_measurement_rgb_frame_(xx, yy, 0) == 0 &&
					wound_3d_measurement_rgb_frame_(xx, yy, 1) == 0 &&
					wound_3d_measurement_rgb_frame_(xx, yy, 2) == 0 ) {
						// this is a background pixel after wound segmentation
						viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 0) = 0;
						viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 1) = 0;
						viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 2) = 0;
				}
			}
		}
	}

	/*
	for(unsigned j = 0; j < curr_depth_frame_.nj(); j++) {
		for(unsigned i = 0; i < curr_depth_frame_.ni(); i++) {
			viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 0) = curr_depth_frame_(i, j, 0);
			viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 1) = curr_depth_frame_(i, j, 0);
			viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 2) = curr_depth_frame_(i, j, 0);
		}
	}
	*/

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// visualize wound contour in the depth view
	// paste the wound_3d_contour_pts_ij_ into the left-bottom of the canvas
	if(wound_3d_measurement_done_) {
		i_offset = 0;
		j_offset = viz_rgb_frame_.nj();

		for(unsigned m = 0; m < wound_3d_contour_pts_ij_.size(); m++) {
			unsigned i = wound_3d_contour_pts_ij_[m](0);
			unsigned j = wound_3d_contour_pts_ij_[m](1);

			int dot_diameter = 3;
			for(unsigned neighbor_j = j-dot_diameter; neighbor_j <= j+dot_diameter; neighbor_j++) {
				for(unsigned neighbor_i = i-dot_diameter; neighbor_i <= i+dot_diameter; neighbor_i++) {

					if( neighbor_i < 0 || neighbor_i >= curr_rgb_in_depth_frame_.ni() || 
						neighbor_j < 0 || neighbor_j >= curr_rgb_in_depth_frame_.nj() ) {

							continue;
					}

					viz_canvas_frame(viz_offset_i_+neighbor_i+i_offset, viz_offset_j_+neighbor_j+j_offset, 0) = 255;	
					viz_canvas_frame(viz_offset_i_+neighbor_i+i_offset, viz_offset_j_+neighbor_j+j_offset, 0) = 0;
					viz_canvas_frame(viz_offset_i_+neighbor_i+i_offset, viz_offset_j_+neighbor_j+j_offset, 0) = 0;								
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// finally, paste the tissue type samples into the viz_canvas_frame
	// paste the tissue_type_samples_ into the very left-bottom of the canvas
	i_offset = 0;
	j_offset = viz_rgb_frame_.nj() + curr_rgb_in_depth_frame_.nj();
	for(unsigned t = 0; t < tissue_type_samples_.size(); t++) {

		for(unsigned j = 0; j < tissue_type_samples_[t].nj(); j++) {
			for(unsigned i = 0; i < tissue_type_samples_[t].ni(); i++) {
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 0) = tissue_type_samples_[t](i, j, 0);
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 1) = tissue_type_samples_[t](i, j, 1);
				viz_canvas_frame(viz_offset_i_+i+i_offset, viz_offset_j_+j+j_offset, 2) = tissue_type_samples_[t](i, j, 2);
			}
		}

		i_offset = i_offset + tissue_type_samples_[t].ni() + viz_horizontal_spacing_for_tissue_type_sample_;		
	}
}

// visualizing to the overlay visualizer
void pu_tissue_analysis_proc::visualize_overlay()
{
	if(viz_) {
		if(viz_->is_initialized()) { 
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// visualize the tissue type percentage info
			int i_offset, j_offset;
			if(tissue_types_classify_result_available_) {
				i_offset = 10;
				j_offset = 10;
				vcl_map<int, vcl_string>::iterator iter2 = tissue_labels_2_types_map_.begin();
				for(; iter2 != tissue_labels_2_types_map_.end(); iter2++) {
					vcl_string tissue_type = iter2->second;
					int tissue_percentage = (int)(tissue_types_classify_percentage_[iter2->first]);
					viz_->set_foreground(0,1,0);
					viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_+j_offset, tissue_type + ":\t" + gevxl::util::to_str(tissue_percentage) + "%", true, false, "below");
					j_offset = j_offset + 20;
				}
			}

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// visualize the wound 3d measurement's length, width, and depth info
			if(wound_3d_measurement_done_) {
				i_offset = 10;
				j_offset = viz_rgb_frame_.nj() + 10;
				int length_in_mm = int(wound_3d_length_in_cm_*10+0.5);
				int width_in_mm = int(wound_3d_width_in_cm_*10+0.5);
				int depth_in_mm = int(wound_3d_depth_in_cm_*10+0.5);
				viz_->set_foreground(0,1,0);
				viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_+j_offset, "length=" + gevxl::util::to_str(length_in_mm) + "mm", true, false, "below");
				viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_+j_offset+20, "width=" + gevxl::util::to_str(width_in_mm) + "mm", true, false, "below");
				viz_->add_text(viz_offset_i_+i_offset, viz_offset_j_+j_offset+40, "depth=" + gevxl::util::to_str(depth_in_mm) + "mm", true, false, "below");

				// add the visualization of length line, width line and deepest depth point
				i_offset = 0;
				j_offset = viz_rgb_frame_.nj();

				viz_->set_line_width(3);

				viz_->set_foreground(1,1,0);
				viz_->add_line(viz_offset_i_+i_offset+wound_3d_length_pts_[0](0), viz_offset_j_+j_offset+wound_3d_length_pts_[0](1), 
											 viz_offset_i_+i_offset+wound_3d_length_pts_[1](0), viz_offset_j_+j_offset+wound_3d_length_pts_[1](1));

				viz_->set_foreground(1,0,1);
				viz_->add_line(viz_offset_i_+i_offset+wound_3d_width_pts_[0](0), viz_offset_j_+j_offset+wound_3d_width_pts_[0](1), 
											 viz_offset_i_+i_offset+wound_3d_width_pts_[1](0), viz_offset_j_+j_offset+wound_3d_width_pts_[1](1));

				viz_->set_foreground(0,1,1);
				viz_->add_circle(viz_offset_i_+i_offset+wound_3d_depth_pt_(0), viz_offset_j_+j_offset+wound_3d_depth_pt_(1), 3);
			}
		}
	}
}

