// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 2/18/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_tissue_analysis_proc_h
#define gevxl_pressure_ulcer_pu_tissue_analysis_proc_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <shape/shape_handle_set.h>
#include <shape/point_handle_set.h>
#include <shape/line_handle_set.h>
#include <shape/rectangle_handle_set.h>
#include <shape/polygon_handle_set.h>

#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_2.h>
#include <vnl/vnl_int_2.h>

#include <img/histogram_fast_rgb.h>

#include "opencv2/imgproc/imgproc.hpp"

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {

		const int HISTOGRAM_FAST_RGB_N = 4;

		class pu_tissue_analysis_proc : public gevxl::framework::process,
																		public gevxl::util::on_off_mixin
		{
		public:

			// constructor
			pu_tissue_analysis_proc(char const *name="gevxl::pressure_ulcer::assessment::pu_tissue_analysis_proc");

			// destructor
			virtual ~pu_tissue_analysis_proc(void); 

			// configure this proc
			virtual bool configure(gevxl::util::config_file &config);

			// initialize the process
			virtual bool initialize(void);

			// uninitialize the process
			virtual void uninitialize(void);

			// the main step function
			virtual bool step(void);

			// to make this work without direct connection to an frame_process,
			// we can make a second call where an image has already been acquired from somewhere else.
			virtual bool step(vil_image_view<vxl_byte> img);

			// set the process's visualizer
			virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

			// set the viz offset
			virtual void set_viz_offset(int viz_offset_i, int viz_offset_j);

			// visualization function
			virtual void visualize(void);

			// visualizing to the viz_canvas_frame
			virtual void visualize_canvas(vil_image_view<vxl_byte> &viz_canvas_frame);

			// visualizing to the overlay visualizer
			virtual void visualize_overlay();

			// set the source process
			void set_source_process(gevxl::vid::generic_frame_process<vxl_byte> *src_proc) { source_proc_ = src_proc; }

      //////////////////////////////////////////////////////////////////////////////////
			// member functions associated with wound segmentation stuff
			void set_wound_segment_rect_handle_set(gevxl::shape::rectangle_handle_set_sptr set) { wound_segment_rect_handle_set_ = set; }
			
			void add_wound_segment_foreground_polygon_handle_set(gevxl::shape::polygon_handle_set_sptr set) { wound_segment_foreground_polygon_handle_sets_.push_back(set); }
			void add_wound_segment_background_polygon_handle_set(gevxl::shape::polygon_handle_set_sptr set) { wound_segment_background_polygon_handle_sets_.push_back(set); }

			void start_wound_segmentation(void);
			void cancel_wound_segmentation(void);
			void end_wound_segmentation(void);

			//////////////////////////////////////////////////////////////////////////////////
			// member functions associated with tissue segmentation stuff
			int get_tissue_segmentation_spatial_radius(void) { return tissue_segmentation_spatial_radius_; }
			void set_tissue_segmentation_spatial_radius(int spatial_radius) { tissue_segmentation_spatial_radius_ = spatial_radius; }

			int get_tissue_segmentation_color_radius(void) { return tissue_segmentation_color_radius_; }
			void set_tissue_segmentation_color_radius(int color_radius) { tissue_segmentation_color_radius_ = color_radius; }

			int get_tissue_segmentation_max_pyramid_level(void) { return tissue_segmentation_max_pyramid_level_; }
			void set_tissue_segmentation_max_pyramid_level(int pyramid_level) { tissue_segmentation_max_pyramid_level_ = pyramid_level; }

			void start_tissue_segmentation(void);
			void end_tissue_segmentation(void);

			bool is_tissue_segmentation_labeling_mouse_clicking_valid(int x, int y);
			const vcl_vector<vcl_string> &get_tissue_types(void) const { return tissue_types_; }
			void set_tissue_segmentation_label(int x, int y, vcl_string tissue_type);

			bool save_to_file_tissue_type_histogram_model(vcl_ofstream &ofs);
			bool load_from_file_tissue_type_histogram_model(vcl_ifstream &ifs);

			void classify_wound_segment_into_tissue_types(vil_image_view<vxl_byte> &image_label_map, 
																										 double &granulation_percentage,
																										 double &slough_percentage,
																										 double &eschar_percentage,
																										 double &bone_percentage);

			//////////////////////////////////////////////////////////////////////////////////
			// member functions associated with wound 3D measurement stuff
			void start_wound_3D_length_width_depth_measurement(void);

			//////////////////////////////////////////////////////////////////////////////////
			// member function to end the wound and tissue analysis for the current frame
			void end_wound_and_tissue_analysis_for_current_frame(void);
			
		private:
						
			// downsample the image view from the original image
			void downsample_image_view(const vil_image_view<vxl_byte> ori_img, vil_image_view<vxl_byte> &scaled_img);

      // input source
			gevxl::vid::generic_frame_process<vxl_byte> *source_proc_;

      // frame nr
      int frame_nr_;

      // current and previous frames
			vil_image_view<vxl_byte> curr_rgb_frame_;
			//vil_image_view<vxl_byte> prev_rgb_frame_;
			
			vil_image_view<vxl_byte> curr_depth_frame_;
			//vil_image_view<vxl_byte> prev_depth_frame_;

			vil_image_view<vxl_byte> curr_rgb_in_depth_frame_;
			//vil_image_view<vxl_byte> prev_rgb_in_depth_frame_;

			vil_image_view<float> curr_xyz_rgb_frame_;
			//vil_image_view<float> prev_xyz_rgb_frame_;

			// visualization downsampling factor
			int viz_downsampling_factor_;
			vil_image_view<vxl_byte> viz_rgb_frame_;

			// high resolution timer
      gevxl::util::time::highres_timer highres_timer_; 

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

			// visualizer offset
			int viz_offset_i_;
			int viz_offset_j_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;

      //////////////////////////////////////////////////////////////////////////////////
			// member variables associated with wound segmentation stuff
			gevxl::shape::rectangle_handle_set_sptr wound_segment_rect_handle_set_;

			vcl_vector<gevxl::shape::polygon_handle_set_sptr> wound_segment_foreground_polygon_handle_sets_;
			vcl_vector<gevxl::shape::polygon_handle_set_sptr> wound_segment_background_polygon_handle_sets_;

			void generate_wound_segment_mask(void);

			vil_image_view<vxl_byte> wound_segment_rgb_frame_;
			vil_image_view<vxl_byte> wound_segment_bgr_frame_;
			vil_image_view<vxl_byte> wound_segment_mask_frame_;

			cv::Mat wound_segment_bgr_mat_;
			cv::Mat wound_segment_mask_mat_;			

			vil_image_view<vxl_byte> wound_segment_result_rgb_frame_;
			vil_image_view<vxl_byte> viz_wound_segment_result_rgb_frame_;

			cv::Rect wound_segment_rect_;
			
			cv::Mat wound_segment_foreground_model_;
			cv::Mat wound_segment_background_model_;
			
			// the viz darken pixel value for the wound segmentation result
			int viz_wound_segment_darken_pixel_value_offset_;

      int wound_segment_stroke_line_thickness_;

			bool wound_segment_started_;
			bool wound_segment_initial_segment_done_;

			//////////////////////////////////////////////////////////////////////////////////
			// member variables associated with the tissue segmentation stuff
			bool tissue_segmentation_started_;

			int tissue_segmentation_spatial_radius_;
			int tissue_segmentation_color_radius_;
			int tissue_segmentation_max_pyramid_level_;

			vil_image_view<vxl_byte> tissue_segmentation_rgb_frame_;
			vil_image_view<vxl_byte> tissue_segmentation_bgr_frame_;
			
			vil_image_view<vxl_byte> tissue_segmentation_result_rgb_frame_;
			vil_image_view<vxl_byte> viz_tissue_segmentation_result_rgb_frame_;

			cv::Mat tissue_segmentation_bgr_mat_;

			vcl_map<long, vcl_vector<vnl_int_2> > tissue_segmentation_result_label_coordinates_;	// for each different mean color, stores the pixel coordinates that fall into this color category.
			vcl_map<long, vcl_vector<vxl_byte> > tissue_segmentation_result_random_colorized_pattern_; // for each different mean color, generates a new color pattern for the pixels that fall into this color category.
			
			vil_image_view<vxl_byte> tissue_segmentation_result_label_colorized_frame_; // based on the mean-shift clustering label index, assign an random color to this label.
			vil_image_view<vxl_byte> viz_tissue_segmentation_result_label_colorized_frame_;

			bool tissue_segmentation_mouse_selection_enabled_;
			vcl_vector<vnl_int_2> tissue_segmentation_mouse_selection_pixels_;

			vil_image_view<vxl_byte> tissue_segmentation_tissue_type_label_frame_;
			vcl_vector<vnl_int_2> tissue_segmentation_tissue_type_already_labeled_pixels_;

			//////////////////////////////////////////////////////////////////////////////////
			// member variables associated with the tissue classification stuff
			vcl_string tissue_type_sample_path_;
			vcl_vector<vcl_string> tissue_type_sample_filenames_;
			int viz_horizontal_spacing_for_tissue_type_sample_;

			vcl_vector<vil_image_view<vxl_byte> > tissue_type_samples_;
			vcl_vector<vcl_string> tissue_types_;

			vcl_map<vcl_string, int> tissue_types_2_labels_map_;
			vcl_map<int, vcl_string> tissue_labels_2_types_map_;

			vcl_map<int, gevxl::img::histogram_fast_rgb<HISTOGRAM_FAST_RGB_N> > tissue_types_color_hist_;
			vil_image_view<vxl_byte> tissue_types_classify_frame_;

			vcl_string tissue_types_color_hist_filename_;
			
			vcl_map<int, vil_rgb<vxl_byte> > tissue_types_colorized_patterns_;
			vil_image_view<vxl_byte> tissue_types_classify_colorized_frame_;

			vcl_map<int, double> tissue_types_classify_percentage_;

			bool tissue_types_classify_result_available_;

			//////////////////////////////////////////////////////////////////////////////////
			// member variables associated with the wound 3D measurement stuff
			bool wound_3d_measurement_started_;
			bool wound_3d_measurement_done_;

			double wound_3d_length_width_cos_angle_thresh_;
			double wound_3d_plane_fit_error_margin_thresh_;
			bool wound_3d_width_pts_computation_in_2d_;

			vil_image_view<vxl_byte> wound_3d_measurement_rgb_frame_;
			vil_image_view<vxl_byte> wound_3d_mask_in_depth_view_frame_;

			vcl_vector<vnl_double_3> wound_3d_contour_pts_;
			vcl_vector<vnl_int_2> wound_3d_contour_pts_ij_;

			double wound_3d_length_in_cm_;
			double wound_3d_width_in_cm_;
			double wound_3d_depth_in_cm_;

			vnl_int_2 wound_3d_length_pts_[2];
			vnl_int_2 wound_3d_width_pts_[2];
			vnl_int_2 wound_3d_depth_pt_;

			// debugging info
			vcl_string rgb_dump_filename_;
			vcl_string rgb_in_depth_dump_filename_;

		};  // end of class pu_tissue_analysis_proc

		typedef vbl_shared_pointer<pu_tissue_analysis_proc> pu_tissue_analysis_proc_sptr;

		} // end of assessment namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif

