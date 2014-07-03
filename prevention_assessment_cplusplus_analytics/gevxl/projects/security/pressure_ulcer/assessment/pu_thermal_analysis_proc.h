// Copyright (C) 2013 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ming-Ching Chang and Ting Yu
/// \date 1/27/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_thermal_analysis_proc_h_
#define gevxl_pressure_ulcer_pu_thermal_analysis_proc_h_

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <threading/mutex.h>

#include <shape/shape_handle_set.h>
#include <shape/rectangle_handle_set.h>

#include <vnl/vnl_int_2.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {
  
			class pu_thermal_analysis_proc : public gevxl::framework::process,
																			 public gevxl::util::on_off_mixin
			{
			public:

				// Constructor
				pu_thermal_analysis_proc (char const *name="gevxl::pressure_ulcer::pu_thermal_analysis_proc"); 

				// Destructor
				virtual ~pu_thermal_analysis_proc(void); 

				// Configure this proc
				virtual bool configure(gevxl::util::config_file &config);

				// Initialize the process
				virtual bool initialize(void);

				// Uninitialize the process
				virtual void uninitialize(void);

				// set the point cloud xyz_rgb data from the depth camera
				void set_point_cloud_xyzrgb_frame(const vil_image_view<float> &xyz_rgb_frame);

				// The main step function
				virtual bool step(void);
	      
				bool step(const vil_image_view<vxl_byte> &thermal_byte_img);
	      
				// set the process's visualizer
				virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

				// set the viz offset
				virtual void set_viz_offset(int viz_offset_i, int viz_offset_j);

				// set the roi rectangle handle set
				void set_roi_rect_handle_set(gevxl::shape::rectangle_handle_set_sptr set) { roi_rect_handle_set_ = set; }

				// perform the roi region thermal analysis
				void perform_roi_thermal_analysis(void);

				// clear the roi region thermal analysis
				void clear_roi_thermal_analysis(void);

				// visualization function
				virtual void visualize(void);

				// visualizing to the viz_canvas_frame
				virtual void visualize_canvas(vil_image_view<vxl_byte> &viz_canvas_frame);

				// visualizing to the overlay visualizer
				virtual void visualize_overlay();

				// get the current skin temperature range frame
				const vil_image_view<vxl_byte> &cur_viz_skin_temperature_range_frame(void) const { return viz_skin_temperature_range_frame_; }

				// set the source process
				void set_source_process(gevxl::vid::generic_frame_process<vxl_byte> *src_proc) { source_proc_ = src_proc; }

				// get GUI configurable parameters
				void get_gui_configurable_params(float &skin_temperature_lower_range_in_C,
																				 float &skin_temperature_higher_range_in_C,
																				 float &delta_temperature_in_heat_map,
																				 float &hot_spot_temperature_diff_thresh,
																				 int &num_of_deviations_as_outlier);
				
				const vcl_vector<vcl_string> &get_viz_modes(void) const { return viz_modes_; }
				bool get_viz_hot_spot_flag(void) { return viz_hot_spot_flag_; }

				// set GUI configurable parameters
				void set_gui_configurable_params(float skin_temperature_lower_range_in_C,
																				 float skin_temperature_higher_range_in_C,
																				 float delta_temperature_in_heat_map,
																				 float hot_spot_temperature_diff_thresh,
																				 int num_of_deviations_as_outlier);

				// get and set the different vis modes
				const int get_viz_mode_selection(void) const { return viz_mode_selection_; }

				void set_viz_mode_selection(int viz_mode_selection) {
					if(viz_mode_selection >= viz_modes_.size() || viz_mode_selection < 0) return;
					viz_mode_selection_ = viz_mode_selection;
				}

				// set if we visualize the hot spot as the white pixel regions
				void set_viz_hot_spot_flag(bool flag) { viz_hot_spot_flag_ = flag; }

			private:
							
				// thermal texture mapping to the depth view
				void thermal_texture_mapping_to_depth_view(void);

				// skin temperature range scaling function
				void skin_temperature_range_scaling(void);

				// Input Source
				gevxl::vid::generic_frame_process<vxl_byte> *source_proc_;

				// The thermal camera's intrinsic, rotation and translation knowledge
				vnl_matrix_fixed<double, 3, 3> cam_K_;
				vnl_matrix_fixed<double, 3, 3> cam_rotation_;
				vnl_matrix_fixed<double, 3, 1> cam_translation_;
				vnl_matrix_fixed<double, 3, 4> cam_extrinsic_mat_;

				vnl_matrix_fixed<double, 3, 4> cam_proj_mat_;	// = cam_K_ * cam_extrinsic_mat_;

				// Input point cloud xyz_rgb frame
				vil_image_view<float> xyz_rgb_frame_;

				// Thermal image overlayed on top of the depth image
				vil_image_view<vxl_byte> thermal_byte_frame_in_depth_view_;
				vil_image_view<float> thermal_frame_in_C_in_depth_view_;
				vil_image_view<vxl_byte> thermal_valid_pixel_mask_in_depth_view_;

				bool thermal_frames_in_depth_view_;

				// high resolution timer
				gevxl::util::time::highres_timer highres_timer_; 

				// visualizer 	
				gevxl::img::visualizer_2d *viz_;

				// visualizer offset
				int viz_offset_i_;
				int viz_offset_j_;

				// visualizer image
				gevxl::img::visualizer_image *viz_img_;

				// the configuration file that the generator needs in order to configure its own parameters.
				gevxl::util::config_file config_;
			
				// frame number count
				int frame_nr_;

				// visualization frame for the range of body skin temperature
				vil_image_view<vxl_byte> viz_skin_temperature_range_frame_;

				// visualization frame for the temperature layering based segmentation
				vil_image_view<vxl_byte> viz_skin_temperature_layer_segmentation_frame_;

				// the rectangle region of interest definition by the user
				gevxl::shape::rectangle_handle_set_sptr roi_rect_handle_set_;
				util::rectangle<int> roi_rect_;
				int roi_rect_to_frame_border_x_margin_;
				int roi_rect_to_frame_border_y_margin_;

				// the color palette used by the thermal camera
				vcl_vector<vcl_vector<vxl_byte> > color_palette_;

				// the median temperature estimate from the rectangle ROI
				float median_temperature_estimate_in_roi_;
				
				float max_temperature_in_roi_;
				float min_temperature_in_roi_;

				float median_deviation_temperature_estimate_in_roi_;
				int num_of_deviations_as_outlier_in_roi_;

				// parameters to be configured for the thermal analysis				
				// visualization range of body skin temperature 
				float skin_temperature_lower_range_in_C_;
				float skin_temperature_higher_range_in_C_;

				// delta temperature for the heat map generation
				float delta_temperature_in_heat_map_;

				// hot spot temperature difference threshold, i.e., 
				// the pixel whose temperature reading is greater than the median_temperature_estimate_in_roi_ with a margin of this threshold is considered as hot spot pixel
				float hot_spot_temperature_diff_thresh_;

				// a string to describe what frame we would like to visualize
				vcl_vector<vcl_string> viz_modes_;
				int viz_mode_selection_;

				// a flag to indicate whether we will visualize the hot spot pixels
				bool viz_hot_spot_flag_;

				// parameters to be computed based on the configured parameters
				int num_of_layers_in_heat_map_;
				vcl_vector<vcl_vector<vxl_byte> > color_palette_in_heat_map_;

				// the pixel coordinates of the hot spot and cold spot pixels
				vcl_vector<vnl_int_2> hot_spot_pixels_coordinate_;
				vcl_vector<vnl_int_2> cold_spot_pixels_coordinate_;

				// other parameters
				float b_ratio_;

				vxl_byte pixel_th_low_;
				vxl_byte pixel_th_high_;
				vxl_byte frame_pixel_max_;
				vxl_byte frame_pixel_min_;

				vil_image_view<vxl_byte> viz_im_;

				// current and previous frames
				vil_image_view<vxl_byte> curr_frame_;
				vil_image_view<vxl_byte> prev_frame_;
	    
			};  // end of class pu_thermal_analysis_proc

			typedef vbl_shared_pointer<pu_thermal_analysis_proc> pu_thermal_analysis_proc_sptr;

		} // end of assessment namespace
  } //end namespace pressure_ulcer
} //end namespace gevxl


#endif
