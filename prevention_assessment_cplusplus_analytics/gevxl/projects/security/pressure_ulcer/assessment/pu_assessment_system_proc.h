// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/09/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_assessment_system_proc_h
#define gevxl_pressure_ulcer_pu_assessment_system_proc_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>
#include <vid/code_stamp_frame_tag_process.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>
#include <util/rectangle.h>

#include "pu_tissue_analysis_proc.h"
#include "pu_3d_recon_proc.h"
#include "pu_thermal_analysis_proc.h"

#include <framework/async_wrapping_process.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {

		class pu_assessment_system_proc : public gevxl::framework::process,
																			public gevxl::util::on_off_mixin
		{
		public:

			// constructor
			pu_assessment_system_proc(char const *name="gevxl::pressure_ulcer::assessment::pu_assessment_system_proc");

			// destructor
			virtual ~pu_assessment_system_proc(void); 

			// configure this proc
			virtual bool configure(gevxl::util::config_file &config);

			// initialize the process
			virtual bool initialize(void);

			// uninitialize the process
			virtual void uninitialize(void);

			// the main step function
			virtual bool step(void);

			//////////////////////////////////////////////////////////////////////////////////
			// python GUI setter and getter methods.

			// set the process's visualizer from GUI.
			virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

			void set_rgb_visualizer(gevxl::img::visualizer_2d *viz) 
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_rgb_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_rgb_visualizer, viz = " << viz << vcl_endl;
				}
				rgb_viz_ = viz; 
			}
			
			void set_depth_visualizer(gevxl::img::visualizer_2d *viz)
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_depth_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_depth_visualizer, viz = " << viz << vcl_endl;
				}
				depth_viz_ = viz; 
			}
			
			void set_thermal_visualizer(gevxl::img::visualizer_2d *viz)
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_thermal_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_thermal_visualizer, viz = " << viz << vcl_endl;
				}
				thermal_viz_ = viz; 
			}
			
      void set_rgb_copy_visualizer(gevxl::img::visualizer_2d *viz) 
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_rgb_copy_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_rgb_copy_visualizer, viz = " << viz << vcl_endl;
				}
				rgb_copy_viz_ = viz; 
			}

			void set_hyper_spectral_visualizer(gevxl::img::visualizer_2d *viz) 
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_hyper_spectral_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_hyper_spectral_visualizer, viz = " << viz << vcl_endl;
				}
				hyperspectral_viz_ = viz; 
			}

      void set_rgb_touch_visualizer(gevxl::img::visualizer_2d *viz) 
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_rgb_touch_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_rgb_touch_visualizer, viz = " << viz << vcl_endl;
				}
				rgb_touch_viz_ = viz; 
			}

      void set_thermal_touch_visualizer(gevxl::img::visualizer_2d *viz) 
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_thermal_touch_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_thermal_touch_visualizer, viz = " << viz << vcl_endl;
				}
				thermal_touch_viz_ = viz; 
			}

      void set_hyper_spectral_touch_visualizer(gevxl::img::visualizer_2d *viz) 
			{ 
				if(verbose_ > 0) {
					vcl_cout << "pu_assessment_system_proc::set_hyper_spectral_touch_visualizer, program continues to this routine." << vcl_endl;
					vcl_cout << "pu_assessment_system_proc::set_hyper_spectral_touch_visualizer, viz = " << viz << vcl_endl;
				}
				hyperspectral_touch_viz_ = viz; 
			}

      // python GUI to set the file storage directories
      void set_rgb_file_directory(const vcl_string dir) { rgb_file_directory_ = dir; }

      void set_depth_file_directory(const vcl_string dir) { depth_file_directory_ = dir; }

      void set_thermal_file_directory(const vcl_string dir) { thermal_file_directory_ = dir; }
      
      void set_hyperspectral_file_directory(const vcl_string dir) { hyperspectral_file_directory_ = dir; }

      // python GUI to control the imaging data capture
      bool take_snapshot(int view_pt_id);

      bool take_hyperspectral_snapshot(void);
			
      // python GUI to control the imaging video data capture
      bool start_recording(int view_pt_id);
      bool stop_recording();

      bool playback_recording(int view_pt_id);

      // visualization function
			virtual void visualize(void);

      // get the current frame from the source process
      const vil_image_view<vxl_byte> &cur_frame(void) const { return viz_canvas_frame_; }

			// get the rgb-d camera source process
      gevxl::vid::generic_frame_process<vxl_byte> &get_rgbd_cam_source_process(void) { return rgbd_cam_source_proc_; }

			// get the thermal camera source process
      gevxl::vid::generic_frame_process<vxl_byte> &get_thermal_cam_source_process(void) { return thermal_cam_source_proc_; }

      // get the hyperspectral camera source process
      gevxl::vid::generic_frame_process<vxl_byte> &get_hyperspectral_cam_source_process(void) { return hyperspectral_cam_source_proc_; }

			//////////////////////////////////////////////////////////////////////////////////
			// the tissue analysis process 
			gevxl::pressure_ulcer::assessment::pu_tissue_analysis_proc *get_tissue_analysis_proc(void) { return &tissue_analysis_proc_; }
			void get_tissue_analysis_viz_offset(int &viz_offset_i, int &viz_offset_j);

			//////////////////////////////////////////////////////////////////////////////////
			// the 3d reconstruction process 
			gevxl::pressure_ulcer::assessment::pu_3d_recon_proc *get_3d_recon_proc(void) { return &threed_recon_proc_; }
			void get_3d_recon_viz_offset(int &viz_offset_i, int &viz_offset_j);

      //////////////////////////////////////////////////////////////////////////////////
			// the thermal analysis process 
			gevxl::pressure_ulcer::assessment::pu_thermal_analysis_proc *get_thermal_analysis_proc(void) { return &thermal_analysis_proc_; }
			void get_thermal_analysis_viz_offset(int &viz_offset_i, int &viz_offset_j);

		private:
			
      // prepare the visualization images for the externally set visualizers
      void prepare_viz_images(void);

      // rescale the source image to the target image size and store it to the target image.
      void rescale_image(const vil_image_view<vxl_byte> &source, vil_image_view<vxl_byte> &target);

      // generate the depth visualization image
      void generate_depth_viz_image(void);

      // update the rgb visualization image image based on the distance measure
      void update_rgb_viz_image(void);
      
			// camera sources
			
			// rgb-d camera
			gevxl::vid::generic_frame_process<vxl_byte> rgbd_cam_source_proc_;
			// rgb-d camera output type
      vcl_string rgbd_cam_source_output_type_;
			gevxl::framework::async_wrapping_process *async_rgbd_cam_source_proc_;

			// thermal camera
			gevxl::vid::generic_frame_process<vxl_byte> thermal_cam_source_proc_;
			// thermal camera output type
			vcl_string thermal_cam_source_output_type_;
			gevxl::framework::async_wrapping_process *async_thermal_cam_source_proc_;

			// hyper-spectral camera
			gevxl::vid::generic_frame_process<vxl_byte> hyperspectral_cam_source_proc_;
      // hyperspectral camera output type
			vcl_string hyperspectral_cam_source_output_type_;
			gevxl::framework::async_wrapping_process *async_hyperspectral_cam_source_proc_;

			// the overall viz_canvas_frame_;
			int viz_canvas_width_;
			int viz_canvas_height_;
			vil_image_view<vxl_byte> viz_canvas_frame_;

			// visualization canvas offset for difference processes
			int tissue_analysis_viz_offset_i_;
			int tissue_analysis_viz_offset_j_;

			int threed_recon_viz_offset_i_;
			int threed_recon_viz_offset_j_;

			int thermal_analysis_viz_offset_i_;
			int thermal_analysis_viz_offset_j_;
			
			// frame number
			int frame_nr_;

      // high resolution timer
      gevxl::util::time::highres_timer highres_timer_; 

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

      // external set visualizers
			gevxl::img::visualizer_2d *rgb_viz_;
			gevxl::img::visualizer_2d *depth_viz_;
			gevxl::img::visualizer_2d *thermal_viz_;

      gevxl::img::visualizer_2d *rgb_copy_viz_;
      gevxl::img::visualizer_2d *hyperspectral_viz_;

      gevxl::img::visualizer_2d *rgb_touch_viz_;
      gevxl::img::visualizer_2d *thermal_touch_viz_;
      gevxl::img::visualizer_2d *hyperspectral_touch_viz_;

      // the visualization images
      int rgb_viz_height_;
      int rgb_viz_width_;
      vil_image_view<vxl_byte> rgb_viz_image_;
      
      int depth_viz_height_;
      int depth_viz_width_;
      vil_image_view<vxl_byte> depth_viz_image_;
      
      int thermal_viz_height_;
      int thermal_viz_width_;
      vil_image_view<vxl_byte> thermal_viz_image_;
      
      int hyperspectral_viz_height_;
      int hyperspectral_viz_width_;
      vil_image_view<vxl_byte> hyperspectral_viz_image_;

      int thermal_touch_viz_height_;
      int thermal_touch_viz_width_;
      vil_image_view<vxl_byte> thermal_touch_viz_image_;
      
      int hyperspectral_touch_viz_height_;
      int hyperspectral_touch_viz_width_;
      vil_image_view<vxl_byte> hyperspectral_touch_viz_image_;

      // the visualization assistive methods
      vcl_string viz_depth_or_color_in_depth_;
      double optimal_snapshot_distance_;
      double optimal_snapshot_distance_tolerance_;
      double current_distance_;
      int distance_rgb_viz_[3]; // r, g, b
      int max_color_offset_;
      gevxl::util::rectangle<int> viz_inner_rect_;
      gevxl::util::rectangle<int> viz_outer_rect_;

      gevxl::util::rectangle<int> depth_in_rgb_viz_inner_rect_;
      gevxl::util::rectangle<int> depth_in_rgb_viz_outer_rect_;

      gevxl::util::rectangle<int> hyperspectral_in_rgb_viz_inner_rect_;
      gevxl::util::rectangle<int> hyperspectral_in_rgb_viz_outer_rect_;

      // imaging data storage directories.
      vcl_string rgb_file_directory_;
      vcl_string depth_file_directory_;
      vcl_string thermal_file_directory_;
      vcl_string hyperspectral_file_directory_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;

			// the tissue analysis process
			gevxl::pressure_ulcer::assessment::pu_tissue_analysis_proc tissue_analysis_proc_;

			// the 3d reconstruction process
			gevxl::pressure_ulcer::assessment::pu_3d_recon_proc threed_recon_proc_;

      // the thermal analysis process
			gevxl::pressure_ulcer::assessment::pu_thermal_analysis_proc thermal_analysis_proc_;

			// debugging verbose_ level
			int verbose_;

		};  // end of class pu_assessment_system_proc

		typedef vbl_shared_pointer<pu_assessment_system_proc> pu_assessment_system_proc_sptr;

		} // end of assessment namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif

