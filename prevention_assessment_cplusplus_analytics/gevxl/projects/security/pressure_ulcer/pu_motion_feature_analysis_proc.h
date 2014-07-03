// Copyright (C) 2013 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 11/07/2013
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_motion_feature_analysis_proc_h
#define gevxl_pressure_ulcer_pu_motion_feature_analysis_proc_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>
#include <framework/async_wrapping_process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>
#include <vid/generic_writer_process.h>
#include <vid/code_stamp_frame_tag_process.h>
#include <vid/io/ffmpeg_writer.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <threading/mutex.h>

#include <fgbg/fgbg_seg_motion_extraction_proc.h>

namespace gevxl {
	namespace pressure_ulcer {

		class pu_motion_feature_analysis_proc : public gevxl::framework::process,
														                public gevxl::util::on_off_mixin
		{
		public:

			// Constructor
      pu_motion_feature_analysis_proc(char const *name="gevxl::pressure_ulcer::pu_motion_feature_analysis_proc"); 

			// Destructor
			virtual ~pu_motion_feature_analysis_proc(void); 

			// Configure this proc
			virtual bool configure(gevxl::util::config_file &config);

			// Initialize the process
			virtual bool initialize(void);

			// Uninitialize the process
			virtual void uninitialize(void);

			// The main step function
			virtual bool step(void);

			// To make this work without direct connection to an frame_process,
			// we can make a second call where an image has already been acquired from somewhere else.
			virtual bool step(vil_image_view<vxl_byte> img);

			//: set the process's visualizer
			virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

			// visualization function
			virtual void visualize(void);

      // get the current frame from the source process
      const vil_image_view<vxl_byte> &cur_frame(void) const { return source_proc_.cur_frame(); }

		private:
						
      // Input Source
			gevxl::vid::generic_frame_process<vxl_byte> source_proc_;

      // Output Writer
      gevxl::vid::generic_writer_process<vxl_byte> writer_proc_;

      // Frame tag process
      vid::code_stamp_frame_tag_process frame_tag_proc_;
			
      // Spatio-temporal filtering process
      fgbg::fgbg_seg_motion_extraction_proc fgbg_seg_motion_extraction_proc_;

      // Spatio-temporal filtering related params
      vcl_vector< vnl_vector<unsigned> > sal_pts_;
      float thresh_sal_frac_;
      float thresh_sal_;

      int frame_nr_;

			vil_image_view<vxl_byte> curr_frame_;
			vil_image_view<vxl_byte> prev_frame_;

      // high resolution timer
      gevxl::util::time::highres_timer highres_timer_; 

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

			// visualizer image
			gevxl::img::visualizer_image *viz_img_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;
		
			// visualizer image
			bool visualize_to_image_;
			vil_image_view<vxl_byte> viz_img_output_;			

			vcl_string viz_video_out_filename_;
			bool viz_video_out_openned_;
			vid::io::ffmpeg_writer viz_video_out_;

		};  // end of class pu_motion_feature_analysis_proc

		typedef vbl_shared_pointer<pu_motion_feature_analysis_proc> pu_motion_feature_analysis_proc_sptr;

	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif

