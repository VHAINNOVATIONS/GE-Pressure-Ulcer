// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/25/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_camera_source_proc_h
#define gevxl_pressure_ulcer_pu_camera_source_proc_h

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

namespace gevxl {
	namespace pressure_ulcer {

		class pu_camera_source_proc : public gevxl::framework::process,
														      public gevxl::util::on_off_mixin
		{
		public:

			// constructor
      pu_camera_source_proc(char const *name="gevxl::pressure_ulcer::pu_camera_source_proc"); 

			// destructor
			virtual ~pu_camera_source_proc(void); 

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

			// visualization function
			virtual void visualize(void);

      // get the current frame from the source process
      const vil_image_view<vxl_byte> &cur_frame(void) const { return source_proc_.cur_frame(); }

      // get the source process type
      vcl_string get_source_process_type(void) { return source_process_type_; }

      // get the source process
      gevxl::vid::generic_frame_process<vxl_byte> &get_source_process(void) { return source_proc_; }
      
		private:
						
      // input source
			gevxl::vid::generic_frame_process<vxl_byte> source_proc_;

      // frame tag process
      vid::code_stamp_frame_tag_process frame_tag_proc_;
			
      // frame nr
      int frame_nr_;

      // current and previous frames
			vil_image_view<vxl_byte> curr_frame_;
			vil_image_view<vxl_byte> prev_frame_;

      // high resolution timer
      gevxl::util::time::highres_timer highres_timer_; 

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;

      // input source process type
      vcl_string source_process_type_;

      // source process's output type
      vcl_string source_output_type_;

		};  // end of class pu_camera_source_proc

		typedef vbl_shared_pointer<pu_camera_source_proc> pu_camera_source_proc_sptr;

	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif

