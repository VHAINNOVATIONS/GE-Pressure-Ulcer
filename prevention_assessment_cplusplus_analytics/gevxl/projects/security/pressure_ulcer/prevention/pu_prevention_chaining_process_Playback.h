// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef gevxl_pressure_ulcer_pu_prevention_chaining_process_h
#define gevxl_pressure_ulcer_pu_prevention_chaining_process_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>
#include <vid/code_stamp_frame_tag_process.h>
#include <vid/io/ffmpeg_writer.h>

#include <pressure_ulcer/prevention/pu_prevention_videoarchive_frame_process.h>
#include <pressure_ulcer/prevention/pu_prevention_videoarchive_writer_process.h>

#include <pressure_ulcer/prevention/pu_prv_rectify_kinect_process.h>
#include <pressure_ulcer/prevention/pu_prv_motion_estimate_process.h>

#include <pressure_ulcer/prevention/pu_prevention_database_writer.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

		class pu_prevention_chaining_process : public gevxl::framework::process,
																					 public gevxl::util::on_off_mixin
		{
		public:

			// constructor
			pu_prevention_chaining_process(char const *name="gevxl::pressure_ulcer::prevention::pu_prevention_chaining_process");

			// destructor
			virtual ~pu_prevention_chaining_process(void); 

			// configure this proc
			virtual bool configure(gevxl::util::config_file &config);

			// initialize the process
			virtual bool initialize(void);

			// uninitialize the process
			virtual void uninitialize(void);

			// the main step function
			virtual bool step(void);

			// set the process's visualizer
			virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

			// visualization function
			virtual void visualize(void);

      // get the current frame from the source process
      //const vil_image_view<vxl_byte> &cur_frame(void) const { return viz_canvas_frame_; }
      const vil_image_view<vxl_byte> &cur_frame(void) const;

			// get the generic source frame process
			gevxl::vid::generic_frame_process<vxl_byte> &get_generic_frame_process(void) { return source_proc_; }

			// get the frame tag process
			gevxl::vid::code_stamp_frame_tag_process &get_code_stamp_frame_tag_process(void) { return frame_tag_proc_; }

      // get the kinect depth rectification process
      gevxl::pressure_ulcer::prevention::pu_prv_rectify_kinect_process *get_rectify_kinect_process(void) { return &rectify_kinect_proc_; }

      // get the motion estimate process
      gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process *get_motion_estimate_process(void) { return &motion_estimate_proc_; }

      // Python interface methods
      bool start_recording(const vcl_string folder);

      bool stop_recording(void);

      void set_database_writer(gevxl::pressure_ulcer::prevention::pu_prevention_database_writer *writer)
      {
        database_writer_ = writer;
      }

		private:
						
      // ensure the avi files have been openned for the ffmpeg writing out
      void ensure_avi_files_openned(const vil_image_view<vxl_byte> &rgb_img, const vil_image_view<vxl_byte> &depth_img);

      // ffmpeg write out 
      void write_out_frames_to_avi_files(const vil_image_view<vxl_byte> &rgb_img, const vil_image_view<vxl_byte> &depth_img);

			// cur frame output type
      vcl_string cur_frame_output_type_;

			// frame number
			int frame_nr_;

      // high resolution timer
      gevxl::util::time::highres_timer highres_timer_; 

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

			// visualizer image
      gevxl::img::visualizer_image *viz_img_;

			// avi video output writers
			int avi_fps_;

			// avi writer for the raw rgb video
			vcl_string rgb_video_out_filename_;
      bool rgb_video_out_openned_;
      gevxl::vid::io::ffmpeg_writer rgb_video_out_;

			// avi writer for the raw depth video
			vcl_string depth_video_out_filename_;
      bool depth_video_out_openned_;
      gevxl::vid::io::ffmpeg_writer depth_video_out_;

			// avi writer for the height_filtered video
      vcl_string height_filtered_video_out_filename_;
      bool height_filtered_video_out_openned_;
      gevxl::vid::io::ffmpeg_writer height_filtered_video_out_;

			// avi writer for the depth_filtered video
			vcl_string depth_filtered_video_out_filename_;
      bool depth_filtered_video_out_openned_;
      gevxl::vid::io::ffmpeg_writer depth_filtered_video_out_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;

			// input source
			gevxl::vid::generic_frame_process<vxl_byte> source_proc_;

			// pu_prevention_videoarchive_frame_process
			gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_frame_process videoarchive_frame_proc_;

			// pu_prevention_videoarchive_writer_process
			gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_writer_process videoarchive_writer_proc_;

			// frame tag process
			gevxl::vid::code_stamp_frame_tag_process frame_tag_proc_;

			// rectify Kinect process
      pu_prv_rectify_kinect_process rectify_kinect_proc_;

      // motion estimate process
      pu_prv_motion_estimate_process motion_estimate_proc_;

      // the database writer
      gevxl::pressure_ulcer::prevention::pu_prevention_database_writer *database_writer_;

      // a flag to indicate the current process is configured to be running as live mode or playback mode
      vcl_string running_mode_;

			// debugging verbose_ level
			int verbose_;
		};  // end of class pu_prevention_chaining_process

		typedef vbl_shared_pointer<pu_prevention_chaining_process> pu_prevention_chaining_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif
