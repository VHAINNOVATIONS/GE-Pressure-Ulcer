// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 02/27/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_prevention_videoarchive_writer_process_h_
#define gevxl_pressure_ulcer_pu_prevention_videoarchive_writer_process_h_

#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>
#include <vid/generic_frame_process.h>
#include <vid/tag_source.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <vid/io/image_list_writer.h>
#include <vcl_iostream.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {
  
			class pu_prevention_videoarchive_writer_process : public gevxl::framework::process,
																												public gevxl::util::tag_source,
																												public gevxl::util::on_off_mixin
																												
			{
			public:

				// Constructor
				pu_prevention_videoarchive_writer_process(char const *name="gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_writer_process"); 

				// Destructor
				virtual ~pu_prevention_videoarchive_writer_process(void); 

				// Configure this proc
				virtual bool configure(gevxl::util::config_file &config);

				// Initialize the process
				virtual bool initialize(void);

				// Uninitialize the process
				virtual void uninitialize(void);

				// The main step function
				virtual bool step(void);
	      
				// connect to a generic frame process. We will save out video.
				void set_source_frame_process(const gevxl::vid::generic_frame_process<vxl_byte> *source) { source_ = source; }

				// connection to a tag source.
				void set_tag_source(const gevxl::vid::tag_source *t) { tag_source_ = t; }

				// get the current frame tag
				virtual const gevxl::vid::frame_tag &cur_frame_tag(void) const { return tag_; }

        // start recording
        bool start_recording(const vcl_string &folder);

        // stop recording
        bool stop_recording(void);

			private:
				
				// private methods
				// create a subfolder
				bool create_subfolder(const vcl_string &subfolder);

				// open the depth_img_seq_writer_ based on the current subfolder path
				bool open(const vcl_string &subfolder);

				// close the depth_img_seq_writer_ based on the current subfolder path
				void close(void);

				// process that serves frames.
				const gevxl::vid::generic_frame_process<vxl_byte> *source_;

				// tag source.
				const gevxl::vid::tag_source *tag_source_;

				// copy of config file.
				gevxl::util::config_file config_;

				// the writer process maintains a tag for the outgoing video. If desired,
				// this process can also be used as a tag source.
				gevxl::vid::frame_tag tag_;

				// image_list_writer as the depth image sequence writer
				gevxl::vid::io::image_list_writer<vxl_uint_16> depth_img_seq_writer_;

				// ofstream to write out the frame tag info;
				vcl_ofstream tag_ofstream_;

				// is writer openned that associate with the current subfolder
				bool writer_openned_;

				// subfolder id
				int subfolder_id_;

				// the time code of the first frame in the current subfolder
				double time_code_of_the_first_frame_in_subfolder_;

				// configurable parameters
				// depth image sequence 
				vcl_string depth_img_seq_root_folder_;

				// depth image sequence filename pattern
				vcl_string depth_img_seq_filename_pattern_;

				// time window in millisecond for the frames placed in one subfolder
				double time_window_in_millisecond_in_subfolder_;

        // stop recording flag
        bool stop_recording_flag_;

			};  // end of class pu_prevention_videoarchive_writer_process

			typedef vbl_shared_pointer<pu_prevention_videoarchive_writer_process> pu_prevention_videoarchive_writer_process_sptr;

		} // end of prevention namespace
  } //end namespace pressure_ulcer
} //end namespace gevxl

#endif
