// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 02/27/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_prevention_videoarchive_frame_process_h_
#define gevxl_pressure_ulcer_pu_prevention_videoarchive_frame_process_h_

#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>
#include <vid/tagged_frame_process.h>
#include <vid/frame_tag.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <vnl/vnl_double_2.h>
#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_matrix_fixed.h>

//#include <vid/io/image_list_reader.h>
#include <vcl_iostream.h>
#include <vcl_deque.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {
  
			class pu_prevention_videoarchive_frame_process : public gevxl::vid::tagged_frame_process<vxl_byte>,																											 
																											 public gevxl::util::on_off_mixin
																												
			{
			public:

				// Constructor
				pu_prevention_videoarchive_frame_process(char const *name="gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_frame_process"); 

				// Destructor
				virtual ~pu_prevention_videoarchive_frame_process(void); 

				// Configure this proc
				virtual bool configure(gevxl::util::config_file &config);

				// Initialize the process
				virtual bool initialize(void);

				// Uninitialize the process
				virtual void uninitialize(void);

				// The main step function
				virtual bool step(void);

        /// Return the current frame
        vil_image_view<vxl_byte> const &cur_frame() const { return depth_byte_frame_; }
	      
				// get the current frame tag
				virtual const gevxl::vid::frame_tag &cur_frame_tag(void) const { return tag_; }

        /// Return RGB frame
        vil_image_view<vxl_byte> const &cur_rgb_frame() const { return rgb_frame_; } 

        /// Return depth frame
        vil_image_view<vxl_uint_16> const &cur_depth_frame() const { return depth_frame_; }

        /// Return depth frame in 8-bit byte
        vil_image_view<vxl_byte> const &cur_depth_byte_frame() const { return depth_byte_frame_; }

        /// Return xyz_rgb frame in float byte
	      bool is_point_cloud_frame_available(void) const { return is_point_cloud_frame_available_; }
        vil_image_view<float> const &cur_xyz_rgb_frame() const { return xyz_rgb_frame_; }

        // start playback
        bool start_playback(const vcl_string &folder);

        // stop playback
        bool stop_playback(void);

			private:

        // copy of config file.
				gevxl::util::config_file config_;

				// the writer process maintains a tag for the outgoing video. If desired,
				// this process can also be used as a tag source.
				gevxl::vid::frame_tag tag_;

        /// Current RGB frame, assumes the black frame for the prevention system
        vil_image_view<vxl_byte> rgb_frame_;

        /// Current depth frame
        vil_image_view<vxl_uint_16> depth_frame_;
        vil_image_view<vxl_byte> depth_byte_frame_;

        /// frame number
        unsigned frame_nr_;

        // the point cloud frame available flag
        bool is_point_cloud_frame_available_;

        vnl_matrix_fixed<double, 3, 3> cam_K_;
	      vnl_matrix_fixed<double, 2, 2> cam_invK22_;

        /// current xyz_rgb frame
        vil_image_view<float> xyz_rgb_frame_;

	      /// pre-computed frame for computing the internal (x,y) frame from the camera intrinsic parameters
	      vil_image_view<float> pre_computed_xy_frame_;
	      bool is_xy_frame_pre_computed_;

        // configurable parameters
				// depth image sequence 
				vcl_string depth_img_seq_root_folder_;

        // depth image sequence filename pattern
				vcl_string depth_img_seq_filename_pattern_;

        // subfolder id
				int subfolder_id_;

        // frame tags loaded from the subfolder
        vcl_deque<gevxl::vid::frame_tag> subfolder_loaded_frame_tags_;

        // frame filenames from the subfolder
	      vcl_deque<vcl_string> subfolder_frame_filenames_;

			};  // end of class pu_prevention_videoarchive_frame_process

			typedef vbl_shared_pointer<pu_prevention_videoarchive_frame_process> pu_prevention_videoarchive_frame_process_sptr;

		} // end of prevention namespace
  } //end namespace pressure_ulcer
} //end namespace gevxl

#endif
