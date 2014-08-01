// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef gevxl_pressure_ulcer_pu_prevention_person_detect_process_h
#define gevxl_pressure_ulcer_pu_prevention_person_detect_process_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <vid/frame_process.h>

#include <threading/thread.h>
#include <threading/mutex.h>

#include <vnl/vnl_double_3x4.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_point_2d.h>

#include <vnl/vnl_int_2.h>

#include <vsl/vsl_binary_io.h>
#include <vsl/vsl_vector_io.h>
#include <vsl/vsl_block_binary.h>

#include <vid/frame_tag.h>

// a process to peform person detection from the rectified kinect view, i.e., the process is to operate 
// on the point cloud data image, where each pixel encodes the (x, y, z), where the z value is the distance
// of the pixel to the ground plane. The process partition the (x,y,z) space into some grid-based representation
// and count how many pixels are binned to each of the grid, and then counting each person detection hypotheses
// how many foreground grids (with enough point cloud points resided) in its hypothesis 
// rectangular (x,y,z) box are including.

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

      struct pu_prevention_person_detect_hypothesis {
        
        // the groundplane center of the person standing
        vgl_point_2d<double> gp_center_pt_;
        // the groundplane bounding box of the person body
        gevxl::util::rectangle<double> gp_bounding_box_;

        // the image pixel position of the groundplane center
        vnl_int_2 gp_center_img_pt_;

        // the 8 cuboid corners of the person body in world 3d
        vcl_vector<vgl_point_3d<double> > cuboid_3d_corners_;
        // the 8 cuboid corners of the person body in image 2d
        vcl_vector<vnl_int_2> cuboid_2d_corners_;

        // the hypothesis score
        double score_;
      };

		class pu_prevention_person_detect_process : public gevxl::framework::process,
                                                public gevxl::threading::thread,
																					      public gevxl::util::on_off_mixin
		{
		public:

			// constructor
			pu_prevention_person_detect_process(char const *name="gevxl::pressure_ulcer::prevention::pu_prevention_person_detect_process");

			// destructor
			virtual ~pu_prevention_person_detect_process(void); 

			// configure this proc
			virtual bool configure(gevxl::util::config_file &config);

			// initialize the process
			virtual bool initialize(void);

			// uninitialize the process
			virtual void uninitialize(void);

      // the main step function
			virtual bool step(void) { vcl_cerr << "Use step(image)" << vcl_endl; return false; }

      // the main step function
			virtual bool step(const gevxl::vid::frame_tag &tag,                       // the frame tag
                        const vil_image_view<vxl_uint_16> &raw_depth_frame,     // the raw depth frame
                        const vil_image_view<float> &rectified_xyz_frame);      // the rectified xyz frame

			// set the process's visualizer
			virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

			// visualization function
			virtual void visualize(void);

			// visualization overlay function
			virtual void visualize_overlay(void);

      // get the current frame from the source process
      const vil_image_view<vxl_byte> &cur_frame(void) const;

      // get the persons_detected_ flag
      bool get_persons_detected_flag(void) { return persons_detected_; }

			// get the person detection time window count
			void get_persons_detection_time_window_count(double &total_persons_detected_flag_count, 
																									 double &total_time_elapse_count,
																									 double &nr_of_no_persons_detected_frames);

			void reset_persons_detection_time_window_count(void);

		private:
	    
      // start the thread
      void run_thread(void);

      // initialize the various precomputations
      bool initialize_precomputations(const vil_image_view<vxl_uint_16> &raw_depth_frame,
                                      const vil_image_view<float> &rectified_xyz_frame);

      // perform the person detection 
      bool detect_persons(const vil_image_view<vxl_uint_16> &raw_depth_frame,
                          const vil_image_view<float> &rectified_xyz_frame);

			/// Deserialize from a stream.
			bool vsl_binary_read( vsl_b_istream &bistr );

			/// Serialize to a stream.
			void vsl_binary_write( vsl_b_ostream &bostr );

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;

      // a busy mutex lock
      bool is_process_busy_;
      gevxl::threading::mutex process_busy_lock_;

			// debugging verbose_ level
			int verbose_;

      // are all precomputations initialized
      bool precomp_initialized_;

      // camera projection matrix based on a few image pixels (u,v) to their (x,y,z) coordinates correspondence
      vnl_double_3x4 proj_mat_;

      // the precomputed hypotheses
      vcl_vector<pu_prevention_person_detect_hypothesis> hypotheses_;

      // flag to indicate whether the persons are detected
      bool persons_detected_;

      // configurable parameters
      // the minimum spatial distance between two detection hypotheses on the ground plane
      double min_spatial_distance_thresh_;

      // the half size of person body for one detection hypothesis
      double radius_of_person_body_;

      // the cuboid height for person detection hypothesis
      double height_of_person_body_;

      // the ground plane z value tolerance
      double gp_z_tolerance_thresh_;

      // the number of randomly sampled correspondence positions for camera projection calibration
      int num_of_random_samples_for_calibration_;

      // the height threshold for the person detection
      double person_detection_height_thresh_;

      // the score threshold for the person detection
      double person_detection_score_thresh_;

			// the person detection time window count
			double total_persons_detected_flag_count_;
			double total_time_elapse_count_;

			double nr_of_no_persons_detected_frames_;

			// the binary serialized file of the pre-computed hypotheses
			vcl_string precomputed_hypotheses_full_filename_;

		};  // end of class pu_prevention_person_detect_process

		typedef vbl_shared_pointer<pu_prevention_person_detect_process> pu_prevention_person_detect_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif
