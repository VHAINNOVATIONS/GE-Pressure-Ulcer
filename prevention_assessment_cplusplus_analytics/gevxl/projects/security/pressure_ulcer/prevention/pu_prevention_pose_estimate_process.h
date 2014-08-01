// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef gevxl_pressure_ulcer_pu_prevention_pose_estimate_process_h
#define gevxl_pressure_ulcer_pu_prevention_pose_estimate_process_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <vid/frame_process.h>
#include <vid/frame_tag.h>

#include <threading/thread.h>
#include <threading/mutex.h>

#include <vgl/vgl_point_3d.h>

#include <vnl/vnl_double_2.h>
#include <vnl/vnl_float_3x3.h>

#include <vcl_iostream.h>

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include <pressure_ulcer/prevention/pu_prv_motion_estimate_process.h>
#include <pressure_ulcer/prevention/pu_prevention_person_detect_process.h>

// a process to detect the particular body pose based on the rectified (x,y,z) point cloud image
// the analytics logic flow is as follows: perform the (x,y,z) point cloud analysis to separate the
// height(z) image into different layers, ground plane, bed plane, and body plane;
// based on these height layers, segment out the body silouette against the ground and bed planes
// extract the contour out of the body silouette segmentation
// classify the contour into different body pose.

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

    enum body_pose{left_side, right_side, back_side, face_side, empty, num_positions};
    const vcl_string body_pose_str[] = {"left_side", "right_side", "back_side", "face_side", "empty", "num_positions"};
		const vcl_string body_pose_str2[] = {"Left Side", "Right Side", "Back", "Stomach", "empty", "num_positions"};

		class pu_prevention_pose_estimate_process : public gevxl::framework::process,
                                                public gevxl::threading::thread,
																					      public gevxl::util::on_off_mixin
		{
		public:

			// constructor
			pu_prevention_pose_estimate_process(char const *name="gevxl::pressure_ulcer::prevention::pu_prevention_pose_estimate_process");

			// destructor
			virtual ~pu_prevention_pose_estimate_process(void); 

			// configure this proc
			virtual bool configure(gevxl::util::config_file &config);

			// initialize the process
			virtual bool initialize(void);

			// uninitialize the process
			virtual void uninitialize(void);

			// the main step function
			virtual bool step(void) { vcl_cerr << "Use step(image)" << vcl_endl; return false; }

      // the main step function
			virtual bool step(const gevxl::vid::frame_tag &tag,                     // the frame tag
                        const vil_image_view<vxl_uint_16> &raw_depth_frame,   // the raw depth frame
                        const vil_image_view<vxl_byte> &filtered_depth_frame, // the filtered depth frame
                        const vil_image_view<float> &rectified_xyz_frame,     // the rectified xyz frame
                        const vgl_point_3d<float>  &origin,                   // origin
                        const vgl_vector_3d<float> &vx,                       // orthonormal basis vector vx
                        const vgl_vector_3d<float> &vy,                       // orthonormal basis vector vy
                        const vgl_vector_3d<float> &vz);                      // orthonormal basis vector vz 

			// set the process's visualizer
			virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

			// visualization function
			virtual void visualize(void);

			// visualization overlay function
			virtual void visualize_overlay(void);

      // get the current frame from the source process
      const vil_image_view<vxl_byte> &cur_frame(void) const;

      // get the pose_changed_ flag with the current pose description
      bool get_pose_changed_flag(vcl_string &current_pose);

			// set the current pose sample labeling
			void set_pose_sample_label(const vcl_string &label);

			// set the motion estimate process
			void set_pu_prv_motion_estimate_process(gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process_sptr motion_estimate_proc) { motion_estimate_proc_sptr_ = motion_estimate_proc; }

			// set the person detection process
			void set_pu_prevention_person_detect_process(gevxl::pressure_ulcer::prevention::pu_prevention_person_detect_process_sptr person_detect_proc) { person_detect_proc_sptr_ = person_detect_proc; }

			// reset the majority voting based final pose estimate decision making
			void reset_classifier_labels_count(void) { classifier_labels_count_.resize(body_pose::num_positions, 0); }

			// get the majority voting based pose estimate
			int get_majority_voting_based_pose_estimate(void);

			// get the current pose estimate
			int get_current_pose_estimate(void) { return label_; }

		private:

      // start the thread
      void run_thread(void);

			// estimate the homography using the 4 corners of the ROI
			bool estimate_homography(const vcl_vector<vnl_double_2> &from_pts, const vcl_vector<vnl_double_2> &to_pts, vnl_float_3x3 &H);

			// create the homography map image, and constrain it within [x0 y0 x1 y1], and if the mapped coordinate is outside of this constraint, set the mapped value to (-1, -1)
			void create_homography_map(const vnl_float_3x3 &H, const int img_width, const int img_height, vil_image_view<int> &map);

			// train the pose classifiers with the loaded training feature data with the ground truth label
			void learn_pose_classifier(void);

      // load the previously trained classifiers
      bool load_pose_classifier(void);

			// the supporting processes
			gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process_sptr motion_estimate_proc_sptr_;
			gevxl::pressure_ulcer::prevention::pu_prevention_person_detect_process_sptr person_detect_proc_sptr_;

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;

      // a busy mutex lock
      bool is_process_busy_;
      gevxl::threading::mutex process_busy_lock_;

			// debugging verbose_ level
			int verbose_;

			// the roi quadrilateral defined in the input raw depth frame
			vcl_vector<vnl_double_2> roi_quadrilateral_;

			// the re-scaled rectangle defined in the canonical space
			int roi_canonical_rectangle_width_;
			int roi_canonical_rectangle_height_;

			// the inverse of the homography that maps the canonical rectangle to the roi quadrilateral
			vnl_float_3x3 inverse_homography_;

			// the inverse homography map
			vil_image_view<int> inverse_homography_map_;

			// the canonical view of the z frame, based on the height from the estimated ground plane
			vil_image_view<float> canonical_z_frame_;

			// the number of x (along the bed head direction) and y (along the bed side direction) grids 
			int num_of_x_grids_;
			int num_of_y_grids_;

			// the grid frame, based on the height from the estimated ground plane
			vil_image_view<float> grid_z_frame_;

			// the computed feature vector
			vcl_vector<float> z_diff_feature_;

			// the provided feature label
			int label_;
			vcl_string label_str_;

			// the video full filename used for collecting the training data
			vcl_string training_video_data_full_filename_;
			vcl_ofstream training_data_feature_label_ofstream_;

			// the pose label groundtruth data full filename
			vcl_string all_training_data_feature_label_full_filename_;
			vcl_ifstream all_training_data_feature_label_ifstream_;

			// classifiers' configurable parameters			
      vcl_string classifier_choice_;

			CvDTree decision_tree_classifier_;
			int decision_tree_max_depth_;
			int num_of_cross_validations_;
      vcl_string decision_tree_classifier_full_filename_;

			CvSVM svm_classifier_;
			int svm_C_;
      vcl_string svm_classifier_full_filename_;

			CvKNearest knearest_classifier_;
			int k_nearest_neighbor_;
      vcl_string knearest_classifier_full_filename_;

      vcl_vector<int> classifier_labels_count_;

		};  // end of class pu_prevention_pose_estimate_process

		typedef vbl_shared_pointer<pu_prevention_pose_estimate_process> pu_prevention_pose_estimate_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif
