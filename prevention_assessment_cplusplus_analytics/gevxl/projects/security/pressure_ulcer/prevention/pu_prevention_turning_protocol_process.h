// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef gevxl_pressure_ulcer_pu_prevention_turning_protocol_process_h
#define gevxl_pressure_ulcer_pu_prevention_turning_protocol_process_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <vid/frame_process.h>

#include <vgl/vgl_point_3d.h>

#include <pressure_ulcer/prevention/pu_prv_motion_estimate_process.h>
#include <pressure_ulcer/prevention/pu_prevention_person_detect_process.h>
#include <pressure_ulcer/prevention/pu_prevention_pose_estimate_process.h>

#include <vid/frame_tag.h>
#include <threading/mutex.h>

// a process to monitor the patient turning protocol, and under the hood the logic is as follows:
// the patient turning protocol process internally maintains a few individual running analytics process
// the patient pose estimate process, the patient body motion estimate process, and the person detection process
// so the reasoning logic is that whenever the patient pose estimate process gets invoked (not necessarily at the frame rate)
// and found out that the patient pose has been changed from the last invoke, and simultaneously 
// the patient body motion estimate process finds out that there were significant motion happened in-between
// these two patient pose estimate change, this is considered as a patient turning event. 
// In addition, the person detection process validates that within this patient turning time window, 
// whether there is anyone nearby, which can indicate there is a caregiver assistance during the 
// patient turning event.

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

		class pu_prevention_turning_protocol_process : public gevxl::framework::process,
																					         public gevxl::util::on_off_mixin
		{
		public:

			// constructor
			pu_prevention_turning_protocol_process(char const *name="gevxl::pressure_ulcer::prevention::pu_prevention_turning_protocol_process");

			// destructor
			virtual ~pu_prevention_turning_protocol_process(void); 

			// configure this proc
			virtual bool configure(gevxl::util::config_file &config);

			// initialize the process
			virtual bool initialize(void);

			// uninitialize the process
			virtual void uninitialize(void);

			// the main step function
			virtual bool step(void) { vcl_cerr << "Use the other step function" << vcl_endl; return false; }

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

      // get the persons_detected_ flag
      bool get_persons_detected_flag(void) { return person_detection_sptr_->get_persons_detected_flag(); }

      // get the pose_changed_ flag with the current pose description
      bool get_pose_changed_flag(vcl_string &current_pose) { return pose_estimate_sptr_->get_pose_changed_flag(current_pose); }

      // get the body_motion_detected_ flag
      bool get_body_motion_detected_flag(void) { return motion_estimate_sptr_->get_body_motion_detected_flag(); }

			// get the motion estimate process sptr
			gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process_sptr get_motion_estimate_process_sptr(void) { return motion_estimate_sptr_; }

			// get the pose estimate process sptr
			gevxl::pressure_ulcer::prevention::pu_prevention_pose_estimate_process_sptr get_pose_estimate_process_sptr(void) { return pose_estimate_sptr_; }

			// get the person detection process sptr
			gevxl::pressure_ulcer::prevention::pu_prevention_person_detect_process_sptr get_person_detect_process_sptr(void) { return person_detection_sptr_; }

			// extract all recently detected turning events from the list
			void extract_turning_events(vcl_vector<vcl_string> &event_strs);

		private:

			// push newly detected turning event into the list
			void push_turning_event(int pose_estimate_label, bool caregiver_assisted_flag, double time_code);

      // visualizer 	
			gevxl::img::visualizer_2d *viz_;

			// the configuration file that the generator needs in order to configure its own parameters.
			gevxl::util::config_file config_;

      // the internally used analytics processes
      // body motion estimate
      gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process_sptr motion_estimate_sptr_;
      // pose estimate process, should let this run in the asynchronous process
      gevxl::pressure_ulcer::prevention::pu_prevention_pose_estimate_process_sptr pose_estimate_sptr_;
      // person detection process, could be run in the asynchronous or synchronous modes
      gevxl::pressure_ulcer::prevention::pu_prevention_person_detect_process_sptr person_detection_sptr_;

			// debugging verbose_ level
			int verbose_;

      // the last turning protocol check time
      double last_protocol_check_time_;

			// the last patient pose estimate from the last turning protocol check time
			int last_pose_estimate_label_;

      // configurable parameters
      // the time elapse between two consecutive protocol status checking
      double protocol_check_time_elapse_in_min_;
      double protocol_check_time_elapse_in_millisec_;

			bool patient_turned_flag_;
			bool caregiver_assisted_flag_;

			int patient_turned_flag_viz_count_;

			double nr_of_no_motion_detected_frames_thresh_;
			double nr_of_no_persons_detected_frames_thresh_;

			double total_motion_detected_flag_count_thresh_;
			double total_persons_detected_flag_count_thresh_;

			int patient_turned_flag_viz_count_thresh_;

			// the detected turning events
			gevxl::threading::mutex turning_events_lock_;
			vcl_vector<vcl_string> turning_events_;

		};  // end of class pu_prevention_turning_protocol_process

		typedef vbl_shared_pointer<pu_prevention_turning_protocol_process> pu_prevention_turning_protocol_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif
