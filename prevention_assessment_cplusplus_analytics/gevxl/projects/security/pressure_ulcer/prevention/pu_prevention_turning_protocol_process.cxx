// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pu_prevention_turning_protocol_process.h"

#include <vcl_iostream.h>

#include <vil/vil_save.h>
#include <vil/vil_load.h>

#include <util/time/global_time.h>

using namespace gevxl;
using namespace gevxl::util::time;
using namespace gevxl::pressure_ulcer::prevention;

pu_prevention_turning_protocol_process::pu_prevention_turning_protocol_process(char const *name)
: gevxl::framework::process(name), 
  viz_(NULL),
  verbose_(0),
  last_protocol_check_time_(-1),
	last_pose_estimate_label_(-1),
	patient_turned_flag_(false),
	caregiver_assisted_flag_(false),
	patient_turned_flag_viz_count_(0)
{
  enable(false);
	turning_events_.clear();
}

pu_prevention_turning_protocol_process::~pu_prevention_turning_protocol_process(void)
{

}

bool pu_prevention_turning_protocol_process::configure(util::config_file &config)
{
	config_ = config;

  bool enabled = false;
  config.get_bool(name()+"::enabled", enabled);
  enable(enabled);

	verbose_ = 0;
	config.get_integer(name()+"::verbose", verbose_);

  motion_estimate_sptr_ = pu_prv_motion_estimate_process_sptr(new pu_prv_motion_estimate_process());
  if( !motion_estimate_sptr_->configure(config) ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::configure(), error configuring motion_estimate_sptr_." << vcl_endl;
		return false;
  }

  pose_estimate_sptr_ = pu_prevention_pose_estimate_process_sptr(new pu_prevention_pose_estimate_process());
  if( !pose_estimate_sptr_->configure(config) ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::configure(), error configuring pose_estimate_sptr_." << vcl_endl;
		return false;
  }

  person_detection_sptr_ = pu_prevention_person_detect_process_sptr(new pu_prevention_person_detect_process());
  if( !person_detection_sptr_->configure(config) ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::configure(), error configuring person_detection_sptr_." << vcl_endl;
		return false;
  }

  // the time elapse between two consecutive protocol status checking
  protocol_check_time_elapse_in_min_ = 5; // perform protocol status update checking every five minute
  config.get_double(name()+"::protocol_check_time_elapse_in_min", protocol_check_time_elapse_in_min_);
  protocol_check_time_elapse_in_millisec_ = protocol_check_time_elapse_in_min_*60*1000;

	nr_of_no_motion_detected_frames_thresh_ = 3;
	config.get_double(name()+"::nr_of_no_motion_detected_frames_thresh", nr_of_no_motion_detected_frames_thresh_);

	nr_of_no_persons_detected_frames_thresh_ = 3;
	config.get_double(name()+"::nr_of_no_persons_detected_frames_thresh", nr_of_no_persons_detected_frames_thresh_);

	total_motion_detected_flag_count_thresh_ = 5;
	config.get_double(name()+"::total_motion_detected_flag_count_thresh", total_motion_detected_flag_count_thresh_);

	total_persons_detected_flag_count_thresh_ = 5;
	config.get_double(name()+"::total_persons_detected_flag_count_thresh", total_persons_detected_flag_count_thresh_);

	patient_turned_flag_viz_count_thresh_ = 100;
	config.get_integer(name()+"::patient_turned_flag_viz_count_thresh", patient_turned_flag_viz_count_thresh_);

	return true;
}

bool pu_prevention_turning_protocol_process::initialize(void)
{
	if(is_enabled() == false) return true;

  if( !motion_estimate_sptr_->initialize() ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::initialize(), error initializing motion_estimate_sptr_." << vcl_endl;
		return false;
  }

  if( !person_detection_sptr_->initialize() ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::initialize(), error initializing person_detection_sptr_." << vcl_endl;
		return false;
  }

	pose_estimate_sptr_->set_pu_prv_motion_estimate_process(motion_estimate_sptr_);
	pose_estimate_sptr_->set_pu_prevention_person_detect_process(person_detection_sptr_);
	if( !pose_estimate_sptr_->initialize() ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::initialize(), error initializing pose_estimate_sptr_." << vcl_endl;
		return false;
  }

  return true;
}

void pu_prevention_turning_protocol_process::uninitialize(void)
{
	if(is_enabled() == false) return;

  motion_estimate_sptr_->uninitialize();
  pose_estimate_sptr_->uninitialize();
  person_detection_sptr_->uninitialize();
}

bool pu_prevention_turning_protocol_process::step(
                        const gevxl::vid::frame_tag &tag,                     // the frame tag
                        const vil_image_view<vxl_uint_16> &raw_depth_frame,   // the raw depth frame
                        const vil_image_view<vxl_byte> &filtered_depth_frame, // the filtered depth frame
                        const vil_image_view<float> &rectified_xyz_frame,     // the rectified xyz frame
                        const vgl_point_3d<float>  &origin,                   // origin
                        const vgl_vector_3d<float> &vx,                       // orthonormal basis vector vx
                        const vgl_vector_3d<float> &vy,                       // orthonormal basis vector vy
                        const vgl_vector_3d<float> &vz)                       // orthonormal basis vector vz
{
	if(is_enabled() == false) return true;

  if( !motion_estimate_sptr_->step(tag, filtered_depth_frame) ) {
		vcl_cerr << "pu_prevention_turning_protocol_process::step(), Error executing motion_estimate_sptr_->step()." << vcl_endl;
		return false;
	}

  if( !person_detection_sptr_->step(tag, raw_depth_frame, rectified_xyz_frame) ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::step(), Error executing person_detection_sptr_->step()." << vcl_endl;
		return false;
  }

  if( !pose_estimate_sptr_->step(tag, raw_depth_frame, filtered_depth_frame, rectified_xyz_frame, origin, vx, vy, vz) ) {
    vcl_cerr << "pu_prevention_turning_protocol_process::step(), Error executing pose_estimate_sptr_->step()." << vcl_endl;
		return false;
  }

  if(last_protocol_check_time_ == -1) {
    last_protocol_check_time_ = tag.get_time_code();
    vcl_cout << "pu_prevention_turning_protocol_process::step, the turning protocol process is just starting." << vcl_endl;
    return true;
  }

  if(tag.get_time_code() - last_protocol_check_time_ < protocol_check_time_elapse_in_millisec_) {
    //vcl_cout << "pu_prevention_turning_protocol_process::step, the elapse time is not enough for the next protocol check." << vcl_endl;
    return true;
  }

  // start to check the protocol
	double total_motion_detected_flag_count, total_time_elapse_count, nr_of_no_motion_detected_frames;
	motion_estimate_sptr_->get_body_motion_detection_time_window_count(total_motion_detected_flag_count, total_time_elapse_count, nr_of_no_motion_detected_frames);

	double total_persons_detected_flag_count, nr_of_no_persons_detected_frames;
	person_detection_sptr_->get_persons_detection_time_window_count(total_persons_detected_flag_count, total_time_elapse_count, nr_of_no_persons_detected_frames);

	if(nr_of_no_motion_detected_frames < nr_of_no_motion_detected_frames_thresh_ || 
		 nr_of_no_persons_detected_frames < nr_of_no_persons_detected_frames_thresh_) {
		
		//vcl_cout << "pu_prevention_turning_protocol_process::step, this is not a good time frame to check the turning protocol." << vcl_endl;
    return true;
	}

	//if(total_motion_detected_flag_count > total_motion_detected_flag_count_thresh_ && pose_estimate_sptr_->get_majority_voting_based_pose_estimate() != last_pose_estimate_label_) {
	//	last_pose_estimate_label_ = pose_estimate_sptr_->get_majority_voting_based_pose_estimate();

	if(total_motion_detected_flag_count > total_motion_detected_flag_count_thresh_ && pose_estimate_sptr_->get_current_pose_estimate() != last_pose_estimate_label_) {
		last_pose_estimate_label_ = pose_estimate_sptr_->get_current_pose_estimate();

		patient_turned_flag_ = true;
		patient_turned_flag_viz_count_ = 0;

		if(total_persons_detected_flag_count > total_persons_detected_flag_count_thresh_) {
			caregiver_assisted_flag_ = true;		
		}
		else {
			caregiver_assisted_flag_ = false;
		}

		push_turning_event(last_pose_estimate_label_, caregiver_assisted_flag_, tag.get_time_code());
	}

	motion_estimate_sptr_->reset_body_motion_detection_time_window_count();
	person_detection_sptr_->reset_persons_detection_time_window_count();
	pose_estimate_sptr_->reset_classifier_labels_count();

	last_protocol_check_time_ = tag.get_time_code();

	/*
  vcl_string current_pose = "";
  if( motion_estimate_sptr_->get_body_motion_detected_flag() &&
      pose_estimate_sptr_->get_pose_changed_flag(current_pose) )
  {
    if(person_detection_sptr_->get_persons_detected_flag()) {
      // caregiver(s) assisted turn
    }
    else {
      // patient self turn
    }
  }

  last_protocol_check_time_ = tag.get_time_code();
	*/

  //-- Visualize the process
	//visualize();

  return true;
}

const vil_image_view<vxl_byte> &pu_prevention_turning_protocol_process::cur_frame(void) const
{
  return vil_image_view<vxl_byte>();
}

void pu_prevention_turning_protocol_process::set_visualizer( gevxl::img::visualizer_2d *viz)
{
	if(is_enabled() == false) return;

	viz_ = viz;

	motion_estimate_sptr_->set_visualizer(viz);

  person_detection_sptr_->set_visualizer(viz);

  pose_estimate_sptr_->set_visualizer(viz);
}

void pu_prevention_turning_protocol_process::visualize(void)
{	
	if(is_enabled() == false) return;
	/*
	vil_image_view<vxl_byte> viz_img;
	viz_img = cur_frame();
	
	if( viz_ && viz_img.size() > 0 ) {
		gevxl::threading::scoped_lock lock( viz_ );
		viz_->initialize();
		if (viz_->is_initialized() ) { 
			
			viz_->set_image( viz_img );
		}
	}

	IF_CAN_VISUALIZE( viz_ ) {
		viz_->flush();
	}
	*/
}

void pu_prevention_turning_protocol_process::visualize_overlay(void)
{
	if(is_enabled() == false) return;

	motion_estimate_sptr_->visualize_overlay();
	
	person_detection_sptr_->visualize_overlay();
	
	pose_estimate_sptr_->visualize_overlay();

	if(patient_turned_flag_ && patient_turned_flag_viz_count_ < patient_turned_flag_viz_count_thresh_) {
		if( viz_ ) {

			viz_->set_foreground(1, 1, 0);
			
			vcl_string viz_str = "patient turned to " + body_pose_str[last_pose_estimate_label_] + " ";
			if(caregiver_assisted_flag_) {
				viz_str = viz_str + "with caregiver(s) assistance";
			}
			else {
				viz_str = viz_str + "by him/her self";
			}

			viz_->add_text( 0, 68,  viz_str );
			patient_turned_flag_viz_count_++;
		}
	}
	else if(patient_turned_flag_viz_count_ >= patient_turned_flag_viz_count_thresh_) {
		patient_turned_flag_ = false;
		patient_turned_flag_viz_count_ = 0;
	}

}

// extract all recently detected turning events from the list
void pu_prevention_turning_protocol_process::extract_turning_events(vcl_vector<vcl_string> &event_strs)
{
	if(is_enabled() == false) return;

	event_strs.clear();

	turning_events_lock_.lock();
	event_strs.insert(event_strs.end(), turning_events_.begin(), turning_events_.end());
	turning_events_.clear();
	turning_events_lock_.unlock();
}

// push newly detected turning event into the list
void pu_prevention_turning_protocol_process::push_turning_event(int pose_estimate_label, bool caregiver_assisted_flag, double time_code)
{
	if(is_enabled() == false) return;

	vcl_string pose_estimate_label_str, caregiver_assisted_flag_str, time_code_str;
	
	pose_estimate_label_str = body_pose_str2[pose_estimate_label];
	
	if(caregiver_assisted_flag) {
		caregiver_assisted_flag_str = "true";
	}
	else {
		caregiver_assisted_flag_str = "false";
	}

	double g_time = util::time::global_time();
	time_code_str = util::time::global_time_as_localtime(g_time);
	
	vcl_string event_str = pose_estimate_label_str + "," + caregiver_assisted_flag_str + "," + time_code_str;
	vcl_cout << "pu_prevention_turning_protocol_process::push_turning_event, event_str = " << event_str << vcl_endl;
	
	turning_events_lock_.lock();
	turning_events_.push_back(event_str);
	turning_events_lock_.unlock();
}
