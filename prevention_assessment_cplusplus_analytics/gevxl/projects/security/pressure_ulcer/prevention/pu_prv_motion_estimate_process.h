// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef pu_prv_motion_estimate_process_h_
#define pu_prv_motion_estimate_process_h_

#include <framework/process.h>
#include <img/visualizer_2d.h>
#include <util/on_off_mixin.h>
#include <img/optical_flow.h>
#include <boost/circular_buffer.hpp>
#include <vid/frame_tag.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

class pu_prv_motion_estimate_process : public gevxl::framework::process, public gevxl::util::on_off_mixin
{
public :

  // constructor
  pu_prv_motion_estimate_process( char const *name = "gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process" );

  // de-constructor
  virtual ~pu_prv_motion_estimate_process( void );

  // configure the process
  virtual bool configure( gevxl::util::config_file &config );

  // initialize the process
  virtual bool initialize(void);

	// uninitialize the process
	virtual void uninitialize(void);

  // step function
  virtual bool step() { vcl_cerr << "Use step(image)" << vcl_endl; return false; }

  // step function
  virtual bool step( const gevxl::vid::frame_tag &tag, const vil_image_view<vxl_byte> &depth_byte_img );

  // set the process's visualizer
	virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

	// visualization function
	virtual void visualize(void);

	// visualization overlay function
	virtual void visualize_overlay(void);

  // get the current frame from the source process
  const vil_image_view<vxl_byte> &cur_frame(void) const;

  // quantify motion with a single number
  float get_scalar_motion_magnitude() { return mh_scalar_magnitude_; }

  // quantify motion with a histogram 
  vcl_vector<int> get_motion_histogram() { return mh_histogram_; }

  // mh_motion_history_img_
  const vil_image_view<vxl_byte> &get_visualization_image() const { return mh_viz_img_; }

  // mh_histogram_img_
  const vil_image_view<vxl_byte> &get_motion_histogram_image() const { return mh_histogram_img_; }

  // get the body motion detected flag
  bool get_body_motion_detected_flag(void);

	// get the no body motion flag
	bool get_no_body_motion_flag(void);

	// get the body motion detection time window count
	void get_body_motion_detection_time_window_count(double &total_motion_detected_flag_count, 
																									 double &total_time_elapse_count,
																									 double &nr_of_no_motion_detected_frames);

	void reset_body_motion_detection_time_window_count(void);


private:

	// compute the motion history image buffer
	void compute_motion_history(// input params
                            const vil_image_view<float>                               &bg_img,
                            const boost::circular_buffer<vil_image_view<vxl_byte> >   &img_buf,
                            unsigned int                                              motion_histogram_img_width, // visualization of motion histogram
                            int                                                       motion_histogram_threshold, // larger value => less noisy
                            bool                                                      enable_morphology_noise_removal,
                            double                                                    motion_histogram_morphology_se_radius,
                            // output params
                            float                     &motion_magnitude,
                            vcl_vector<int>           &motion_histogram,
                            vil_image_view<vxl_byte>  &motion_history_img,
                            vil_image_view<vxl_byte>  &motion_histogram_img
                            );

	// Control the rate at which background is updated 
	void update_background(const vil_image_view<vxl_byte> &img, vil_image_view<float> &bg_img);

  // config file
  gevxl::util::config_file config_;

  // for display and visualization of image processing result
  vil_image_view<vxl_byte> colorized_img_;

  // current and previous depth images
  vil_image_view<vxl_byte> prev_img_, cur_img_;

  // optical flow estimator, not used yet
  gevxl::img::optical_flow optical_flow_estimator_;

  // for motion history (prefix mh_ for motion history )
  const unsigned int     mh_history_size_;
  vil_image_view<float>  mh_bg_img_; // background image 
  float                  mh_scalar_magnitude_; // motion quantified as a single number
  vcl_vector<int>        mh_histogram_; // motion quantified with a histogram
  int                    mh_viz_width_;  // width for visualizing the histogram
  int                    mh_threshold_; // high value => less noisy
  double                 mh_morphology_se_radius_; // radius of morphological structuring element used for removing spurious foreground
  bool                   enable_morphology_noise_removal_;

  boost::circular_buffer< vil_image_view<vxl_byte> > mh_img_buf_; // stores previous frames

  vil_image_view<vxl_byte> mh_viz_img_; // a 3 channel color image
  
  vil_image_view<vxl_byte> mh_histogram_img_; // a histogram visualization

	// the roi mask image
	vil_image_view<vxl_byte> roi_mask_img_;
	int roi_mask_area_;

  // visualizer 	
	gevxl::img::visualizer_2d *viz_;

	// the motion scalar magnitude threshold
	double mh_scalar_magnitude_high_thresh_;
	double mh_scalar_magnitude_low_thresh_;

	// the motion scalar magnitude time window count
	double total_motion_detected_flag_count_;
	double total_time_elapse_count_;

	double nr_of_no_motion_detected_frames_;
};

typedef vbl_shared_pointer<pu_prv_motion_estimate_process> pu_prv_motion_estimate_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif
