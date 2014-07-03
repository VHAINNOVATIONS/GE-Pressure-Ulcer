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

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

class pu_prv_motion_estimate_process : public gevxl::framework::process, public gevxl::util::on_off_mixin
{
public :

  //:
  pu_prv_motion_estimate_process(  char const *name = "gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process" );

  //:
  virtual ~pu_prv_motion_estimate_process( void );

  //:
  virtual bool configure( gevxl::util::config_file &config );

  //:
  bool step() {vcl_cerr<<"Use step(image)"<<vcl_endl; return false;}

  //:
  bool step( const vil_image_view<vxl_byte> &depth_byte_img );

  //: Quantify motion with a single number
  float get_scalar_motion_magnitude() { return mh_scalar_magnitude_; }

  //: Quantify motion with a histogram 
  vcl_vector<int> get_motion_histogram() { return mh_histogram_; }

  //: mh_motion_history_img_
  const vil_image_view<vxl_byte> &get_visualization_image() const { return mh_viz_img_; }

  //:
  const vil_image_view<vxl_byte> &get_motion_histogram_image() const { return mh_histogram_img_; }


private:

  //:
  gevxl::util::config_file config_;

  //: for display and visualization of image processing result
  vil_image_view<vxl_byte> colorized_img_;

  //:
  vil_image_view<vxl_byte> prev_img_, cur_img_;

  //:
  gevxl::img::optical_flow optical_flow_estimator_;

  //: for motion history (prefix mh_ for motion history )
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

};

typedef vbl_shared_pointer<pu_prv_motion_estimate_process> pu_prv_motion_estimate_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif
