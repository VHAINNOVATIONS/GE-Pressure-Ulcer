// Copyright (C) 2013 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Yi Xu
/// \date 1/24/2014
/// \brief the wrapper for the camera that supports Intel Peception Library (PCSDK)
/// - Original version

#ifndef pxc_camera_h_
#define pxc_camera_h_

//#ifdef GEVXL_VID_HAS_PCSDK
//#include <util_pipeline.h>
//#endif

#include "vnl/vnl_matrix.h"
#include "vil/vil_image_view.h"

#include <threading/thread.h>
#include <threading/mutex.h>

#include <util/time/highres_time.h>

class UtilPipeline;
class PXCImage;

namespace gevxl {
namespace vid {

class pxc_camera : public gevxl::threading::thread
{
public:

  // constructor	
  pxc_camera(bool live, vcl_string pxc_filename, int img_height, int img_width, int depth_height, int depth_width, bool asynchronous_grabbing = true);

  // deconstructor
  ~pxc_camera();

  bool get_next_frames(vil_image_view<vxl_uint_16> &depth_frame, 
                       vil_image_view<vxl_byte> &rgb_frame,
                       vil_image_view<float> &xyz_rgb_frame,                       
											 vil_image_view<int> &depth_to_rgb_coordinate_map,
                       double &captured_frame_system_time);

  int get_img_height(void) { return img_height_; }
  int get_img_width(void) { return img_width_; }

  int get_depth_height(void) { return depth_height_; }
  int get_depth_width(void) { return depth_width_; }

  bool is_pxc_initialized(void) { return pxc_initialized_; }

private:

  void run_thread(void);

  void capture_next_frames(void);

  void copy_video_frames(vil_image_view<vxl_uint_16> &depth_frame, 
                         vil_image_view<vxl_byte> &rgb_frame,
                         vil_image_view<float> &xyz_rgb_frame,                         
												 vil_image_view<int> &depth_to_rgb_coordinate_map);

  // pxc driver initialization variables
  static gevxl::threading::mutex pxc_driver_mutex_;
  static bool pxc_initialized_;
  static bool pxc_disconnected_;

  // pxc utility pipeline
  UtilPipeline *pp_;

  // pxc video stream frames
  PXCImage *pxc_depth_frame_;
  PXCImage *pxc_rgb_frame_;

  // local copies
  vil_image_view<vxl_uint_16> local_depth_frame_;
  vil_image_view<vxl_byte> local_rgb_frame_;
  vil_image_view<float> local_xyz_rgb_frame_;
  
	vil_image_view<int> local_depth_to_rgb_coordinate_map_;

  // flag indicating whether the pxc_camera performs asynchronous grabbing
  bool asynchronous_grabbing_;
  bool new_frame_ready_;

  // the mutex lock that only locks the grabbing slot of the VideoStreams
  gevxl::threading::mutex grab_slot_lock_;

  // the height and width of the rgb and depth image
  int img_height_;
  int img_width_;
  int depth_height_;
  int depth_width_;

  // local frame copy captured time using the system time
  gevxl::util::time::highres_time hires_time_;
  double captured_frame_system_time_;
};

}} // namespace

#endif
