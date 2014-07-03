// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef gevxl_pu_prv_motion_demo_chain_process_h
#define gevxl_pu_prv_motion_demo_chain_process_h

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_shared_pointer.h>

#include <framework/process.h>
#include <framework/async_wrapping_process.h>

#include <img/visualizer_2d.h>
#include <img/visualizer_image.h>

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>
#include <vid/generic_writer_process.h>
#include <vid/code_stamp_frame_tag_process.h>
#include <vid/io/ffmpeg_writer.h>

#include <util/on_off_mixin.h>
#include <util/time/highres_timer.h>

#include <threading/mutex.h>

#include <pressure_ulcer/prevention/pu_prv_rectify_kinect_process.h>
#include <pressure_ulcer/prevention/pu_prv_motion_estimate_process.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

class pu_prv_motion_demo_chain_process : public gevxl::framework::process, public gevxl::util::on_off_mixin
{
public:

  // Constructor
  pu_prv_motion_demo_chain_process(char const *name="pu_prv_motion_demo_chain_process"); 

  // Destructor
  virtual ~pu_prv_motion_demo_chain_process(void); 

  // Configure this proc
  virtual bool configure(gevxl::util::config_file &config);

  // Initialize the process
  virtual bool initialize(void);

  // Uninitialize the process
  virtual void uninitialize(void);

  // The main step function
  virtual bool step(void); 

  //: set the process's visualizer
  virtual void set_visualizer(gevxl::img::visualizer_2d *viz);

  // visualization function
  virtual void visualize(void);

  // get the current frame from the source process
  const vil_image_view<vxl_byte> &cur_frame(void) const { return source_proc_.cur_frame(); }

private:

  // Input Source
  gevxl::vid::generic_frame_process<vxl_byte> source_proc_;

  // Output Writer
  gevxl::vid::generic_writer_process<vxl_byte> writer_proc_;

  // Frame tag process
  gevxl::vid::code_stamp_frame_tag_process frame_tag_proc_;

  int frame_nr_;

  // high resolution timer
  gevxl::util::time::highres_timer highres_timer_; 

  // visualizer 	
  gevxl::img::visualizer_2d *viz_;

  // visualizer image
  gevxl::img::visualizer_image *viz_img_;

  // the configuration file that the generator needs in order to configure its own parameters.
  gevxl::util::config_file config_;

  vcl_string viz_video_out_filename_;
  bool viz_video_out_openned_;
  gevxl::vid::io::ffmpeg_writer viz_video_out_;

  //: Rectify Kinect process
  pu_prv_rectify_kinect_process rectify_kinect_proc_;

  //:
  pu_prv_motion_estimate_process motion_estimate_proc_;


};  // end of class pu_prv_motion_demo_chain_process

typedef vbl_shared_pointer<pu_prv_motion_demo_chain_process> pu_prv_motion_demo_chain_process_sptr;

		} // end of prevention namespace
	} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif

