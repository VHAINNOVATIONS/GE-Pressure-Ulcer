// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 2/18/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_prevention_frame_h
#define gevxl_pressure_ulcer_pu_prevention_frame_h

#include <vcl_cstdlib.h>
#include <vcl_fstream.h>

#include <vbl/vbl_shared_pointer.h>
#include <img/visualizer_2d_buffered.h>

#include <gui/wx/app.h>
#include <gui/wx/frame.h>
#include <gui/wx/wxid.h>
#include <gui/wx/canvas.h>
#include <gui/wx/direct_visualizer.h>
#include <gui/wx/show_splash.h>
#include <gui/wx/settings_dialog.h>
#include <gui/wx/frame_record_client.h>

#include <gui/utils.h>

// prevention chaining proc
#include <pressure_ulcer/prevention/pu_prevention_chaining_process.h>

// pose estimate client
#include <pressure_ulcer/prevention/gui/pu_pose_estimate_client.h>

#include <util/config_file.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {

/////////////////////////////////////////////////////////////////////////
//
// The implementation of the frame.
//
// This frame essentially visualizes a process that is controlled by
// a process controller and which supports the following
// functions:
//
// set_visualizer(visualizer *viz) -- tells process how to visualize
//
// vil_image_view<vxl_byte> &cur_frame(void) -- returns current frame
// unsigned cur_frame_num(void) -- returns current frame that is processed
// unsigned length(void) -- returns length of video
// seek(unsigned n) -- tells process to go to given frame
//
/////////////////////////////////////////////////////////////////////////
class pu_prevention_frame : public gevxl::gui::wx::frame  
{
public:

  /// Default constructor.
  pu_prevention_frame(const vcl_string &title)
    : frame(NULL, title, wxDefaultPosition, wxSize(640,480)), 
      canvas_(NULL),
      record_client_(NULL),
			prevention_chaining_proc_(NULL),
			pose_estimate_client_(NULL)			
  { };

  virtual ~pu_prevention_frame(void);

	/// Configure the prevention frame from the config_file
	bool configure(gevxl::util::config_file &config);

	/// Tell this GUI about the prevention chaining process.
	void set_prevention_chaining_process(gevxl::pressure_ulcer::prevention::pu_prevention_chaining_process *p) { prevention_chaining_proc_ = p; }  

protected:

  bool initialize(void); 

  bool is_scroll_bar_active(void) { return true; }

  /// The canvas that we allocate.
  gevxl::gui::wx::canvas *canvas_;

  /// The main OpenGL visualizer. Will be owned by process.
  gevxl::gui::wx::direct_visualizer viz_;

  /// The overlay visualizer. Will be owned by GUI.
  gevxl::img::visualizer_2d_buffered overlay_viz_;

	//gevxl::gui::wx::canvas_record_client *canvas_record_client_;
  gevxl::gui::wx::frame_record_client *record_client_;

	/// This is where we hold the process to the prevention chaining proc.
	gevxl::pressure_ulcer::prevention::pu_prevention_chaining_process *prevention_chaining_proc_;

  /// client to handle the GUI behavior for pose estimate
  pu_pose_estimate_client *pose_estimate_client_;

	/// the configuration object
	gevxl::util::config_file config_;	
};
typedef vbl_shared_pointer<pu_prevention_frame> pu_prevention_frame_sptr;

}}}  // end of the namespace

#endif