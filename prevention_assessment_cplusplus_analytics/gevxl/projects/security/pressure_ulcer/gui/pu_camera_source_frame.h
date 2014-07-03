// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/30/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_camera_source_frame_h
#define gevxl_pressure_ulcer_pu_camera_source_frame_h

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

#include <pressure_ulcer/pu_camera_source_proc.h>
#include <pressure_ulcer/gui/pu_pixel_picking_client.h>

namespace gevxl {
	namespace pressure_ulcer {

/////////////////////////////////////////////////////////////////////////
//
// A client to handle modify parameters of processes the application.
//
/////////////////////////////////////////////////////////////////////////
class options_handler_client : public gevxl::gui::wx::handler_client
{
public:
  //:
  options_handler_client( void ) { };

  //:
  virtual ~options_handler_client() { };

  //:
  virtual void update_menu (wxMenu *menu );

  //:
  void on_mouse( gevxl::gui::event_source *source, wxMouseEvent &event) {}

  virtual void on_key( gevxl::gui::event_source *source, wxKeyEvent &event) {}

  virtual bool configure(gevxl::util::config_file &config) { return true; };

  // process that contains the options we are interested in toggling  
  void set_proc(gevxl::pressure_ulcer::pu_camera_source_proc *p) { proc_ = p; }
  //void set_proc(gevxl::pressure_ulcer::pu_thermal_analysis_proc *p) { proc_ = p; }

  void on_configure(wxCommandEvent &evt);

private:
  
  gevxl::pressure_ulcer::pu_camera_source_proc *proc_;
  //gevxl::pressure_ulcer::pu_thermal_analysis_proc *proc_;
  
};

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
class pu_camera_source_frame : public gevxl::gui::wx::frame  
{
public:

  /// Default constructor.
  pu_camera_source_frame(const vcl_string &title)
    : frame(NULL, title, wxDefaultPosition, wxSize(640,480)), 
      canvas_(NULL),
      proc_(NULL),
      record_client_(NULL),
      options_handler_client_(NULL),
      pixel_picking_client_(NULL)
  { };

  virtual ~pu_camera_source_frame(void);

  /// Tell this GUI about the process.
  void set_process(gevxl::pressure_ulcer::pu_camera_source_proc *p) { proc_ = p; } 
  //void set_process(gevxl::pressure_ulcer::pu_thermal_analysis_proc *p) { proc_ = p; } 

protected:

  bool initialize(void); 

  bool is_scroll_bar_active(void) { return true; }

  /// The canvas that we allocate.
  gevxl::gui::wx::canvas *canvas_;

  /// The main OpenGL visualizer. Will be owned by process.
  gevxl::gui::wx::direct_visualizer viz_;

  /// The overlay visualizer. Will be owned by GUI.
  gevxl::img::visualizer_2d_buffered overlay_viz_;

  /// This is where we do all the processing.
  gevxl::pressure_ulcer::pu_camera_source_proc *proc_;
  //gevxl::pressure_ulcer::pu_thermal_analysis_proc *proc_;

  //gevxl::gui::wx::canvas_record_client *canvas_record_client_;
  gevxl::gui::wx::frame_record_client *record_client_;

  // Client to handle the toggling of options in the program
  options_handler_client *options_handler_client_;

  // cleint to handle the pixel picking behavior to visualize the pixel's attributes
  pu_pixel_picking_client *pixel_picking_client_;

};
typedef vbl_shared_pointer<pu_camera_source_frame> pu_camera_source_frame_sptr;

}}  // end of the namespace

#endif