// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/25/2014
/// \par Modifications:

#include <vcl_cstdlib.h>
#include <vcl_fstream.h>

#include <vbl/vbl_shared_pointer.h>
#include <vul/vul_arg.h>
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

#include <framework/process_control.h>

#include <pressure_ulcer/gui/pu_camera_source_frame.h>
#include <pressure_ulcer/pu_camera_source_proc.h>

// Our namespace.
using namespace gevxl::gui;
using namespace gevxl::gui::wx;
using namespace gevxl::pressure_ulcer;

/////////////////////////////////////////////////////////////////////////
//
// The main application.
//
/////////////////////////////////////////////////////////////////////////
class MyApp : public wx::app
{
  virtual bool OnInit();

  virtual int OnExit();

  /// Example process.
  gevxl::pressure_ulcer::pu_camera_source_proc proc_;  

  /// Process control that controls above process.
  gevxl::framework::process_control control_;

  // The main frame window.
  pu_camera_source_frame_sptr frame_; 
};

// This is a wxWidgets construct. It hides main().
IMPLEMENT_APP(MyApp)

// This function can be viewed as the traditional main().
bool MyApp::OnInit()
{  
   vul_arg<vcl_string> a_config_file( "-c", "Config file", "C:/GEVXL/gevxl/projects/security/pressure_ulcer/config/pu_camera_source.tconf" );
   gevxl::util::config_file config;

#if wxUSE_UNICODE 
  convert_unicode_to_ansi(argc, argv);
  vul_arg_parse(argc, argv_, false);  
  config.parse_arguments(argc, argv_); // process remaining command-line args
#else
  vul_arg_parse(argc, argv, false);
  config.parse_arguments(argc, argv); // process remaining command-line args
#endif

 // if( a_config_file.set() )
  config.read( a_config_file().c_str() );  // process entire config file

  utils::ensure_terminal_output();

  if(!proc_.configure(config) || !proc_.initialize())
  {
    vcl_cerr << "Error, failed to initialize or configure process.\n";
    return EXIT_FAILURE;
  }

  control_.set_process(&proc_);
  control_.initialize_control(control_.ASYNCHRONOUS);
  control_.execute_one_step(true);
  control_.execute_continously();
 
  // Create the main frame window.
  frame_ = pu_camera_source_frame_sptr(new pu_camera_source_frame("Pressure Ulcer Camera Source Demo"));
  frame_->set_process(&proc_);
  frame_->set_process_control(&control_);
  frame_->enable_scroll_bar(true);
  frame_->init(); // Main initialization (which is the actual GUI).
  frame_->activate_scroll_bar(false);
  frame_->Show(true);// Show the frame (i.e., GUI).

  // Return and start the wxWidgets event loop.
  return true;
}

int MyApp::OnExit()
{
  vcl_cout << "Shutting down process and process control.\n";

  vcl_cout << "Main done. \n";

  vcl_exit(EXIT_SUCCESS);

  return 1;
}
