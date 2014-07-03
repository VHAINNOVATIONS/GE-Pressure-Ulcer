// Copyright (C) 2006 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include <vcl_fstream.h>
#include <vcl_cstdlib.h>
#include <vil/vil_load.h>
#include <gui/wx/app.h>
#include <wx/menu.h>
#include <gui/wx/wxid.h>
#include <detectors/va/gui/va_image_frame.h>
#include <util/config_file.h>

using namespace gevxl;
using namespace gevxl::gui;
using namespace gevxl::img;
using gevxl::gui::wx::wxid;
using namespace gesec::detectors;

// ///////////////////////////////////////////////////////////////////////
// The main application.
// ///////////////////////////////////////////////////////////////////////
  
class MyApp : public gui::wx::app
{
    virtual bool OnInit();
};

IMPLEMENT_APP(MyApp)
  
bool MyApp::OnInit()
{
  utils::ensure_terminal_output();
  
  vcl_cout << "Usage: sec_va_image_exec -c config.tconf" << vcl_endl;

  vul_arg<char *> 
    config_file_name("-c", "<config file name>", "config.tconf");
  
  // now parse the input parameters 
  //vul_arg_parse(argc, argv);

#if wxUSE_UNICODE 
  convert_unicode_to_ansi(argc, argv);
  vul_arg_parse(argc, argv_, false);  
  //config.parse_arguments(argc, argv_); // process remaining command-line args
#else
  vul_arg_parse(argc, argv, false);
  //config.parse_arguments(argc, argv); // process remaining command-line args
#endif

  // now make a stream to the config file, load the config file 
  // and then close the stream
  
  util::config_file config;
  vcl_ifstream config_file(config_file_name());
  if(config_file){
    config.read(config_file);
    config_file.close();
  }
  
  
  // Create the main frame window
  va_image_frame *frame=new va_image_frame(NULL, 
					   wxT("wx Client Example"),
					   wxDefaultPosition,
					   wxDefaultSize);
  
  frame->configure(&config);
  frame->init(false);
  frame->ClearBackground();
  
  // Show the frame
  frame->Show(true);
  
  return true;
}

