// Copyright (C) 2012 General Electric Company
//  
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
// 
// 
//GE
/// \file
/// \author Yi (Jessica) Yao
/// \date 06/05/2012
/// \par Modifications:
/// - Original version (header created with VS macro)

#include <vcl_cstdlib.h>
#include <vul/vul_arg.h>

#include <gui/wx/app.h>
#include <gui/utils.h>

#include "smartroom/depth/gui/sec_skeleton_frame.h"
//#include "smartroom/depth/bed_detector_depth_based_process.h"

using namespace gevxl;
using namespace gesec::smartroom;

struct MyApp : public gui::wx::app
{
  MyApp();
  virtual bool OnInit();
  virtual int OnExit();

  //sec_depth_frame * frame_;
  gesec::smartroom::skeleton_control_process_chain *process_chain_;
  framework::process_control control_;

  //gui::wx::direct_visualizer viz_;
  //gevxl::img::visualizer_2d_buffered overlay_viz_;
  util::config_file config_;  
};

// This is a wxWidgets construct. It hides main().
IMPLEMENT_APP(MyApp)

MyApp::MyApp() 
{
}


bool MyApp::OnInit()
{
  vul_arg<vcl_string> a_config_file( "-c", "Config file" );

  char **my_argv;

  my_argv=new char *[argc+1];

  for(int i=0;i<argc;i++)
  {
    vcl_string av=vcl_string(wxString(argv[i]).mb_str());
    my_argv[i]=new char [av.length()+1];
    vcl_strcpy(my_argv[i],av.c_str());
    my_argv[i][av.length()]=0;
  }
  my_argv[argc]=NULL;

  vul_arg_parse( argc, my_argv, false );

  //config_.parse_arguments( argc, argv );
  config_.read( a_config_file().c_str() );

	//to start playing video directly when program starts or not
  bool direct_render = false;
  config_.get_bool( "skeleton::direct_render", direct_render );

  sec_skeleton_frame *frame_=new gesec::smartroom::sec_skeleton_frame;
  process_chain_ = new gesec::smartroom::skeleton_control_process_chain("skeleton::chaining_process");

  process_chain_->configure(config_);
  process_chain_->set_visualizer(&(frame_->viz_), &(frame_->overlay_viz_));
  process_chain_->set_up_process_chain();

  control_.set_process( process_chain_ );

  // Construct and setup the main window
  frame_->set_process_control( &control_ );
  frame_->enable_scroll_bar(true);
  frame_->set_process(process_chain_);
  frame_->init();
  vcl_cout << "About to call Show" << vcl_endl;
  frame_->Show();
  vcl_cout << "Show called" << vcl_endl;


  vcl_cout << "Running" << vcl_endl;
  control_.initialize_control(control_.ASYNCHRONOUS);
  if( direct_render ) {
    control_.execute_continously();
  } else {
    control_.execute_one_step();
  }

  return true;
}

int MyApp::OnExit()
{
  if(process_chain_)
  {
    if (!control_.is_terminated())
    {
      control_.terminate_execution( /*wait until terminated=*/ true );
      control_.wait_until_terminated();
    }
    delete process_chain_;
    process_chain_=NULL;
  }
  exit(EXIT_SUCCESS);
  return EXIT_SUCCESS;
}
