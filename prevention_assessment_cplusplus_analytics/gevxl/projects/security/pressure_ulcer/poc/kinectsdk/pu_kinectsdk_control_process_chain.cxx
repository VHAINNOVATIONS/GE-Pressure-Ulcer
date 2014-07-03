//
// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
//

/// \file uav_control_process_chain.cxx
/// \brief a chain of processes used for uav project. 
///
/// \author Yi (Jessica) Yao
/// \data   06/05/2012
/// modified by Ting Yu, 01/12/2014

#include <framework/timer_process.h>
#include <framework/function_caller_process.h>
#include <img/visualizer_initialize_process.h>
#include <img/visualizer_flush_process.h>
#include <boost/bind.hpp>

#include <pressure_ulcer/poc/kinectsdk/pu_kinectsdk_control_process_chain.h>

using namespace gevxl;
using namespace gevxl::pressure_ulcer::kinectsdk;

pu_kinectsdk_control_process_chain::pu_kinectsdk_control_process_chain(char const* name)
 : framework::process( name ),
   draw_viz_(NULL),
   gui_based_(true)
{   

}

pu_kinectsdk_control_process_chain::~pu_kinectsdk_control_process_chain()
{

}

 /// set visualizer: direct_visualizer is necessary, overlay_viz is not
void pu_kinectsdk_control_process_chain::set_visualizer(gui::wx::direct_visualizer *viz,
                                                          img::visualizer_2d_buffered *overlay_viz)
{
    viz_ = viz;
    overlay_viz_ = overlay_viz;   
    gui_based_ = true;
}

void pu_kinectsdk_control_process_chain::set_visualizer(gevxl::img::visualizer_2d *viz,
                                                 img::visualizer_2d_buffered *overlay_viz)
{
    draw_viz_ = viz;
    overlay_viz_ = overlay_viz;
    gui_based_ = false;
}

bool pu_kinectsdk_control_process_chain::configure(util::config_file& config)
{  
  if (!config.get_bool(name()+"::direct_render", direct_render_))
    direct_render_ = false;

  if (!config.get_bool(name()+"::is_recording", is_recording_))
    is_recording_ = false;

  config_ = config;

  return true;
}

bool pu_kinectsdk_control_process_chain::step()
{
    return proc_chain_.step();    
}  

static bool visualize_image( vid::frame_process<vxl_byte>* src,
                              img::visualizer_2d* viz)
{
  IF_CAN_VISUALIZE( viz ) {
    viz->set_image( src->cur_frame() );
  }

  return true;
}

void pu_kinectsdk_control_process_chain::set_up_process_chain()
{    
  if(gui_based_ && direct_render_) {
    draw_viz_ = viz_;
  }

  if (!direct_render_) {
    draw_viz_ = overlay_viz_;
  }

  #ifdef GEVXL_DEPTH_HAS_KinectSDK

    if (gui_based_) {

       // Setup visualization initialization 
      img::visualizer_initialize_process *viz_init_proc = new img::visualizer_initialize_process;
      viz_init_proc->set_visualizer(viz_);
      viz_init_proc->set_clear(true);
      proc_chain_.add_process( viz_init_proc, /*transfer ownership=*/ true );

      if (overlay_viz_) {
          img::visualizer_initialize_process *viz_overlay_init_proc = new img::visualizer_initialize_process;
          viz_overlay_init_proc->set_visualizer(overlay_viz_);
          viz_overlay_init_proc->set_clear(true);
          proc_chain_.add_process( viz_overlay_init_proc, /*transfer ownership=*/ true );

          draw_viz_ = overlay_viz_;
      }
    }

    kinfu_proc_ = new gevxl::pressure_ulcer::kinectsdk::pu_kinectsdk_kinfu_process();    
    kinfu_proc_->set_visualizer(draw_viz_);
    if (!kinfu_proc_->configure(config_)) {
      vcl_cerr << "ERROR: gevxl::pressure_ulcer::kinectsdk::pu_kinect_kinfu_process: configure error" << vcl_endl;
    }
    if (!kinfu_proc_->initialize())
      vcl_cerr << "ERROR: gevxl::pressure_ulcer::kinectsdk::pu_kinect_kinfu_process: initialize error" << vcl_endl;
    proc_chain_.add_process(kinfu_proc_, true);
 
    vid_writer_ = new vid::generic_writer_process<vxl_byte>( "gevxl::pressure_ulcer::kinectsdk::pu_kinectsdk_control_process_chain::generic_writer_process" );
    gevxl::vid::frame_process<vxl_byte> *frame_proc = dynamic_cast<gevxl::vid::frame_process<vxl_byte> *>(kinfu_proc_);
    
    if (vid_writer_) {
      vid_writer_->set_frame_process(frame_proc);
      if( !vid_writer_->configure(config_) ) {
        vcl_cerr << "ERROR: gevxl::pressure_ulcer::kinectsdk::pu_kinectsdk_control_process_chain: video writer didn't configure\n";
      }
      proc_chain_.add_process( vid_writer_, true );
    }
    else {
      vcl_cerr << "ERROR: gevxl::pressure_ulcer::kinectsdk::pu_kinectsdk_control_process_chain: video writer source frame process is invalid!\n";
    }

    if (gui_based_) {
      // flushing visualizer
      img::visualizer_flush_process* viz_flush_proc = new img::visualizer_flush_process;
      viz_flush_proc->set_visualizer(viz_);
      proc_chain_.add_process( viz_flush_proc, /*transfer ownership=*/ true );  
    }

  #endif

}

void pu_kinectsdk_control_process_chain::uninitialize(void)
{
  proc_chain_.uninitialize();
}

