//
// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
//

#ifndef gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_control_process_chain_h_
#define gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_control_process_chain_h_

/// \author Yi (Jessica) Yao
/// \data   06/05/2012
/// modified by Ting Yu, 01/12/2014

#include <gui/wx/direct_visualizer.h>
#include <img/visualizer_2d_buffered.h>

#include <framework/chaining_process.h>
#include <framework/timer_process.h>

#include <vid/generic_frame_process.h>
#include <vid/generic_writer_process.h>

#ifdef GEVXL_DEPTH_HAS_KinectSDK
#include <pressure_ulcer/poc/kinectsdk/pu_kinectsdk_kinfu_process.h>
#endif

namespace gevxl {
  namespace pressure_ulcer {
    namespace kinectsdk {

class pu_kinectsdk_control_process_chain : public gevxl::framework::process     
{  
public: 

    pu_kinectsdk_control_process_chain(char const* name="gevxl::pressure_ulcer::kinectsdk::pu_kinectsdk_control_process_chain");
    ~pu_kinectsdk_control_process_chain();

    /// this process must have a visualizer    
    /// set visualizer: direct_visualizer is necessary, overlay_viz is not
    void set_visualizer(gevxl::gui::wx::direct_visualizer *viz, gevxl::img::visualizer_2d_buffered *overlay_viz = NULL);

    void set_visualizer(gevxl::img::visualizer_2d *viz, gevxl::img::visualizer_2d_buffered *overlay_viz = NULL);

    /// must call after set_visualizer() since the chain include
    /// visualizer_initialize_process()
    void set_up_process_chain();
    
    /// configure the process
    bool configure( gevxl::util::config_file& config );
    
    bool step();

    gevxl::vid::generic_frame_process<vxl_byte> *frame_process() { return frame_src_;}

    void uninitialize(void);
   
private:
  
    gevxl::util::config_file config_;

    gevxl::framework::chaining_process proc_chain_; 
    
    /// video source process
    gevxl::vid::generic_frame_process<vxl_byte> *frame_src_;

    #ifdef GEVXL_DEPTH_HAS_KinectSDK
    gevxl::pressure_ulcer::kinectsdk::pu_kinectsdk_kinfu_process *kinfu_proc_;
    #endif

    gevxl::vid::generic_writer_process<vxl_byte> *vid_writer_;

    gevxl::framework::timer_process *timer_proc_;

    // /// The visualizer that does the OpenGL rendering into the canvas.
    gevxl::gui::wx::direct_visualizer *viz_;

    /// The visualizer that buffers draw commands from clients.
    /// The content of this visualizer is extracted by viz_ and 
    /// rendered into the canvas.
    gevxl::img::visualizer_2d_buffered *overlay_viz_;

    gevxl::img::visualizer_2d *draw_viz_;

    int cur_frame_nr_;
		
    bool direct_render_;
    bool is_recording_;

    bool gui_based_;
   };
} // end namespace kinectsdk
} // end namespace pressure_ulcer
} // end namespace gevxl

#endif //gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_control_process_chain_h_
