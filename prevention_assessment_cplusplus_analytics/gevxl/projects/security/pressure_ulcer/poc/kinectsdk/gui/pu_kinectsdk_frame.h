//
// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
//

#ifndef sec_skeleton_frame_h_
#define sec_skeleton_frame_h_

/// \file
/// \author Yi Yao
/// \date 6/5/2012
/// \par Modifications:
/// - Original version

// Put includes here.
#include <gui/wx/frame.h>
#include <gui/wx/direct_visualizer.h>
#include <gui/wx/canvas.h>
#include <gui/wx/client.h>
#include <gui/wx/canvas_view_client.h>
#include <gui/wx/canvas_record_client.h>
#include <gui/wx/process_control_client.h>
#include <gui/wx/frame_record_client.h>
#include <gui/utils.h>

#include <img/visualizer_2d_buffered.h>

#include "smartroom/depth/skeleton_control_process_chain.h"


namespace gesec { 
namespace smartroom {

class sec_skeleton_frame: public gevxl::gui::wx::frame
{
public:
    sec_skeleton_frame();
    ~sec_skeleton_frame();

    bool initialize();  
    
    gevxl::gui::wx::canvas * get_canvas(void) {return canvas_;}

    void set_process(gesec::smartroom::skeleton_control_process_chain *proc)
       { control_proc_ = proc; 
         control_proc_->set_visualizer(&viz_, &overlay_viz_); }

    /// The visualizer that does the OpenGL rendering into the canvas.
    gevxl::gui::wx::direct_visualizer viz_;

    /// The visualizer that buffers draw commands from clients.
    /// The content of this visualizer is extracted by viz_ and 
    /// rendered into the canvas.
    gevxl::img::visualizer_2d_buffered overlay_viz_;

         
protected:
    void setup_canvas(void);
    void setup_layout(void);
    void setup_visualization(void);
    void setup_clients(void);
    void setup_menus(void);     
   
private:
    /// Pointer to canvas (OpenGL rendering area).
    gevxl::gui::wx::canvas *canvas_;     

    gesec::smartroom::skeleton_control_process_chain *control_proc_;

    gevxl::gui::wx::canvas_record_client *record_client_;

};

} // end smartroom namespace
} // end gesec namespace

#endif
