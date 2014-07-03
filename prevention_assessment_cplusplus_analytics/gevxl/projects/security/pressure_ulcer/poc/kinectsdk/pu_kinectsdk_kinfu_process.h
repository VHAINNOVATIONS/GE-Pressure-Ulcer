// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#ifndef gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_kinfu_process_h_
#define gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_kinfu_process_h_

/// \file 
/// \author Yi (Jessica) Yao
/// \date 6/8/2012
/// modified by Ting Yu, 01/12/2014
/// \par Modifications: 
/// - Original versioin

#include <vid/frame_process.h>
#include <vid/generic_frame_process.h>

#include <img/visualizer_2d.h>

#include <util/config_file.h> 
#include <util/time/highres_time.h>

#include <vcl_string.h>
#include <vcl_vector.h>

#include <vil/vil_rgb.h>

#include <windows.h>
#include <NuiApi.h>

#include <pressure_ulcer/poc/kinectsdk/pu_kinectsdk_sensor.h>

#define MAX_DEPTH 10000

namespace gevxl { 
  namespace pressure_ulcer {
    namespace kinectsdk {

class pu_kinectsdk_kinfu_process : public gevxl::vid::frame_process<vxl_byte> 
{
  public: 

    //////////////////////////////////////////////////////////////////
    // Standard Methods
    //////////////////////////////////////////////////////////////////

    /// Default constructor.
    pu_kinectsdk_kinfu_process(char const *name="gevxl::pressure_ulcer::kinectsdk::pu_kinectsdk_kinfu_process");

    /// Default destructor.
    virtual ~pu_kinectsdk_kinfu_process(void);

    //////////////////////////////////////////////////////////////////
    // Methods
    //////////////////////////////////////////////////////////////////

    virtual bool configure(gevxl::util::config_file& config );

    /// Return the current frame
    vil_image_view<vxl_byte> const & cur_frame() const { return frame_; }

    void enable_visualize(bool flag) {visualize_ = flag;}
    bool is_visualize(void) const {return visualize_;}
    void set_visualizer(gevxl::img::visualizer_2d *viz) { viz_ = viz; kinectsdk_sensor_.set_visualizer(viz); }

    //virtual bool configure(const gevxl::util::config_file& config );
    virtual bool initialize(void);
    virtual void uninitialize(void);
    virtual bool step(void);
    virtual void visualize(gevxl::img::visualizer_2d *viz);

  protected:

    void step_frame(void);
    
    gevxl::img::visualizer_2d *viz_;
    bool visualize_;

    bool enabled_;

    /// Current frame
    bool frame_set_;
    vil_image_view<vxl_byte> frame_;
    
    unsigned frame_width_;
    unsigned frame_height_;
    
    vil_image_view<vxl_byte> depth_frame_;

    //set time when no time tag is available
    double global_start_time_; 
    gevxl::util::time::highres_time hires_time_;     
    double system_start_time_;

    //// Current Kinect
    pu_kinectsdk_sensor          kinectsdk_sensor_;
    bool                         kinectsdk_sensor_flag_;
    
    NUI_IMAGE_TYPE               depth_type_;
    NUI_IMAGE_RESOLUTION         depth_res_;
    bool                         near_mode_flag_;
    
    NUI_IMAGE_TYPE               color_type_;
    NUI_IMAGE_RESOLUTION         color_res_;
};

typedef vbl_shared_pointer<pu_kinectsdk_kinfu_process> pu_kinectsdk_kinfu_process_sptr;

}}} //  namespace kinectsdk, pressure_ulcer, gevxl

#endif //   gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_kinfu_process_h_
