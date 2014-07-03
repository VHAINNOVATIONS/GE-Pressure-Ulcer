//
// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
//

#ifndef gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_sensor_h_
#define gevxl_pressure_ulcer_kinectsdk_pu_kinectsdk_sensor_h_

/// \author Yi (Jessica) Yao
/// \data   06/08/2012
/// modified by Ting Yu, 01/12/2014

#pragma once
#include <windows.h>

#include <vcl_iostream.h>
#include <vcl_vector.h>

#include <vnl/vnl_double_2.h>

#include <img/visualizer_2d.h>

#ifdef GEVXL_DEPTH_HAS_KinectSDK
  #include <FaceTrackLib.h>
  #include <NuiApi.h>
#endif

namespace gevxl {
  namespace pressure_ulcer {
    namespace kinectsdk {

  class pu_kinectsdk_sensor
{
public:
    pu_kinectsdk_sensor();
    ~pu_kinectsdk_sensor();

    void set_sensor(INuiSensor * sensor) { sensor_ = sensor; }

    HRESULT init(NUI_IMAGE_TYPE depthType, NUI_IMAGE_RESOLUTION depthRes, BOOL bNearMode, 
                  BOOL bFallbackToDefault, 
                  NUI_IMAGE_TYPE colorType, NUI_IMAGE_RESOLUTION colorRes);

    void release();

    void set_visualizer(gevxl::img::visualizer_2d *viz) { viz_ = viz; }
    virtual void visualize(gevxl::img::visualizer_2d *viz);

    //GetRGBBuffer()->CopyTo;
    //GetDepthBuffer()->CopyTo;

private:

    static DWORD WINAPI process_thread(PVOID p_param);
    
    void got_rgb_alert(void);
    void got_depth_alert(void);

    INuiSensor *sensor_;
    
    NUI_IMAGE_FRAME *nui_depth_image_frame_;
    NUI_IMAGE_FRAME *nui_rgb_image_frame_;

    HANDLE next_depth_frame_event_;
    HANDLE next_rgb_frame_event_;
    
    HANDLE depth_stream_handle_;
    HANDLE rgb_stream_handle_;

    HANDLE th_nui_process_;
    HANDLE ev_nui_process_stop_;

    bool nui_initialized_; 
    int frames_total_;
    
    gevxl::img::visualizer_2d *viz_;

};
} // namespace kinectsdk
} // namespace pressure_ulcer
} // namespace gevxl
#endif 