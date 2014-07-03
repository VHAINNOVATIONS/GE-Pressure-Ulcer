// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file 
/// \author Yi (Jessica) Yao
/// \date 6/8/2012
/// modified by Ting Yu, 01/12/2014
/// \par Modifications: 
/// - Original versioin

#include <vcl_utility.h>

#include <vil/vil_image_view.h>
#include <vil/vil_plane.h>
#include <vil/vil_save.h>

#include <util/time/global_time.h>
#include <util/string.h>

#include <img/integral_image.h>

#include <pressure_ulcer/poc/kinectsdk/pu_kinectsdk_kinfu_process.h>

using namespace gevxl::pressure_ulcer::kinectsdk;

/// Default constructor
pu_kinectsdk_kinfu_process::pu_kinectsdk_kinfu_process(const char *name) : 
 gevxl::vid::frame_process<vxl_byte>(name),
 visualize_(false),
 enabled_(false)
{    
    depth_type_ = NUI_IMAGE_TYPE_DEPTH;
    depth_res_ = NUI_IMAGE_RESOLUTION_640x480;
    
    near_mode_flag_ = false;
    
    color_type_ = NUI_IMAGE_TYPE_COLOR;
    color_res_ = NUI_IMAGE_RESOLUTION_640x480;
	      
    frame_set_ = false;
}

/// Default destructor
pu_kinectsdk_kinfu_process::~pu_kinectsdk_kinfu_process(void)
{

}

bool pu_kinectsdk_kinfu_process::configure(gevxl::util::config_file& config )
{
  if (!config.get_bool(name()+"::visualize", visualize_))
    visualize_ = false;

  if (!config.get_bool(name()+"::enabled", enabled_))
    enabled_ = false;

  if (!config.get_bool(name()+"::near_mode_flag", near_mode_flag_))
    near_mode_flag_ = false;

  return true;
}

//initialize
bool pu_kinectsdk_kinfu_process::initialize(void)
{ 
    INuiSensor * pNuiSensor;

    int iSensorCount = 0;
    HRESULT hr = NuiGetSensorCount(&iSensorCount);
    if (hr < 0) {
        return hr;
    }

    // Look at each Kinect sensor
    int i = 0;
    for (int i = 0; i < iSensorCount; ++i) {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr)) {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (SUCCEEDED(hr)) {
          kinectsdk_sensor_.set_sensor(pNuiSensor);
          break;
        }
        // This sensor wasn't OK, so release it since we're not using it
        else {
          pNuiSensor->Release();
        }
    }

    // Try to get the Kinect camera to work
    hr = kinectsdk_sensor_.init(depth_type_, depth_res_, near_mode_flag_, FALSE, color_type_, color_res_);
    if (SUCCEEDED(hr)) {
        kinectsdk_sensor_flag_ = TRUE;        
    }
    else {
        kinectsdk_sensor_flag_ = FALSE;
        vcl_cerr << "Could not initialize the Kinect sensor.\n";
        return false;
    }
    
    return true;
}

void pu_kinectsdk_kinfu_process::uninitialize(void)
{
  frame_.clear();
}

bool pu_kinectsdk_kinfu_process::step(void)
{
  if(!enabled_) return true;

  step_frame();

  //visuualize results
  visualize(viz_);

  return true;
}

void pu_kinectsdk_kinfu_process::visualize(gevxl::img::visualizer_2d *viz)
{
  if (!enabled_ || !visualize_)
    return;

  viz_->lock();
  viz_->set_image( frame_ );
  viz_->unlock();
  kinectsdk_sensor_.visualize(viz);  

  return;
}

void pu_kinectsdk_kinfu_process::step_frame(void)
{
    BOOL ret = TRUE;

    return;
}

