//
// Copyright (C) 2012 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.
//
/// \author Yi (Jessica) Yao
/// \data   06/08/2012
/// modified by Ting Yu, 01/12/2014

#include <math.h>
#include <pressure_ulcer/poc/kinectsdk/pu_kinectsdk_sensor.h>

using namespace gevxl::pressure_ulcer::kinectsdk;

pu_kinectsdk_sensor::pu_kinectsdk_sensor()
{
    sensor_ = NULL;

    next_depth_frame_event_ = NULL;
    next_rgb_frame_event_ = NULL;
    
    depth_stream_handle_ = NULL;
    rgb_stream_handle_ = NULL;
    
    th_nui_process_ = NULL;
    ev_nui_process_stop_ = NULL;
    
    nui_initialized_ = false;
    frames_total_ = 0;
}

pu_kinectsdk_sensor::~pu_kinectsdk_sensor()
{
    release();
}

HRESULT pu_kinectsdk_sensor::init(NUI_IMAGE_TYPE depthType, NUI_IMAGE_RESOLUTION depthRes, BOOL bNearMode, 
                                  BOOL bFallbackToDefault, 
                                  NUI_IMAGE_TYPE colorType, NUI_IMAGE_RESOLUTION colorRes)
{
    HRESULT hr = E_UNEXPECTED;
    if(NULL == sensor_) {
      return hr;
    }

    release(); // Deal with double initializations.

    if(colorType != NUI_IMAGE_TYPE_COLOR || depthType != NUI_IMAGE_TYPE_DEPTH) {
        return E_INVALIDARG;
    }

    DWORD width = 0;
    DWORD height = 0;

    NuiImageResolutionToSize(colorRes, width, height);
    NuiImageResolutionToSize(depthRes, width, height);

    frames_total_ = 0;
    
    next_depth_frame_event_ = CreateEvent(NULL, TRUE, FALSE, NULL);
    next_rgb_frame_event_ = CreateEvent(NULL, TRUE, FALSE, NULL);
    
    DWORD dwNuiInitDepthFlag = NUI_INITIALIZE_FLAG_USES_DEPTH;

    hr = sensor_->NuiInitialize(dwNuiInitDepthFlag | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR);
    if (FAILED(hr)) {
        return hr;
    }

    nui_initialized_ = true;

	  hr = sensor_->NuiImageStreamOpen(colorType, colorRes, 0, 2, 
                                      next_rgb_frame_event_, &rgb_stream_handle_);
    if (FAILED(hr)) {
        return hr;
    }

    hr = sensor_->NuiImageStreamOpen(depthType, depthRes, (bNearMode) ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0, 2,
                                      next_depth_frame_event_, &depth_stream_handle_);
    
    if (FAILED(hr)) {
        if(bNearMode && bFallbackToDefault) {
            hr = sensor_->NuiImageStreamOpen(depthType, depthRes, 0, 2,
                                              next_depth_frame_event_, &depth_stream_handle_);
        }

        if(FAILED(hr)) {
            return hr;
        }
    }

    // Start the Nui processing thread
    ev_nui_process_stop_ = CreateEvent(NULL, TRUE, FALSE, NULL);
    th_nui_process_ = CreateThread(NULL, 0, process_thread, this, 0, NULL);

    nui_depth_image_frame_ = new NUI_IMAGE_FRAME;
    nui_rgb_image_frame_ = new NUI_IMAGE_FRAME;

    return hr;
}

void pu_kinectsdk_sensor::release()
{
    // Stop the Nui processing thread
    if(NULL != ev_nui_process_stop_) {
        // Signal the thread
        SetEvent(ev_nui_process_stop_);

        // Wait for thread to stop
        if(NULL != th_nui_process_) {
            WaitForSingleObject(th_nui_process_, INFINITE);
            CloseHandle(th_nui_process_);
            th_nui_process_ = NULL;
        }

        CloseHandle(ev_nui_process_stop_);
        ev_nui_process_stop_ = NULL;
    }

    if (nui_initialized_) {
        sensor_->NuiShutdown();
    }
    nui_initialized_ = false;

    if (next_depth_frame_event_ && next_depth_frame_event_ != INVALID_HANDLE_VALUE) {
        CloseHandle(next_depth_frame_event_);
        next_depth_frame_event_ = NULL;
    }

    if (next_rgb_frame_event_ && next_rgb_frame_event_ != INVALID_HANDLE_VALUE) {
        CloseHandle(next_rgb_frame_event_);
        next_rgb_frame_event_ = NULL;
    }
}

DWORD WINAPI pu_kinectsdk_sensor::process_thread(LPVOID p_param)
{
    pu_kinectsdk_sensor *pthis = (pu_kinectsdk_sensor *)p_param;
    
    HANDLE          hEvents[3];

    // Configure events to be listened on
    hEvents[0] = pthis->ev_nui_process_stop_;
    hEvents[1] = pthis->next_depth_frame_event_;
    hEvents[2] = pthis->next_rgb_frame_event_;    

    // Main thread loop
    while (true) {
        // Wait for an event to be signaled
        WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]), hEvents, FALSE, 100);

        // If the stop event is set, stop looping and exit
        if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->ev_nui_process_stop_, 0)) {
            break;
        }

        // Process signal events
        if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->next_depth_frame_event_, 0)) {
            pthis->got_depth_alert();
            pthis->frames_total_++;
        }

        if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->next_rgb_frame_event_, 0)) {
            pthis->got_rgb_alert();
        }        
    }

    return 0;
}

void pu_kinectsdk_sensor::got_rgb_alert(void)
{
    //NUI_IMAGE_FRAME* pImageFrame = NULL;

    HRESULT hr = sensor_->NuiImageStreamGetNextFrame(rgb_stream_handle_, 0, nui_rgb_image_frame_);
    if (FAILED(hr)) {
        return;
    }

    INuiFrameTexture* pTexture = nui_rgb_image_frame_->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch) {   
        // Copy video frame to face tracking
        //TY to be continued: memcpy(m_VideoBuffer->GetBuffer(), PBYTE(LockedRect.pBits), min(m_VideoBuffer->GetBufferSize(), UINT(pTexture->BufferLen())));
    }
    else {
      vcl_cerr <<   "Buffer length of received texture is bogus\r\n";
      //OutputDebugString(L"Buffer length of received texture is bogus\r\n");
    }

    hr = sensor_->NuiImageStreamReleaseFrame(rgb_stream_handle_, nui_rgb_image_frame_);
}


void pu_kinectsdk_sensor::got_depth_alert(void)
{
    //NUI_IMAGE_FRAME* pImageFrame = NULL;

    HRESULT hr = sensor_->NuiImageStreamGetNextFrame(depth_stream_handle_, 0, nui_depth_image_frame_);
    if (FAILED(hr)) {
        return;
    }

    INuiFrameTexture* pTexture = nui_depth_image_frame_->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch) {   
        // Copy depth frame to face tracking
        //TY to be continued: memcpy(m_DepthBuffer->GetBuffer(), PBYTE(LockedRect.pBits), min(m_DepthBuffer->GetBufferSize(), UINT(pTexture->BufferLen())));
    }
    else {
      vcl_cerr <<   "Buffer length of received depth texture is bogus\r\n";
      //OutputDebugString( L"Buffer length of received depth texture is bogus\r\n" );
    }

    hr = sensor_->NuiImageStreamReleaseFrame(depth_stream_handle_, nui_depth_image_frame_);
}

void pu_kinectsdk_sensor::visualize(gevxl::img::visualizer_2d *viz)
{
  viz_->lock();
  
  
  viz_->unlock();
  
  return;
}

