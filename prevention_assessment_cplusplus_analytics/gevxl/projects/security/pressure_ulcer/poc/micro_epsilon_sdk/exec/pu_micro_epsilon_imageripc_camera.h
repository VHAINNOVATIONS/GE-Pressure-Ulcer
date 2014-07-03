// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/22/2014
/// \brief the wrapper for the Micro-Epsilon camera
/// - Original version

#ifndef micro_epsilon_imageripc_camera_h_
#define micro_epsilon_imageripc_camera_h_

#include "vnl/vnl_matrix.h"
#include "vil/vil_image_view.h"

#include <threading/thread.h>
#include <threading/mutex.h>

#include <windows.h>
#include "ImagerIPC2.h"

#include "pu_micro_epsilon_imageripc_setup_thread.h"

namespace gevxl { 
namespace vid {

class micro_epsilon_imageripc_camera : public gevxl::threading::thread
{
public:

  // constructor
  micro_epsilon_imageripc_camera();
  // deconstructor
  ~micro_epsilon_imageripc_camera();

  void get_next_frame(vil_image_view<vxl_uint_16> &thermal_frame);

  int get_img_height(void) { return img_height_; }
  int get_img_width(void) { return img_width_; }

private:

  // initialize camera
  void initialize_camera(void);

  // run thread
  void run_thread(void);

  // capture next frame
  void capture_next_frame(void);

  // copy video frame buffer
  void copy_video_frame_ref(vil_image_view<vxl_uint_16> &thermal_frame);

  // callback functions for the 
  static HRESULT WINAPI OnServerStopped(int reason);
  static HRESULT WINAPI OnFrameInit(int width, int height, int depth);
  static HRESULT WINAPI OnNewFrame(char *buffer, int frame_counter);
  static HRESULT WINAPI OnNewFrameEx(void *buffer, FrameMetadata *frame_meta_data);
  static HRESULT WINAPI OnInitCompleted(void);
  static HRESULT WINAPI OnConfigChanged(long reserved);
  static HRESULT WINAPI OnFileCommandReady(wchar_t *filepath);

  // micro epsilon imageripc initialization variables
  static gevxl::threading::mutex micro_epsilon_imageripc_mutex_;
  static bool micro_epsilon_imageripc_initialized_;
  static bool micro_epsilon_imageripc_shutdowned_;

  // setup
  micro_epsilon_imageripc_setup_thread micro_epsilon_imageripc_setup_;
  
  // live camera or ONI file
  bool live_;

  // new frame ready flag
  bool new_frame_ready_;

  // the mutex lock that only locks the grabbing slot of the VideoStreams
  gevxl::threading::mutex grab_slot_lock_;

  // the height and width of the rgb and depth image
  int img_height_;
  int img_width_;
};

}} // end namespaces

#endif
