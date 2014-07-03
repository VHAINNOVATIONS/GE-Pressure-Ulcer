// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include <vcl_iostream.h>

#include "pu_micro_epsilon_imageripc_camera.h"
#include <threading/sleep.h>

#include <vil/vil_save.h>
#include <vul/vul_sprintf.h>

using namespace gevxl::vid;

gevxl::threading::mutex micro_epsilon_imageripc_camera::micro_epsilon_imageripc_mutex_;
bool micro_epsilon_imageripc_camera::micro_epsilon_imageripc_initialized_ = false;
bool micro_epsilon_imageripc_camera::micro_epsilon_imageripc_shutdowned_ = false;

micro_epsilon_imageripc_camera::micro_epsilon_imageripc_camera()
: live_(true),   
  new_frame_ready_(false)
{
	// check whether the micro epsilon imageripc has been initialized or not
	micro_epsilon_imageripc_mutex_.lock();
	if(!micro_epsilon_imageripc_initialized_) {

		initialize_camera();
		if(false) {
			vcl_cerr << "micro_epsilon_imageripc_camera::micro_epsilon_imageripc_camera(), Initialize micro epsilon imageripc failed." << vcl_endl;
			micro_epsilon_imageripc_mutex_.unlock();
			return;
		}
		// the micro epsilon imageripc has been successfully initialized
		micro_epsilon_imageripc_initialized_ = true;

	}
	micro_epsilon_imageripc_mutex_.unlock();	
}


void micro_epsilon_imageripc_camera::initialize_camera(void)
{
  // do something with the pu_micro_epsilon imageripc setup thread to setup the imageripc
  // to be continued
  micro_epsilon_imageripc_setup_.set_fpOnServerStopped( &(micro_epsilon_imageripc_camera::OnServerStopped) );
  micro_epsilon_imageripc_setup_.set_fpOnFrameInit( &(micro_epsilon_imageripc_camera::OnFrameInit) );
  micro_epsilon_imageripc_setup_.set_fpOnNewFrame( &(micro_epsilon_imageripc_camera::OnNewFrame) );
  micro_epsilon_imageripc_setup_.set_fpOnNewFrameEx( &(micro_epsilon_imageripc_camera::OnNewFrameEx) );
  micro_epsilon_imageripc_setup_.set_fpOnInitCompleted( &(micro_epsilon_imageripc_camera::OnInitCompleted) );
  micro_epsilon_imageripc_setup_.set_fpOnConfigChanged( &(micro_epsilon_imageripc_camera::OnConfigChanged) );
  micro_epsilon_imageripc_setup_.set_fpOnFileCommandReady( &(micro_epsilon_imageripc_camera::OnFileCommandReady) );

  micro_epsilon_imageripc_setup_.start_thread();

	img_height_ = 288;
	img_width_ = 382;

  new_frame_ready_ = false;

	//start_thread();	
}

micro_epsilon_imageripc_camera::~micro_epsilon_imageripc_camera()
{
	// shutdown the micro epsilon imageripc if it has not been shutdowned yet.
	micro_epsilon_imageripc_mutex_.lock();
	if(!micro_epsilon_imageripc_shutdowned_) {
		// do something here to shut down the micro epsilon imageripc
		// the micro epsilon imageripc is shutdowned
		micro_epsilon_imageripc_shutdowned_ = true;
	}
	micro_epsilon_imageripc_mutex_.unlock();

	set_time_to_stop_flag();
	wait_until_stopped();
}

void micro_epsilon_imageripc_camera::get_next_frame(vil_image_view<vxl_uint_16> &thermal_frame)
{
  // allocate the vxl_image_view memories if they haven't been allocated yet.
  if(thermal_frame.ni() != img_width_ || thermal_frame.nj() != img_height_ || thermal_frame.nplanes() != 1) {
    thermal_frame = vil_image_view<vxl_uint_16>(img_width_, img_height_, 1, 1);
    thermal_frame.fill(0);
  }

  // asynchronous grabbing		
	grab_slot_lock_.lock();
	// copy the data out of the buffer from the micro-epsilon imageripc's buffer
  // to the vxl_image_view (depth_frame)
  if(new_frame_ready_) {
    copy_video_frame_ref(thermal_frame);
    new_frame_ready_ = false;
  }
	grab_slot_lock_.unlock();
}

void micro_epsilon_imageripc_camera::run_thread(void)
{
	do {
		// Thread is now grabbing frames
		capture_next_frame();
	}
	while(!get_time_to_stop_flag());
}

void micro_epsilon_imageripc_camera::capture_next_frame(void)
{
  // do something with the micro-epsilon imageripc's callback

  grab_slot_lock_.lock();
  // do something
  new_frame_ready_ = true;
	grab_slot_lock_.unlock();
  
  // probably it's better to sleep a little bit ?
  gevxl::threading::sleep(20);
}

void micro_epsilon_imageripc_camera::copy_video_frame_ref(vil_image_view<vxl_uint_16> &depth_frame)
{
  
}

// callback functions for the 
HRESULT WINAPI micro_epsilon_imageripc_camera::OnServerStopped(int reason)
{
  vcl_cout << "In OnServerStopped()" << vcl_endl;
  return 0;
}
  
HRESULT WINAPI micro_epsilon_imageripc_camera::OnFrameInit(int width, int height, int depth)
{
  vcl_cout << "In OnFrameInit()" << vcl_endl;
  return 0;
}

HRESULT WINAPI micro_epsilon_imageripc_camera::OnNewFrame(char *buffer, int frame_counter)
{
  vcl_cout << "In OnNewFrame()" << vcl_endl;
  return 0;
}
  
HRESULT WINAPI micro_epsilon_imageripc_camera::OnNewFrameEx(void *buffer, FrameMetadata *frame_meta_data)
{
  vcl_cout << "In OnNewFrameEx()" << vcl_endl;
  return 0;
}
  
HRESULT WINAPI micro_epsilon_imageripc_camera::OnInitCompleted(void)
{
  vcl_cout << "In OnInitCompleted()" << vcl_endl;
  return 0;
}
  
HRESULT WINAPI micro_epsilon_imageripc_camera::OnConfigChanged(long reserved)
{
  vcl_cout << "In OnConfigChanged()" << vcl_endl;
  return 0;
}
  
HRESULT WINAPI micro_epsilon_imageripc_camera::OnFileCommandReady(wchar_t *filepath)
{
  vcl_cout << "In OnFileCommandReady()" << vcl_endl;
  return 0;
}
