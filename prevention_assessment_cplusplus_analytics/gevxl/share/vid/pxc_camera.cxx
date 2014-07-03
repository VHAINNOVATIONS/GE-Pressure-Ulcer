// Copyright (C) 2013 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "pxc_camera.h"
#include <threading/sleep.h>

#include <vil/vil_save.h>
#include <vul/vul_sprintf.h>
#include <vcl_limits.h>

#ifdef GEVXL_VID_HAS_PCSDK
  #include <util_pipeline.h>
  #include <pxcsmartptr.h>
  #include <pxcsession.h>
  #include <util_capture.h>
  #include <util_render.h>
  #include <pxcprojection.h>
  #include <pxcmetadata.h>
#endif

#if defined(VCL_WIN32) && defined(VCL_VC)
  #include <Windows.h>
#else
  #include <sys/time.h>
  #include <sys/stat.h>
#endif

using namespace gevxl::vid;

gevxl::threading::mutex pxc_camera::pxc_driver_mutex_;
bool pxc_camera::pxc_initialized_ = false;
bool pxc_camera::pxc_disconnected_ = false;

pxc_camera::pxc_camera(bool live, vcl_string pxc_filename, int img_height, int img_width, int depth_height, int depth_width, bool asynchronous_grabbing)
: new_frame_ready_(false)
{
	pp_ = NULL;

  pxc_depth_frame_ = NULL;
  pxc_rgb_frame_ = NULL;

	const char *c = pxc_filename.c_str();
	const size_t cSize = strlen(c)+1;
  wchar_t *wc = new wchar_t[cSize];
  mbstowcs(wc, c, cSize);

	// live = true, (pxc_filename != "") or (pxc_filename = "")
	// live = false, (pxc_filename != "")
	if(live == true) {
		if(pxc_filename != "") {
			// live capturing and recording
			pp_ = new UtilPipeline(0, wc, true);
		}
		else {
			// live capturing but no recording
			pp_ = new UtilPipeline(0, 0, false);
		}
	}
	else if(live == false) {
		if(pxc_filename != "") {
			// playback video
			pp_ = new UtilPipeline(0, wc, false);
		}		
	}

	img_height_ = img_height;
  img_width_ = img_width;

	depth_height_ = depth_height;
  depth_width_ = depth_width;

	// check whether the pxc driver has been initialized or not
	pxc_driver_mutex_.lock();

	if(!pxc_initialized_) {

    pp_->EnableImage(PXCImage::COLOR_FORMAT_RGB32, img_width_, img_height_);
    //pp_->EnableImage(PXCImage::COLOR_FORMAT_DEPTH);
    pp_->EnableImage(PXCImage::COLOR_FORMAT_VERTICES);
    bool rc = pp_->Init();

		if(!rc) 
    {
			vcl_cerr << "pxc_camera::pxc_camera(), Initialize PXC camera failed" << vcl_endl;
			pxc_driver_mutex_.unlock();
			return;
		}

		// the pxc camera has been successfully initialized
		pxc_initialized_ = true;
	}
	pxc_driver_mutex_.unlock();
  
	// create local buffers
  local_depth_frame_ = vil_image_view<vxl_uint_16>(depth_width_, depth_height_, 1, 1);
  local_depth_frame_.fill(0);

  local_rgb_frame_ = vil_image_view<vxl_byte>(img_width_, img_height_, 1, 3);    
  local_rgb_frame_.fill(0);

  local_xyz_rgb_frame_ = vil_image_view<float>(depth_width_, depth_height_, 1, 6);
  local_xyz_rgb_frame_.fill(0.0);
  
  local_depth_to_rgb_coordinate_map_ = vil_image_view<int>(depth_width_, depth_height_, 1, 2);
	local_depth_to_rgb_coordinate_map_.fill(-1);

  new_frame_ready_ = false;

  // start the grabbing thread
	asynchronous_grabbing_ = asynchronous_grabbing;
	if(asynchronous_grabbing_) {
		start_thread();
	}
}

pxc_camera::~pxc_camera()
{
	// shutdown the pxc camera if it has not been shutdowned yet.
	pxc_driver_mutex_.lock();
	if(!pxc_disconnected_) {
		pp_->Close();
		pxc_disconnected_ = true;
	}
	pxc_driver_mutex_.unlock();

	// check if the device has really been initialized or not
	if(!pxc_initialized_) return;

	if(asynchronous_grabbing_) {
		set_time_to_stop_flag();
		wait_until_stopped();
	}

  if(!pp_) {
    delete pp_;
    pp_ = NULL;
  }

	pxc_initialized_ = false;
}

// retouine that gives the next frame
bool pxc_camera::get_next_frames(vil_image_view<vxl_uint_16> &depth_frame, 
                                 vil_image_view<vxl_byte> &rgb_frame,
                                 vil_image_view<float> &xyz_rgb_frame,                                 
																 vil_image_view<int> &depth_to_rgb_coordinate_map,
                                 double &captured_frame_system_time)
{
  if(asynchronous_grabbing_) {
		// asynchronous grabbing
		
    bool new_frame_ready = false;
    
    grab_slot_lock_.lock();
    new_frame_ready = new_frame_ready_;
    grab_slot_lock_.unlock();

    if(!new_frame_ready) {
      // new frame is not ready yet, return false
      return false;
    }

		grab_slot_lock_.lock();
		// copy the data out of the depth_frame_ref_, rgb_frame_ref_
    // to the vxl_image_view (depth_frame, rgb_frame)
    if(new_frame_ready_) {
      
      // set the captured time
      captured_frame_system_time = captured_frame_system_time_;

      // copy local frames to outside
      rgb_frame.deep_copy(local_rgb_frame_);
      depth_frame.deep_copy(local_depth_frame_);
      xyz_rgb_frame.deep_copy(local_xyz_rgb_frame_);
      
			depth_to_rgb_coordinate_map.deep_copy(local_depth_to_rgb_coordinate_map_);

      new_frame_ready_ = false;
    }
		grab_slot_lock_.unlock();
	}
	else {
		// synchronous grabbing
		if(!pp_->AcquireFrame(true)) {
      vcl_cerr << "pxc_camera::get_next_frames(), Acquire failed" << vcl_endl;
      return false;
    }

    pxc_rgb_frame_ = pp_->QueryImage(PXCImage::IMAGE_TYPE_COLOR);
    pxc_depth_frame_ = pp_->QueryImage(PXCImage::IMAGE_TYPE_DEPTH);

    //hires_time_.set_to_system_time();
    //captured_frame_system_time_ = hires_time_.get_time_in_ms();

		FILETIME ft;
		GetSystemTimeAsFileTime( &ft );
		// ft values are in 100 nanoseconds = 10^-7 seconds, so divide by
		// 10^-4 to get milliseconds.
		captured_frame_system_time_ = (ft.dwHighDateTime * 4294967296.0 + ft.dwLowDateTime) / 10000.0;

    // copy the data out of the pxc_depth_frame_, pxc_color_frame_
    // to the vxl_image_view (depth_frame, rgb_frame)
    
    // set the captured time
    captured_frame_system_time = captured_frame_system_time_;
    // copy the frames to outside
    copy_video_frames(depth_frame, rgb_frame, xyz_rgb_frame, depth_to_rgb_coordinate_map);

    pp_->ReleaseFrame();
  }

  return true;
}

void pxc_camera::run_thread(void)
{
	do {
		// Thread is now grabbing frames
		capture_next_frames();
	}
	while(!get_time_to_stop_flag());
}

// used in thread to grab next frames
void pxc_camera::capture_next_frames(void)
{
  grab_slot_lock_.lock();

 	if(!pp_->AcquireFrame(true)) {
    vcl_cerr << "pxc_camera::capture_next_frames(), Acquire failed" << vcl_endl;
    return;
  }

  pxc_rgb_frame_ = pp_->QueryImage(PXCImage::IMAGE_TYPE_COLOR);
  pxc_depth_frame_ = pp_->QueryImage(PXCImage::IMAGE_TYPE_DEPTH); 
  
  //hires_time_.set_to_system_time();
  //captured_frame_system_time_ = hires_time_.get_time_in_ms();

	FILETIME ft;
	GetSystemTimeAsFileTime( &ft );
	// ft values are in 100 nanoseconds = 10^-7 seconds, so divide by
	// 10^-4 to get milliseconds.
	captured_frame_system_time_ = (ft.dwHighDateTime * 4294967296.0 + ft.dwLowDateTime) / 10000.0;

  // because the way PCSDK pipeline works, have to copy the frame here before releasing
  copy_video_frames(local_depth_frame_, local_rgb_frame_, local_xyz_rgb_frame_, local_depth_to_rgb_coordinate_map_);

  new_frame_ready_ = true;

  pp_->ReleaseFrame();

  grab_slot_lock_.unlock();
 
  // probably it's better to sleep a little bit ?
  gevxl::threading::sleep(1);
}

void pxc_camera::copy_video_frames(vil_image_view<vxl_uint_16> &depth_frame, 
                                   vil_image_view<vxl_byte> &rgb_frame,
                                   vil_image_view<float> &xyz_rgb_frame,                                   
																	 vil_image_view<int> &depth_to_rgb_coordinate_map)
{
  ////////////////////////////////////////////////////////////////////////
  UtilCapture *cap = pp_->QueryCapture();
  PXCCapture *capture = cap->QueryCapture();
  PXCSession *session = pp_->QuerySession();

  // special depth values for saturated and low-confidence pixels
  pxcF32 dvalues[2];

  // projection serializable identifier
  pxcUID prj_value;
  PXCSmartPtr<PXCProjection> projection;

  // setup the projection info for getting a nice depth map
  pxcStatus sts = cap->QueryDevice()->QueryPropertyAsUID(PXCCapture::Device::PROPERTY_PROJECTION_SERIALIZABLE, &prj_value);
  if(sts >= PXC_STATUS_NO_ERROR) {
    // create properties for checking if depth values are bad (by low confidence and by saturation)
    cap->QueryDevice()->QueryProperty(PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE, &dvalues[0]);
    cap->QueryDevice()->QueryProperty(PXCCapture::Device::PROPERTY_DEPTH_SATURATION_VALUE, &dvalues[1]);

    session->DynamicCast<PXCMetadata>()->CreateSerializable<PXCProjection>(prj_value, &projection);
  }
  ////////////////////////////////////////////////////////////////////////

  // pointers to the PXC images
  PXCImage::ImageData idata;
  pxc_rgb_frame_->AcquireAccess(PXCImage::ACCESS_READ, &idata);
  unsigned char *rgb_image_data = (unsigned char *)idata.planes[0];
  
  PXCImage::ImageData ddata;
  pxc_depth_frame_->AcquireAccess(PXCImage::ACCESS_READ, &ddata);
  short *depth_image_data = (short *)ddata.planes[0];

  // copy rgb	
  for (int yy=0;yy<img_height_;yy++)
  {
    for (int xx=0;xx<img_width_;xx++)
    {
       unsigned char *rgb_ptr = rgb_image_data + 3*(yy*img_width_+xx);
       rgb_frame(xx,yy,0) = rgb_ptr[2];
       rgb_frame(xx,yy,1) = rgb_ptr[1];
       rgb_frame(xx,yy,2) = rgb_ptr[0];
    }
  }

   // pointers to the vil images 
  //vxl_uint_16 *depth_frame_ptr = depth_frame.top_left_ptr();

  // copy depth
  //int depth_size_ = depth_width_*depth_height_;
  //for(int i = 0; i < depth_size_; i++) {
  //   depth_frame_ptr[i] = depth_image_data[i];
  //}

  // generate the xyz_rgb frame
  float *uvmap = (float*)ddata.planes[2];
  
  int xx, yy;
  unsigned int idx;
  short x, y, z;
  float r, g, b;

  //int count_of_good_rgb = 0;
  //int count_of_good_point_cloud = 0;	
  for(int j = 0; j < depth_height_ ; j++)
  {
    for(int i = 0; i < depth_width_ ; i++)
    {
      xx = (int) (uvmap[(j*depth_width_+i)*2+0]*img_width_+0.5f);
      yy = (int) (uvmap[(j*depth_width_+i)*2+1]*img_height_+0.5f);

			depth_to_rgb_coordinate_map(i,j,0) = xx;
			depth_to_rgb_coordinate_map(i,j,1) = yy;

      idx = 3*(j*depth_width_ + i);

      x = depth_image_data[idx];
      y = depth_image_data[idx+1];
      z = depth_image_data[idx+2];

      r = 0.0f;
      g = 0.0f;
      b = 0.0f;

      if(xx < 0 || xx >= img_width_ || yy < 0 || yy >= img_height_)
      {
      }
      else
      {
        unsigned char *rgb_ptr = rgb_image_data + 3*(yy*img_width_+xx);

        r = float(rgb_ptr[2]);
        g = float(rgb_ptr[1]);
        b = float(rgb_ptr[0]);

        //count_of_good_rgb++;
      }

      if(z == dvalues[0] || z == dvalues[1]) {
        // saturated and low-confidence depth pixel, throw it away, both xyz value and rgb value
        depth_frame(i,j) = 0;

        // normalize vertices to meters
        xyz_rgb_frame(i,j,0) = 0.0f;
        xyz_rgb_frame(i,j,1) = 0.0f;
        xyz_rgb_frame(i,j,2) = 0.0f;

        xyz_rgb_frame(i,j,3) = 0.0f;
        xyz_rgb_frame(i,j,4) = 0.0f;
        xyz_rgb_frame(i,j,5) = 0.0f;        
      }
      else {
        // valid depth pixel
        depth_frame(i,j) = z;

        // normalize vertices to meters
        xyz_rgb_frame(i,j,0) = float(x) / 1000.0f;
        xyz_rgb_frame(i,j,1) = float(y) / 1000.0f;
        xyz_rgb_frame(i,j,2) = float(z) / 1000.0f;

        // this is a valid depth pixel, but its rgb value may still be not available, as it may be outside of the RGB field of view.
        xyz_rgb_frame(i,j,3) = r;
        xyz_rgb_frame(i,j,4) = g;
        xyz_rgb_frame(i,j,5) = b;

        //count_of_good_point_cloud++;        
      }      
    }
  }

  //vcl_cout << "# of good RGB pixels = " << count_of_good_rgb << vcl_endl;
  //vcl_cout << "# of good point cloud pixels = " << count_of_good_point_cloud << vcl_endl;

  pxc_rgb_frame_->ReleaseAccess(&idata);
  pxc_depth_frame_->ReleaseAccess(&ddata);
}
