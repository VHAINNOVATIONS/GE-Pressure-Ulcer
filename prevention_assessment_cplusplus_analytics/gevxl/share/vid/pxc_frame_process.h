// Copyright (C) 2013 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Yi Xu
/// \date 1/24/2014
/// \brief the frame process wrapping around the pxc_camera
/// - Original version

#ifndef pxc_frame_process_h_
#define pxc_frame_process_h_

#include <vcl_string.h>
#include <vil/vil_image_view.h>
#include <vid/frame_tag.h>
#include <vid/tagged_frame_process.h>
#include <util/config_file.h>

#include <util_pipeline.h>

#include "pxc_camera.h"

#include <vcl_iostream.h>

namespace gevxl {
namespace vid {

/// Capture frames from an Intel Peceptual Computing (PXC) compliant camera using the PCSDK
class pxc_frame_process : public gevxl::vid::tagged_frame_process<vxl_byte>
{
public:

  /// Create a video source process
  ///
  /// The configuration parameter names, e.g. \c type, will be
  /// prefixed by \a process_name. This defaults to
  /// "vid::pxc_frame_process", so the source type is by default read
  /// from vid::pxc_frame_process::type.
  ///
  pxc_frame_process( char const * process_name = "vid::pxc_frame_process" );

  virtual ~pxc_frame_process();

  /// Configure the camera and the process
  virtual bool configure( gevxl::util::config_file & config );

  /// Set the visualization mode
  void change_viz_mode(void)
  { 
    if(viz_mode_ == 0) {
      viz_mode_ = 1;
    }
    else {
      viz_mode_ = 0;
    }
  }

  /// Move to the next frame
  virtual bool step();

  /// Return the current frame
  vil_image_view<vxl_byte> const &cur_frame() const;

  /// Return the tag associated with the current frame
  virtual const gevxl::vid::frame_tag &cur_frame_tag(void) const { return tag_; }

  /// Return RGB frame
  vil_image_view<vxl_byte> const &cur_rgb_frame() const; 

  /// Return depth frame
  vil_image_view<vxl_uint_16> const &cur_depth_frame() const; 

  /// Return xyz_rgb frame in float byte
  vil_image_view<float> const &cur_xyz_rgb_frame() const;

	/// Return depth to rgb coordinate map
  vil_image_view<int> const &cur_depth_to_rgb_coordinate_map() const; 

	/// Return depth frame in 8-bit byte
  vil_image_view<vxl_byte> const &cur_depth_byte_frame() const;

  /// Return RGB frame in depth view
  vil_image_view<vxl_byte> const &cur_rgb_frame_in_depth_view() const;

	/// Set the file storage directory
	void set_file_storage_directory(const vcl_string &filepath) { filepath_ = filepath; }

	/// Set whether we save out the frames
	void set_save_out_flag(bool flag);
	bool get_save_out_flag(void) { return save_out_; }
	
  /// Control command to take snapshot images
  bool take_snapshot(const vcl_string &rgb_file_directory, const vcl_string &depth_file_directory, 
                      const vcl_string &viewpoint_description);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // the new implementation to support the GUI controlled recording and playback
  /// Control command to start/stop/playback recording
  bool start_recording(const vcl_string &rgb_file_directory, const vcl_string &depth_file_directory, 
                       const vcl_string &viewpoint_description);

  bool stop_recording(void);

  bool playback_recording(const vcl_string &rgb_file_directory, const vcl_string &depth_file_directory, 
                          const vcl_string &viewpoint_description);

private:

  /// Initialize the camera.
  void camera_initialize();

  /// Uninitialize the camera.
  void camera_uninitialize();

	/// Camera frames save.
	bool camera_frames_save(const int frame_nr);

	/// Load the frame tag information if it is in the playback mode.
	bool frame_tags_load(void);

	/// Load the frames if it is in the playback mode.
	bool playback_frames_load(const int frame_nr);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // the new implementation to support the GUI controlled recording and playback
  // record one frame
  bool recording_frame(void);

  // playback one frame
  bool playback_frame(void);

private:

	// file storage directory
	vcl_string filepath_;

	// live or playback model
	vcl_string capture_mode_;	// = camera or playback

	// boolean flag to indicate whether we save out the frames
	bool save_out_;

  /// asynchronous grabbing flag
  bool asynchronous_grabbing_;

  /// Configuration file
  gevxl::util::config_file config_;

  /// Tag for the current frame
  gevxl::vid::frame_tag tag_;

	/// Tag for the saved out frame
	gevxl::vid::frame_tag saved_out_tag_;
	
	/// Frame tags loaded from the file
	vcl_vector<gevxl::vid::frame_tag> loaded_tags_;

	/////////////////////////////////////////////////////
  /// Current RGB frame
  vil_image_view<vxl_byte> rgb_frame_;

  /// Current depth frame
  vil_image_view<vxl_uint_16> depth_frame_;
  
  /// Current xyz_rgb frame
  vil_image_view<float> xyz_rgb_frame_;

  /// Current depth to rgb coordinate mapping frame
  vil_image_view<int> depth_to_rgb_coordinate_map_;

	/////////////////////////////////////////////////////
	/// Current depth byte frame, derived frame
	vil_image_view<vxl_byte> depth_byte_frame_;

  /// Current RGB frame in depth view, derived frame
  vil_image_view<vxl_byte> rgb_frame_in_depth_view_;
	
	/////////////////////////////////////////////////////

	/// camera
  pxc_camera *camera_;
	bool camera_live_;	
  vcl_string pxc_filename_;
	
  /// Is the camera initialized?
  bool camera_initialized_;

	/// Current frame
  vil_image_view<vxl_byte> frame_;
 
  /// Verbosity level
  int verbose_;

  /// 0: visualize color image
  /// 1: visualize depth image  
  int viz_mode_;

  /// frame number
  unsigned frame_nr_;

	/// saved out frame number 
	unsigned saved_out_frame_nr_;

	/// captured frame system time
  double captured_frame_system_time_;

  // maximum frame waiting time in millisecond.
	int max_frame_waiting_time_in_ms_;

	// the height and width of the rgb and depth image
	int img_height_;
  int img_width_;
  int depth_height_;
  int depth_width_;

	// ofstream to write out the frame tag info;
	vcl_ofstream frame_tag_ofstream_;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // the new implementation to support the GUI controlled recording and playback
  // ofstream to write out the frame tag info for the GUI controlled recording;
  vcl_ofstream gui_rgb_frame_tag_ofstream_;
  vcl_ofstream gui_depth_frame_tag_ofstream_;

  vcl_ifstream *gui_rgb_frame_tag_ifstream_;
  vcl_ifstream *gui_depth_frame_tag_ifstream_;

  // boolean flag to indicate whether the GUI controls to record frames
	bool gui_recording_flag_;
  bool gui_stop_recording_flag_;

  // boolean flag to indicate whether the GUI controls to playback frames
  bool gui_playback_flag_;
  
  // rgb_file_directory, depth_file_directory string
  vcl_string rgb_file_directory_;
  vcl_string depth_file_directory_;
};

}} // namespace

#endif // pxc_frame_process_h_
