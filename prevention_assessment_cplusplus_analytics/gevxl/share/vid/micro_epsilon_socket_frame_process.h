// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/25/2014
/// \brief the frame_process for the Micro-Epsilon camera through socket communication
/// - Original version

#ifndef micro_epsilon_socket_frame_process_h_
#define micro_epsilon_socket_frame_process_h_

#include <vcl_string.h>
#include <vil/vil_image_view.h>
#include <vid/frame_tag.h>
#include <vid/tagged_frame_process.h>
#include <util/config_file.h>

#include <util/time/highres_time.h>
//#include "micro_epsilon_socket_camera.h"
#include <vcl_iostream.h>

namespace gevxl {
namespace vid {

  // forward declaration
  class micro_epsilon_socket_camera;

/// Capture frames from a socket stream that connects to a Micro-Epsilon ImagerIPC camera
///
class micro_epsilon_socket_frame_process : public gevxl::vid::tagged_frame_process<vxl_byte>
{
public:

  micro_epsilon_socket_frame_process( char const * process_name = "vid::micro_epsilon_socket_frame_process" );

  virtual ~micro_epsilon_socket_frame_process();

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

  /// Return the tag associated with the current frame
  virtual const gevxl::vid::frame_tag &cur_frame_tag(void) const { return tag_; }

  /// Return the current frame
  vil_image_view<vxl_byte> const &cur_frame() const;

  /// Return the float thermal frame
  vil_image_view<float> const &cur_thermal_frame_in_C() const; 
  vil_image_view<float> const &cur_thermal_frame_in_F() const; 

  /// Return the grayscale thermal image
  vil_image_view<vxl_byte> const &cur_thermal_byte_frame() const;

  /// Return the color palette based thermal image
  vil_image_view<vxl_byte> const &cur_thermal_color_palette_frame() const; 

	/// Return the color palette 
	const vcl_vector<vcl_vector<vxl_byte> > &get_color_palette(void) const { return color_palette_; }

	/// Set the file storage directory
	void set_file_storage_directory(const vcl_string &filepath) { filepath_ = filepath; }

	/// Set whether we save out the frames
	void set_save_out_flag(bool flag);
	bool get_save_out_flag(void) { return save_out_; }

  /// Control command to take snapshot image
  bool take_snapshot(const vcl_string &thermal_file_directory, 
                      const vcl_string &viewpoint_description);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // the new implementation to support the GUI controlled recording and playback
  /// Control command to start/stop/playback recording
  bool start_recording(const vcl_string &thermal_file_directory, 
                        const vcl_string &viewpoint_description);

  bool stop_recording(void);

  bool playback_recording(const vcl_string &thermal_file_directory, 
                          const vcl_string &viewpoint_description);


private:

	bool read_in_color_palette(vcl_string filepath);

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

  /// Configuration file
  gevxl::util::config_file config_;

  /// Tag for the current frame
  gevxl::vid::frame_tag tag_;

	/// Tag for the saved out frame
	gevxl::vid::frame_tag saved_out_tag_;

	/// Frame tags loaded from the file
	vcl_vector<gevxl::vid::frame_tag> loaded_tags_;

	/////////////////////////////////////////////////////
  /// Current thermal frame
	vil_image_view<float> thermal_frame_;

	/////////////////////////////////////////////////////
	vil_image_view<float> thermal_frame_in_C_;
  vil_image_view<float> thermal_frame_in_F_;
  vil_image_view<vxl_byte> thermal_byte_frame_;
  vil_image_view<vxl_byte> thermal_color_palette_frame_;

  /// Is the camera initialized?
  bool initialized_;

  /// is frame initialized?
  bool frame_initialized_;

  /// Verbosity level
  int verbose_;

  /// 0: visualize grayscale thermal image, i.e., thermal_byte_frame_
  /// 1: visualize color palette based thermal image, i.e., thermal_color_palette_frame_  
  int viz_mode_;

  /// frame number
  unsigned frame_nr_;

	/// saved out frame number 
	unsigned saved_out_frame_nr_;

  /// time management
  double captured_frame_system_time_;

  /// the height and width
  int img_height_;
  int img_width_;

  /// port number 
  int port_;

  /// whether the camera is upside down and left to right flipped
  bool upsidedown_leftright_flipping_;

	/// color palette
	vcl_vector<vcl_vector<vxl_byte> > color_palette_;

	/// maximum frame waiting time in millisecond.
	int max_frame_waiting_time_in_ms_;

  /// micro epsilon socket camera
  micro_epsilon_socket_camera *camera_;

	/// ofstream to write out the frame tag info;
	vcl_ofstream frame_tag_ofstream_;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // the new implementation to support the GUI controlled recording and playback
  // ofstream to write out the frame tag info for the GUI controlled recording;
  vcl_ofstream gui_thermal_frame_tag_ofstream_;
  vcl_ifstream *gui_thermal_frame_tag_ifstream_;

  // boolean flag to indicate whether the GUI controls to record frames
	bool gui_recording_flag_;
  bool gui_stop_recording_flag_;

  // boolean flag to indicate whether the GUI controls to playback frames
  bool gui_playback_flag_;
  
  // rgb_file_directory, depth_file_directory string
  vcl_string thermal_file_directory_;
};

}} // end namespaces

#endif
