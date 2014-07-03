// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 5/25/2014
/// \brief the frame_process for the Bayspec camera through socket communication
/// - Original version

#ifndef bayspec_socket_frame_process_h_
#define bayspec_socket_frame_process_h_

#include <vcl_string.h>
#include <vil/vil_image_view.h>
#include <vid/frame_tag.h>
#include <vid/tagged_frame_process.h>
#include <util/config_file.h>

#include <util/time/highres_time.h>
#include <vcl_iostream.h>

namespace gevxl {
namespace vid {

  // forward declaration
  class bayspec_socket_camera;

/// Capture frames from a socket stream that connects to a Micro-Epsilon ImagerIPC camera
///
class bayspec_socket_frame_process : public gevxl::vid::tagged_frame_process<vxl_byte>
{
public:

  bayspec_socket_frame_process( char const * process_name = "vid::bayspec_socket_frame_process" );

  virtual ~bayspec_socket_frame_process();

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

  /// Return the current three wavelength bands frame
  vil_image_view<vxl_byte> const &cur_three_wavelength_bands_frame() const;

  /// Set the file storage directory
	void set_file_storage_directory(const vcl_string &filepath) { file_storage_directory_ = filepath; }

	/// Set whether we save out the frames
	void set_save_out_flag(bool flag);
	bool get_save_out_flag(void) { return save_out_; }

  /// Control command to take snapshot image
  bool take_snapshot(const vcl_string &hyperspectral_file_directory);

private:

  /// Load the frame tag information if it is in the playback mode.
	bool frame_tags_load(void);

  /// Load the frames if it is in the playback mode.
	bool playback_frames_load(const int frame_nr);

  /// Retrieve the frame from the camera's file system.
  void retrieve_last_captured_frame(const vcl_string &file_system_root_folder);

  /// Rotating the captured frame according to the rotation_info_ configuration description
  void rotate_captured_frame(void);

	// file storage directory
	vcl_string file_storage_directory_;

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

  /// Is the camera initialized?
  bool initialized_;

  /// is frame initialized?
  bool frame_initialized_;

	/// Verbosity level
  int verbose_;

  /// 0: visualize grayscale image based on one wavelength band,
  /// 1: visualize color image based on three selected wavelength bands.
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

  /// camera server host name or ip address
  vcl_string server_host_or_ip_;

  /// port number 
  vcl_string port_;

	/// maximum frame waiting time in millisecond.
	int max_frame_waiting_time_in_ms_;

  /// camera control command sending interval elapse in milliseconds
  int camera_com_msg_interval_elapse_in_ms_;

  /// frame nr of last sent-out camera control command
  unsigned frame_nr_of_last_camera_com_msg_;

  /// the timestamp of last sent-out camera control command 
  double timestamp_of_last_camera_com_msg_;

  /// bayspec socket camera
  bayspec_socket_camera *camera_;	

	/// ofstream to write out the frame tag info;
	vcl_ofstream frame_tag_ofstream_;

  /// white reference folder info
  vcl_string white_ref_folder_;

  /// dark background folder info
  vcl_string dark_bkg_folder_;

  /// the current frame
  int wavelength_band_idx_;
  vil_image_view<vxl_byte> frame_;

  /// the selected three channel frames
  vcl_vector<int> three_wavelength_bands_idx_;  // [3]
  vil_image_view<vxl_byte> three_wavelength_bands_frame_;
  vil_image_view<vxl_byte> three_one_wavelength_band_frames_[3];

  /// the complete wavelength band frames
  vcl_vector<double> wavelength_bands_;
  vcl_vector<vil_image_view<vxl_byte> > wavelength_band_frames_;

  // rotating the captured frames 90 degree clockwise or counter-clockwise or clockwise_180 or upside_down or none
  vcl_string rotation_info_;
};

}} // end namespaces

#endif
