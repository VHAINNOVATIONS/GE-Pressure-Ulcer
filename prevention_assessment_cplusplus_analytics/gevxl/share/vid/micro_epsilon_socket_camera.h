// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/25/2014
/// \brief the wrapper for the Micro-Epsilon camera through socket communication
/// - Original version

#ifndef micro_epsilon_socket_camera_h_
#define micro_epsilon_socket_camera_h_

#include "vil/vil_image_view.h"

#include <threading/thread.h>
#include <threading/mutex.h>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <util/time/highres_time.h>

namespace gevxl { 
namespace vid {

const int MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH = 20000000;

class micro_epsilon_socket_camera
{
public:

  // constructor
  micro_epsilon_socket_camera(int img_height, int img_width, int port, bool upsidedown_leftright_flipping = true);

  // deconstructor
  ~micro_epsilon_socket_camera();

  // initialize socket server
  void initialize_socket_server(void);

  // get the next frame
  bool get_next_frame(vil_image_view<float> &thermal_frame, double &captured_frame_system_time);

  // get the image size
  int get_img_height(void) { return img_height_; }
  int get_img_width(void) { return img_width_; }

  // copy the message buffer from the socket session
  void copy_message_buffer(char *buffer, int length);

private:

  // copy thermal frame buffer
  bool copy_next_frame(vil_image_view<float> &thermal_frame, double &captured_frame_system_time);

  // live socket camera or saved thermal video file
  bool live_;

  // new frame ready flag  
  bool new_frame_ready_;

  // the mutex lock that only locks the grabbing slot  
  gevxl::threading::mutex write_slot_lock_;

  // the height and width
  int img_height_;
  int img_width_;

  // socket message buffer
  // [0] is for incoming socket message
  // [1] is for micro_epsilon_socket_frame_process to reconstruct the frame for read out.
  int write_message_length_;
  int read_message_length_;
  
  char write_message_buffer_[MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH];
  char read_message_buffer_[MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH];
  
  // socket port #
  int port_;

  // whether the camera is upside down and left to right flipped
  bool upsidedown_leftright_flipping_;

	// local frame copy captured time using the system time
	gevxl::util::time::highres_time hires_time_;

	int captured_year_;
	int captured_month_;
	int captured_day_;
	int captured_hour_;
	int captured_minute_;
	int captured_second_;
	int captured_millisecond_;

  double captured_frame_system_time_;
};

}} // end namespaces

#endif
