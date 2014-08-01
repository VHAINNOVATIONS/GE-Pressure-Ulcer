// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "micro_epsilon_socket_camera.h"
#include <threading/sleep.h>

#include <vil/vil_save.h>
#include <vul/vul_sprintf.h>
#include <util/string.h>

#include <vcl_iostream.h>
#include <vcl_sstream.h>

#if defined(VCL_WIN32) && defined(VCL_VC)
#include <Windows.h>
#else
#include <sys/time.h>
#include <sys/stat.h>
#endif

using namespace gevxl::vid;
using namespace gevxl::util;

//////////////////////////////////////////////////////////////////////////////////////////////////
// session of sockets communications
typedef class micro_epsilon_socket_session
{
public:
  micro_epsilon_socket_session(boost::asio::io_service& io_service, micro_epsilon_socket_camera *camera) 
    : socket_(io_service),
    camera_(camera) 
  {
    message_length_ = 0;
    message_count_ = 0;
  }

  boost::asio::ip::tcp::socket &socket(){ return socket_; }

  void start() {
    
    // read the message sent by the client in asnyc fashion
    socket_.async_read_some(boost::asio::buffer(received_data_, MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH),
      boost::bind(&micro_epsilon_socket_session::handle_read, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
  }

  void handle_read(const boost::system::error_code& error, size_t bytes_transferred) {

    if (!error) {

      // copy the data out from the received_data to message_buffer_
      memcpy(message_buffer_+message_length_, received_data_, bytes_transferred);
      message_length_ += bytes_transferred;

      if((char)(received_data_[bytes_transferred-1]) == '^') {

        if(message_count_ % 100 == 0) {
          vcl_cout << "message_length_ = " << message_length_ << vcl_endl;
        }

        // copy the data out from the message_buffer_ to the camera's buffer
        if(camera_) {
          camera_->copy_message_buffer(message_buffer_, message_length_);
        }

        memset(message_buffer_, 0, MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH);
        message_length_ = 0;
        message_count_++;

        if(message_count_ % 100 == 0) {
          vcl_cout << "total message_count_ = " << message_count_ << vcl_endl;
        }
      }

      memset(received_data_, 0, MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH);
      
      if(camera_->get_camera_shutdown_flag() == false) {
        // only maintain the socket to handle the session start() and handle_read() when the camera_shutdown_flag_ is false 
        start();
      }
      else {
        // the camera_shutdown_flag_ is true, so let's close the socket
        camera_ = NULL;
        socket_.close();
      }
    }
    else {
      delete this;
    }
  }

private:
  boost::asio::ip::tcp::socket socket_;    

  char received_data_[MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH];
  char message_buffer_[MICRO_EPSILON_SOCKET_MAX_MESSAGE_LENGTH];

  int message_count_;
  int message_length_;

  micro_epsilon_socket_camera *camera_;

} micro_epsilon_socket_session;

// the class for implementing the server
class socket_camera_server
{
public:
  socket_camera_server(boost::asio::io_service& io_service, short port,  micro_epsilon_socket_camera *camera)
    : io_service_(io_service),
    acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
  {
    camera_ = camera;

    // start a new micro_epsilon_socket_session
    micro_epsilon_socket_session *new_session = new micro_epsilon_socket_session(io_service_, camera_);

    // start an asynchronous accept.
    acceptor_.async_accept(new_session->socket(),
      boost::bind(&socket_camera_server::handle_accept, this, new_session, boost::asio::placeholders::error));
  }

  // event handler
  void handle_accept(micro_epsilon_socket_session *new_session, const boost::system::error_code &error)
  {

    if (!error) {
      new_session->start();

      // whenever there is a new message, start a brand new session to process
      // future messages
      new_session = new micro_epsilon_socket_session(io_service_, camera_);
      acceptor_.async_accept(new_session->socket(), 
        boost::bind(&socket_camera_server::handle_accept, this, new_session, boost::asio::placeholders::error));

      vcl_cout << "socket_camera_server::handle_accept, new client connection established." << vcl_endl;
    }
    else {
      if(NULL != new_session) {
        delete new_session;
        new_session = NULL;
      }
    }
  }

private:
  boost::asio::io_service& io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  micro_epsilon_socket_camera *camera_;
};

void socket_server_func(int port, micro_epsilon_socket_camera *camera)
{
  try {
    boost::asio::io_service io_service;
    socket_camera_server server(io_service, port, camera);
    io_service.run();
  }
  catch (std::exception& ex) {
    vcl_cerr << "socket_server_func: Exception: " << ex.what() << vcl_endl;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////

micro_epsilon_socket_camera::micro_epsilon_socket_camera(int img_height, int img_width, int port, bool upsidedown_leftright_flipping)
: img_height_(img_height), 
img_width_(img_width), 
port_(port), 
upsidedown_leftright_flipping_(upsidedown_leftright_flipping),
live_(true),   
new_frame_ready_(false),
write_message_length_(0),
read_message_length_(0),
captured_year_(0),
captured_month_(0),
captured_day_(0),
captured_hour_(0),
captured_minute_(0),
captured_second_(0),
captured_millisecond_(0),
captured_frame_system_time_(0.0),
socket_camera_server_thread_(NULL),
camera_shutdown_flag_(false)
{

}

micro_epsilon_socket_camera::~micro_epsilon_socket_camera()
{
  //socket_camera_server_thread_->join();

  delete socket_camera_server_thread_;
  socket_camera_server_thread_ = NULL;
}

void micro_epsilon_socket_camera::set_camera_shutdown_flag(void)
{
  camera_shutdown_flag_ = true;
}

bool micro_epsilon_socket_camera::get_camera_shutdown_flag(void)
{
  return camera_shutdown_flag_;
}

void micro_epsilon_socket_camera::initialize_socket_server(void)
{
  new_frame_ready_ = false;
  socket_camera_server_thread_ = new boost::thread(socket_server_func, port_, this);
}

bool micro_epsilon_socket_camera::get_next_frame(vil_image_view<float> &thermal_frame, double &captured_frame_system_time)
{
  // allocate the vxl_image_view memories if they haven't been allocated yet.
  if(thermal_frame.ni() != img_width_ || thermal_frame.nj() != img_height_ || thermal_frame.nplanes() != 1) {
    thermal_frame = vil_image_view<float>(img_width_, img_height_, 1, 1);
    thermal_frame.fill(0);
  }

  bool new_frame_ready = false;

  write_slot_lock_.lock();
  new_frame_ready = new_frame_ready_;
  write_slot_lock_.unlock();

  if(!new_frame_ready) {
    return false;
  }

  // copy the buffer out from the write_slot to the read_slot
  write_slot_lock_.lock();
  memcpy(read_message_buffer_, write_message_buffer_, write_message_length_);
  read_message_length_ = write_message_length_;
  new_frame_ready_ = false;
  write_slot_lock_.unlock();

  // now based on the read_slot read_message_buffer_, reconstruct the frame
  bool result = copy_next_frame(thermal_frame, captured_frame_system_time);
  return result;
}

bool micro_epsilon_socket_camera::copy_next_frame(vil_image_view<float> &thermal_frame, double &captured_frame_system_time)
{
  bool result = true;

  int width = 0, height = 0;

  // read the data from buffer to a string
  vcl_string str;
  str.assign(read_message_buffer_, read_message_length_);

  // place the string into a istringstream construct.
  vcl_istringstream isstream(str);

  // captured time
  if( (isstream >> captured_year_) == false ) return false;
  if( (isstream >> captured_month_) == false ) return false;
  if( (isstream >> captured_day_) == false ) return false;
  if( (isstream >> captured_hour_) == false ) return false;
  if( (isstream >> captured_minute_) == false ) return false;
  if( (isstream >> captured_second_) == false ) return false;
  if( (isstream >> captured_millisecond_) == false ) return false;	

  SYSTEMTIME st;
  st.wYear = captured_year_;
  st.wMonth = captured_month_;
  st.wDayOfWeek = 0;
  st.wDay = captured_day_;
  st.wHour = captured_hour_;
  st.wMinute = captured_minute_;
  st.wSecond = captured_second_;
  st.wMilliseconds = captured_millisecond_;

  //////////////////////////////////////////////////////////////////////////////////////
  FILETIME ft;

  /*
  // Option 1: use the captured_time from the .NET capturing program's SYSTEMTIME, which seems not working.
  if( ! SystemTimeToFileTime( &st, &ft ) ) {
  vcl_cerr << "micro_epsilon_socket_camera::copy_next_frame: SystemTimeToFileTime failed with code " << GetLastError() << vcl_endl;
  return false;
  }
  */
  // Option 2: use the captured_time at the local system time, which may not reflect the true capturing time 
  // because of the socket transmission, but we can't do anything about it.
  GetSystemTimeAsFileTime( &ft );

  // ft values are in 100 nanoseconds = 10^-7 seconds, so divide by 10^-4 to get milliseconds.
  captured_frame_system_time_ = (ft.dwHighDateTime * 4294967296.0 + ft.dwLowDateTime) / 10000.0;
  //////////////////////////////////////////////////////////////////////////////////////

  captured_frame_system_time = captured_frame_system_time_;

  // frame size, frame_width and frame_height
  if( (isstream >> width) == false ) return false;
  if( (isstream >> height) == false ) return false;

  if(width != img_width_ || height != img_height_) {
    //return false;
    //vcl_cout << "micro_epsilon_socket_camera::copy_next_frame, the received image size does not match with the expected size." << vcl_endl;
  }

  if(img_width_ < width || img_height_ < height) {
    vcl_cerr << "micro_epsilon_socket_camera::copy_next_frame, the received image size is greater than the expected size, which is incorrect." << vcl_endl;
    return false;
  }

  // image buffer
  int val = 0;

  // filling with the received buffer data
  if(upsidedown_leftright_flipping_) {
    // perform upside down and left to right flipping
    for(unsigned j = 0; j < height; j++) {
      for(unsigned i = 0; i < width; i++) {
        if( (isstream >> val) == false ) return false;
        thermal_frame(width - i - 1, height - j - 1) = val;			
      }
    }
  }
  else {
    for(unsigned j = 0; j < height; j++) {
      for(unsigned i = 0; i < width; i++) {
        if( (isstream >> val) == false ) return false;
        thermal_frame(i, j) = val;			
      }
    }
  }


  // filling the missing columns
  for(unsigned j = 0; j < height; j++) {
    for(unsigned i = width; i < img_width_; i++) {
      thermal_frame(i, j) = thermal_frame(width-1, j);
    }
  }

  // filling the missing rows
  for(unsigned j = height; j < img_height_; j++) {
    for(unsigned i = 0; i < img_width_; i++) {
      thermal_frame(i, j) = thermal_frame(i, height-1);
    }
  }

  return true;
}

void micro_epsilon_socket_camera::copy_message_buffer(char *buffer, int length)
{
  write_slot_lock_.lock();

  memcpy(write_message_buffer_, buffer, length); 
  write_message_length_ = length;
  new_frame_ready_ = true;

  write_slot_lock_.unlock();
}
