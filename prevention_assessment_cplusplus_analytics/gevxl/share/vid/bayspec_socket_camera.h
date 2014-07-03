// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 5/25/2014
/// \brief the wrapper for the Bayspec camera through socket communication
/// - Original version

#ifndef bayspec_socket_camera_h_
#define bayspec_socket_camera_h_

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>

#include <vcl_string.h>
#include <vcl_deque.h>
#include <vcl_cstddef.h>
#include <vcl_iostream.h>

namespace gevxl { 
namespace vid {

// command message class
class command_message
{
public:
  enum { header_length = 4 };
  enum { max_body_length = 512 };

  command_message()
    : body_length_(0)
  {
  }

  const char* data() const
  {
    return data_;
  }

  char* data()
  {
    return data_;
  }

  vcl_size_t length() const
  {
    return header_length + body_length_;
  }

  const char* body() const
  {
    return data_ + header_length;
  }

  char* body()
  {
    return data_ + header_length;
  }

  vcl_size_t body_length() const
  {
    return body_length_;
  }

  void body_length(vcl_size_t new_length)
  {
    body_length_ = new_length;
    if (body_length_ > max_body_length)
      body_length_ = max_body_length;
  }

  bool decode_header()
  {
    using namespace std; // For strncat and atoi.
    char header[header_length + 1] = "";
    strncat(header, data_, header_length);
    body_length_ = atoi(header);
    if (body_length_ > max_body_length)
    {
      body_length_ = 0;
      return false;
    }
    return true;
  }

  void encode_header()
  {
    using namespace std; // For sprintf and memcpy.
    char header[header_length + 1] = "";
    sprintf(header, "%4d", body_length_);
    memcpy(data_, header, header_length);
  }

private:
  char data_[header_length + max_body_length];
  vcl_size_t body_length_;
};

typedef vcl_deque<command_message> command_message_queue;

// command client class that handles asynchronized writing.
class command_client
{
public:
  command_client(boost::asio::io_service& io_service, boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
    : io_service_(io_service),
      socket_(io_service)
  {
    boost::asio::async_connect(socket_, endpoint_iterator,
        boost::bind(&command_client::handle_connect, this,
          boost::asio::placeholders::error));
  }

  void write(const command_message& msg)
  {
    io_service_.post(boost::bind(&command_client::do_write, this, msg));
  }

  void close()
  {
    io_service_.post(boost::bind(&command_client::do_close, this));
  }

private:

  void handle_connect(const boost::system::error_code& error)
  {
    if (!error)
    {
      boost::asio::async_read(socket_,
          boost::asio::buffer(read_msg_.data(), command_message::header_length),
          boost::bind(&command_client::handle_read_header, this,
            boost::asio::placeholders::error));
    }
  }

  void handle_read_header(const boost::system::error_code& error)
  {
    if (!error && read_msg_.decode_header())
    {
      boost::asio::async_read(socket_,
          boost::asio::buffer(read_msg_.body(), read_msg_.body_length()),
          boost::bind(&command_client::handle_read_body, this,
            boost::asio::placeholders::error));
    }
    else
    {
      do_close();
    }
  }

  void handle_read_body(const boost::system::error_code& error)
  {
    if (!error)
    {
      vcl_cout.write(read_msg_.body(), read_msg_.body_length());
      vcl_cout << "\n";
      boost::asio::async_read(socket_,
          boost::asio::buffer(read_msg_.data(), command_message::header_length),
          boost::bind(&command_client::handle_read_header, this,
            boost::asio::placeholders::error));
    }
    else
    {
      do_close();
    }
  }

  void do_write(command_message msg)
  {
    bool write_in_progress = !write_msgs_.empty();
    write_msgs_.push_back(msg);
    if (!write_in_progress)
    {
      boost::asio::async_write(socket_,
          boost::asio::buffer(write_msgs_.front().data(),
            write_msgs_.front().length()),
          boost::bind(&command_client::handle_write, this,
            boost::asio::placeholders::error));
    }
  }

  void handle_write(const boost::system::error_code& error)
  {
    if (!error)
    {
      write_msgs_.pop_front();
      if (!write_msgs_.empty())
      {
        boost::asio::async_write(socket_,
            boost::asio::buffer(write_msgs_.front().data(),
              write_msgs_.front().length()),
            boost::bind(&command_client::handle_write, this,
              boost::asio::placeholders::error));
      }
    }
    else
    {
      do_close();
    }
  }

  void do_close()
  {
    socket_.close();
  }

private:
  boost::asio::io_service& io_service_;
  boost::asio::ip::tcp::socket socket_;
  command_message read_msg_;
  command_message_queue write_msgs_;
};


class bayspec_socket_camera
{
public:

  // constructor
  bayspec_socket_camera(const vcl_string &server_host_or_ip, const vcl_string &server_port);

  // deconstructor
  ~bayspec_socket_camera();

  // initialize socket client
  bool initialize_socket_client(void);

  // uninitialize socket client
  bool uninitialize_socket_client(void);

  // send command message
  bool send_command_message(const vcl_string &comm_msg);

private:

  vcl_string server_host_or_ip_;

  vcl_string server_port_;

  // boost asio related variables
  boost::asio::io_service *io_service_;
  
  boost::asio::ip::tcp::resolver *resolver_;
    
  boost::asio::ip::tcp::resolver::query *query_;

  boost::asio::ip::tcp::resolver::iterator *iterator_;

  command_client *command_client_;

  boost::thread *thread_;
  
};

}} // end namespaces

#endif
