// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include "bayspec_socket_camera.h"

#include <vcl_iostream.h>
#include <vcl_sstream.h>

#include <util/string.h>

#if defined(VCL_WIN32) && defined(VCL_VC)
  #include <Windows.h>
#else
  #include <sys/time.h>
  #include <sys/stat.h>
#endif

using namespace gevxl::vid;
using namespace gevxl::util;
using boost::asio::ip::tcp;

bayspec_socket_camera::bayspec_socket_camera(const vcl_string &server_host_or_ip, const vcl_string &server_port)
: server_host_or_ip_(server_host_or_ip),
  server_port_(server_port),
  command_client_(NULL),
  thread_(NULL),
  io_service_(NULL),
  resolver_(NULL),
  query_(NULL)
{

}

bayspec_socket_camera::~bayspec_socket_camera()
{

}

bool bayspec_socket_camera::initialize_socket_client(void)
{
  io_service_ = new boost::asio::io_service();
  
  resolver_ = new boost::asio::ip::tcp::resolver(*io_service_);
    
  query_ = new boost::asio::ip::tcp::resolver::query(server_host_or_ip_.c_str(), server_port_.c_str());

  iterator_ = new boost::asio::ip::tcp::resolver::iterator();
  *iterator_ = resolver_->resolve(*query_);

  command_client_ = new command_client(*io_service_, *iterator_);

  thread_ = new boost::thread(boost::bind(&boost::asio::io_service::run, io_service_));

  return true;
}

bool bayspec_socket_camera::uninitialize_socket_client(void)
{
  if(command_client_ != NULL && thread_ != NULL) {
    command_client_->close();
    thread_->join();

    delete thread_;
    thread_ = NULL;

    delete command_client_;
    command_client_ = NULL;

    delete iterator_;
    iterator_ = NULL;

    delete query_;
    query_ = NULL;

    delete resolver_;
    resolver_ = NULL;

    delete io_service_;
    io_service_ = NULL;
  }

  return true;
}

bool bayspec_socket_camera::send_command_message(const vcl_string &comm_msg)
{
  if(command_client_ == NULL) {
    vcl_cerr << "bayspec_socket_camera::send_command_message(), command_client_ is not initialized properly." << vcl_endl;
    return false;
  }

  command_message msg;

  msg.body_length(comm_msg.length());
  memcpy(msg.body(), comm_msg.c_str(), comm_msg.length());

  msg.encode_header();
  command_client_->write(msg);

  return true;
}
