//GE

//#include "pu_micro_epsilon_socket_camera.h"
#include <vcl_string.h>
#include <threading/sleep.h>

// Include these boost headers here instead of h file.
// Otherwise, it will complain WinSock.h has been included
// This is because wx includes window.h which includes WinSock.h
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <vcl_iostream.h>

using boost::asio::ip::tcp;

// ********************************
// Session of sockets communications
class session
{
public:
  session(boost::asio::io_service& io_service) : socket_(io_service)
  {
    message_length_ = 0;
    message_count_ = 0;
  }

  tcp::socket& socket(){ return socket_; }

  void start()
  {
      // read the message sent my client in asnyc fashion
      socket_.async_read_some(boost::asio::buffer(data_, 1000000),
          boost::bind(&session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
  }

  void handle_read(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error)
    {
      short eventID = (unsigned char)data_[6];
      //vcl_cout << eventID << vcl_endl;

      message_length_ += bytes_transferred;
      if((char)(data_[bytes_transferred-1]) == '^') {
        vcl_cout << "message_length_ = " << message_length_ << vcl_endl;
        message_length_ = 0;
        message_count_++;
        vcl_cout << "total message_count_ = " << message_count_ << vcl_endl;
      }
      start();
    }
    else
    {
      delete this;
    }
  }

private:
  tcp::socket socket_;
  //enum { max_length = 1000000 };
  char data_[1000000];

  int message_count_;
  int message_length_;
};

// The calss for implementing the server
class micro_epsilon_socket_camera_server
{
public:
  micro_epsilon_socket_camera_server(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
  {
    // start a new session
    session *new_session = new session(io_service_);

    // start an asynchronous accept.
    acceptor_.async_accept(new_session->socket(),
        boost::bind(&micro_epsilon_socket_camera_server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
  }

  // event handler
  void handle_accept(session *new_session,
      const boost::system::error_code &error)
  {

    if (!error)
    {
      new_session->start();
      
      // whenever there is a new message, start a brand new session to process
      // future messages
      new_session = new session(io_service_);
      acceptor_.async_accept(new_session->socket(),
          boost::bind(&micro_epsilon_socket_camera_server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
    }
    else
    {
      delete new_session;
    }
  }

private:
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
};

// The server thread that processes client requests
void serverFunc(int port)
{
  try
  {
    boost::asio::io_service io_service;
    micro_epsilon_socket_camera_server s(io_service, port);
    io_service.run();
    }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

// exp01.cpp : Defines the entry point for the console application.
int main (int argc, char **argv)
{
  // start the server here	
  boost::thread socketCameraServer(serverFunc, 9080);  

  for(int i = 0; i < 100; i++) {
    gevxl::threading::sleep(10000000);
  }

  return 0;
}



	
