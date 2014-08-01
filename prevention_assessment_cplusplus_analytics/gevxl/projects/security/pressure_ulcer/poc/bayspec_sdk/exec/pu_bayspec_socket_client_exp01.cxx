// pu_bayspec_socket_client_exp01.cpp : Defines the entry point for the console application.
//

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <deque>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include <string>
#include <vector>

using boost::asio::ip::tcp;

class command_message
{
public:
  enum { header_length = 4 };
  enum { max_body_length = 1024 };

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

  size_t length() const
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

  size_t body_length() const
  {
    return body_length_;
  }

  void body_length(size_t new_length)
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
  size_t body_length_;
};

typedef std::deque<command_message> command_message_queue;

class command_client
{
public:
  command_client(boost::asio::io_service& io_service,
      tcp::resolver::iterator endpoint_iterator)
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
      std::cout.write(read_msg_.body(), read_msg_.body_length());
      std::cout << "\n";
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
  tcp::socket socket_;
  command_message read_msg_;
  command_message_queue write_msgs_;
};

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: command_client <host> <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;

    tcp::resolver resolver(io_service);
    tcp::resolver::query query(argv[1], argv[2]);
    tcp::resolver::iterator iterator = resolver.resolve(query);

    command_client c(io_service, iterator);

    boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));
  
    std::vector<int> commands_ids;
    commands_ids.resize(5);
    std::string commands[5];

    commands_ids[0] = 0;
    commands[0] = "set_file_system_root_folder=D:\\yuting\\data\\pressure_ulcer\\hyper_spectral\\poc\\BaySpecCameraServerTest";
    commands_ids[1] = 1;
    commands[1] = "set_white_ref_folder=D:\\yuting\\data\\pressure_ulcer\\hyper_spectral\\poc\\WhiteRefDarkBkg\\12-24-59RES2048_EXP0030_BIT08-White";
    commands_ids[2] = 2;
    commands[2] = "set_dark_bkg_folder=D:\\yuting\\data\\pressure_ulcer\\hyper_spectral\\poc\\WhiteRefDarkBkg\\12-26-37RES2048_EXP0030_BIT08-Dark";
    commands_ids[3] = 3;
    commands[3] = "capture_images=0";
    commands_ids[4] = 4;
    commands[4] = "set_sleep_time=1000";

    for(unsigned m = 0; m < 5; m++) {
      std::cout << commands_ids[m] << "   " << commands[m] << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "input_id = " << std::endl;

    char line[command_message::max_body_length + 1];
    while (std::cin.getline(line, command_message::max_body_length + 1))
    {
      using namespace std; // For strlen and memcpy.
      command_message msg;
     
      //msg.body_length(strlen(line));
      //memcpy(msg.body(), line, msg.body_length());
      
      // based on the input, 
      std::string cmd;
      cmd.assign(line, strlen(line));
      bool valid_input = true;

      if(cmd == "0") {
        msg.body_length(commands[0].length());
        memcpy(msg.body(), commands[0].c_str(), commands[0].length());
      }
      else if(cmd == "1") {
        msg.body_length(commands[1].length());
        memcpy(msg.body(), commands[1].c_str(), commands[1].length());
      }
      else if(cmd == "2") {
        msg.body_length(commands[2].length());
        memcpy(msg.body(), commands[2].c_str(), commands[2].length());
      }
      else if(cmd == "3") {
        msg.body_length(commands[3].length());
        memcpy(msg.body(), commands[3].c_str(), commands[3].length());
      }
      else if(cmd == "4") {
        msg.body_length(commands[4].length());
        memcpy(msg.body(), commands[4].c_str(), commands[4].length());
      }
      else {
        valid_input = false;
      }

      if(valid_input) {
        msg.encode_header();
        c.write(msg);
      }

      std::cout << std::endl;
      std::cout << std::endl;
      for(unsigned m = 0; m < 5; m++) {
        std::cout << commands_ids[m] << "   " << commands[m] << std::endl;
      }
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "input_id = " << std::endl;
    }

    c.close();
    t.join();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
