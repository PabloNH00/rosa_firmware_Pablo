#pragma once

#include <rclcpp/rclcpp.hpp>
#include "io_context.hpp"
#include "serial_driver/serial_driver.hpp"
#include "rosa_messages.h"

#include <string>

using namespace drivers::serial_driver;
//one using per line C++14 
using drivers::common::IoContext;
using std::string;
using std::unique_ptr;

// SerialPort class
namespace ROSA{
class RosaPort
{
  rclcpp::Node::SharedPtr _node;
  string _port_name{};
  uint32_t _bauds; 
  unique_ptr<IoContext> m_owned_ctx{};
  unique_ptr<SerialPortConfig> port_config;
  unique_ptr<SerialDriver> serial_driver;
  rclcpp::TimerBase::SharedPtr serial_wdt_; //shared ptr to watchdog timer (wd_check)
  ROSAmens::MsgReader reader;
  ROSAmens::CircularBuffer<100> mens_buffer{};
  std::mutex _mutex;
public:
  // Constructor that takes a shared pointer to a node
  RosaPort(rclcpp::Node::SharedPtr node, string port_name, uint32_t bauds);
  ~RosaPort(){  if (m_owned_ctx) m_owned_ctx->waitForExit();}
  bool there_is_message();
  ROSAmens get_message();
  void send_message(const ROSAmens &m);

private:
  void wd_check(); // Function to check the serial port state
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);
}; //class SerialPort

// Typedef for a shared pointer to a Hardware object
typedef std::shared_ptr<RosaPort> RosaPortPtr;
} //namespace ROSA

