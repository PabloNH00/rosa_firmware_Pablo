#include "rosa_port.h"

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ROSA
{
RosaPort::RosaPort(rclcpp::Node::SharedPtr node, string port_name, uint32_t bauds): 
    m_owned_ctx{new IoContext()},
    serial_driver{new SerialDriver(*m_owned_ctx)},
    _node{node},
    _port_name{port_name}, 
    _bauds{bauds}
{
  RCLCPP_INFO(_node->get_logger(), "ROSA driver is starting...");

  port_config = std::make_unique<SerialPortConfig>(
    bauds,FlowControl::NONE,Parity::NONE, StopBits::ONE);

  
  
  // Initialise and open serial port
  try {
    serial_driver->init_port(port_name, *port_config);
    if (!serial_driver->port()->is_open()) {
      serial_driver->port()->open();
      serial_driver->port()->async_receive(
        std::bind(&RosaPort::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      _node->get_logger(), "Error creating serial port: %s - %s",
      port_name.c_str(), ex.what());
  }

  // Create a watchdog timer for serial port monitoring
  serial_wdt_ = _node->create_wall_timer(std::chrono::seconds(1), std::bind(&RosaPort::wd_check, this));
  RCLCPP_INFO(_node->get_logger(), "Rosa Serial Port interface is ready");
}

//Check if serial port is open, if it's closed then reopen the port
void RosaPort::wd_check()
{
    RCLCPP_DEBUG(_node->get_logger(), "Checking port...");
    
    try{
      if (!serial_driver->port()->is_open()) {
      RCLCPP_DEBUG(_node->get_logger(), "Port closed, reopening...");
        serial_driver->port()->open();
        serial_driver->port()->async_receive(
          std::bind(&RosaPort::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        RCLCPP_DEBUG(_node->get_logger(), "Port open.");
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        _node->get_logger(), "Error creating serial port: %s - %s",
        _port_name.c_str(), ex.what());
    }

}



// Callback function for reading from serial
void RosaPort::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
{
  for(auto c:buffer){
    if(reader.add_uchar(c)){
      const std::lock_guard<std::mutex> lock(_mutex);
      mens_buffer.push_single(reader.getMessage());
    }
  }
  
}

bool RosaPort::there_is_message(){
  const std::lock_guard<std::mutex> lock(_mutex);
  return mens_buffer.there_is_msg();
}
ROSAmens RosaPort::get_message(){
  const std::lock_guard<std::mutex> lock(_mutex);
  return mens_buffer.getMessage();
}
void RosaPort::send_message(const ROSAmens &m){
  std::vector<uint8_t> data(m.begin(), m.end());
  serial_driver->port()->async_send(data);
}


} //namespace ROSA