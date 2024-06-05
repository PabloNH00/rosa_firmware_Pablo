#pragma once

//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

//Package headers
#include "msg_converters/converters.hpp"
#include "rosa_port.h"

//STL includes
#include <chrono>
#include <memory>
#include <string>
#include <vector>


namespace ROSA{
class RosaDriver : public rclcpp::Node
{    
  std::string frame_id, robot_frame_id, odometry_topic, cmd_topic;

public:
  RosaDriver() : Node("rosa_driver_node"){}
  // Function to initialize the driver
  void init();
  
  // main regular func to update the odometry
  void loop();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);

  
private:
  // Function to update the driver
  void update();

  // Pointer to the Hardware object
  RosaPortPtr hw_ptr;

  rclcpp::TimerBase::SharedPtr _timer;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;

  /// To broadcast TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

  /// Velocity received on command.
  geometry_msgs::msg::Twist target_cmd_vel;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_msg;
  geometry_msgs::msg::TransformStamped transform_msg;
};

} //namespace ROSA