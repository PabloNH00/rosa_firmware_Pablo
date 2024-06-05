/*
 * This node communicates with Robotont hardware
 */
#include <rclcpp/rclcpp.hpp>
#include "rosa_driver.h"


int main(int argc, char** argv)
{  
    rclcpp::init(argc, argv);
    auto rosa_driver_node = std::make_shared<ROSA::RosaDriver>();
    rosa_driver_node->init(); 
    rclcpp::spin(rosa_driver_node);
    rclcpp::shutdown();
}