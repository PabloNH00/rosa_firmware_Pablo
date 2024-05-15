cmake_minimum_required(VERSION 3.5)
#project(robotont_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(io_context REQUIRED)
#find_package(pluginlib REQUIRED)
find_package(serial_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
#find_package(asio_cmake_module REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(ASIO REQUIRED)

ament_auto_find_build_dependencies()


#find_package(asio_cmake_module REQUIRED)
#find_package(ASIO REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/shared_src
		    ${serial_driver_INCLUDE_DIRS})

file(GLOB SOURCE_FILES
 ${PROJECT_SOURCE_DIR}/rosa_driver/*.cpp
 )
add_executable(rosa_driver ${SOURCE_FILES})
ament_target_dependencies(rosa_driver io_context serial_driver rclcpp geometry_msgs nav_msgs  tf2_geometry_msgs)

#add_executable(fake_rosa_driver fake_rosa_driver_node.cpp)
#ament_target_dependencies(fake_driver_node io_context serial_driver rclcpp geometry_msgs #nav_msgs pluginlib tf2_geometry_msgs)


install(TARGETS
  rosa_driver
  #fake_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

#ament_package()
