cmake_minimum_required(VERSION 3.5)
project(rosa_firmware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()
#ADD_DEFINITIONS(-DBUILD_VERSION=”${BUILD_VERSION}” )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets Network REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
 ${rclcpp_INCLUDE_DIRS}
 ${std_msgs_INCLUDE_DIRS}
 ${PROJECT_SOURCE_DIR}/qt_interface
 ${PROJECT_SOURCE_DIR}/shared_src
 )
file(GLOB SOURCE_FILES
 ${PROJECT_SOURCE_DIR}/qt_interface/*.cpp
 )
 file(GLOB UI_SOURCE_FILES
 ${PROJECT_SOURCE_DIR}/src/*.ui
 )
  file(GLOB RC_SOURCE_FILES
 ${PROJECT_SOURCE_DIR}/qt_interface/*.qrc
 )

add_executable(rosa_qt_esp32 ${SOURCE_FILES} ${UI_SOURCE_FILES} ${RC_SOURCE_FILES})
ament_target_dependencies(rosa_qt_esp32
 rclcpp
 std_msgs
 geometry_msgs
 #rmw_implementation
 )

target_link_libraries(rosa_qt_esp32
 Qt5::Widgets Qt5::Core Qt5::Network
 )
 
 install(TARGETS
  rosa_qt_esp32
  DESTINATION lib/${PROJECT_NAME}
  )
  
#install(TARGETS rosa_qt_esp32
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#  )
 # Install launch files.
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)
ament_package()
