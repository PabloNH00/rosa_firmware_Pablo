cmake_minimum_required(VERSION 3.5)
project(rosa_firmware)



set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(ament_cmake REQUIRED)
include(qt_interface/CMakeLists.txt)
include(rosa_driver/CMakeLists.txt)



ament_package()
