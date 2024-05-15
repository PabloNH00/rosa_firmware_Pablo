
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()



find_package(Qt5 COMPONENTS Core Widgets Network SerialPort REQUIRED)


include_directories(
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
ament_target_dependencies(rosa_qt_esp32)

target_link_libraries(rosa_qt_esp32
 Qt5::Widgets Qt5::Core Qt5::Network Qt5::SerialPort
 )
 
 install(TARGETS
  rosa_qt_esp32
  DESTINATION lib/${PROJECT_NAME}
  )

