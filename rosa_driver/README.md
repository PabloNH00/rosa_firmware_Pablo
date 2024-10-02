# rosa_driver
ROS2 node for controlling the ROSA robot

Serial_driver package depends on "ros-humble-asio-cmake-module", if it is not installed you have to install it manually by doing:

    sudo apt install ros-humble-asio-cmake-module

Once installed, update the remaining dependencies and build the package to launch

    sudo rosdep update
    rosdep install --from-paths src -y --ignore-src
    colcon build

    ros2 run rosa_firmware rosa_driver --ros-args -p "device_name:=/dev/ttyUSB2"
    ros2 topic echo /odom --field pose.pose


