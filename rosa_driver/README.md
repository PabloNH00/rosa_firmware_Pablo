# rosa_driver
ROS2 node for controlling the ROSA robot

  

    sudo rosdep update
    rosdep install --from-paths src -y --ignore-src
    colcon build

    ros2 run rosa_firmware rosa_driver --ros-args -p "device_name:=/dev/ttyUSB2"
    ros2 topic echo /odom --field pose.pose


