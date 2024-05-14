# ROSAfirmware
ESP32 Firmware for the robot rosa v3.0

  Because the ESP32 has two cores and one of them handles WiFi and BT communications, it is possible to easily debug the code using them.

  For this reason, an application (node) is included in QT that allows to monitor and even emulate the Robot with the ESP32 in-the-loop. If there is no ROS2 node communicating with the micro, it is possible to connect to it through the serial port and read and write the commands and configuration.

