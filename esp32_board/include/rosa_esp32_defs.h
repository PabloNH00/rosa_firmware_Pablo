#pragma once

//DEBUG OPTIONS
//#define DEBUG_GAME_PAD 

//ROSA DEFINITIONS
#define COMMANDS_BAUD_RATE 38400 //bauds
#define ROS_UPDATE_INFO_RATE 50 //ms
#define WIFI_UPDATE_INFO_RATE 255 //ms
#define ROSA_HEARTBEAT 500       //ms
#define TIME_OUT_WIFI_MASTER 5000  //after 5 seconds without wifi messages, no output messages will be sent


#define ROSA_CONF_FILE_VERSION 1
#define SET_DEFAULT_CONFIGURATION_DATA 0



class RosaDefs{
    public:
    static char ROBOT_NAME[100]; 
    static uint8_t IP_ADDRESS[4]; //180
    static uint8_t GATEWAY_ADDRESS[4]; //180 (IP address of the router)
    static uint8_t SUBNET_MASK[4]; //180
    static char WIFI_SSID[50]; //"RM2_WIFI"
    static char WIFI_KEY[50]; //"R0BOMINERS*"
    static bool readConfiguration();
    static bool writeConfiguration();
};
