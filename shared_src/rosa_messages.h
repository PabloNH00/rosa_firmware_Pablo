#pragma once
#include "spslib.h"

#define ROSA_OUTPUT_UDP_PORT 12001 //QT lisening udp port
#define ROSA_INPUT_UDP_PORT 12000 //ESP32 lisening udp port

typedef SPS::Message<> ROSAmens;

//ROSA ROS driver messages IDs
//ROSA->ROS2
#define ROSA_ODOMETRY 0x01
#define ROSA_DEBUG_TXT 0x10

//ROS2->ROSA
#define ROSA_CMD_VEL 0x91
#define ROSA_RESET_ODOMETRY 0x92

//QT->ROSA
#define ROSA_SET_MASTER_IP 0xE1
#define ROSA_GET_WIFI_CONFIG 0xE2
#define ROSA_SET_WIFI_INFO 0xE3
#define ROSA_ENABLE_ROBOCLAWS 0xE4
//ROSA->QT
#define ROSA_NAME 0xF1
#define ROSA_ROBOT_DATA 0xF2
#define ROSA_WIFI_INFO 0xF3
#define ROSA_WIFI_CONFIGURED 0xF4

/////////////utility inline functions///////////////////////////////////////////
inline ROSAmens text_message(SPS::uchar_t cmd, const char *text)
{
    ROSAmens m(cmd);
    m.write_cstring(text);
    return m;
}
inline ROSAmens debug_text_message(const char *text){return text_message(ROSA_DEBUG_TXT,text);}
inline ROSAmens name_message(const char *name){return text_message(ROSA_NAME,name);}

inline ROSAmens odometry_message(float x, float y, float yaw)
{
    ROSAmens m(ROSA_ODOMETRY);
    m.write<float>(x);
    m.write<float>(y);
    m.write<float>(yaw);
    return m;
}
struct RobotData{
   int32_t current_velocity[4]{};
   int32_t target_velocity[4]{};
   int32_t encoder_counts[4]{};
   float battery_voltage=0;  
};
inline ROSAmens robot_data_message(const RobotData& data){
    ROSAmens m(ROSA_ROBOT_DATA);
    m.write_array<int32_t>(data.current_velocity,4);
    m.write_array<int32_t>(data.target_velocity,4);
    m.write_array<int32_t>(data.encoder_counts,4);
    m.write<float>(data.battery_voltage);
    return m;

}
struct WiFiData{
    uint8_t ip[4]{};
    uint8_t gateway[4]{};
    uint8_t mask[4]{};
    char ssid[50]{};
    char key[50]{};
};
inline ROSAmens info_wifi_message(SPS::uchar_t cmd, const WiFiData &data){
    ROSAmens m(cmd);
    m.write_array<uint8_t>(data.ip,4);
    m.write_array<uint8_t>(data.gateway,4);
    m.write_array<uint8_t>(data.mask,4);
    m.write_cstring(data.ssid);
    m.write_cstring(data.key);
    return m;
}
