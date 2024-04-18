#pragma once
#include "spslib.h"

typedef SPS::Message<> ROSAmens;
//DEBUG OPTIONS
//#define DEBUG_GAME_PAD 

//ROSA DEFINITIONS
#define COMMANDS_BAUD_RATE 38400 //bauds
#define ROS_UPDATE_INFO_RATE 100 //ms
#define ROSA_HEARTBEAT 500       //ms


//ROSA ROS driver messages IDs
//ROSA->ROS2
#define ROSA_ODOMETRY 0x01
#define ROSA_DEBUG_TXT 0x10

//ROS2->ROSA
#define ROSA_CMD_VEL 0xF1


//utility inline functions
inline ROSAmens debug_text_message(const char *text)
{
    ROSAmens m(ROSA_DEBUG_TXT);
    m.write_cstring(text);
    return m;
}