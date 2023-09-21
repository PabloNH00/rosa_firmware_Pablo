#pragma once
#include <Arduino.h>
#include <RoboClaw.h>

#define RC_BAUD_RATE 38400
#define RC_TIMEOUT 1000 //1s of coms timeout
#define RC_MAX_MOTOR_SPEED 50

#define RC_LEFT_PORT Serial1
#define RC_LEFT_RX 26
#define RC_LEFT_TX 27

#define RC_RIGHT_PORT Serial2
#define RC_RIGHT_RX 16
#define RC_RIGHT_TX 17

#define RC_ID 0x80   //both ROBOCLAWS HAVE THE SAME ADDRESS
class RobotDrive{
    RoboClaw rc_left =  RoboClaw(&RC_LEFT_PORT, RC_TIMEOUT);
    RoboClaw rc_right = RoboClaw(&RC_RIGHT_PORT,RC_TIMEOUT);
    bool move_commands_enabled = true;
  public:
    void setup();
    void loop();
    void enable_move_commands(){move_commands_enabled=true;}
    void disable_move_commands(){move_commands_enabled=false;}
    void set_velocity(float vx, float vy, float vr);
    void emergency_stop();
   
};