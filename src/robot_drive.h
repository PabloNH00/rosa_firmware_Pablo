#pragma once
#include <Arduino.h>
#include "RoboClaw.h"

#define RC_BAUD_RATE 38400
#define RC_TIMEOUT 1000 //1s of coms timeout
//motors should work at 3K rpm as max
//encoders 300 cpr  si es enc simple 15K si x4 60K
#define ENC_CPR  300
#define REDUCTION 40
#define MEC_RAD 0.076F
#define ROSA_LENGTH 0.68181F
#define ROSA_WIDTH 0.68181F

#define RC_MAX_MOTOR_SPEED 60000 //TODO max counts/sec
#define CPR_2_RADS (3.1415F*2.0F/(ENC_CPR*REDUCTION))
#define RC_LEFT_PORT Serial2
#define RC_LEFT_RX 26
#define RC_LEFT_TX 27

#define RC_RIGHT_PORT Serial1
#define RC_RIGHT_RX 16
#define RC_RIGHT_TX 17

#define RC_ID 0x80   //both ROBOCLAWS HAVE THE SAME ADDRESS

union int32_u{
  uint32_t t_uint;
  int32_t t_int;
  float t_float;
};
enum rosa_motors : char{m1_left, m2_left, m1_right, m2_right};
class RobotDrive{
    RoboClaw rc_left =  RoboClaw(&RC_LEFT_PORT, RC_TIMEOUT);
    RoboClaw rc_right = RoboClaw(&RC_RIGHT_PORT,RC_TIMEOUT);
    //internal state vars
    bool move_commands_enabled = true;
    int32_u current_velocity[4]{};
    float battery_voltage=0;
    int32_u encoder_counts[4]{};
    float x_pos,y_pos,yaw;
    //commanded
    int32_u target_velocity[4]{};
    void read_encoders();
  public:
    void setup();
    void loop();
    void enable_move_commands(){move_commands_enabled=true;}
    void disable_move_commands(){move_commands_enabled=false;}
    void set_velocity(float vx, float vy, float vr);
    void emergency_stop();
   void reset_odometry();
};