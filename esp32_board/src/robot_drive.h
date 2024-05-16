#pragma once
#include <Arduino.h>
#include "robo_claw.h"
#include "rosa_messages.h"
#include "rosa_esp32_utils.h"

#define RC_BAUD_RATE 38400
constexpr uint16_t ROBOT_DRIVER_WATCHDOG = 2000;
//motors should work at 3K rpm as max
//encoders 300 cpr  si es enc simple 15K si x4 60K
constexpr int ENC_CPR  = 300;
constexpr int REDUCTION = 40;
constexpr float MEC_RAD = 0.076F;
constexpr float ROSA_LENGTH = 0.68181F;
constexpr float ROSA_WIDTH = 0.68181F;
constexpr float LX = ROSA_LENGTH/2.F;
constexpr float LY = ROSA_WIDTH/2.F;
constexpr float LXY = LX+LY;
#define MAX_MOTOR_SPEED 8.0F //max rads/sec
#define MAX_FORWARD_SPEED 2.5F //m/s
#define MAX_LATERAL_SPEED 2.5F  //m/s
#define MAX_ROT_SPEED 4.0F //rad/s
constexpr float CPR_2_RADS = 3.1415F*2.0F/(ENC_CPR*REDUCTION);
constexpr float RADS_2_CPR = 1.0F/CPR_2_RADS;
constexpr float factor_speed = 1;
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
struct Odometry{
  float x_pos=0;
  float y_pos=0;
  float yaw=0;
};

class RobotDrive{
    RoboClawDriver rc_left{RC_LEFT_PORT, RC_BAUD_RATE, RC_ID};
    RoboClawDriver rc_right{RC_RIGHT_PORT, RC_BAUD_RATE, RC_ID};
    //internal state vars
    bool mock_hardware = false; //set to true to run in a ESP32 without robot
    bool move_commands_enabled = true;
    int32_u current_velocity[4]{};
    float battery_voltage=0;
    int32_u encoder_counts[4]{};
    float x_pos,y_pos,yaw;
    //commanded
    int32_u target_velocity[4]{};
    void read_encoders();
    void command_speed();
    TIMER watch_dog{ROBOT_DRIVER_WATCHDOG};
  public:
    void setup();
    void loop();
    void enable_move_commands(){move_commands_enabled=true;}
    void disable_move_commands(){move_commands_enabled=false;}
    void set_relative_velocity(float vx, float vy, float vr);
    void set_velocity(float vx, float vy, float vr);
    void emergency_stop();
   void reset_odometry();
   void enable(bool en=true){mock_hardware=!en;}
   Odometry get_odometry(){return Odometry{x_pos, y_pos, yaw};}
   RobotData get_robot_data();
   static void FK(const float v[4], float &vx, float &vy, float &vr);
   static void IK(const float &vx, const float &vy, const float &vr, float vm[4]);
};