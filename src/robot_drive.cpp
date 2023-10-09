#include "robot_drive.h"
#include "utils.h"
void RobotDrive::setup(){
 RC_LEFT_PORT.begin(RC_BAUD_RATE, SERIAL_8N1, RC_LEFT_RX, RC_LEFT_TX);
 RC_RIGHT_PORT.begin(RC_BAUD_RATE, SERIAL_8N1, RC_RIGHT_RX, RC_RIGHT_TX);
}
void RobotDrive::loop(){
  static TIMER read_timer(50), command_timer(50); //20 times per sec
  if(command_timer()){
    rc_left.SpeedM1M2(RC_ID,
      target_velocity[m1_left].t_uint, target_velocity[m2_left].t_uint );
    rc_right.SpeedM1M2(RC_ID,
      target_velocity[m1_right].t_uint, target_velocity[m2_right].t_uint );
  } else
  if(read_timer()){
    rc_left.ReadISpeeds(RC_ID, 
      current_velocity[m1_left].t_uint, current_velocity[m2_left].t_uint );
    rc_right.ReadISpeeds(RC_ID, 
      current_velocity[m1_right].t_uint, current_velocity[m2_right].t_uint );
  }

}
void RobotDrive::emergency_stop(){
    for (auto &v:target_velocity)v.t_int=0;
    rc_left.SpeedM1M2(RC_ID,0,0);
    rc_right.SpeedM1M2(RC_ID,0,0);
}
void RobotDrive::set_velocity(float vx, float vy, float vr)
{
    if(!move_commands_enabled)return; //nothing is executed
    
    //compute vx, vy, vr
    float vm[] = { -vx +vy  +vr, +vx +vy  -vr, -vx +vy  -vr, +vx +vy  +vr};
    
    float max=0;
    for(auto v:vm)max=fabs(v)>max?fabs(v):max;
    if(max>1.0) for(auto &v:vm)v/=max;
    
    //scale to the roboclaw units: transform % to encoders speeds
    for(auto &v:vm)v*=RC_MAX_MOTOR_SPEED;
    //store as integers in counts per sec
    for(int i=0;i<4;i++)target_velocity[0].t_int=vm[i];

}