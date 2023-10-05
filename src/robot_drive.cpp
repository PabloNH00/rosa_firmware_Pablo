#include "robot_drive.h"
#include "utils.h"
void RobotDrive::setup(){
 RC_LEFT_PORT.begin(RC_BAUD_RATE, SERIAL_8N1, RC_LEFT_RX, RC_LEFT_TX);
 RC_RIGHT_PORT.begin(RC_BAUD_RATE, SERIAL_8N1, RC_RIGHT_RX, RC_RIGHT_TX);
}
void RobotDrive::loop(){
  static TIMER update_rcs(500); 
  int v[4]{};
  if(update_rcs()){
   /* v[0] = rc_left.ReadSpeedM1(RC_ID);
    v[1] = rc_left.ReadSpeedM2(RC_ID);
    v[2] = rc_right.ReadSpeedM1(RC_ID);
    v[3] = rc_right.ReadSpeedM2(RC_ID);*/
  
  }
}
void RobotDrive::emergency_stop(){
    rc_left.SpeedM1M2(RC_ID,0,0);
    rc_right.SpeedM1M2(RC_ID,0,0);
}
void RobotDrive::set_velocity(float vx, float vy, float vr)
{
    if(!move_commands_enabled)return; //nothing is executed
    Serial.println("MOVE********************");
    //compute vx, vy, vr
  
    float vm[] = { -vx +vy  +vr, +vx +vy  -vr, -vx +vy  -vr, +vx +vy  +vr};
    
    float max=0;
    for(auto v:vm)max=fabs(v)>max?fabs(v):max;
    if(max>1.0) for(auto &v:vm)v/=max;
    //scale to the roboclaw units:
    for(auto &v:vm)v*=RC_MAX_MOTOR_SPEED;
    int32_t vm_int32[4]={vm[0],vm[1],vm[2],vm[3]};
    
    rc_left.SpeedM1M2(RC_ID,vm[0],vm[1]);
    rc_right.SpeedM1M2(RC_ID,vm[2],vm[3]);
}