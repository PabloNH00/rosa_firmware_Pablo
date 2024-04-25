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
//verified with multiple tests IK->FK->IK
//the loss of gdl have to be considered inb order to correctly test the IK
inline void RobotDrive::FK(const float vm[4], float &vx, float &vy, float &vr)
{
    vx=MEC_RAD*(-vm[0]+vm[1]-vm[2]+vm[3])/4;
    vy=MEC_RAD*(vm[0]+vm[1]+vm[2]+vm[3])/4;
    vr=MEC_RAD*(vm[0]-vm[1]-vm[2]+vm[3])/(4*LXY);
}
inline void RobotDrive::IK(const float &vx, const float &vy, const float &vr, float vm[4])
{
  vm[0] = (-vx +vy  +vr*LXY)/MEC_RAD; 
  vm[1] = (+vx +vy  -vr*LXY)/MEC_RAD; 
  vm[2] = (-vx +vy  -vr*LXY)/MEC_RAD; 
  vm[3] = (+vx +vy  +vr*LXY)/MEC_RAD;
}
//Sets the robot speed proportional to the maximun speed (-1.0, 1.0)
void RobotDrive::set_relative_velocity(float vx, float vy, float vr)
{
    if(!move_commands_enabled)return; //nothing is executed
    
    //compute vx, vy, vr
    float vm[4] ; IK(vx*MAX_FORWARD_SPEED, vy*MAX_LATERAL_SPEED, vr*MAX_ROT_SPEED, vm);

    //normalization to RC_MAX_MOTOR_SPEED
    float max=0;
    for(auto v:vm)max=fabs(v)>max?fabs(v):max;
    if(max>MAX_MOTOR_SPEED) for(auto &v:vm)v/=max;

    //scale to the roboclaw units: transform rads/sec to encoders speeds
    for(auto &v:vm)v*=RADS_2_CPR;

    //store as integers in counts per sec
    for(int i=0;i<4;i++)target_velocity[0].t_int=static_cast<int32_t>(vm[i]);

}
//Sets the robot speed proportional to the maximun speed (-1.0, 1.0)
void RobotDrive::set_velocity(float vx, float vy, float vr)
{
    if(!move_commands_enabled)return; //nothing is executed
    
    //compute the motors speeds as function of vx, vy, vr
    float vm[4]; IK(vx, vy, vr, vm);

    //scale to the roboclaw units: transform rads/sec to encoders speeds
    for(auto &v:vm)v*=RADS_2_CPR;

    //store as integers in counts per sec
    for(int i=0;i<4;i++)target_velocity[0].t_int=static_cast<int32_t>(vm[i]);

}

//if needed it is possible to deal  with overflows and unerflows (use ReqdM1Encoder instead)
//overflow, underflow will happen after 84Km so probably it is not neccesary
//it also computes in m the displacement of each wheel
void RobotDrive::read_encoders(){
  int32_u encs[4];
  float angs[4]{}; //radians
  if(rc_left.ReadEncoders(RC_ID, encs[0].t_uint,encs[1].t_uint) && 
     rc_right.ReadEncoders(RC_ID, encs[2].t_uint,encs[3].t_uint)){
     //odometry is computed, and enc counts updated only if all readings are ok 
     for(int i=0;i<4;i++){
        angs[i]=(encs[i].t_int-encoder_counts[i].t_int);
        encoder_counts[i]=encs[i];
     }
     //from encoder counts to rads 
     for(auto &r:angs)r*=CPR_2_RADS;
     //compute displacements 
     float d_xr, d_yr, d_yaw; FK(angs,d_xr, d_yr, d_yaw);
     
     //odometry update x_pos, y_pos, yaw
     x_pos+= d_xr * cos( yaw + d_yaw/2 )-d_yr * sin( yaw + d_yaw/2);
     y_pos+= d_xr * sin( yaw + d_yaw/2 )+d_yr * cos( yaw + d_yaw/2);
     yaw += d_yaw;

  }
}
//reset odometry (0, 0, 0), and the roboclaw counts
void RobotDrive::reset_odometry(){
  rc_left.ResetEncoders(RC_ID);
  rc_right.ResetEncoders(RC_ID);
  for(auto &enc:encoder_counts)enc.t_int=0;
  x_pos=y_pos=yaw=0.0F;
}

/*
notas del robogait:
   
   M1M2speed : 37
   readnecoder Ispeed: 79
   read intensitites: 49
   read batery voltage: 24
   read status: 90
   
*/