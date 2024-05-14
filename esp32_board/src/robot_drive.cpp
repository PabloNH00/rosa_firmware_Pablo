#include "robot_drive.h"
#include "rosa_esp32_utils.h"
void RobotDrive::setup(){
 rc_left.begin(RC_LEFT_RX, RC_LEFT_TX);
 rc_right.begin(RC_RIGHT_RX, RC_RIGHT_TX);
}
void RobotDrive::loop(){
  static TIMER sinc_timer(50), bat_timer(1000); //20 times per sec,1 per sec
  if(sinc_timer()){
    static int cuentas=0;
    if(cuentas==0)command_speed();
    if(cuentas==1)read_encoders();
    if(++cuentas>1)cuentas=0;
  }else
  if(bat_timer()){
    if(!mock_hardware)rc_left.read_battery(battery_voltage);
    else battery_voltage=(battery_voltage>14?12:battery_voltage+0.1);
  }

}
void RobotDrive::emergency_stop(){
    for (auto &v:target_velocity)v.t_int=0;
    if(mock_hardware)return;
    rc_left.set_speeds(0,0);
    rc_right.set_speeds(0,0);
}
//verified with multiple tests IK->FK->IK
//the loss of gdl have to be considered inb order to correctly test the IK
inline void RobotDrive::FK(const float vm[4], float &vx, float &vy, float &vr)
{ 
   vx=MEC_RAD*(vm[0]+vm[1]+vm[2]+vm[3])/4;
    vr=MEC_RAD*(-vm[0]+vm[1]+vm[2]-vm[3])/4;
    vy=MEC_RAD*(vm[0]-vm[1]+vm[2]-vm[3])/(4*LXY);
}
inline void RobotDrive::IK(const float &vx, const float &vy, const float &vr, float vm[4])
{
  vm[0] = (vx -vr  +vy*LXY)/MEC_RAD; //vy+ ->vx+
  vm[1] = (vx +vr  -vy*LXY)/MEC_RAD; //vx+ ->vy-
  vm[2] = (vx +vr  +vy*LXY)/MEC_RAD; 
  vm[3] = (vx -vr  -vy*LXY)/MEC_RAD;
}
//Sets the robot speed proportional to the maximun speed (-1.0, 1.0)
void RobotDrive::set_relative_velocity(float vx, float vy, float vr)
{
    if(!move_commands_enabled)return; //nothing is executed
    //compute vx, vy, vr
    float vm[4] ; IK(vx*MAX_FORWARD_SPEED, vy*MAX_LATERAL_SPEED, vr*MAX_ROT_SPEED, vm);
    //scale to the roboclaw units: transform rads/sec to encoders speeds
    for(auto &v:vm)v*=RADS_2_CPR*factor_speed;
    //store as integers in counts per sec
    for(int i=0;i<4;i++)target_velocity[i].t_int=static_cast<int32_t>(vm[i]);
    
          

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
    for(int i=0;i<4;i++)target_velocity[i].t_int=static_cast<int32_t>(vm[i]);

}
void RobotDrive::command_speed(){
  if(mock_hardware)return;
  rc_left.set_speeds(target_velocity[m1_left].t_int, target_velocity[m2_left].t_int);
  rc_right.set_speeds(target_velocity[m1_right].t_int, target_velocity[m2_right].t_int);
}

//if needed it is possible to deal  with overflows and unerflows (use ReqdM1Encoder instead)
//overflow, underflow will happen after 84Km so probably it is not neccesary
//it also computes in m the displacement of each wheel
void RobotDrive::read_encoders(){
  int32_u encs[4];
  float angs[4]{}; //radians
  bool left{},right{};
  if(mock_hardware){ //ENCODERS EMULATION
    static auto last_time=millis();
    float dt=(millis()-last_time)/1000.F;
    last_time=millis();
    left=right=true;
    for(int i=0;i<4;i++){
      constexpr int acc = 500;
      auto v_err=target_velocity[i].t_int-current_velocity[i].t_int;
      if(abs(v_err)<acc) current_velocity[i].t_int=target_velocity[i].t_int;
      else current_velocity[i].t_int += (v_err >0 ? acc:-acc);
      encs[i].t_int=encoder_counts[i].t_int+dt*current_velocity[i].t_int;
    }
  }  else  {
    left = rc_left.read_encoders(encs[0].t_uint,encs[1].t_uint);
    right = rc_right.read_encoders(encs[2].t_uint,encs[3].t_uint);
    if(left)WIFI_DEBUG("LEFT LEIDO");
    if(right)WIFI_DEBUG("RIGHT LEIDO");
  }
  if(left && right){
     //odometry is computed, and enc counts updated only if all readings are ok 
     //DEBUG_PRINTF("m1:%d\n",encs[0].t_int);
     for(int i=0;i<4;i++){
        angs[i]=(encs[i].t_int-encoder_counts[i].t_int);
        encoder_counts[i].t_int=encs[i].t_int;
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

  for(auto &enc:encoder_counts)enc.t_int=0;
  x_pos=y_pos=yaw=0.0F;
  if(mock_hardware)return;
  rc_left.reset_encoders();
  rc_right.reset_encoders();
}

RobotData RobotDrive::get_robot_data(){
  if(mock_hardware)battery_voltage=23.+(millis()%20)*0.1F;
  return RobotData{
  {current_velocity[0].t_int, current_velocity[1].t_int,current_velocity[2].t_int,current_velocity[3].t_int},
  {target_velocity[0].t_int,target_velocity[1].t_int,target_velocity[2].t_int,target_velocity[3].t_int},
  {encoder_counts[0].t_int,encoder_counts[1].t_int,encoder_counts[2].t_int,encoder_counts[3].t_int},
  battery_voltage
  };
}
