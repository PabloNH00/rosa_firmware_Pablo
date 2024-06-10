#include <Arduino.h>
#include <robot_drive.h>
#include "rosa_esp32_defs.h"
#include "rosa_gamepad.h"
#include "rosa_esp32_utils.h"
#include "rosa_messages.h"
#include "rosa_wifi.h"

 //ROSA COMPONENTS
RobotDrive robot;
ROSAgamepad gamepad;

void handle_serial_port();
void handle_gamepad();
//MAIN command execution function prototipe
ROSAmens proccess_message( ROSAmens &m);

inline void send_message(const ROSAmens &m){
    if(!m.size)return;
    Serial.write(m.data,m.datagram_size());
}
void send_wifi_regular_messages()
{
if(!RosaWiFi::isConected2Master())return;
RosaWiFi::sendMessage(robot_data_message(robot.get_robot_data()));
//auto [x, y,yaw] = robot.get_odometry();
//RosaWiFi::sendMessage(odometry_message(x, y,yaw));
auto [x, y,yaw, vx,vy,vyaw] = robot.get_extended_odometry();
RosaWiFi::sendMessage(extended_odometry_message(x,y,yaw,vx,vy,vyaw));
}
void setup()
 {
  RosaDefs::readConfiguration();  //gets config data from a configurable internal file
  Serial.begin(COMMANDS_BAUD_RATE);
  robot.setup();
  gamepad.setup();
   
   RosaWiFi::setup(); //wifi initialization
   delay(1000);
 }
 
 void loop()
 {
    static TIMER heart_beat(ROSA_HEARTBEAT), update_ros(ROS_UPDATE_INFO_RATE), update_wifi(WIFI_UPDATE_INFO_RATE);
    handle_serial_port();
    handle_gamepad();
    RosaWiFi::loop();
    robot.loop();

    if(heart_beat())//do something to show that the uc is alive
    {

    }
    if(update_ros())//send odometry and status data to ros2
    {
        //deberia incluir un timeout que detecte si hay comunicacion
        
        //DEBUG_PRINTF("x:%5.2F y:%5.2f yaw:%5.2F",x,y,yaw);
        //auto [x, y,yaw] = robot.get_odometry();
        //send_message(odometry_message(x,y,yaw));
        auto [x, y,yaw, vx,vy,vyaw] = robot.get_extended_odometry();
        send_message(extended_odometry_message(x,y,yaw,vx,vy,vyaw));
        
    }
    if(update_wifi())send_wifi_regular_messages();
    
 }

void handle_serial_port(){
static ROSAmens::MsgReader reader; //the reader must be persistent
while (Serial.available()) {
    if (reader.add_uchar(Serial.read())) {
        auto &&m = reader.getMessage();
        send_message(proccess_message(m));

    }
}//while
}
/*****************************************
 Reads the gamepad if present. If the gamepad is in control
 then disables the command movements and takes control
 otherwise it only updates the gamepad info
*****************************************/
void handle_gamepad()
{
    gamepad.loop();
    robot.enable_move_commands();
    if(gamepad.is_controlling()){ 
        if(gamepad.emergency_stop())robot.emergency_stop();
        else{
            robot.set_relative_velocity( 
                gamepad.get_lx(), //forward
                gamepad.get_ly(), //sideward
                gamepad.get_ry()  //rotation
                );
            #ifdef DEBUG_GAME_PAD
            
            char txt[100];
            sprintf(txt,"%5.2f %5.2f %5.2f",-gamepad.get_ry(),-gamepad.get_rx(),gamepad.get_ly());
            send_message(debug_text_message(txt));
            #endif
        }
        robot.disable_move_commands();//only the gamepad moves the robot
    }
}

ROSAmens proccess_message( ROSAmens &m)
{
    WIFI_DEBUG("SERIAL COMMAND RECEIVED");
    switch (m.id) {
        case ROSA_CMD_VEL: {
            //message of CMD_VEL interpretation, get values
            auto vx=m.read<float>(), vy=m.read<float>(), vr=m.read<float>();
            //do something with that values...
            //DEBUG_PRINTF("cmd_vel:%4.2f %4.2f %4.2f",vx,vy,vr);
            robot.set_velocity(vx,vy,vr);
        }break;
        case ROSA_STOP: 
            WIFI_DEBUG("EMERGENCY STOP");
            robot.emergency_stop();
        break;
        case ROSA_RESET_ODOMETRY:
            //message that resets the robot odometry
            WIFI_DEBUG("RESET ODOMETRY");
            robot.reset_odometry();
        break;
        case ROSA_GET_WIFI_CONFIG:{
            WIFI_DEBUG("ROSA_GET_WIFI_CONFIG");
            ROSAmens resp(ROSA_WIFI_INFO);
            resp.write_array<uint8_t>(RosaDefs::IP_ADDRESS,4);
            resp.write_array<uint8_t>(RosaDefs::GATEWAY_ADDRESS,4);
            resp.write_array<uint8_t>(RosaDefs::SUBNET_MASK,4);
            resp.write_cstring(RosaDefs::WIFI_SSID);
            resp.write_cstring(RosaDefs::WIFI_KEY);
            return resp;
        }
        case ROSA_SET_WIFI_INFO:{
            m.read_array<uint8_t>(RosaDefs::IP_ADDRESS,4);
            m.read_array<uint8_t>(RosaDefs::GATEWAY_ADDRESS,4);
            m.read_array<uint8_t>(RosaDefs::SUBNET_MASK,4);
            m.read_cstring(RosaDefs::WIFI_SSID,50);
            m.read_cstring(RosaDefs::WIFI_KEY,50);
            uint8_t res=RosaDefs::writeConfiguration();
            return ROSAmens(ROSA_WIFI_CONFIGURED,res);
        }
        case ROSA_ENABLE_ROBOCLAWS:{
            auto val=m.read<uint8_t>();
            robot.enable(val);
            break;
        }
        default:
            //unknown message
            ;
        }
    return ROSAmens::none();
}