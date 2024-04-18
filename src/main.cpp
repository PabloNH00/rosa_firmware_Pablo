#include <Arduino.h>
#include <robot_drive.h>
#include "rosa_defs.h"
#include "rosa_gamepad.h"
#include "utils.h"


 //ROSA COMPONENTS
RobotDrive robot;
ROSAgamepad gamepad;

void handle_serial_port();
void handle_gamepad();
inline void send_message(const ROSAmens &m){
    Serial.write(m.data,m.datagram_size());
}
void setup()
 {

  Serial.begin(COMMANDS_BAUD_RATE);
  robot.setup();
  gamepad.setup();
 }
 
 void loop()
 {
    static TIMER led_timer(ROSA_HEARTBEAT), update_ros(ROS_UPDATE_INFO_RATE);
    handle_serial_port();
    robot.loop();

    if(led_timer())//do something to show that the uc is alive
    {

    }
    if(update_ros())//send odometry and status data to ros2
    {

    }
    
 }

void handle_serial_port(){
static ROSAmens::MsgReader reader; //the reader must be persistent

while (Serial.available()) {
    if (reader.add_uchar(Serial.read())) {
        auto m = reader.getMessage();
        /*switch (m.id) {
            case 10: {
            //message of id 10 interpretation, get values
            m.read_cstring(buffer, 100);
            int value = m.read<int>();
            m.read_array<int>(vector, 3);
            //do something with that values...
        }break;
        case 2:
            //message 2 interpretation
            //...
        break;
        default:
            //unknown message
            ;
        }*/
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
            robot.set_velocity( 
                -gamepad.get_ry(), //forward
                -gamepad.get_rx(), //sideward
                gamepad.get_ly()  //rotation
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

