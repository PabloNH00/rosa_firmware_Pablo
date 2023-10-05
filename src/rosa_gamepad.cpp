#include "rosa_gamepad.h"




void ROSAgamepad::setup(){//a global object Ps3Controller is intantiated with the include
          Ps3.begin(); 
}
inline float normalize_int8(int8_t val){
    return abs(val)<10?0:val/100.0F;
}
void ROSAgamepad::loop(){
    //a global object Ps3Controller is intantiated with the include
    //only the used buttons and controls are updated and normalized
   lx=normalize_int8(Ps3.data.analog.stick.lx);
   ly=normalize_int8(Ps3.data.analog.stick.ly);
   rx=normalize_int8(Ps3.data.analog.stick.rx);
   ry=normalize_int8(Ps3.data.analog.stick.ry);
   update_capture_control();

}
//checks buttons and joytick that enables de manual control
void ROSAgamepad::update_capture_control(){
    static int counts=0;
    manual_control=false;
    if(lx||ly||rx||ry)manual_control = true;
    //trigers disable any movement command
    if( Ps3.data.button.l2 && Ps3.data.button.r2) manual_control = true;
     if( Ps3.data.button.cross) manual_control = true;
     if(manual_control)
        counts=100;
    if((counts>0)&&(!manual_control)){
            counts--;
            manual_control=true;
    }
 
}

