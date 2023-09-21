#pragma once
#include <Arduino.h>
#include <Ps3Controller.h>

struct ROSAgamepad{
    float lx=0.0F, ly=0.0F, rx=0.0F, ry=0.0F;
    bool manual_control = false;
    void update_capture_control(); //chack conditions to capture control
  public:
    void setup();
    void loop();
    float get_lx(){return lx;}
    float get_ly(){return ly;}
    float get_rx(){return rx;}
    float get_ry(){return ry;}
    bool is_controlling(){return manual_control;}
    bool emergency_stop(){return Ps3.data.button.cross;}
};