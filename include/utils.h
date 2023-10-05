#pragma once
#include <cstdint>
#include <Arduino.h>
class TIMER{
  ulong lt=0;
  uint16_t _time;
  public:
  TIMER(uint16_t time):_time(time){
    lt=millis();
  }
  bool operator()(){
    if(millis()-lt>_time){
      lt=millis();
      return true;
    }
  return false;
  }
};

  
