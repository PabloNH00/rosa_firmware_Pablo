#pragma once
#include <cstdint>
#include <Arduino.h>
#include "rosa_wifi.h"
#define ENABLE_WIFI_DEBUG 1
#define ENABLE_BT_DEBUG_RCV 0
#define ENABLE_DEBUG_PRINTF 

#define WIFI_DEBUG(X) {if(ENABLE_WIFI_DEBUG)RosaWiFi::sendText(X);}
#ifdef ENABLE_DEBUG_PRINTF
#define DEBUG_PRINTF(...) {if(ENABLE_WIFI_DEBUG)RosaWiFi::sendPrint(__VA_ARGS__);}
#else 
#define DEBUG_PRINTF(...) ;
#endif


class TIMER{
  ulong lt=0;
  uint16_t _time;
  public:
  TIMER(uint16_t time):_time(time){
    lt=millis();
  }
  //returns true if the time exceeds the conf time
  bool operator()(){
    if(millis()-lt>_time){
      lt=millis();
      return true;
    }
  return false;
  }
  //useful to implement a polling watchdog
  void reset(){lt=millis();}
};

  
