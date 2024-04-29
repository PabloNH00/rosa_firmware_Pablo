#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "rosa_messages.h"
#include "rosa_esp32_defs.h"

#ifdef _WIN32
    #include "AsyncUdp.h"
#else
    #include "AsyncUDP.h"
#endif

class RosaWiFi{

    static enum WIFI_STATE {NONE, INITIALIZING, CONNECTED} state;
    static long _timeout;
    static long _time;
    static long _stop_time;
    static long _time_watch_dog;
    static AsyncUDP udp;
    static ROSAmens::MsgReader msg_reader;
    static const char * translateEncryptionType(wifi_auth_mode_t encryptionType);
    static ROSAmens::CircularBuffer<> buffer;
public:
    static IPAddress ip_remote;
    static void scanNetworks();
    static void init(int timeout=5000, const char *ssid=RosaDefs::WIFI_SSID, const char *psswd=RosaDefs::WIFI_KEY);
    static void setup();
    static void loop();
    static void listenUDP();
    static bool isConnected(){return state==WIFI_STATE::CONNECTED;}
    static ROSAmens executeWifiMessage(const ROSAmens &m);
    static void sendMessage(const ROSAmens &m, const IPAddress &ip=ip_remote);
    static bool isConected2Master(){return((isConnected())&&(RosaWiFi::ip_remote[0]));}
    static void sendText(const char *text);
    static void sendPrint(const char *fmt, ...);

};

