#pragma once
#include <Arduino.h>
#include "RMDefinitions.h"
#include "BluetoothSerial.h"
#include "RMMsg.h"


class RMBT{
    static String _bt_name;
    static BluetoothSerial port;
    static RmMsg last_message;
    static MsgReader msg_reader;
public:
    static void setup(const char *bt_name = RMDefs::MODULE_NAME);
    static void sendText(const char *text);
    static void sendPrint(const char *fmt, ...);
    static bool hasClient(){return port.hasClient();}
    static void sendMessage(const RmMsg &m);
    static bool readMessage(); //return true if there is a new message. if true The message is saved on last_message
    static RmMsg getMessage(){return last_message;}
    static void loop();
};
