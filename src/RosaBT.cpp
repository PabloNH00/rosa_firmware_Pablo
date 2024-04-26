#include "RMBlueTooth.h"
#include <stdarg.h>
#include "RMDebug.h"


String RMBT::_bt_name;
BluetoothSerial  RMBT::port;
RmMsg RMBT::last_message;
MsgReader RMBT::msg_reader;
void RMBT::setup(const char *bt_name )
{
 _bt_name=String(bt_name);
 port.begin(bt_name);
}
void RMBT::loop()
{
  if(!RMBT::hasClient())return;
  //someone is connected to BT
  while(RMBT::readMessage()){
    RMBT::sendMessage(executeMessage(RMBT::getMessage()));
    BT_DEBUG("message completed");
  }//while
}
void RMBT::sendText(const char *text)
{
      RmMsg &&rep_msg = text_message(text);
      port.write(rep_msg.data,rep_msg.size+3);
}
void RMBT::sendPrint(const char *fmt, ... )
{
    char text[300];
    va_list myargs;
    va_start(myargs,fmt);
    vsprintf(text,fmt,myargs);
    va_end(myargs);
    RmMsg &&rep_msg = text_message(text);
    port.write(rep_msg.data,rep_msg.size+3);
}
void RMBT::sendMessage(const RmMsg &m)
{
    if(m.size)port.write(m.data,m.size+3);
}
bool RMBT::readMessage()
{
    while(port.available()){
      uchar_t aux=port.read();
      if(msg_reader.add_uchar(aux)){
        last_message=msg_reader.getMessage();
        return true;
      }
      
    }
    return false;
} 