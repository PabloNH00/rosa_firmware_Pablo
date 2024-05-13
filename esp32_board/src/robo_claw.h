#include <Arduino.h>



#define RC_TIMEOUT 1000 //1ms of coms timeout
#define RC_TIMEOUT2 500 //us

enum  RC_CMDS:uint8_t{CMD_RESET_ENC=20, CMD_READ_BATTERY=24, CMD_SPEEDS=37, CMD_READ_ENCODERS=78};
class RoboClawDriver{
    HardwareSerial &port;
    uint baudrate;
    uint8_t ID;
    void clr_rx();
    static uint16_t crc16(uint8_t *packet, uint8_t nBytes);
    static void add_crc16(uint8_t *packet, uint8_t n){
        auto crc=crc16(packet,n);
        packet[n]=(uint8_t)((crc>>8)&0xFF);
        packet[n+1]=(uint8_t)(crc&0xFF);
    }

    //low level methods
    bool command(uint8_t cmd);
    bool command(uint8_t cmd, int32_t d1, int32_t d2);
    bool read_command(uint8_t cmd, bool tx_crc, uint8_t *rxbuffer, uint8_t rx_n);
    bool wait_ack();
  public:
    RoboClawDriver(HardwareSerial &serial, uint brate = 38400 , uint8_t id=0x80):
                port{serial}, baudrate{brate},ID{id}{}
    void begin(uint8_t rx, uint8_t tx){
        port.begin(baudrate,SERIAL_8N1,rx,tx);
    }
    bool reset_encoders(){ return command(CMD_RESET_ENC);}
    bool set_speeds(int m1, int m2){ return command(CMD_SPEEDS,m1,m2);}
    bool read_encoders(uint32_t& e1, uint32_t& e2);
    bool read_battery(float &bat);
    
};
//wait at maximun RC_TIMEOUT us for the ack byte 0xff
inline bool RoboClawDriver::wait_ack(){
    uint32_t start = micros();
	while(!port.available())
		if((micros()-start)>RC_TIMEOUT)return false;   
	return (port.read()==0xFF);
}
