#include "robo_claw.h"
#include "rosa_esp32_utils.h"
uint16_t RoboClawDriver::crc16(uint8_t *packet, uint8_t nBytes){
    uint16_t crc=0;
    for (size_t byte = 0; byte < nBytes; byte++) {
        crc = crc ^ ((uint16_t) packet[byte] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000)crc = (crc << 1) ^ 0x1021;
                else crc = crc << 1;
            }
        }
    return crc;
}
void RoboClawDriver::clr_rx(){
    while(port.available())port.read();
}
bool RoboClawDriver::command(uint8_t cmd)
{
    uint8_t packet[4]{ID,cmd};

    clr_rx();
    add_crc16(packet,sizeof(packet)-2);
    port.write(packet,sizeof(packet));
    return wait_ack();
}
bool RoboClawDriver::command(uint8_t cmd, int32_t d1, int32_t d2){
    uint8_t packet[12]{ID,cmd};

        // RoboClaw expects big endian / MSB first
        packet[2] = (uint8_t) ((d1 >> 24) & 0xFF);
        packet[3] = (uint8_t) ((d1 >> 16) & 0xFF);
        packet[4] = (uint8_t) ((d1 >> 8) & 0xFF);
        packet[5] = (uint8_t) (d1 & 0xFF);

        packet[6] = (uint8_t) ((d2 >> 24) & 0xFF);
        packet[7] = (uint8_t) ((d2 >> 16) & 0xFF);
        packet[8] = (uint8_t) ((d2 >> 8) & 0xFF);
        packet[9] = (uint8_t) (d2 & 0xFF);
    clr_rx();
    add_crc16(packet,sizeof(packet)-2);
    port.write(packet,sizeof(packet));
    return wait_ack();
}

bool RoboClawDriver::read_encoders(uint32_t& e1, uint32_t& e2)
{
    uint8_t rx[10];
    if(read_command(CMD_READ_ENCODERS, false, rx, 10)){
        e1= (rx[0]<<24)|(rx[1]<<16)|(rx[2]<<8)|(rx[3]); 
        e2= (rx[4]<<24)|(rx[5]<<16)|(rx[6]<<8)|(rx[7]);
        return true;
    }
    return false;
}
bool RoboClawDriver::read_battery(float &bat)
{
    uint8_t rx[4];
    if(read_command(CMD_READ_BATTERY, false, rx, 4)){
        uint16_t e1= (rx[0]<<8)|(rx[1]); 
        bat=e1/10.0;
        return true;
    }
    return false;
} 
bool RoboClawDriver::read_command(uint8_t cmd, bool tx_crc, uint8_t *rxbuffer, uint8_t rx_n)
{
    uint8_t packet[4]{ID,cmd};
    clr_rx();
    if(tx_crc){
        add_crc16(packet,sizeof(packet)-2);
        port.write(packet,sizeof(packet));
    }else port.write(packet,sizeof(packet-2));
    
    uint32_t start = micros();
    uint8_t n=0;
	while((micros()-start)<RC_TIMEOUT+RC_TIMEOUT2*rx_n){
        if(port.available())rxbuffer[n++]=port.read();
        if(n==rx_n)break;
    }
    if(rx_n!=n){
    DEBUG_PRINTF("READ ERR: esperados%d leidos%d\n",rx_n,n);
    return false;
    }
    //check crc
    uint16_t crc=crc16(rxbuffer,n-2);
    uint16_t crc_rx=((int16_t)(rxbuffer[rx_n-2]) << 8)|(rxbuffer[rx_n-1]);
    if(crc!=crc_rx) DEBUG_PRINTF("CRC ERR: esperados%d leidos%d\n",crc,crc_rx);
    return (crc==crc_rx);		 
}