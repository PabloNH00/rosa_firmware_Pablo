//RMerin global variables implementations
#include "rosa_esp32_utils.h"
#include "rosa_esp32_defs.h"
//configurable global variables
#include "SPIFFS.h"


//#define ETSIDI

char RosaDefs::ROBOT_NAME[100]="ROSA";
uint8_t RosaDefs::SUBNET_MASK[4]={255, 255, 255, 0};

#ifdef ETSIDI
uint8_t RosaDefs::IP_ADDRESS[4]={192, 168, 253, 9};
uint8_t RosaDefs::GATEWAY_ADDRESS[4]={192, 168, 253, 61};
 char RosaDefs::WIFI_SSID[50]="RM2_WIFI";
 char RMRosaDefsDefs::WIFI_KEY[50]="R0BOMINERS*";
#else
  uint8_t RosaDefs::IP_ADDRESS[4]={192, 168, 1, 109};       
  uint8_t RosaDefs::GATEWAY_ADDRESS[4]={192, 168, 1, 1}; 
  char RosaDefs::WIFI_SSID[50]="MHG";   
  char RosaDefs::WIFI_KEY[50]="fll100218"; 
#endif

//Reads the configuration file if exists. Creates the configuration file 
// with the default configuration otherwise
bool RosaDefs::readConfiguration(){
//Intenta la  apertura de ficheros: montando el sistema
  if(!SPIFFS.begin(true)){ //formats SPIFFS on error
     WIFI_DEBUG("An Error has occurred while mounting SPIFFS");
     return false;
  }
  #if SET_DEFAULT_CONFIGURATION_DATA == 1
    BT_DEBUG("Removing the previous configuration file");
    if (!SPIFFS.remove("/configuration.txt"))
      BT_DEBUG("Configuration file does not exist");
  #endif

  //if no file exists, create the file with default values
  if(!SPIFFS.exists("/configuration.txt")){
    WIFI_DEBUG("No configuration file found. Writing a new one");
    return writeConfiguration();
  }
  
  File conf_file = SPIFFS.open("/configuration.txt");
  if(!conf_file){
    WIFI_DEBUG("Unable to open the configuration file");
    return false;
  }
  //reads the binary data at the begining
  //version(u8)
  int file_version=0;
  if(conf_file.available())file_version=conf_file.read();
  if(file_version!=ROSA_CONF_FILE_VERSION){
      conf_file.close();
      WIFI_DEBUG("Conf Version changed: reseting conf file.");
      return writeConfiguration();
  }
  //as a closed sytem it is, no reading errors are considered
  conf_file.read(IP_ADDRESS,4);
  DEBUG_PRINTF("config file: IP = %d.%d.%d.%d",IP_ADDRESS[0],IP_ADDRESS[1],IP_ADDRESS[2],IP_ADDRESS[3]);
  conf_file.read(GATEWAY_ADDRESS,4);
  conf_file.read(SUBNET_MASK,4);
  ROBOT_NAME[conf_file.readBytesUntil('\0',ROBOT_NAME, 99)]='\0';
  WIFI_SSID[conf_file.readBytesUntil('\0',WIFI_SSID, 49)]='\0';
  WIFI_KEY[conf_file.readBytesUntil('\0',WIFI_KEY, 49)]='\0';
  conf_file.close();
  return true;
}

bool RosaDefs::writeConfiguration(){
 File conf_file = SPIFFS.open("/configuration.txt",FILE_WRITE);
  if(!conf_file){
    WIFI_DEBUG("Unable to write the configuration file");
    return false;
  }
  conf_file.write(ROSA_CONF_FILE_VERSION);
  conf_file.write(IP_ADDRESS,4);
  conf_file.write(GATEWAY_ADDRESS,4);
  conf_file.write(SUBNET_MASK,4);
  conf_file.write((uint8_t *)ROBOT_NAME,strlen(ROBOT_NAME)+1);
  conf_file.write((uint8_t *)WIFI_SSID,strlen(WIFI_SSID)+1);
  conf_file.write((uint8_t *)WIFI_KEY,strlen(WIFI_KEY)+1);
  conf_file.close();
  return true;
}