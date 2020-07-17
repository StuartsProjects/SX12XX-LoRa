/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 28/05/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//**************************************************************************************************
// 1) Hardware related definitions and options - specify lora board type and pins here
//**************************************************************************************************

//These are the pin definitions for one of my own boards, the Easy Mikrobus Pro Mini,
//be sure to change the definitions to match your own setup. 

//#define NSS 10                                  //select on LoRa device
//#define NRESET 9                                //reset on LoRa device
//#define DIO0 3                                  //DIO0 on LoRa device, used for RX and TX done 
#define LED1 8                                  //On board LED, high for on
#define BATVREADON 8                            //Pin that turns on the resistor divider to read battery volts
#define ONE_WIRE_BUS 4                          //for DS18B20 temperature sensor 
#define ADMultiplier 10.42                      //adjustment to convert AD value read into mV of battery voltage 
#define SupplyAD A0                             //Resistor divider for battery connected here 
#define SDCS  30                                //pin number for device select on SD card module


#define RXpin A3                                //pin number for GPS RX input into Arduino - TX from GPS
#define TXpin A2                                //pin number for GPS TX output from Arduino- RX into GPS

#define GPSPOWER -1                             //Pin that powers GPS on\off, set to -1 if not used
#define GPSONSTATE HIGH                         //logic level to turn GPS on via pin GPSPOWER 
#define GPSOFFSTATE LOW                         //logic level to turn GPS off via pin GPSPOWER 

//#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using

//**************************************************************************************************
// 2) Program Options
//**************************************************************************************************

//#define ClearAllMemory                           //Clears memory of stored tracker information, counts, errors etc
#define SleepTimesecs 15

//**************************************************************************************************
// 4) GPS Options
//**************************************************************************************************

#define GPSBaud 9600                              //GPS Baud rate

#define USESOFTSERIALGPS                          //need to include this if using softserial for GPS, otherwise hardware serial assumed      

//#define HARDWARESERIALPORT Serial1              //if your using hardware serial for the GPS, define it here  

const uint16_t WaitGPSFixSeconds = 60;            //when in flight the time to wait for a new GPS fix 

#define GPS_Library <UBLOXSerialGPS.h>            //use library file for UBLOX GPS                    
//#define GPS_Library <QuectelSerialGPS.h>        //use library file for Quectel GPS

                                                   
//**************************************************************************************************
// 7) Memory settings - define the type of memory to use for non-Volatile storage.
//    Default is internal ATmega device EEPROM but EEPROM has a limited write endurance of 'only' 
//    100,000 writes. Since the non-Volatile memory selected is written to at each transmission loop
//    and error, its highly recommended to use one of the FRAM options, these have an endurance of
//    100,000,000,000,000 writes.   
//**************************************************************************************************

//#define Memory_Library <EEPROM_Memory.h>
#define Memory_Library <FRAM_MB85RC16PNF.h>
//#define Memory_Library <FRAM_FM24CL64.h>

int16_t Memory_Address = 0x50;                     //default I2C address of MB85RC16PNF and FM24CL64 FRAM

