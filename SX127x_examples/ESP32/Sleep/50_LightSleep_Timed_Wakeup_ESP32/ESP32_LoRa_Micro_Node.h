/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 03/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Mikrobus Pro Mini,
//be sure to change the definitions to match your own setup. Some pins such as DIO1,
//DIO2, BUZZER SWITCH1 may not be in used by this sketch so they do not need to be
//connected and should be set to -1.

#define NSS 5                                   //select on LoRa device
#define SCK 18                                  //SCK on SPI3
#define MISO 19                                 //MISO on SPI3 
#define MOSI 23                                 //MOSI on SPI3 

#define NRESET 27                               //reset on LoRa device
#define DIO0 35                                 //DIO0 on LoRa device, used for RX and TX done 
#define DIO1 -1                                 //DIO1 on LoRa device, normally not used so set to -1
#define DIO2 -1                                 //DIO2 on LoRa device, normally not used so set to -1
#define LED1 2                                  //On board LED, high for on
#define BUZZER -1                               //Buzzer if fitted, high for on. Set to -1 if not used      
#define VCCPOWER 14                             //pin controls power to external devices. lora and SD card for example

#define RXpin 17                                //pin number for GPS RX input into Arduino - TX from GPS
#define TXpin 16                                //pin number for GPS TX output from Arduino- RX into GPS

#define GPSPOWER 26                              //Pin that controls power to GPS, set to -1 if not used
#define GPSONSTATE LOW                             //logic level to turn GPS on via pin GPSPOWER 
#define GPSOFFSTATE HIGH                           //logic level to turn GPS off via pin GPSPOWER 
#define GPSserial Serial2                          //define GPSserial as ESP32 Serial2 

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using


