/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 05/11/22

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

#define NSS 10                                  //select on LoRa device
#define NRESET 9                                //reset on LoRa device
#define DIO0 3                                  //DIO0 on LoRa device, used for RX and TX done 
#define LED1 8                                  //On board LED, high for on
#define SDCS 4                                  //CS pin for SD card      
#define SWITCH1 2                               //pin number of switch for selecting tracker mode

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting
