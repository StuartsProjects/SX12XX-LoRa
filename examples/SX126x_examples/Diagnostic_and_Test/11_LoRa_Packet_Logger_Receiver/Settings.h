/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 01/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

#define NSS 10                                  //select on LoRa device
#define NRESET 9                                //reset on LoRa device
#define RFBUSY 7                                //SX12XX busy pin 
#define DIO1 3                                  //DIO1 on LoRa device, used for RX and TX done 
#define LED1 8                                  //On board LED, high for on
#define BUZZER -1                               //normally not used so set to -1


#define LORA_DEVICE DEVICE_SX1262               //this is the device we are using

//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting

const int8_t TXpower = 2;                       //LoRa TX power

#define packet_delay 1000                       //mS delay between packets
