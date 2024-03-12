/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 06/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

#define NSS 10
#define RFBUSY 7
#define NRESET 9
#define LED1 8
#define DIO1 3

#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using  

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate

#define TXpower  10                              //power for transmissions in dBm

#define packet_delay 1000                        //mS delay between packets
