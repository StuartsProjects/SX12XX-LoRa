/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 19/06/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup. BUZZER may not be used
//by this sketch so they do not need to be connected and should be set to -1.

#define NSS 10                                  //select pin on lora device
#define NRESET 9                                //reset pin on lora device
#define RFBUSY 7                                //busy pin on lora device  
#define DIO1 3                                  //DIO1 pin on lora device, used for RX and TX done 

#define LED1 8                                  //on board LED, high for on
#define BUZZER -1                               //pin for buzzer, on when logic high

#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate

const int8_t TXpower = 10;                      //LoRa transmit power in dBm

const uint16_t packet_delay = 1000;             //mS delay between packets

#define RXBUFFER_SIZE 32                        //RX buffer size  
const uint16_t packetCRCcheck = 0x3F83;         //CRC to check RX packet for
const uint8_t packetCRClengthcheck = 23;        //packet length to check for
