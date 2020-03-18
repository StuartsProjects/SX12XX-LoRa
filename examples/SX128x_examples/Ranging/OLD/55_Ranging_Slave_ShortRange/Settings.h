/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 13/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//Note - The Master and Slave programs use a common 'Settings.h' file so its easy to be sure that master
//and slave have the same settings. Just copy this file to both master and slave. A consequence of this 
//is that some settings below may not be needed by this program. 


//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Mikrobus Pro Mini,
//be sure to change the definitions to match your own setup. Some pins such as DIO2,
//DIO3, BUZZER may not be in used by this sketch so they do not need to be
//connected and should be included and be set to -1.

#define NSS 10
#define RFBUSY 7
#define NRESET 9
#define LED1 8
#define DIO1 3
#define DIO2 -1                 //not used 
#define DIO3 -1                 //not used
#define RX_EN -1                //pin for RX enable, used on some SX1280 devices, set to -1 if not used
#define TX_EN -1                //pin for TX enable, used on some SX1280 devices, set to -1 if not used  
       
#define ENABLEDISPLAY           //enable this define for display option 

#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using  

//LoRa Modem Parameters
const uint32_t  Frequency = 2445000000;          //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes  
const uint8_t Bandwidth = LORA_BW_1600;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF5;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate

const uint8_t TXpower = 10;                      //Power for transmissions in dBm

/*******************************************************************************************
Calibration value - The calibration value below will depend on the LoRa bandwidth used and can
also vary depending on the actual hardware. Changing the spreading factor has a small
affect. Typical values are LORA_BW_0400,10450  LORA_BW_0800,12870 LORA_BW_1600,16800
*******************************************************************************************/ 

const uint16_t Calibration = 13150;               //Ranging calibration value, needs to be appropriate for bandwidth 

const uint32_t ranging_address = 16;             //Must match address in receiver
const uint16_t TXtimeoutmS = 1000;               //ranging TX timeout in mS
const uint8_t range_count = 5;                   //number of ranging requests at each attempt
const uint16_t waittimemS = 5000;                //wait this long in mS for packet before assuming timeout


const uint16_t packet_delay = 0;                 //mS delay between ranging packets
const float distance_adjustment = 1.0;           //adjustment to calculated distance


