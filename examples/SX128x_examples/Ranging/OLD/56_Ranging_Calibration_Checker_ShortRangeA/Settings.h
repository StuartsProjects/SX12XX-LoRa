/*****************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 13/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, be sure to change them to match
//your own setup

#define NSS 10
#define RFBUSY 7
#define NRESET 9
#define LED1 8
#define DIO1 3
#define DIO2 -1                 //not used 
#define DIO3 -1                 //not used

#define RX_EN -1                //pin for RX enable, used on some SX1280 devices, set to -1 if not used
#define TX_EN -1                //pin for TX enable, used on some SX1280 devices, set to -1 if not used 

#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using

//*******  Setup Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t  Frequency = 2445000000;          //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes  

#define Bandwidth LORA_BW_1600                   //LoRa bandwidth
#define SpreadingFactor LORA_SF5                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate
#define RangingRole RANGING_MASTER               //Rnaging role  
#define RangingTXPower 0                         //Transmit power used 
#define RangingAddress 16                        //must match address in recever

//uint16_t const CalibrationStart = 12000;         //calibration value for ranging, SF10, BW400      
//uint16_t const CalibrationEnd = 15000;           //calibration value for ranging, SF10, BW400

#define waittimemS 5000                          //wait this long in mS for packet before assuming timeout
#define rangingTXTimeoutmS 10000                 //ranging TX timeout in mS
#define packet_delaymS 250                       //forced extra delay in mS between ranging requests 

