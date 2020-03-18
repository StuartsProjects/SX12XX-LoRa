/*
******************************************************************************************************

lora Programs for Arduino

Copyright of the author Stuart Robinson

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

Changes:

To Do:

******************************************************************************************************
*/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, be sure to change them to match
//your own setup

#define NSS 10
#define RFBUSY 7
#define NRESET 9
#define LED1 8
#define DIO1 3
#define DIO2 -1                   //not used 
#define DIO3 -1                   //not used                      
#define BUZZER A0                 //connect a buzzer here if wanted


#define ENABLEDISPLAY           //enable use of SSD1306 OLED

#define SX1280_TXBUFF_SIZE 32
#define SX1280_RXBUFF_SIZE 32


//*******  Setup Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t  Frequency = 2445000000;          //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes  

#define Bandwidth LORA_BW_0800                   //LoRa bandwidth
#define SpreadingFactor LORA_SF8                //LoRa spreading factor
#define CodeRate LORA_CR_4_8                     //LoRa coding rate

#define TXaddress 16                             //Must match address in receiver
#define Calibration 11350                        //calibration value this module for ranging, SF10, BW400
//#define waittimemS 5000                        //wait this long in mS for packet before assuming timeout
#define rangingTXTimeoutmS 1000                  //ranging TX timeout in mS
#define range_count 5                            //number of ranging requests at each attempt
#define packet_delaymS 0                         //mS delay between packets

const bool reset_device = false;                  //enable device reset between ranging requests.
const int8_t RangingTXPower = 10;                //Transmit power 
const float distance_adjustment = 1.0;        //adjustment to calculated distance 

