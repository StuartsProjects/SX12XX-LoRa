/*****************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

// #define SX128XPA                                 //if your device has PA, uncomment this


//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.
#ifndef SX128XPA
  #define NSS 10
  #define RFBUSY 7
  #define NRESET 9
  #define LED1 8
  #define DIO1 3

  #define RX_EN -1
  #define TX_EN -1
#endif

//These are the pin definitions for the LilyGo T3S3 V1.2 (taken from here: https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_S3_V1.2.pdf)
//be sure to change the definitions to match your own setup.
#ifdef SX128XPA
  #define NSS 7                                   //select pin on LoRa device
  #define SCK 5                                   //SCK on SPI3
  #define MISO 3                                  //MISO on SPI3 
  #define MOSI 6                                  //MOSI on SPI3 

  #define NRESET 8                                //reset pin on LoRa device
  #define RFBUSY 36

  #define LED1 37                                 //on board LED, high for on
  #define DIO1 9                                  //DIO1 pin on LoRa device, used for RX and TX done 
  #define DIO2 33                                 //DIO2 pin on LoRa device, normally not used so set to -1 
  #define DIO3 34                                 //DIO3 pin on LoRa device, normally not used so set to -1
  #define RX_EN 21                                //pin for RX enable, used on some SX128X devices, set to -1 if not used
  #define TX_EN 10                                //pin for TX enable, used on some SX128X devices, set to -1 if not used
#endif

#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using

//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t Bandwidth = LORA_BW_0800;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF8;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t Calibration = 11350;              //Manual Ranging calibration value

const int8_t TXpower = 10;                       //Transmit power used
const uint32_t RangingAddress = 16;              //must match address in master

const uint16_t  rangingRXTimeoutmS = 0xFFFF;     //ranging RX timeout in mS
