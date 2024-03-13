/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 19/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

const int8_t NSS = 10;                          //select on LoRa device
const int8_t NRESET = 9;                        //reset on LoRa device
const int8_t RFBUSY = 7;                        //RF busy on LoRa device
const int8_t DIO1 = 3;                          //DIO1 on LoRa device, used for RX and TX done
const int8_t LED1 = 8;                          //On board LED, logic high is on
const int8_t BUZZER = -1;                       //pin for buzzer, set to -1 if not used

#define LORA_DEVICE DEVICE_SX1262               //this is the device we are using

const int8_t OUTPUT0 = 2;
const int8_t OUTPUT1 = 4;
const int8_t OUTPUT2 = A3;
const int8_t OUTPUT3 = A2;

const uint32_t RXIdentity = 1234554321;         //define an identity number, the receiver must use the same number
//range is 0 to 4294967296

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

#define TXpower  10                             //power for transmissions in dBm
