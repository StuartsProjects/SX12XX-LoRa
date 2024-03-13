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

#define LORA_DEVICE DEVICE_SX1280               //this is the device we are using

const int8_t joystickX1 = A2;                   //analog pin for the joystick 1 X pot
const int8_t joystickY1 = A3;                   //analog pin for the joystick 1 Y pot
const int8_t SWITCH1 = 2;                       //switch on joystick, set to -1 if not used

const uint32_t TXIdentity = 123 ;               //define a transmitter number, the receiver must use the same number
//range is 0 to 255



//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate

const uint8_t PacketLength = 5;                  //packet length is fixed
const int8_t TXpower = 10;                       //LoRa transmit power in dBm
