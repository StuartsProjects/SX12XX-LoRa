/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 01/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

#define NSS 10                      //select pin on LoRa device
#define NRESET 9                    //reset pin on LoRa device
#define LED1 8                      //on board LED, high for on
#define RFBUSY 7                    //SX126X busy pin 
#define DIO1 3                      //DIO1 pin on LoRa device, used for RX and TX done 

#define LORA_DEVICE DEVICE_SX1262   //this is the device we are using


//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting

//for SX1262, SX1268 power range is +22dBm to -9dBm
//for SX1261, power range is +15dBm t0 -9dBm
const int8_t TXpower = 10;                      //Transmit power used when sending packet starting test sequence
const int8_t start_power = 10;                  //link test starts at this transmit power
const int8_t end_power = -8;                    //and ends at this power
const uint8_t ThisNode = 'T';                   //this identifies the node in transmissions


#define packet_delay 1000                       //mS delay between packets
#define mode_delaymS 2000                       //mS delay after sending start test sequence  
