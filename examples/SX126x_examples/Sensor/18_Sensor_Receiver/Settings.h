/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 17/12/19

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup. Some pins such as DIO2,
//DIO3, BUZZER may not be in used by this sketch so they do not need to be
//connected and should be set to -1.

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define LED1 8                                  //on board LED, high for on
#define RFBUSY 7                                //SX126X busy pin 
#define DIO1 3                                  //DIO1 pin on LoRa device, used for RX and TX done 

#define BATVREADON 8                            //when high turns on the resistor divider to measure voltage, -1 if not used
#define BATTERYAD A7                            //Resitor divider for battery connected here, -1 if not used
#define ADMultiplier 10.00                      //adjustment to convert AD value read into mV of battery voltage
#define DIODEMV 98                              //mV voltage drop accross diode @ low idle current

#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using


//***************  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting

const int8_t TXpower = 2;                       //LoRa TX power

#define packet_delay 1000                       //mS delay between packets
#define This_Node 'B'                           //this is the node that the remote sensors send data to   

//****************  Setup Display Parameters Here  ****************

//const uint8_t dispfont = u8x8_font_chroma48medium8_r;  //display font from u8g2 library
