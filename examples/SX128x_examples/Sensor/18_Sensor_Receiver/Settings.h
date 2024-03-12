/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 29/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

#define NSS 10
#define RFBUSY 7
#define NRESET 9
#define LED1 8
#define DIO1 3

#define BATVREADON 8                            //when high turns on the resistor divider to measure voltage, -1 if not used
#define BATTERYAD A7                            //Resistor divider for battery connected here, -1 if not used
#define ADMultiplier 10.00                      //adjustment to convert AD value read into mV of battery voltage
#define DIODEMV 98                              //mV voltage drop accross diode @ low idle current

#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using 


//***************  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_0400;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate

const int8_t TXpower = 10;                       //Power for transmissions in dBm

#define packet_delay 1000                        //mS delay between packets
#define This_Node 'B'                            //this is the node that the remote sensors send data to   
