/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 23/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define LED1 8                                  //on board LED, high for on
#define RFBUSY 7                                //SX126X busy pin 
#define DIO1 3                                  //DIO1 pin on LoRa device, used for RX and TX done 

#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using


//*******  Setup Direct Modem Parameters Here ! ***************

const uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes
const uint16_t deviation = 10000;               //deviation, total frequency shift low to high
const float adjustfreq = 0.9;                   //adjustment to tone frequency

const int8_t TXpower = 10;                      //LoRa transmit power in dBm
