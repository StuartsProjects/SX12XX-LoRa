//HAB2_Board_Definitions.h

/*
*******************************************************************************************************************************
  Easy Build LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 02/10/19

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
  of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  Hardware definitions for the Easy_Mikrobus_DIP board.

  To Do:

*******************************************************************************************************************************
*/

#define SWITCH1 0                    //pin number to attach switch 

#define lora_NSS 5                   //pin number where the NSS line for the LoRa device is connected
#define SDCS  13                     //ESP32 pin number for device select on SD card module
#define VCCPOWER 14                  //pin controls power to external devices
#define lora_DIO2 15                 //pin number for DIO2 pin on LoRa device
#define lora_TonePin 15              //pin number for LoRa radio tone generation, connects to LoRa device pin DIO2
#define GPSTX 16                     //pin number for TX output from Arduino - RX into GPS
#define GPSRX 17                     //pin number for RX input into Arduino - TX from GPS
#define BATREAD 25                   //pin that switches on supply measure resistor devider
#define GPSPOWER 26                  //pin controls power to external devices
#define lora_NReset 27               //pin where LoRa device reset line is connected
#define ONE_WIRE_BUS 33              //pin for one wire bus devices  
#define lora_DIO1 34                 //pin connected to DIO1 on LoRa device
#define lora_DIO0 35                 //pin number for DIO0 pin on LoRa device
#define SupplyAD 36                  //pin for reading supply voltage
#define WAKE 39                      //Wake signal form TPL5010 (if fitted)




 






#define ADMultiplier 10              //multiplier for supply volts calculation, default


