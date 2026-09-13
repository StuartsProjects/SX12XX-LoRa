/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 10/09/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This transmitter program demonstrates the use of the LoRa RX duty cycle receive
  whereby the LoRa packet receiver automatically switches between receive and sleep modes. With the
  receiver not powered and listening for long periods there are significant power savings and the normal
  active receive current of 7mA or so can be reduced to an average of 500uA. This low average current
  makes it feasible to keep the LoRa receiver active, wating for a wake packet, which can be used to
  bring the ESP32S3 out of sleep mode. 

  The program was tested using an ESP32S3 Dev Board.

  The matching receiver program 3_LoRa_Receiver_RXdutycycle_DeepSleep_SF12BW125 has notes on how the receiver
  wake and sleep periods are calculated and those calculations for the LoRa settings used, SF12 and bandwidth
  125Khz show that a preamble length of 144 symbols is required for the transmitted packet.
    
  The code for the packets uses reliable packets which have a 16bit NetworkID and payload CRC automatically
  appended to the end of the packet. These are checked in the receiver code for a match so the chance of a
  false packet detect on the receiver is remote.      
  
  When the program starts there will be a serial menu displayed in the serial monitor. You can use the menu
  to change LoRa settings, set the number of the node you want to wake and send the wake packet.

  You can also press the BOOT switch on the ESP32S3 Dev board to transmit the wake packet. 
  
  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>     //The LoRa device is SPI based so load the SPI library
#include <SX126X.h>  //Include the appropriate library

SX126X LoRa;  //Create a library class instance called LoRa

//hardware pins for ESP32S3 Development board
#define LORA_NSS 14                //select pin on LoRa device
#define LORA_RESET 21              //reset pin on LoRa device
#define LORA_BUSY 47               //SX126X busy pin
#define LORA_DIO1 1                //DIO1 pin on LoRa device, used for RX and TX done
#define LED1 RGB_BUILTIN           //GPIO 97
#define SWITCH1 0                  //boot swtch on GPIO0
#define WAKEUP_GPIO GPIO_NUM_1     //LoRa pin for ext interrupt wakeup
#define LORA_DEVICE DEVICE_SX1262  //we need to define the device we are using

#define LORA_MOSI 2   //LoRa MOSI pin
#define LORA_SCK 42   //LoRa SCK pin
#define LORA_MISO 41  //LoRa MISO pin

//program Settings
uint32_t Frequency = 434000000;       //Frequency of transmissions in hertz
uint32_t Offset = 0;                  //Offset frequency for calibration purposes
uint8_t SpreadingFactor = LORA_SF12;  //LoRa spreading factor
uint8_t Bandwidth = LORA_BW_125;      //LoRa bandwidth
uint8_t CodeRate = LORA_CR_4_5;       //LoRa coding rate
uint8_t Optimisation = LDRO_AUTO;     //Low data rate optimisation setting, normally set to auto
int8_t TXpower = 10;                  //LoRa transmit power in dBm
uint16_t NetworkID = 0x3210;          //a unique identifier to go out with LoRa reliable packets
uint16_t PreambleSymbols = 146;       //length of packet preamble in symbols
uint16_t MenuTimeout = 30000;         //Timeout for serial menus, included just for compatibility
uint8_t ListenNodeNumber = 1;
uint16_t PacketCount = 0;
uint8_t TXPacketL;
uint8_t TXPayloadL;
uint32_t TXPacketCount = 0;
uint8_t TXActions;
uint32_t Packetus;
bool numbervalid;

#define ATWake 0xF8  //packet type for Wake

#include "SerialMenu.h"


void loop() {
  menu_LoRa(MenuTimeout);
}


void led_Flash(uint16_t flashes, uint16_t delaymS) {
  uint16_t index;

  for (index = 1; index <= flashes; index++) {
    digitalWrite(LED1, HIGH);  //Turn on the LED
    delay(delaymS);
    digitalWrite(LED1, LOW);  //Turn off the LED
  }
}


void setup() {
  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH);
  pinMode(SWITCH1, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  led_Flash(2, 250);  //flash LED to sugnal wakeup

  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println(__DATE__);
  Serial.println(__FILE__);
  Serial.println(F("Program starting"));

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  if (LoRa.begin(LORA_NSS, LORA_RESET, LORA_BUSY, LORA_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa Device found"));
  } else {
    Serial.println(F("No device responding"));
    while (1)
      ;
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);  //configure frequency and LoRa settings
  LoRa.setPacketParams(PreambleSymbols, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);

  Serial.println(F("Transmitter ready"));
  Serial.println();
}
