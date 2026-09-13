/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 10/09/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/******************************************************************************************************
  Program Operation - This is a LoRa test transmitter designed to be used with a program that is using
  LoRa RXdutycycle receive mode, 1_LoRa_Receiver_RXdutycycle_LightSleep_SF7BW125. A packet containing
  ASCII text is sent according to the frequency and LoRa settings specified at the beginning of the
  sketch. The packet is sent with an extended number of preamble symbols to meet the requirements of
  the receivers RXdutycycle mode. 

  The details of the packet sent and any errors are shown on the Serial Monitor, see below;

  10dBm Packet> LoRa00006  BytesSent,10  PacketsSent,6

  To send the packet that should wakeup the Receiver, press the Boot button on the Transmitters ESP32S3
  Dev board.
 
  Serial monitor baud rate is set at 115200
*******************************************************************************************************/

#include <SPI.h>     //The LoRa device is SPI based so load the SPI library
#include <SX126X.h>  //include the library which support RXdutycycle

SX126X LoRa;  //Create a library class instance called LoRa

///hardware pins for ESP32S3 Development board
#define LORA_NSS 14                //select pin on LoRa device
#define LORA_RESET 21              //reset pin on LoRa device
#define LORA_BUSY 47               //SX126X busy pin
#define LORA_DIO1 1                //DIO1 pin on LoRa device, used for RX and TX done
#define LED1 RGB_BUILTIN           //GPIO 97
#define SWITCH1 0                  //boot swtch on GPIO0
#define LORA_DEVICE DEVICE_SX1262  //we need to define the device we are using

#define LORA_MOSI 2   //LoRa MOSI pin
#define LORA_SCK 42   //LoRa SCK pin
#define LORA_MISO 41  //LoRa MISO pin

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;      //Frequency of transmissions in hertz
const uint32_t Offset = 0;                 //Offset frequency for calibration purposes
const uint8_t SpreadingFactor = LORA_SF7;  //LoRa spreading factor
const uint8_t Bandwidth = LORA_BW_125;     //LoRa bandwidth
const uint8_t CodeRate = LORA_CR_4_5;      //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;    //Low data rate optimisation setting, normally set to auto
uint16_t PreambleSymbols = 909;            //length of packet preamble in symbols
const int8_t TXpower = 10;                 //LoRa transmit power in dBm

uint8_t TXPacketL;
uint16_t TXPacketCount;

uint8_t buff[] = "LoRa00000";  //The message to send


void loop() {

  Serial.println(F("Press Boot switch to transmit wake packet"));

  while (digitalRead(SWITCH1))
    ;  //wait for boot switch to be pressed for packet transmission

  TXPacketCount++;
  Serial.print(F("TXPacketCount "));
  Serial.println(TXPacketCount);

  //form the ASCII packet
  buff[4] = TXPacketCount / 10000 + '0';
  buff[5] = ((TXPacketCount % 10000) / 1000) + '0';
  buff[6] = ((TXPacketCount % 1000) / 100) + '0';
  buff[7] = ((TXPacketCount % 100) / 10) + '0';
  buff[8] = TXPacketCount % 10 + '0';

  Serial.print(TXpower);  //print the transmit power defined
  Serial.print(F("dBm > "));
  Serial.flush();

  TXPacketL = sizeof(buff);  //set TXPacketL to length of array

  LoRa.printASCIIPacket(buff, TXPacketL);  //print the buffer (the sent packet) as ASCII

  digitalWrite(LED1, HIGH);                                     //Turn on the LED
  if (LoRa.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX))  //will return packet length sent if OK, otherwise 0 if transmit error
  {
    packet_is_OK();
  } else {
    packet_is_Error();  //transmit packet returned 0, there was an error
  }
  digitalWrite(LED1, LOW);  //Turn off the LED

  delay(2000);  //have a small delay between packets
  Serial.println();
}


void packet_is_OK() {
  //if here packet has been sent OK
  Serial.print(F("  Bytes,"));
  Serial.print(TXPacketL);  //print transmitted packet length
  Serial.print(F("  TX,"));
  Serial.print(TXPacketCount);  //print total of packets sent OK
}


void packet_is_Error() {
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LoRa.readIrqStatus();  //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);  //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);  //print IRQ status
  LoRa.printIrqStatus();         //prints the text of which IRQs set
}


void led_Flash(uint16_t flashes, uint16_t delaymS) {
  uint16_t index;

  for (index = 1; index <= flashes; index++) {
    digitalWrite(LED1, HIGH);  //Turn on the LED
    delay(delaymS);
    digitalWrite(LED1, LOW);  //Turn off the LED
    delay(delaymS);
  }
}


void setup() {
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  led_Flash(5, 250);  //flash LED to signal board boot

  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  if (LoRa.begin(LORA_NSS, LORA_RESET, LORA_BUSY, LORA_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  } else {
    Serial.println(F("No LoRa device responding"));
    while (1)
      ;
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);  //configure frequency and LoRa settings
  LoRa.setPacketParams(PreambleSymbols, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);

  Serial.print(F("Transmitter ready"));
  Serial.println();
  TXPacketCount = 0;
}
