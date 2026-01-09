/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 09/01/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/******************************************************************************************************
  Program Operation - This is a simple LoRa test transmitter. A packet containing ASCII text is sent
  according to the frequency and LoRa settings specified at the beginning of the sketch. The GPIO pins
  used to access the LoRa device need to be defined also. The SPI pins for the Arduino board used must
  be connected to the LoRa device.

  The details of the packet sent and any errors are shown on the Serial Monitor, together with the transmit
  power used and the packet length. The matching receive program, '4_LoRa_Receiver' or '4_LoRa_ReceiverIRQ'
  can be used to check the packets are being sent correctly, the frequency and LoRa settings used must be
  the same for the Transmit and Receive programs. Sample Serial Monitor output;

  10dBm Packet> LoRa 00006  BytesSent,11  PacketsSent,6

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

#include <SPI.h>                                //The LoRa device is SPI based so load the SPI library                                         
#include <SX126XLT.h>                           //Include the appropriate library  

SX126XLT LT;                                    //Create a library class instance called LT

//LoRa Device pin numbers
#define NSS 10                                  //Select pin on LoRa device
#define NRESET 9                                //Reset pin on LoRa device
#define DIO1 3                                  //DIO1 pin on LoRa device, used for sensing RX and TX done 
#define RFBUSY 7                                //Busy pin on LoRa device 
#define LORA_DEVICE DEVICE_SX1262               //We need to define the device we are using

//SPI interface pin numbers
#define SCK 18
#define MISO 19
#define MOSI 23

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;           //Frequency of transmissions in hertz
const uint32_t Offset = 0;                      //Offset frequency for calibration purposes
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //Low data rate optimisation setting, normally set to auto
const int8_t TXpower = 10;                      //LoRa transmit power in dBm

uint8_t TXPacketL;
uint32_t TXPacketCount = 0;
uint16_t PacketCount = 0;

uint8_t buff[] = "LoRa 00000";                  //The message to send


void loop()
{
  PacketCount++;

  buff[5] = PacketCount / 10000 + '0';
  buff[6] = ((PacketCount % 10000) / 1000) + '0';
  buff[7] = ((PacketCount % 1000) / 100) + '0';
  buff[8] = ((PacketCount % 100) / 10) + '0';
  buff[9] = PacketCount % 10 + '0' ;

  Serial.print(TXpower);                                       //print the transmit power defined
  Serial.print(F("dBm > "));
  Serial.flush();

  TXPacketL = sizeof(buff);                                    //set TXPacketL to length of array

  LT.printASCIIPacket(buff, TXPacketL);                        //print the buffer (the sent packet) as ASCII

  if (LT.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX))   //will return packet length sent if OK, otherwise 0 if transmit error
  {
    TXPacketCount++;
    packet_is_OK();
  }
  else
  {
    packet_is_Error();                                         //transmit packet returned 0, there was an error
  }

  delay(500);                                                  //have a delay between packets
  Serial.println();
}


void packet_is_OK()
{
  //if here packet has been sent OK
  Serial.print(F("  Bytes,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  Serial.print(F("  TX,"));
  Serial.print(TXPacketCount);                         //print total of packets sent OK
}


void packet_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                      //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                        //print IRQ status
  LT.printIrqStatus();                                 //prints the text of which IRQs set
}


void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("3_LoRa_Transmitter Starting"));

  //SPI.begin();
  SPI.begin(SCK, MISO, MOSI);

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1);
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation); //configure frequency and LoRa settings

  Serial.print(F("Transmitter ready"));
  Serial.println();
}
