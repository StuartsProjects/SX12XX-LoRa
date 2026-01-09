/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 03/01/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/******************************************************************************************************
  Program Operation - This is a simple LoRa test transmitter. A packet containing ASCII text is sent
  according to the frequency and LoRa settings specified at the beginning of the sketch. The GPIO pins
  used to access the LoRa device need to be defined also. The SPI pins for the Arduino board used must
  be connected to the LoRa device. This program does not need the DIO1 pin on the LoRa device connected.

  The details of the packet sent and any errors are shown on the Serial Monitor, together with the transmit
  power used and the packet length. The matching receive program, '4_LoRa_Receiver' or '4_LoRa_ReceiverIRQ'
  can be used to check the packets are being sent correctly, the frequency and LoRa settings used must be
  the same for the Transmit and Receive programs. Sample Serial Monitor output;

  10dBm Packet> LoRa 00006  BytesSent,11  PacketsSent,6

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/


#include <SPI.h>                                 //the SX128X device is SPI based so load the SPI library                                         
#include <SX128XLT.h>                            //include the appropriate LoRa library  

SX128XLT LT;                                     //create a library class instance called LT

#define NSS 10
#define RFBUSY 7
#define NRESET 9
#define LED1 8
#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using  

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_0400;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const int8_t TXpower = 10;                       //LoRa transmit power in dBm

uint8_t TXPacketL;
uint32_t TXPacketCount;
uint16_t PacketCount = 0;

uint8_t buff[] = "LoRa 00000";

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

  digitalWrite(LED1, HIGH);

  if (LT.transmitIRQ(buff, TXPacketL, 10000, TXpower, WAIT_TX))   //will return packet length sent if OK, otherwise 0 if transmit, timeout 10 seconds
  {
    TXPacketCount++;
    packet_is_OK();
  }
  else
  {
    packet_is_Error();                                 //transmit packet returned 0, there was an error
  }

  digitalWrite(LED1, LOW);
  Serial.println();
  delay(500);                                          //have a delay between packets
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
  Serial.println(F("3_LoRa_TransmitterIRQ Starting"));

  SPI.begin();

  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, RFBUSY, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1);
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  Serial.print(F("Transmitter ready"));
  Serial.println();
}
