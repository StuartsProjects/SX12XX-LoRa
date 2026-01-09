/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 03/01/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - The program listens for incoming packets using the frequency and LoRa settings
  specified at the beginning of the sketch. The GPIO pins used to access the LoRa device need to be
  defined also. The SPI pins for the Arduino board used must be connected to the LoRa device.
  This program does not need the DIO1 pin on the LoRa device connected.

  There is a printout of the valid packets received, the packet is assumed to be in ASCII printable text,
  if its not ASCII text characters from 0x20 to 0x7F, expect weird things to happen on the Serial Monitor.
  The LED will flash for each packet received.

  Sample serial monitor output;

  3s  LoRa 00005,RSSI,-39dBm,SNR,13dB,Length,11,Packets,5,Errors,0,IRQreg,16

  If there is a packet error it might look like this, which is showing a CRC error in the received packet,

  1189s PacketError,RSSI,-111dBm,SNR,-12dB,Length,0,Packets,26,Errors,1,IRQreg,70,IRQ_HEADER_VALID,IRQ_CRC_ERROR,IRQ_RX_DONE

  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

#include <SPI.h>                                //the LoRa device is SPI based so load the SPI library
#include <SX126XLT.h>                           //include the appropriate library   

SX126XLT LT;                                    //create a library class instance called LT

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define RFBUSY 7                                //SX126X busy pin
#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using
#define RXBUFFER_SIZE 32                        //RX buffer size

uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into

uint8_t RXPacketL;                              //stores length of packet received
int8_t  PacketRSSI;                             //stores RSSI of received packet
int8_t  PacketSNR;                              //stores signal to noise ratio (SNR) of received packet


void loop()
{
  RXPacketL = LT.receiveIRQ(RXBUFFER, RXBUFFER_SIZE, 60000, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout

  PacketRSSI = LT.readPacketRSSI();              //read the received packets RSSI value
  PacketSNR = LT.readPacketSNR();                //read the received packets SNR value

  if (RXPacketL == 0)                            //if the LT.receive() function detects an error RXpacketL is 0
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }

  Serial.println();
}


void packet_is_OK()
{
  uint16_t IRQStatus;

  RXpacketCount++;
  IRQStatus = LT.readIrqStatus();                  //read the LoRa device IRQ status register
  printElapsedTime();                              //print elapsed time to Serial Monitor

  Serial.print(F("  "));
  LT.printASCIIPacket(RXBUFFER, RXPacketL);        //print the packet as ASCII characters

  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
}


void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register

  printElapsedTime();                               //print elapsed time to Serial Monitor

  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the real packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();                            //print the names of the IRQ registers set
  }
}


void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}


void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("4_LoRa_ReceiverIRQ Starting"));
  Serial.println();

  SPI.begin();

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

  LT.setupLoRa(434000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO);   //configure frequency and LoRa settings

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();
}
