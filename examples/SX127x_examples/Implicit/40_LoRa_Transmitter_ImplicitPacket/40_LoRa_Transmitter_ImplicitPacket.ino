/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 04/01/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - This is an example of the use of implicit or fixed length LoRa packets.
  Implicit packets have no header so both transmitter and receiver need to be programmed with the packet
  length in use. The use of spreading factor 6 requires implicit packets and together with a bandwidth
  of 500khz, leads to the shortest possible and lowest air time packets.

  This example sends a buffer that is 23 characters long and that length must be defined in Settings.h
  as the constant 'PacketLength'.

  A packet containing ASCII text is sent according to the frequency and LoRa settings specified in the
  'Settings.h' file. The pins to access the lora device need to be defined in the 'Settings.h' file also.

  The details of the packet sent and any errors are shown on the Serial Monitor, together with the transmit
  power used, the packet length and the CRC of the packet. The matching receive program,
  '41_LoRa_Receiver_ImplicitPackets' can be used to check the packets are being sent correctly, the
  frequency and LoRa settings (in Settings.h) must be the same for the Transmit and Receive program.

  Sample Serial Monitor output;

  10dBm Packet> Hello World 123456*  BytesSent,23  CRC,DAAB  TransmitTime,8mS  PacketsSent,1

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

#include <SPI.h>                                               //the lora device is SPI based so load the SPI library                                         
#include <SX127XLT.h>                                          //include the appropriate library  
#include "Settings.h"                                          //include the setiings file, frequencies, LoRa settings etc   

SX127XLT LT;                                                   //create a library class instance called LT

uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;

uint8_t buff[] = "Hello World 1234567890";                     //buffer length must be 23, and defined in constant PacketLength


void loop()
{
  Serial.print(TXpower);                                       //print the transmit power defined
  Serial.print(F("dBm "));
  Serial.print(F("Packet> "));
  Serial.flush();

  TXPacketL = sizeof(buff);                                    //set TXPacketL to length of array, must be 19 characters
  buff[TXPacketL - 1] = '*';                                   //replace null character at buffer end so its visible on reciver

  LT.printASCIIPacket(buff, TXPacketL);                        //print the buffer (the sent packet) as ASCII

  digitalWrite(LED1, HIGH);
  startmS =  millis();                                         //start transmit timer
  if (LT.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX))   //will return packet length sent if OK, otherwise 0 if transmit error
  {
    endmS = millis();                                          //packet sent, note end time
    TXPacketCount++;
    packet_is_OK();
  }
  else
  {
    packet_is_Error();                                         //transmit packet returned 0, there was an error
  }

  digitalWrite(LED1, LOW);
  Serial.println();
  delay(packet_delay);                                         //have a delay between packets
}


void packet_is_OK()
{
  //if here packet has been sent OK
  uint16_t localCRC;

  Serial.print(F("  BytesSent,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  localCRC = LT.CRCCCITT(buff, TXPacketL, 0xFFFF);
  Serial.print(F("  CRC,"));
  Serial.print(localCRC, HEX);                         //print CRC of sent packet
  Serial.print(F("  TransmitTime,"));
  Serial.print(endmS - startmS);                       //print transmit time of packet
  Serial.print(F("mS"));
  Serial.print(F("  PacketsSent,"));
  Serial.print(TXPacketCount);                         //print total of packets sent OK
}


void packet_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                      //read the the interrupt register
  Serial.print(F("SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                        //print IRQ status
  LT.printIrqStatus();                                 //prints the text of which IRQs set
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setupLoRa()
{
  //this setup is used so as the implicit packet type,LORA_PACKET_FIXED_LENGTH, is used
  LT.setMode(MODE_STDBY_RC);                              //got to standby mode to configure device
  LT.setPacketType(PACKET_TYPE_LORA);                     //set for LoRa transmissions
  LT.setRfFrequency(Frequency, Offset);                   //set the operating frequency
  LT.calibrateImage(0);                                   //run calibration after setting frequency
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);  //set LoRa modem parameters
  LT.setBufferBaseAddress(0x00, 0x00);                    //where in the SX buffer packets start, TX and RX
  LT.setPacketParams(8, LORA_PACKET_FIXED_LENGTH, PacketLength, LORA_CRC_ON, LORA_IQ_NORMAL);  //set packet parameters
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);              //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
  LT.setHighSensitivity();                                //set for highest sensitivity at expense of slightly higher LNA current
  //This is the typical IRQ parameters set, actually excecuted in the transmit function
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);   //set for IRQ on TX done
}


void setup()
{
  pinMode(LED1, OUTPUT);                                   //setup pin as output for indicator LED
  led_Flash(2, 125);                                       //two quick LED flashes to indicate program start

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("40_LoRa_Transmitter_ImplicitPacket Starting"));

  SPI.begin();

  //SPI beginTranscation is normally part of library routines, but if it is disabled in library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);                                   //two further quick LED flashes to indicate device found
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                 //long fast speed LED flash indicates device error
    }
  }

  //this function call sets up the device for LoRa using the settings from the Settings.h file
  setupLoRa();

  Serial.println();
  LT.printModemSettings();                                //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x00, 0x4F);                         //print contents of device registers, normally 0x00 to 0x4F
  Serial.println();
  Serial.println();

  Serial.print(F("Transmitter ready"));
  Serial.println();
}
