/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 06/11/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a program that transfers a file using data transfer (DT) packet functions
  from the SX128X library to send a file from the SD card on one Arduino to the SD card on another Arduino.
  Arduino DUEs were used for the test and this example transfers an JPG image.

  DT packets can be used for transfering large amounts of data in a sequence of packets or segments,
  in a reliable and resiliant way. The file open requests to the remote receiver, each segement sent and
  the remote file close will all keep transmitting until a valid acknowledge comes from the receiver.
  Use this transmitter with the matching receiver program, 234_SDfile_Transfer_Receiver.ino.

  On transmission the NetworkID and CRC of the payload are appended to the end of the packet by the library
  routines. The use of a NetworkID and CRC ensures that the receiver can validate the packet to a high degree
  of certainty.

  The transmitter sends the sequence of segments in order. If the sequence fails for some reason, the receiver
  will return a NACK packet to the transmitter requesting the segment sequence it was expecting.

  Details of the packet identifiers, header and data lengths and formats used are in the file;
  'Data transfer packet definitions.md' in the \SX128X_examples\DataTransfer\ folder.

  The transfer can be carried out using LoRa packets, max segment size (defined by DTSegmentSize) is 245 bytes
  for LoRa and 117 bytes for FLRC.

  Comment out one of the two following lines at the head of the program to select LoRa or FLRC;

  #define USELORA                              //enable this define to use LoRa packets
  #define USEFLRC                              //enable this define to use FLRC packets

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>

#include <SX128XLT.h>
#include <ProgramLT_Definitions.h>
#include "DTSettings.h"                      //LoRa or FLRC settings etc.
#include <arrayRW.h>

SX128XLT LoRa;                               //create an SX128XLT library instance called LoRa

#define PRINTSEGMENTNUM                      //enable this define to print segment numbers 
//#define DEBUG                              //enable this define to print debug info for segment transfers
//#define DEBUGSD                            //enable this defien to print SD file debug info
//#define ENABLEFILECRC                      //enable this define to uses and show file CRCs
//#define DISABLEPAYLOADCRC                  //enable this define if you want to disable payload CRC checking

//#define USELORA                            //enable this define to use LoRa packets
#define USEFLRC                              //enable this define to use FLRC packets

//#define SDLIB                              //define SDLIB for SD.h or SDFATLIB for SDfat.h
#define SDFATLIB

#include "DTSDlibrary.h"
#include "DTLibrary.h"

//choice of files to send
char DTFileName[] = "/$50SATL.JPG";          //file length 63091 bytes, file CRC 0x59CE
//char DTFileName[] = "/$50SATS.JPG";        //file length 6880 bytes, file CRC 0x0281
//char DTFileName[] = "/$50SATT.JPG";        //file length 1068 bytes, file CRC 0x6A02


void loop()
{
  Serial.println(("Transfer started"));

  do
  {
    DTStartmS = millis();

    //opens the local file to send and sets up transfer parameters
    if (startFileTransfer(DTFileName, sizeof(DTFileName), DTSendAttempts))
    {
      Serial.print(DTFileName);
      Serial.println(F(" opened OK on remote"));
      printLocalFileDetails();
      Serial.println();
      NoAckCount = 0;
    }
    else
    {
      Serial.print(DTFileName);
      Serial.println(F("  Error opening remote file - restart transfer"));
      DTFileTransferComplete = false;
      continue;
    }

    delay(packetdelaymS);

    if (!sendSegments())
    {
      Serial.println();
      Serial.println(F("**********************************************************"));
      Serial.println(F("Error - Segment write with no file open - Restart received"));
      Serial.println(F("**********************************************************"));
      Serial.println();
      continue;
    }

    if (endFileTransfer(DTFileName, sizeof(DTFileName)))         //send command to close remote file
    {
      DTSendmS = millis() - DTStartmS;                  //record time taken for transfer
      Serial.print(DTFileName);
      Serial.println(F(" closed OK on remote"));
      beginarrayRW(DTheader, 4);
      DTDestinationFileLength = arrayReadUint32();
      Serial.print(F("Acknowledged remote destination file length "));
      Serial.println(DTDestinationFileLength);
      if (DTDestinationFileLength != DTSourceFileLength)
      {
        Serial.println(F("ERROR - file lengths do not match"));
      }
      else
      {
        Serial.println(F("File lengths match"));
      }
#ifdef ENABLEFILECRC
      DTDestinationFileCRC = arrayReadUint16();
      Serial.print(F("Acknowledged remote destination file CRC 0x"));
      Serial.println(DTDestinationFileCRC, HEX);
      if (DTDestinationFileCRC != DTSourceFileCRC)
      {
        Serial.println(F("ERROR - file CRCs do not match"));
      }
      else
      {
        Serial.println(F("File CRCs match"));
      }
#endif
      DTFileTransferComplete = true;
    }
    else
    {
      DTFileTransferComplete = false;
      Serial.println(F("ERROR send close remote destination file failed - program halted"));
    }
  }
  while (!DTFileTransferComplete);


  Serial.print(F("NoAckCount "));
  Serial.println( NoAckCount);
  Serial.println();

  DTsendSecs = (float) DTSendmS / 1000;
  Serial.print(F("Transmit time "));
  Serial.print(DTsendSecs, 3);
  Serial.println(F("secs"));
  Serial.print(F("Transmit rate "));
  Serial.print( (DTDestinationFileLength * 8) / (DTsendSecs), 0 );
  Serial.println(F("bps"));
  Serial.println(("Transfer finished"));

  Serial.println(("Program halted"));
  while (1);
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


void setup()
{
  pinMode(LED1, OUTPUT);                          //setup pin as output for indicator LED
  led_Flash(2, 125);                              //two quick LED flashes to indicate program start
  setDTLED(LED1);                                 //setup LED pin for data transfer indicator

  Serial.begin(115200);
  Serial.println();
  Serial.println(F(__FILE__));
  Serial.flush();

  SPI.begin();

  if (LoRa.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("LoRa device error"));
    while (1)
    {
      led_Flash(50, 50);                          //long fast speed flash indicates device error
    }
  }

#ifdef USELORA
  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  Serial.println(F("Using LoRa packets"));
#endif

#ifdef USEFLRC
  LoRa.setupFLRC(Frequency, Offset, BandwidthBitRate, CodingRate, BT, Syncword);
  Serial.println(F("Using FLRC packets"));
#endif

  LoRa.printOperatingSettings();
  Serial.println();
  LoRa.printModemSettings();
  Serial.println();

  Serial.print(F("Initializing SD card..."));

  if (DTSD_initSD(SDCS))
  {
    Serial.println(F("SD Card initialized."));
  }
  else
  {
    Serial.println(F("SD Card failed, or not present."));
    while (1) led_Flash(100, 25);
  }

  Serial.println();
  DTSD_printDirectory();
  Serial.println();

#ifdef DISABLEPAYLOADCRC
  LoRa.setReliableConfig(NoReliableCRC);
#endif

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    Serial.println(F("Payload CRC disabled"));
  }
  else
  {
    Serial.println(F("Payload CRC enabled"));
  }

  DTFileTransferComplete = false;

  Serial.println(F("SDfile transfer ready"));
  Serial.println();
}
