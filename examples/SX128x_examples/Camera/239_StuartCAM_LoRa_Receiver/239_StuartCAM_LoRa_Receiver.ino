/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 12/11/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a demonstration program that uses LoRa to receive an image taken with a
  remote OV2640 camera. The LoRa transfer is carried out using the data transfer functions of the
  SX12XX-LoRa library.

  The Arducam software on the transmitter uses program 238_StuartCAM_LoRa_Remote_Camera to take an image
  and then transfers it as a file across to this receiver, when the transfer is complete the file, with
  the original name, is saved as a file on a SD card.

  Program uses an ILI931 TFT display to show progress of the transfer. Arduino DUE was used.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define Program_Version "V1.0"

#define USELORA                             //enable this define to use LoRa packets
//#define USEFLRC                           //enable this define to use FLRC packets

#include <SPI.h>

#include <SX128XLT.h>                      //SX12XX-LoRa library
#include <ProgramLT_Definitions.h>         //part of SX12XX-LoRa library
#include "Settings.h"                      //LoRa settings etc.
#include <arrayRW.h>                       //part of SX12XX-LoRa library    

SX128XLT LoRa;                             //create an SX128XLT library instance called LoRa

//#define SDLIB                            //define SDLIB for SD.h or SDFATLIB for SDfat.h
#define SDFATLIB

#define PRINTSEGMENTNUM                    //enable this define to print segment numbers during data transfer
//#define DEBUG                            //enable this define to show data transfer debug info
//#define DEBUGSD                          //enable this define to show SD debug info
//#define ENABLEFILECRC                    //enable this define to uses and show file CRCs
//#define DISABLEPAYLOADCRC                //enable this define if you want to disable payload CRC checking

#include "Adafruit_GFX.h"                  //get library here > https://github.com/adafruit/Adafruit-GFX-Library  
#include "Adafruit_ILI9341.h"              //get library here > https://github.com/adafruit/Adafruit_ILI9341

Adafruit_ILI9341 disp = Adafruit_ILI9341(DISPCS, DISPDC, DISPRESET);  //for dispaly defaults, textsze and rotation, see Settings.h

uint8_t RXPacketL;                         //length of received packet
uint8_t RXPacketType;                      //type of received packet, segment write, ACK, NACK etc
uint16_t RXErrors;                         //count of packets received with error
uint8_t RXFlags;                           //DTflags byte in header, could be used to control actions in TX and RX
uint8_t RXHeaderL;                         //length of header
uint8_t RXDataarrayL;                      //length of data array\segment
int16_t PacketRSSI;                        //stores RSSI of received packet
int8_t  PacketSNR;                         //stores signal to noise ratio of received packet
uint16_t TXNetworkID;                      //this is used to store the 'network' number, receiver must have the same networkID
uint16_t TXArrayCRC;                       //should contain CRC of data array transmitted
uint8_t  TXPacketL;                        //length of transmitted packet
uint16_t AckCount;                         //keep a count of acks that are received within timeout period
uint16_t NoAckCount;                       //keep a count of acks not received within timeout period
uint16_t LocalPayloadCRC;                  //for calculating the local data array CRC
uint16_t DTDestinationFileCRC;             //CRC of complete file received
uint16_t DTSourceFileCRC;                  //CRC returned of the remote saved file
uint32_t DTDestinationFileLength;          //length of file written on the destination\receiver
uint32_t DTSourceFileLength;               //length of file at source\transmitter
uint32_t DTStartmS;                        //used for timeing transfers
bool DTFileOpened;                         //bool to flag when file has been opened
uint16_t DTSegment = 0;                    //current segment number
uint16_t DTSegmentNext;                    //next segment expected
uint16_t DTReceivedSegments;               //count of segments received
uint16_t DTSegmentLast;                    //last segment processed
uint8_t DTLastSegmentSize;                 //size of the last segment
uint16_t DTNumberSegments;                 //number of segments for a file transfer
uint16_t DTSentSegments;                   //count of segments sent
bool DTFileTransferComplete;               //bool to flag file transfer complete
uint32_t DTSendmS;                         //used for timing transfers
float DTsendSecs;                          //seconds to transfer a file
char DTfilenamebuff[DTfilenamesize];

int DTLED = -1;                            //pin number for indicator LED, if -1 then not used

uint8_t DTheader[16];                      //header array
uint8_t DTdata[245];                       //data/segment array

#ifdef SDFATLIB
#include <SdFat.h>
SdFat SD;
File dataFile;                              //name the file instance needed for SD library routines
#endif

#ifdef SDLIB
#include <SD.h>
File dataFile;                              //name the file instance needed for SD library routines
#endif


void loop()
{
  receiveaPacketDT();
}


bool sendFile(char *DTFileName, uint8_t namelength)
{
  // Start filesend process
  // This routine allows the file transfer to be run with a function call of sendFile(FileName, sizeof(FileName));

  do
  {
    NoAckCount = 0;
    DTStartmS = millis();

    //opens the local file to send and sets up transfer parameters
    if (startFileTransfer(DTFileName, namelength, DTSendAttempts))
    {
      Serial.print(DTFileName);
      Serial.println(F(" opened OK on remote"));
      printLocalFileDetails();
      Serial.println();
    }
    else
    {
      Serial.println(F("*************************"));
      Serial.println(DTFileName);
      Serial.println(F("ERROR opening remote file"));
      Serial.println(F("Restarting transfer"));
      Serial.println(F("*************************"));
      DTFileTransferComplete = false;
      continue;
    }

    delay(packetdelaymS);

    if (!sendSegments())
    {
      Serial.println();
      Serial.println(F("************************"));
      Serial.println(DTFileName);
      Serial.println(F("ERROR in sendSegments()"));
      Serial.println(F("Restarting transfer"));
      Serial.println(F("***********************"));
      Serial.println();
      DTFileTransferComplete = false;
      continue;
    }

    delay(packetdelaymS);

    if (endFileTransfer(DTFileName, namelength))         //send command to close remote file
    {
      DTSendmS = millis() - DTStartmS;                   //record time taken for transfer
      Serial.print(DTFileName);
      Serial.println(F(" closed OK on remote"));
      beginarrayRW(DTheader, 4);
      DTDestinationFileLength = arrayReadUint32();
      Serial.print(F("Acknowledged remote destination file length "));
      Serial.println(DTDestinationFileLength);

      if (DTDestinationFileLength != DTSourceFileLength)
      {
        Serial.println(F("*******************************"));
        Serial.println(DTFileName);
        Serial.println(F("ERROR file lengths do not match"));
        Serial.println(F("Restarting transfer"));
        Serial.println(F("*******************************"));
        DTFileTransferComplete = false;
        continue;
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
        Serial.println(F("****************************"));
        Serial.println(DTFileName);
        Serial.println(F("ERROR file CRCs do not match"));
        Serial.println(F("Restarting transfer"));
        Serial.println(F("****************************"));
        DTFileTransferComplete = false;
        continue;
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
      Serial.println(F("******************************"));
      Serial.println(DTFileName);
      Serial.println(F("ERROR close remote file failed"));
      Serial.println(F("Restarting transfer"));
      Serial.println(F("******************************"));
      DTFileTransferComplete = false;
      continue;
    }
  }
  while (!DTFileTransferComplete);

  Serial.print(F("NoAckCount "));
  Serial.println(NoAckCount);
  DTsendSecs = (float) DTSendmS / 1000;
  Serial.print(F("Transmit time "));
  Serial.print(DTsendSecs, 3);
  Serial.println(F("secs"));
  Serial.print(F("Transmission rate "));
  Serial.print( (DTDestinationFileLength * 8) / (DTsendSecs), 0 );
  Serial.println(F("bps"));
  Serial.println(("Transfer finished"));
  return true;
}


bool sendFileSegment(uint16_t segnum, uint8_t segmentsize)
{
  // Send file segment as payload in a packet

  uint8_t ValidACK;

  DTSD_readFileSegment(DTdata, segmentsize);
  build_DTSegmentHeader(DTheader, DTSegmentWriteHeaderL, segmentsize, segnum);

#ifdef PRINTSEGMENTNUM
  //Serial.print(F("Segment,"));
  Serial.println(segnum);
  setCursor(0, 5);
  disp.print(segnum);
#endif

#ifdef DEBUG
  Serial.print(F(" "));
  printheader(DTheader, DTSegmentWriteHeaderL);
  Serial.print(F(" "));
  printdata(DTdata, segmentsize);                           //print segment size of data array only
#endif

  do
  {
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }

    TXPacketL = LoRa.transmitDT(DTheader, DTSegmentWriteHeaderL, (uint8_t *) DTdata, segmentsize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }

    if (TXPacketL == 0)                                     //if there has been an error TXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
    }

    ValidACK = LoRa.waitACKDT(DTheader, DTSegmentWriteHeaderL, ACKsegtimeoutmS);
    RXPacketType = DTheader[0];

    if (ValidACK > 0)
    {
      if (RXPacketType == DTSegmentWriteNACK)
      {
        DTSegment = DTheader[4] +  (DTheader[5] << 8);      //load what the segment number should be
        RXHeaderL = DTheader[2];
        Serial.println();
        Serial.println(F("************************************"));
        Serial.print(F("Received restart request at segment "));
        Serial.println(DTSegment);
        printheader(DTheader, RXHeaderL);
        Serial.println();
        Serial.print(F("Seek to file location "));
        Serial.println(DTSegment * DTSegmentSize);
        Serial.println(F("************************************"));
        Serial.println();
        Serial.flush();
        DTSD_seekFileLocation(DTSegment * DTSegmentSize);
      }

      //ack is valid, segment was acknowledged if here

      if (RXPacketType == DTStartNACK)
      {
        Serial.println(F("Received restart request"));
        return false;
      }

      if (RXPacketType == DTSegmentWriteACK)
      {
        AckCount++;
#ifdef DEBUG
        printAckBrief();
        //printAckDetails()
#endif
        DTSegment++;                  //increase value for next segment
        return true;
      }
    }
    else
    {
      NoAckCount++;
      Serial.print(F("Error no ACK received "));
      Serial.println(NoAckCount);

      if (NoAckCount > NoAckCountLimit)
      {
        Serial.println(F("ERROR NoACK limit reached"));
        return false;
      }
    }
  } while (ValidACK == 0);

  return true;
}


bool startFileTransfer(char *buff, uint8_t filenamesize, uint8_t attempts)
{
  // Start file transfer, open local file first then remote file.

  uint8_t ValidACK;

  Serial.print(buff);
  Serial.println(F(" Start file transfer"));
  DTSourceFileLength = DTSD_openFileRead(buff);     //get the file length

  if (DTSourceFileLength == 0)
  {
    Serial.print(F("Error - opening local file "));
    Serial.println(buff);
    return false;
  }

#ifdef ENABLEFILECRC
  DTSourceFileCRC = DTSD_fileCRCCCITT(DTSourceFileLength);        //get file CRC from position 0 to end
#endif

  DTNumberSegments = DTSD_getNumberSegments(DTSourceFileLength, DTSegmentSize);
  DTLastSegmentSize = DTSD_getLastSegmentSize(DTSourceFileLength, DTSegmentSize);
  build_DTFileOpenHeader(DTheader, DTFileOpenHeaderL, filenamesize, DTSourceFileLength, DTSourceFileCRC, DTSegmentSize);
  LocalPayloadCRC = LoRa.CRCCCITT((uint8_t *) buff, filenamesize, 0xFFFF);

  do
  {
    Serial.println(F("Send open remote file request"));

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    TXPacketL = LoRa.transmitDT(DTheader, DTFileOpenHeaderL, (uint8_t *) buff, filenamesize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }

#ifdef DEBUG
    Serial.print(F("Send attempt "));
    Serial.println(DTSendAttempts - attempts + 1);
#endif

    attempts--;
    TXNetworkID = LoRa.getTXNetworkID(TXPacketL);     //get the networkID appended to packet
    TXArrayCRC = LoRa.getTXPayloadCRC(TXPacketL);     //get the payload CRC appended to packet

#ifdef DEBUG
    Serial.print(F("TXNetworkID,0x"));
    Serial.print(TXNetworkID, HEX);                   //get the NetworkID of the packet just sent, its placed at the packet end
    Serial.print(F(",TXarrayCRC,0x"));
    Serial.println(TXArrayCRC, HEX);                  //get the CRC of the data array just sent, its placed at the packet end
    Serial.println();
#endif

    if (TXPacketL == 0)                               //if there has been a send and ack error, TXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
    }

    Serial.print(F("Wait ACK"));
    ValidACK = LoRa.waitACKDT(DTheader, DTFileOpenHeaderL, ACKopentimeoutmS);
    RXPacketType = DTheader[0];

    if ((ValidACK > 0) && (RXPacketType == DTFileOpenACK))
    {
      Serial.println(F(" - Valid ACK"));
#ifdef DEBUG
      printPacketHex();
#endif
    }
    else
    {
      NoAckCount++;
      Serial.print(F(" - No ACK received "));
      Serial.println(NoAckCount);
#ifdef DEBUG
      printACKdetail();
      Serial.print(F("  ACKPacket "));
      printPacketHex();
#endif
      if (NoAckCount > NoAckCountLimit)
      {
        Serial.println(F("ERROR NoACK limit reached"));
        return false;
      }
      Serial.println();
    }
  }
  while ((ValidACK == 0) && (attempts != 0));

  if (attempts == 0)
  {
    return false;
  }

  return true;
}


bool endFileTransfer(char *buff, uint8_t filenamesize)
{
  // End file transfer, close local file first then remote file

  uint8_t ValidACK;

  DTSD_closeFile();
  build_DTFileCloseHeader(DTheader, DTFileCloseHeaderL, filenamesize, DTSourceFileLength, DTSourceFileCRC, DTSegmentSize);

  do
  {
    printSeconds();
    Serial.println(F(" Send close remote file"));

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    TXPacketL = LoRa.transmitDT(DTheader, DTFileCloseHeaderL, (uint8_t *) buff, filenamesize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }

    TXNetworkID = LoRa.getTXNetworkID(TXPacketL);
    TXArrayCRC = LoRa.getTXPayloadCRC(TXPacketL);

#ifdef DEBUG
    Serial.print(F("TXNetworkID,0x"));
    Serial.print(TXNetworkID, HEX);               //get the NetworkID of the packet just sent, its placed at the packet end
    Serial.print(F(",TXarrayCRC,0x"));
    Serial.println(TXArrayCRC, HEX);              //get the CRC of the data array just sent, its placed at the packet end
    Serial.println();
#endif

    if (TXPacketL == 0)                           //if there has been a send and ack error, RXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
    }

    ValidACK = LoRa.waitACKDT(DTheader, DTFileCloseHeaderL, ACKclosetimeoutmS);
    RXPacketType = DTheader[0];

    if ((ValidACK > 0) && (RXPacketType == DTFileCloseACK))
    {
#ifdef DEBUG
      printPacketHex();
#endif
    }
    else
    {
      NoAckCount++;
      Serial.println(F("No ACK received "));
      Serial.println(NoAckCount);
      if (NoAckCount > NoAckCountLimit)
      {
        Serial.println(F("ERROR NoACK limit reached"));
        return false;
      }
#ifdef DEBUG
      Serial.println();
      Serial.print(F("  ACKPacket "));
      printPacketHex();
      Serial.println();
#endif
    }
  }
  while (ValidACK == 0);

  return true;
}


void build_DTFileOpenHeader(uint8_t *header, uint8_t headersize, uint8_t datalength, uint32_t filelength, uint16_t filecrc, uint8_t segsize)
{
  // This builds the header buffer for the filename to send

  beginarrayRW(header, 0);             //start writing to array at location 0
  arrayWriteUint8(DTFileOpen);         //byte 0, write the packet type
  arrayWriteUint8(0);                  //byte 1, initial DTflags byte, not used here
  arrayWriteUint8(headersize);         //byte 2, write length of header
  arrayWriteUint8(datalength);         //byte 3, write length of dataarray
  arrayWriteUint32(filelength);        //byte 4,5,6,7, write the file length
  arrayWriteUint16(filecrc);           //byte 8, 9, write file CRC
  arrayWriteUint8(segsize);            //byte 10, segment size
  arrayWriteUint8(0);                  //byte 11, unused byte
  endarrayRW();
}


void build_DTSegmentHeader(uint8_t *header, uint8_t headersize, uint8_t datalen, uint16_t segnum)
{
  // This builds the header buffer for a segment transmit

  beginarrayRW(header, 0);             //start writing to array at location 0
  arrayWriteUint8(DTSegmentWrite);     //write the packet type
  arrayWriteUint8(0);                  //initial DTflags byte, not used here
  arrayWriteUint8(headersize);         //write length of header
  arrayWriteUint8(datalen);            //write length of data array
  arrayWriteUint16(segnum);            //write the DTsegment number
  endarrayRW();
}


void build_DTFileCloseHeader(uint8_t *header, uint8_t headersize, uint8_t datalength, uint32_t filelength, uint16_t filecrc, uint8_t segsize)
{
  // This builds the header buffer for the filename passed

  beginarrayRW(header, 0);             //start writing to array at location 0
  arrayWriteUint8(DTFileClose);        //byte 0, write the packet type
  arrayWriteUint8(0);                  //byte 1, initial DTflags byte, not used here
  arrayWriteUint8(headersize);         //byte 2, write length of header
  arrayWriteUint8(datalength);         //byte 3, write length of dataarray
  arrayWriteUint32(filelength);        //byte 4,5,6,7, write the file length
  arrayWriteUint16(filecrc);           //byte 8, 9, write file CRC
  arrayWriteUint8(segsize);            //byte 10, segment size
  arrayWriteUint8(0);                  //byte 11, unused byte
  endarrayRW();
}


void printLocalFileDetails()
{
  Serial.print(F("Source file length "));
  Serial.print(DTSourceFileLength);
  Serial.println(F(" bytes"));
#ifdef ENABLEFILECRC
  Serial.print(F("Source file CRC is 0x"));
  Serial.println(DTSourceFileCRC, HEX);
#endif
  Serial.print(F("Segment Size "));
  Serial.println(DTSegmentSize);
  Serial.print(F("Number segments "));
  Serial.println(DTNumberSegments);
  Serial.print(F("Last segment size "));
  Serial.println(DTLastSegmentSize);
}


bool sendSegments()
{
  // Start the file transfer at segment 0
  DTSegment = 0;
  DTSentSegments = 0;

  dataFile.seek(0);                       //ensure at first position in file

  while (DTSegment < (DTNumberSegments - 1))
  {
#ifdef DEBUG
    printSeconds();
#endif

    if (sendFileSegment(DTSegment, DTSegmentSize))
    {
      DTSentSegments++;
    }
    else
    {
      return false;
    }
    delay(packetdelaymS);
  };

  //printSeconds();
  Serial.println(F("Last segment"));

  if (!sendFileSegment(DTSegment, DTLastSegmentSize))
  {
    return false;
  }

  return true;
}


void printheader(uint8_t *hdr, uint8_t hdrsize)
{
  Serial.print(F("HeaderBytes,"));
  Serial.print(hdrsize);
  Serial.print(F(" "));
  printarrayHEX(hdr, hdrsize);
}


void printSeconds()
{
  float secs;
  secs = ( (float) millis() / 1000);
  Serial.print(secs, 2);
  Serial.print(F(" "));
}


void printdata(uint8_t *dataarray, uint8_t arraysize)
{
  Serial.print(F("DataBytes,"));
  Serial.print(arraysize);
  Serial.print(F("  "));
  printarrayHEX((uint8_t *) dataarray, 16);             //There is a lot of data to print so only print first 16 bytes
}


void printAckDetails()
{
  PacketRSSI = LoRa.readPacketRSSI();
  PacketSNR = LoRa.readPacketSNR();
  Serial.print(F("AckCount,"));
  Serial.print(AckCount);
  Serial.print(F(",NoAckCount,"));
  Serial.print(NoAckCount);
  Serial.print(F(",AckRSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,AckSNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB"));
  Serial.println();
}


void printAckBrief()
{
  PacketRSSI = LoRa.readPacketRSSI();
  Serial.print(F(",AckRSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm"));
}


void printACKdetail()
{
  Serial.print(F("ACKDetail"));
  Serial.print(F(",RXNetworkID,0x"));
  Serial.print(LoRa.getRXNetworkID(RXPacketL), HEX);
  Serial.print(F(",RXPayloadCRC,0x"));
  Serial.print(LoRa.getRXPayloadCRC(RXPacketL), HEX);
  Serial.print(F(",RXPacketL,"));
  Serial.print(RXPacketL);
  Serial.print(F(" "));
  LoRa.printReliableStatus();
  Serial.println();
}


void printPacketHex()
{
  RXPacketL = LoRa.readRXPacketL();
  Serial.print(RXPacketL);
  Serial.print(F(" bytes > "));
  if (RXPacketL > 0)
  {
    LoRa.printSXBufferHEX(0, RXPacketL - 1);
  }
}


void printPacketDetails()
{
  PacketRSSI = LoRa.readPacketRSSI();
  PacketSNR = LoRa.readPacketSNR();
  Serial.print(F(" RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm"));

#ifdef DEBUG
  Serial.print(F(",SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dBm,RXOKCount,"));
  Serial.print(DTReceivedSegments);
  Serial.print(F(",RXErrs,"));
  Serial.print(RXErrors);
  Serial.print(F(" RX"));
  printheader(DTheader, RXHeaderL);
#endif
}


void printPacketRSSI()
{
  PacketRSSI = LoRa.readPacketRSSI();
  Serial.print(F(" RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm"));
}


void readHeaderDT()
{
  // The first 6 bytes of the header contain the important stuff, so load it up
  // so we can decide what to do next.
  beginarrayRW(DTheader, 0);                      //start buffer read at location 0
  RXPacketType = arrayReadUint8();                //load the packet type
  RXFlags = arrayReadUint8();                     //initial DTflags byte, not used here
  RXHeaderL = arrayReadUint8();                   //load the header length
  RXDataarrayL = arrayReadUint8();                //load the datalength
  DTSegment = arrayReadUint16();                  //load the segment number
}


void printSourceFileDetails()
{
  Serial.print(DTfilenamebuff);
  Serial.print(F(" Source file length is "));
  Serial.print(DTSourceFileLength);
  Serial.println(F(" bytes"));
#ifdef ENABLEFILECRC
  Serial.print(F(" Source file CRC is 0x"));
  Serial.println(DTSourceFileCRC, HEX);
#endif
}


void printDestinationFileDetails()
{
  Serial.print(F("Destination file length "));
  Serial.print(DTDestinationFileLength);
  Serial.println(F(" bytes"));
  if (DTDestinationFileLength != DTSourceFileLength)
  {
    Serial.println(F("ERROR - file lengths do not match"));
  }
  else
  {
    Serial.println(F("File lengths match"));
  }

#ifdef ENABLEFILECRC
  Serial.print(F("Destination file CRC is 0x"));
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
}


bool processFileClose()
{
  // Code for closing local SD file and sending request to remote
  uint32_t transferms;

  Serial.print((char*) DTfilenamebuff);
  Serial.println(F(" File close request"));

  if (DTFileOpened)                                     //check if file has been opened, close it if it is
  {
    if (SD.exists(DTfilenamebuff))                      //check if file exists
    {
      DTSD_closeFile();
      transferms = millis() - DTStartmS;
      Serial.print(F("Transfer time "));
      Serial.print(transferms);
      Serial.print(F("mS"));
      Serial.println();
      Serial.println(F("File closed"));

      setCursor(0, 0);
      disp.print(F("Transfer finished"));
      setCursor(0, 3);
      disp.print(F("          "));
      setCursor(0, 3);
      disp.print(transferms);
      disp.print(F(" mS"));

      DTFileOpened = false;
      DTDestinationFileLength = DTSD_openFileRead(DTfilenamebuff);
#ifdef ENABLEFILECRC
      DTDestinationFileCRC = DTSD_fileCRCCCITT(DTDestinationFileLength);
#endif
      beginarrayRW(DTheader, 4);                       //start writing to array at location 12
      arrayWriteUint32(DTDestinationFileLength);       //write file length of file just written just written to ACK header
      arrayWriteUint16(DTDestinationFileCRC);          //write CRC of file just written to ACK header

      printDestinationFileDetails();
    }
  }
  else
  {
    Serial.println(F("File already closed"));
    delay(DuplicatedelaymS);

  }

  delay(ACKdelaymS);
#ifdef DEBUG
  Serial.println(F("Sending ACK"));
#endif
  DTheader[0] = DTFileCloseACK;

  if (DTLED >= 0)
  {
    digitalWrite(DTLED, HIGH);
  }
  LoRa.sendACKDT(DTheader, DTFileCloseHeaderL, TXpower);
  if (DTLED >= 0)
  {
    digitalWrite(DTLED, LOW);
  }

  Serial.println();
#ifdef DEBUG
  DTSD_printDirectory();
  Serial.println();
  Serial.println();
#endif
  return true;
}


bool processFileOpen(uint8_t *buff, uint8_t filenamesize)
{
  // Code for opening local SD file and sending request to remote

  beginarrayRW(DTheader, 4);                      //start buffer read at location 4
  DTSourceFileLength = arrayReadUint32();         //load the file length of the remote file being sent
  DTSourceFileCRC = arrayReadUint16();            //load the CRC of the source file being sent
  memset(DTfilenamebuff, 0, DTfilenamesize);      //clear DTfilenamebuff to all 0s
  memcpy(DTfilenamebuff, buff, filenamesize);     //copy received DTdata into DTfilenamebuff
  Serial.print((char*) DTfilenamebuff);
  Serial.print(F(" SD File Open request"));
  Serial.println();
  printSourceFileDetails();

  if (DTSD_openNewFileWrite(DTfilenamebuff))      //open file for write at beginning, delete if it exists
  {
    Serial.print((char*) DTfilenamebuff);
    Serial.println(F(" DT File Opened OK"));
    Serial.println(F("Waiting transfer"));
    DTSegmentNext = 0;                            //since file is opened the next sequence should be the first
    DTFileOpened = true;
    DTStartmS = millis();
  }
  else
  {
    Serial.print((char*) DTfilenamebuff);
    Serial.println(F(" File Open fail"));
    DTFileOpened = false;
    return false;
  }

  setCursor(0, 0);
  disp.print(F("Transfer started "));
  setCursor(0, 1);
  disp.print(F("             "));                    //clear previous filename
  setCursor(0, 1);
  disp.print(DTfilenamebuff);
  setCursor(0, 2);
  disp.print(F("            "));                     //clear previous file size
  setCursor(0, 2);
  disp.print(DTSourceFileLength);
  disp.print(F(" bytes"));
  setCursor(0, 3);
  disp.print(F("          "));                       //clear transfer time
  setCursor(0, 5);
  disp.print(F("    "));

  DTStartmS = millis();
  delay(ACKdelaymS);
#ifdef DEBUG
  Serial.println(F("Sending ACK"));
#endif
  DTheader[0] = DTFileOpenACK;                    //set the ACK packet type

  if (DTLED >= 0)
  {
    digitalWrite(DTLED, HIGH);
  }
  LoRa.sendACKDT(DTheader, DTFileOpenHeaderL, TXpower);
  if (DTLED >= 0)
  {
    digitalWrite(DTLED, LOW);
  }
  DTSegmentNext = 0;                               //after file open, segment 0 is next

  return true;
}



bool processSegmentWrite()
{
  // Code for dealing with segment writes
  // checks that the sequence of segment writes is correct

  if (!DTFileOpened)
  {
    //something is wrong, have received a request to write a segment but there is no file opened
    //need to reject the segment write with a restart NACK
    Serial.println();
    Serial.println(F("***************************************************"));
    Serial.println(F("Error - Segment write with no file open - send NACK"));
    Serial.println(F("***************************************************"));
    Serial.println();
    DTheader[0] = DTStartNACK;
    delay(ACKdelaymS);
    delay(DuplicatedelaymS);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTStartHeaderL, TXpower);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return false;
  }

  if (DTSegment == DTSegmentNext)
  {
    DTSD_writeSegmentFile(DTdata, RXDataarrayL);
    //DTSD_fileFlush();

#ifdef PRINTSEGMENTNUM
    //Serial.print(F("Segment,"));
    Serial.println(DTSegment);
    setCursor(0, 7);
    disp.print(F("    "));
    setCursor(0, 7);
    disp.print(DTSegment);
#endif

#ifdef DEBUG
    Serial.print(F("  Bytes,"));
    Serial.print(RXDataarrayL);
    //printPacketDetails();
    printPacketRSSI();
    Serial.println(F(" SendACK"));
#endif

    DTheader[0] = DTSegmentWriteACK;
    delay(ACKdelaymS);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    DTReceivedSegments++;
    DTSegmentLast = DTSegment;                  //so we can tell if sequece has been received twice
    DTSegmentNext = DTSegment + 1;
    return true;
  }

  if (DTSegment == DTSegmentLast)
  {
    Serial.print(F("ERROR segment "));
    Serial.print(DTSegment);
    Serial.println(F(" already received "));
    delay(DuplicatedelaymS);
#ifdef DEBUG
    printPacketDetails();
    printPacketRSSI();
#endif
    DTheader[0] = DTSegmentWriteACK;
    delay(ACKdelaymS);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return true;
  }

  if (DTSegment != DTSegmentNext )
  {
    Serial.print(F(" ERROR Received Segment "));
    Serial.print(DTSegment);
    Serial.print(F(" expected "));
    Serial.print(DTSegmentNext);
    Serial.print(F(" "));

#ifdef DEBUG
    printPacketDetails();
    printPacketRSSI();
#endif

    DTheader[0] = DTSegmentWriteNACK;
    DTheader[4] = lowByte(DTSegmentNext);
    DTheader[5] = highByte(DTSegmentNext);
    Serial.print(F(" Send NACK for segment "));
    Serial.print(DTSegmentNext);
    delay(ACKdelaymS);
    delay(DuplicatedelaymS);                   //add an extra delay here to stop repeated segment sends
    Serial.println();
    Serial.println();
    Serial.println(F("*****************************************"));
    Serial.print(F("Transmit restart request for segment "));
    Serial.println(DTSegmentNext);
    printheader(DTheader, RXHeaderL);
    Serial.println();
    Serial.println(F("*****************************************"));
    Serial.println();
    Serial.flush();
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return false;
  }

  return true;
}


bool processPacket(uint8_t packettype)
{
  // Decide what to do with an incoming packet

  if (packettype == DTSegmentWrite)
  {
    processSegmentWrite();
    return true;
  }

  if (packettype == DTFileOpen)
  {
    processFileOpen(DTdata, RXDataarrayL);
    return true;
  }

  if (packettype == DTFileClose)
  {
    processFileClose();
    return true;
  }

  return true;
}


bool receiveaPacketDT()
{
  // Receive Data transfer packets

  RXPacketType = 0;
  RXPacketL = LoRa.receiveDT(DTheader, HeaderSizeMax, (uint8_t *) DTdata, DataSizeMax, NetworkID, RXtimeoutmS, WAIT_RX);

  if (DTLED >= 0)
  {
    digitalWrite(DTLED, HIGH);
  }

#ifdef DEBUG
  printSeconds();
#endif

  if (RXPacketL > 0)
  {
    //if the LT.receiveDT() returns a value > 0 for RXPacketL then packet was received OK
    //then only action payload if destinationNode = thisNode
    readHeaderDT();                      //get the basic header details into global variables RXPacketType etc
    processPacket(RXPacketType);         //process and act on the packet
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return true;
  }
  else
  {
    //if the LoRa.receiveDT() function detects an error RXOK is 0

    RXErrors++;
#ifdef DEBUG
    Serial.print(F("PacketError"));
    printPacketDetails();
    LoRa.printReliableStatus();
    LoRa.printIrqStatus();
    Serial.println();
#endif

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return false;
  }
}


void setDTLED(int8_t pinnumber)
{
  if (pinnumber >= 0)
  {
    DTLED = pinnumber;
    pinMode(pinnumber, OUTPUT);
  }
}


bool DTSD_dumpFileASCII(char *buff)
{

  File dataFile = SD.open(buff);                  //open the test file note that only one file can be open at a time,

  if (dataFile)                                   //if the file is available, read from it
  {
    while (dataFile.available())
    {
      Serial.write(dataFile.read());
    }
    dataFile.close();
    return true;
  }
  else
  {
    return false;
  }
}


bool DTSD_dumpFileHEX(char *buff)
{
  //Note, this function will return true if the SD card is removed.
  uint16_t Loopv1, Loopv2;
  uint8_t fileData;
  uint32_t filesize;

  if (!SD.exists(buff))
  {
    return false;
  }

  dataFile = SD.open(buff);
  filesize = dataFile.size();
  filesize--;                                      //file data locations are from 0 to (filesize -1);
  Serial.print(F("Lcn    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  if (dataFile)                                    //if the file is available, read from it
  {
    while (dataFile.available())
    {
      for (Loopv1 = 0; Loopv1 <= filesize;)
      {
        Serial.print(F("0x"));
        if (Loopv1 < 0x10)
        {
          Serial.print(F("0"));
        }
        Serial.print((Loopv1), HEX);
        Serial.print(F("  "));
        for (Loopv2 = 0; Loopv2 <= 15; Loopv2++)
        {
          fileData = dataFile.read();
          if (fileData < 0x10)
          {
            Serial.print(F("0"));
          }
          Serial.print(fileData, HEX);
          Serial.print(F(" "));
          Loopv1++;
        }
        Serial.println();
      }
    }
    dataFile.close();
    return true;
  }
  else
  {
    Serial.println(F("File not available"));
    return false;
  }
}


bool DTSD_initSD(uint8_t CSpin)
{
  if (SD.begin(CSpin))
  {
    return true;
  }
  else
  {
    return false;
  }
}


uint32_t DTSD_getFileSize(char *buff)
{
  uint32_t filesize;

  if (!SD.exists(buff))
  {
    return 0;
  }

  dataFile = SD.open(buff);
  filesize = dataFile.size();
  dataFile.close();
  return filesize;
}


#ifdef SDFATLIB
void DTSD_printDirectory()
{
  dataFile = SD.open("/");
  Serial.println(F("Card directory"));
  SD.ls("/", LS_R);
}
#endif


#ifdef SDLIB
void DTSD_printDirectory()
{
  dataFile = SD.open("/");

  Serial.println(F("Card directory"));

  while (true)
  {
    File entry =  dataFile.openNextFile();
    if (! entry)
    {
      //no more files
      break;
    }
    Serial.print(entry.name());
    if (entry.isDirectory())
    {
      Serial.println(F("/"));
      DTSD_printDirectory();
    }
    else
    {
      //files have sizes, directories do not
      Serial.print(F("\t\t"));
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
  Serial.println();
}
#endif


bool DTSD_dumpSegmentHEX(uint8_t segmentsize)
{
  uint16_t Loopv1, Loopv2;
  uint8_t fileData;

  Serial.print(F("Print segment of "));
  Serial.print(segmentsize);
  Serial.println(F(" bytes"));
  Serial.print(F("Lcn    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  if (dataFile)                                    //if the file is available, read from it
  {
    for (Loopv1 = 0; Loopv1 < segmentsize;)
    {
      Serial.print(F("0x"));
      if (Loopv1 < 0x10)
      {
        Serial.print(F("0"));
      }
      Serial.print((Loopv1), HEX);
      Serial.print(F("  "));
      for (Loopv2 = 0; Loopv2 <= 15; Loopv2++)
      {
        //stop printing if all of segment has been printed
        if (Loopv1 < segmentsize)
        {
          fileData = dataFile.read();
          if (fileData < 0x10)
          {
            Serial.print(F("0"));
          }
          Serial.print(fileData, HEX);
          Serial.print(F(" "));
          Loopv1++;
        }
      }
      Serial.println();
    }
    return true;
  }
  else
  {
    return false;
  }
}


uint32_t DTSD_openFileRead(char *buff)
{
  uint32_t filesize;

  dataFile = SD.open(buff);
  filesize = dataFile.size();
  dataFile.seek(0);
  return filesize;
}


uint16_t DTSD_getNumberSegments(uint32_t filesize, uint8_t segmentsize)
{
  uint16_t segments;
  segments = filesize / segmentsize;

  if ((filesize % segmentsize) > 0)
  {
    segments++;
  }
  return segments;
}


uint8_t DTSD_getLastSegmentSize(uint32_t filesize, uint8_t segmentsize)
{
  uint8_t lastsize;

  lastsize = filesize % segmentsize;
  if (lastsize == 0)
  {
    lastsize = segmentsize;
  }
  return lastsize;
}


bool DTSD_openNewFileWrite(char *buff)
{
  if (SD.exists(buff))
  {
    Serial.print(buff);
    Serial.println(F(" File exists - deleting"));
    SD.remove(buff);
  }

  if (dataFile = SD.open(buff, FILE_WRITE))
  {
    Serial.print(buff);
    Serial.println(F(" SD File opened"));
    return true;
  }
  else
  {
    Serial.print(buff);
    Serial.println(F(" ERROR opening file"));
    return false;
  }
}


bool DTSD_openFileWrite(char *buff, uint32_t position)
{
  dataFile = SD.open(buff, FILE_WRITE);   //seems to operate as append
  dataFile.seek(position);                //seek to first position in file

  if (dataFile)
  {
    return true;
  }
  else
  {
    return false;
  }
}


uint8_t DTSD_readFileSegment(uint8_t *buff, uint8_t segmentsize)
{
  uint8_t index = 0;
  uint8_t fileData;

  while (index < segmentsize)
  {
    fileData = (uint8_t) dataFile.read();
    buff[index] = fileData;
    index++;
  };

  if (index == segmentsize)
  {
    return segmentsize;            //if all written return segment size
  }
  else
  {
    return index - 1;               //if not all written return number bytes read
  }
}


uint8_t DTSD_writeSegmentFile(uint8_t *buff, uint8_t segmentsize)
{
  uint8_t index, byteswritten = 0;

  for (index = 0; index < segmentsize; index++)
  {
    dataFile.write(buff[index]);
    byteswritten++;
  }
  return byteswritten;
}


void DTSD_seekFileLocation(uint32_t position)
{
  dataFile.seek(position);                       //seek to position in file
  return;
}


uint16_t DTSD_createFile(char *buff)
{
  //creats a new filename use this definition as the base;
  //char filename[] = "/SD0000.txt";     //filename used as base for creating logfile, 0000 replaced with numbers
  //the 0000 in the filename is replaced with the next number avaialable

  uint16_t index;

  for (index = 1; index <= 9999; index++)
  {
    buff[3] = index / 1000 + '0';
    buff[4] = ((index % 1000) / 100) + '0';
    buff[5] = ((index % 100) / 10) + '0';
    buff[6] = index % 10 + '0' ;
    if (! SD.exists(buff))
    {
      // only open a new file if it doesn't exist
      dataFile = SD.open(buff, FILE_WRITE);
      break;
    }
  }

  if (!dataFile)
  {
    return 0;
  }

  return index;                                      //return number of logfile created
}


uint16_t DTSD_fileCRCCCITT(uint32_t fsize)
{
  uint32_t index;
  uint16_t CRCcalc;
  uint8_t j, filedata;

  CRCcalc = 0xFFFF;                                  //start value for CRC16

  for (index = 0; index < fsize; index++)
  {
    filedata = dataFile.read();
    CRCcalc ^= (((uint16_t) filedata ) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRCcalc & 0x8000)
        CRCcalc = (CRCcalc << 1) ^ 0x1021;
      else
        CRCcalc <<= 1;
    }
  }

#ifdef DEBUGSDLIB
  Serial.print(F(" {SDlibrary} "));
  Serial.print(index);
  Serial.print(F(" Bytes checked - CRC "));
  Serial.println(CRCcalc, HEX);
#endif

  return CRCcalc;
}


void DTSD_fileFlush()
{
  dataFile.flush();
}


void DTSD_closeFile()
{
  dataFile.close();                                   //close local file
}


void setCursor(uint8_t lcol, uint8_t lrow)
{
  disp.setCursor((lcol * 6 * textscale), (lrow * 9 * textscale));
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
  pinMode(LED1, OUTPUT);                       //setup pin as output for indicator LED
  led_Flash(2, 125);                           //two quick LED flashes to indicate program start
  setDTLED(LED1);                              //setup LED pin for data transfer indicator

  digitalWrite(DISPCS, HIGH);
  pinMode(DISPCS, OUTPUT);                     //disable ILI9341 for now
  digitalWrite(NSS, HIGH);
  pinMode(NSS, OUTPUT);                        //disable LoRa device for now

  if (TOUCHCS >= 0)
  {
    digitalWrite(TOUCHCS, HIGH);               //disable touch IC on ILI9341
    pinMode(TOUCHCS, OUTPUT);
  }

  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print(F("Version "));
  Serial.println(F(Program_Version));

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

  Serial.println();
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
    while (1) led_Flash(100, 50);
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

  disp.begin();
  disp.fillScreen(ILI9341_BLACK);
  disp.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  disp.setRotation(rotation);
  disp.setTextSize(textscale);
  setCursor(0, 0);
  disp.print(F("Waiting Transfer"));

  DTSegmentNext = 0;
  DTFileOpened = false;

  Serial.println(F("SDfile transfer receiver ready"));
  Serial.println();
}
