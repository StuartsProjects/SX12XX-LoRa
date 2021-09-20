/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 20/09/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a simulation test program for the use of a data transfer (DT) packet to send
  a file from the SD card on one Arduino to the SD card on another Arduino, Arduino DUEs were used for the
  test.

  DT packets can be used for transfering large amounts of data in a sequence of packets or segments,
  in a reliable and resiliant way. The remote file open request, the segements sent and the remote file close
  will be transmitted until a valid acknowledge comes from the receiver. Use with the receiver program,
  234_SDfile_Transfer_ReceiverIRQ.ino or 236_SDfile_Transfer_ReceiverIRQ.ino.

  Each DT packet contains a variable length header array and a variable length data array as the payload.
  On transmission the NetworkID and CRC of the payload are appended to the end of the packet by the library
  routines. The use of a NetworkID and CRC ensures that the receiver can validate the packet to a high degree
  of certainty. The receiver will not accept packets that dont have the appropriate NetworkID or payload CRC
  at the end of the packet.

  The transmitter sends a sequence of segments in order and the receiver keeps track of the sequence. If
  the sequence fails for some reason, the receiver will return a NACK packet to the transmitter requesting
  the segment sequence it was expecting.

  This is a version of example 235_SDfile_Transfer_Transmitter.ino that does not require the use of the DIO0
  pin to check for transmit or receive done. In addition no NRESET pin is needed either, so its a program for
  use with a minimum pin count Arduino. Leave the DIO0 and NRESET pins on the LoRa device not connected.

  Details of the packet identifiers, header and data lengths and formats used are in the file
  Data_transfer_packet_definitions.md in the \SX127X_examples\DataTransfer\ folder.

  Notes:

  When DTSement++ at end of if (sendFileSegment(DTSegment, DTSegmentSize)) why is does file close eventually sent ?
  When file transfer interrupted, reset of TX, why does file close report as if file transfer completed OK, CRC etc ?


  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>

#include <SX127XLT.h>
#include <ProgramLT_Definitions.h>
#include "Settings.h"                        //LoRa settings etc.
#include <arrayRW.h>
#include "Variables.h"

SX127XLT LoRa;                               //create an SX127XLT library instance called LoRa

#include <SdFat.h>
SdFat SD;
File dataFile;                               //name the file instance needed for SD library routines

uint8_t DTheader[16];                        //header array
uint8_t DTdata[245];                         //data/segment array

//choice of files to send
//char DTfilenamebuff[] = "/$50SAT_Large.JPG";   //file length 63091 bytes, file CRC 0x59CE
//char DTfilenamebuff[] = "/$50SAT_Small.JPG";   //file length 6880 bytes, file CRC 0x0281
char DTfilenamebuff[] = "/$50SAT_Tiny.JPG";    //file length 1068 bytes, file CRC 0x6A02

//#define DEBUG
//#define DEBUGSDLIB                           //enable define to see more detail for SD operation


void loop()
{
  do
  {
    if (startFileTransfer(DTfilenamebuff, sizeof(DTfilenamebuff), DTSendAttempts))   //opens the local file to send and sets up transfer parameters
    {
      Serial.print(DTfilenamebuff);
      Serial.println(" opened OK on remote");
      DTLocalFileCRC = DTSD_fileCRCCCITT(DTLocalFileLength);
      DTNumberSegments = DTSD_getNumberSegments(DTLocalFileLength, DTSegmentSize);
      DTLastSegmentSize = DTSD_getLastSegmentSize(DTLocalFileLength, DTSegmentSize);
      printLocalFileDetails();
      Serial.println();
      NoAckCount = 0;
    }
    else
    {
      Serial.print(DTfilenamebuff);
      Serial.println("  Error opening remote file - restart transfer");
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

    if (endFileTransfer(DTfilenamebuff, sizeof(DTfilenamebuff)))        //send command to close remote file
    {
      //the header returned from file close contains a 16bit CRC of the file saved on the remotes SD
      Serial.print(DTfilenamebuff);
      Serial.println(" closed OK on remote");
      beginarrayRW(DTheader, 4);
      DTRemoteFileLength = arrayReadUint32();
      DTRemoteFileCRC = arrayReadUint16();
      Serial.print(F("Acknowledged remote file length "));
      Serial.println(DTRemoteFileLength);
      Serial.print(F("Acknowledged remote file CRC 0x"));
      Serial.println(DTRemoteFileCRC, HEX);
      DTFileTransferComplete = true;
    }
    else
    {
      DTFileTransferComplete = false;
      Serial.println(F("ERROR send close remote file failed - program halted"));
    }
  }
  while (!DTFileTransferComplete);

  Serial.print("NoAckCount ");
  Serial.println( NoAckCount);
  Serial.print("Total file transmit time ");
  Serial.print(millis() -  DTStartmS);
  Serial.println("mS");
  Serial.println();
  Serial.println("Transfer finished - program halted");
  while (1);
}


bool sendFileSegment(uint16_t segnum, uint8_t segmentsize)
{
  //****************************************************************
  //Send file segment
  //****************************************************************

  uint8_t ValidACK;
  uint8_t index;

  //read in segmentsize bytes from SD file
  for (index = 0; index < segmentsize; index++)
  {
    DTdata[index] = dataFile.read();
  }

  build_DTSegmentHeader(DTheader, DTSegmentWriteHeaderL, segmentsize, segnum);

  Serial.print(F("Send Segment,"));
  Serial.print(segnum);
#ifdef DEBUG
  Serial.print(F(" "));
  printheader(DTheader, DTSegmentWriteHeaderL);
  Serial.print(F(" "));
  printdata(DTdata, segmentsize);                         //print segment size of data array only
#endif
  Serial.println();

  do
  {
    digitalWrite(LED1, HIGH);
    TXPacketL = LoRa.transmitDTIRQ(DTheader, DTSegmentWriteHeaderL, (uint8_t *) DTdata, segmentsize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    digitalWrite(LED1, LOW);

    if (TXPacketL == 0)                                        //if there has been a send TXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
    }

    ValidACK = LoRa.waitACKDTIRQ(DTheader, DTSegmentWriteHeaderL, ACKtimeoutDTmS);

    RXPacketType = DTheader[0];

    if (ValidACK > 0)
    {
      if (RXPacketType == DTSegmentWriteNACK)
      {
        DTSegment = DTheader[4] +  (DTheader[5] << 8);                 //load what the segment should be
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
        Serial.println(F("Received DTStartNACK "));
        return false;
      }

      if (RXPacketType == DTSegmentWriteACK)
      {
        readACKHeader();
        AckCount++;
        printPacketDetails();
        DTSegment++;                  //increase value for next segment
        return true;
      }
    }
    else
    {
      NoAckCount++;
      Serial.print(F("Error No Ack "));
      Serial.print(F("NoAckCount,"));
      Serial.print(NoAckCount);
      LoRa.printReliableStatus();
      Serial.println();
    }
  } while (ValidACK == 0);

  return true;
}


void readACKHeader()
{
  //the first 6 bytes of the segment write header contain the important stuff, so load it up
  //so we can decide what to do next.
  beginarrayRW(DTheader, 0);                      //start buffer read at location 0
  RXPacketType = arrayReadUint8();                //load the packet type
  RXFlags = arrayReadUint8();                     //initial DTflags byte, not used here
  RXHeaderL = arrayReadUint8();                   //load the header length
  RXDataarrayL = arrayReadUint8();                //load the datalength
  DTSegment = arrayReadUint16();                  //load the segment number
}


void printdata(uint8_t *dataarray, uint8_t arraysize)
{
  Serial.print(F("DataBytes,"));
  Serial.print(arraysize);
  Serial.print(F("  "));
  printarrayHEX((uint8_t *) dataarray, arraysize);
}


void printheader(uint8_t *hdr, uint8_t hdrsize)
{
  Serial.print(F("HeaderBytes,"));
  Serial.print(hdrsize);
  Serial.print(F(" "));
  printarrayHEX(hdr, hdrsize);
}


void build_DTSegmentHeader(uint8_t *header, uint8_t headersize, uint8_t datalen, uint16_t segnum)
{
  //this builds the header buffer for the a segment transmit
  beginarrayRW(header, 0);                //start writing to array at location 0
  arrayWriteUint8(DTSegmentWrite);        //write the packet type
  arrayWriteUint8(0);                     //initial DTflags byte, not used here
  arrayWriteUint8(headersize);            //write length of header
  arrayWriteUint8(datalen);               //write length of data array
  arrayWriteUint16(segnum);               //write the DTsegment number
  endarrayRW();
}


void printPacketDetails()
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


bool startFileTransfer(char *buff, uint8_t filenamesize, uint8_t attempts)
{
  //****************************************************************
  //Start; open local file first then remote file.
  //****************************************************************

  uint8_t ValidACK;

  Serial.print(F("Start file transfer "));
  Serial.println(buff);

  DTLocalFileLength = DTSD_openFileRead(buff);                             //get the file length

  if (DTLocalFileLength == 0)
  {
    Serial.print(F("Error - opening local file "));
    Serial.println(buff);
    return false;
  }

  DTLocalFileCRC = DTSD_fileCRCCCITT(DTLocalFileLength);                        //get file CRC from position 0 to end

  build_DTFileOpenHeader(DTheader, DTFileOpenHeaderL, filenamesize, DTLocalFileLength, DTLocalFileCRC, DTSegmentSize);
  LocalPayloadCRC = LoRa.CRCCCITT((uint8_t *) buff, filenamesize, 0xFFFF);

  do
  {
    Serial.println(F("Transmit open remote file request"));

    digitalWrite(LED1, HIGH);
    TXPacketL = LoRa.transmitDTIRQ(DTheader, DTFileOpenHeaderL, (uint8_t *) buff, filenamesize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    digitalWrite(LED1, LOW);

#ifdef DEBUG
    Serial.print(F("Send attempt "));
    Serial.println(DTSendAttempts - attempts + 1);
#endif

    attempts--;



    TXNetworkID = LoRa.getTXNetworkID(TXPacketL);          //get the networkID appended to packet
    TXArrayCRC = LoRa.getTXPayloadCRC(TXPacketL);          //get the payload CRC appended to packet

#ifdef DEBUG
    Serial.print(F("TXNetworkID,0x"));
    Serial.print(TXNetworkID, HEX);               //get the NetworkID of the packet just sent, its placed at the packet end
    Serial.print(F(",TXarrayCRC,0x"));
    Serial.println(TXArrayCRC, HEX);              //get the CRC of the data array just sent, its placed at the packet end
    Serial.println();
#endif

    if (TXPacketL == 0)                           //if there has been a send and ack error, TXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
    }

    Serial.print(F("Wait ACK "));

    ValidACK = LoRa.waitACKDTIRQ(DTheader, DTFileOpenHeaderL, ACKtimeoutDTmS);

    RXPacketType = DTheader[0];

    if ((ValidACK > 0) && (RXPacketType == DTFileOpenACK))
    {
      Serial.println(F(" - Valid ACK received"));
#ifdef DEBUG
      printPacketHex();
#endif
    }
    else
    {
      NoAckCount++;
      Serial.println(F("No Valid ACK received"));
#ifdef DEBUG
      printACKdetail();
      Serial.print(F("ACKPacket "));
      printPacketHex();
#endif
      Serial.println();
    }
    Serial.println();
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
  //****************************************************************
  //End file transfer, close local file first then remote file.
  //****************************************************************

  uint8_t ValidACK;

  Serial.print(F("End file transfer "));
  Serial.println(buff);

  DTSD_closeFile();

  build_DTFileCloseHeader(DTheader, DTFileCloseHeaderL, filenamesize, DTLocalFileLength, DTLocalFileCRC, DTSegmentSize);

  do
  {
    Serial.println(F("Transmit close remote file request"));

    digitalWrite(LED1, HIGH);
    TXPacketL = LoRa.transmitDTIRQ(DTheader, DTFileCloseHeaderL, (uint8_t *) buff, filenamesize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    digitalWrite(LED1, LOW);

    TXNetworkID = LoRa.getTXNetworkID(TXPacketL);
    TXArrayCRC = LoRa.getTXPayloadCRC(TXPacketL);

#ifdef DEBUG
    Serial.print(F("TXNetworkID,0x"));
    Serial.print(TXNetworkID, HEX);               //get the NetworkID of the packet just sent, its placed at the packet end
    Serial.print(F(",TXarrayCRC,0x"));
    Serial.println(TXArrayCRC, HEX);              //get the CRC of the data array just sent, its placed at the packet end
    Serial.println();
#endif

    if (TXPacketL == 0)                                //if there has been a send and ack error, RXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
    }

    Serial.print(F("Wait ACK "));

    ValidACK = LoRa.waitACKDTIRQ(DTheader, DTFileCloseHeaderL, ACKtimeoutDTmS);

    RXPacketType = DTheader[0];

    if ((ValidACK > 0) && (RXPacketType == DTFileCloseACK))
    {
      Serial.println(F(" - Valid ACK received"));
#ifdef DEBUG
      printPacketHex();
#endif
    }
    else
    {
      NoAckCount++;
      Serial.println(F("No Valid ACK received"));
      printACKdetail();
#ifdef DEBUG
      Serial.print(F("ACKPacket "));
      printPacketHex();
#endif
      Serial.println();
    }
    Serial.println();
  }
  while (ValidACK == 0);

  return true;
}


void build_DTFileOpenHeader(uint8_t *header, uint8_t headersize, uint8_t datalength, uint32_t filelength, uint16_t filecrc, uint8_t segsize)
{
  //this builds the header buffer for the filename passed

  beginarrayRW(header, 0);             //start writing to array at location 0
  arrayWriteUint8(DTFileOpen);         //byte 0, write the packet type
  arrayWriteUint8(0);                  //byte 1, initial DTflags byte, not used here
  arrayWriteUint8(headersize);         //byte 2, write length of header
  arrayWriteUint8(datalength);         //byte 3, write length of dataarray
  arrayWriteUint32(filelength);        //byte 4,5,6,7, write the file length
  arrayWriteUint16(filecrc);           //byte 8, 9, write dataarray (filename) CRC
  arrayWriteUint8(segsize);            //byte 10, segment size
  arrayWriteUint8(0);                  //byte 11, unused byte
  endarrayRW();
}


void build_DTFileCloseHeader(uint8_t *header, uint8_t headersize, uint8_t datalength, uint32_t filelength, uint16_t filecrc, uint8_t segsize)
{
  //this builds the header buffer for the filename passed

  beginarrayRW(header, 0);             //start writing to array at location 0
  arrayWriteUint8(DTFileClose);         //byte 0, write the packet type
  arrayWriteUint8(0);                  //byte 1, initial DTflags byte, not used here
  arrayWriteUint8(headersize);         //byte 2, write length of header
  arrayWriteUint8(datalength);         //byte 3, write length of dataarray
  arrayWriteUint32(filelength);        //byte 4,5,6,7, write the file length
  arrayWriteUint16(filecrc);           //byte 8, 9, write dataarray (filename) CRC
  arrayWriteUint8(segsize);            //byte 10, segment size
  arrayWriteUint8(0);                  //byte 11, unused byte
  endarrayRW();
}


void printLocalFileDetails()
{
  Serial.print(DTfilenamebuff);
  Serial.print(" LocalFilelength is ");
  Serial.print(DTLocalFileLength);
  Serial.println(" bytes");
  Serial.print(DTfilenamebuff);
  Serial.print(" LocalFileCRC is 0x");
  Serial.println(DTLocalFileCRC, HEX);
  Serial.print("DTSegmentSize ");
  Serial.println(DTSegmentSize);
  Serial.print("Number Segments ");
  Serial.println(DTNumberSegments);
  Serial.print("DTLastSegmentSize ");
  Serial.println(DTLastSegmentSize);
}


void printPacketHex()
{
  RXPacketL = LoRa.readRegister(REG_RXNBBYTES);
  Serial.print(RXPacketL);
  Serial.print(" bytes > ");
  if (RXPacketL > 0)
  {
    LoRa.printSXBufferHEX(0, RXPacketL - 1);
  }
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


bool sendSegments()
{
  //start the file transfer at segment 0
  DTSegment = 0;
  DTSentSegments = 0;

  dataFile.seek(0);                       //ensure at first position in file

  while (DTSegment < (DTNumberSegments - 1))
  {
    printSeconds();

    if (sendFileSegment(DTSegment, DTSegmentSize))
    {
      Serial.println();
      DTSentSegments++;
    }
    else
    {
      Serial.println(F("ERROR in sendFileSegment"));
      Serial.println();
      return false;
    }

    delay(packetdelaymS);
  };

  Serial.println("Last segment");

  if (!sendFileSegment(DTSegment, DTLastSegmentSize))
  {
    Serial.println(F("ERROR in sendFileSegment"));
    return false;
  }

  return true;
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


void printSeconds()
{
  float secs;
  secs = ( (float) millis() / 1000);
  Serial.print(secs, 3);
  Serial.print(F(" "));
}


//***********************************************************************************************
// Code for dealing with SD
//***********************************************************************************************

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


void DTSD_printDirectory()
{
  dataFile = SD.open("/");
  Serial.println("Card directory");
  SD.ls("/", LS_R);
}


uint32_t DTSD_openFileRead(char *buff)
{
  uint32_t filesize;
  dataFile = SD.open(buff);
  filesize = dataFile.size();
  dataFile.seek(0);
  return filesize;
}


void DTSD_closeFile()
{
  dataFile.close();                                      //close local file
}


uint16_t DTSD_fileCRCCCITT(uint32_t fsize)
{
  //does a CRC calculation on the file open via dataFile
  uint32_t index;
  uint16_t CRCcalc;
  uint8_t j, filedata;

  CRCcalc = 0xFFFF;                                //start value for CRC16
  dataFile.seek(0);                                //be sure at start of file position

  for (index = 0; index < fsize; index++)
  {
    filedata = dataFile.read();
#ifdef DEBUGSDLIB
    Serial.print(" 0x");
    Serial.print(filedata, HEX);
#endif
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
  Serial.print(" {DEBUGSDLIB} ");
  Serial.print(index);
  Serial.print(" Bytes checked - CRC ");
  Serial.println(CRCcalc, HEX);
#endif

  return CRCcalc;
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


void DTSD_seekFileLocation(uint32_t position)
{
  dataFile.seek(position);                       //seek to position in file
  return;
}


void setup()
{
  pinMode(LED1, OUTPUT);                          //setup pin as output for indicator LED
  led_Flash(2, 125);                              //two quick LED flashes to indicate program start

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("3_SDfile_Transfer_TransmitterIRQ starting"));
  Serial.flush();

  SPI.begin();

  if (LoRa.begin(NSS, LORA_DEVICE))
  {
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("LoRa Device error"));
    while (1)
    {
      led_Flash(50, 50);                          //long fast speed flash indicates device error
    }
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  Serial.print("Initializing SD card...");

  if (DTSD_initSD(SDCS))
  {
    Serial.println("Card initialized.");
  }
  else
  {
    Serial.println("Card failed, or not present.");
    while (1) led_Flash(100, 25);
  }

  Serial.println();
  DTSD_printDirectory();
  Serial.println();
  Serial.println();
  Serial.println(F("SDfile transfer ready"));
  Serial.println();

  //enable the following line if you want to disable payload CRC checking
  //LoRa.setReliableConfig(NoReliableCRC);                   //disable payload CRC check

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    Serial.println(F("Payload CRC disabled"));
  }
  else
  {
    Serial.println(F("Payload CRC enabled"));
  }

  DTFileTransferComplete = false;

}
