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
  will be transmitted until a valid acknowledge comes from the receiver. Use with the matching receiver
  program, 4_SDfile_Transfer_Receiver.ino.

  Each DT packet contains a variable length header array and a variable length data array as the payload.
  On transmission the NetworkID and CRC of the payload are appended to the end of the packet by the library
  routines. The use of a NetworkID and CRC ensures that the receiver can validate the packet to a high degree
  of certainty. The receiver will not accept packets that dont have the appropriate NetworkID or payload CRC
  at the end of the packet.

  The transmitter sends a sequence of segments in order and the receiver keeps track of the sequence. If
  the sequence fails for some reason, the receiver will return a NACK packet to the transmitter requesting
  the segment sequence it was expecting.

  Details of the packet identifiers, header and data lengths and formats used are in the file
  Data_transfer_packet_definitions.md in the \SX127X_examples\DataTransfer\ folder.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>

#include <SX127XLT.h>
#include <ProgramLT_Definitions.h>
#include "Settings.h"                      //LoRa settings etc.
#include <arrayRW.h>
#include "Variables.h"

SX127XLT LoRa;                             //create an SX127XLT library instance called LoRa

#include <SdFat.h>
SdFat SD;
File dataFile;                             //name the file instance needed for SD routines

uint8_t DTheader[16];                      //header array
uint8_t DTdata[245];                       //data/segment array

char DTfilenamebuff[DTfilenamesize];

//#define DEBUG                            //enable define to see more detail
//#define DEBUGSDLIB                       //enable define to see more detail for SD operation


void loop()
{
  receiveaPacketDT();
}


bool receiveaPacketDT()
{
  //****************************************************************
  //Receive Data transfer packets
  //****************************************************************

  RXPacketType = 0;

  RXPacketL = LoRa.receiveDT(DTheader, HeaderSizeMax, (uint8_t *) DTdata, DataSizeMax, NetworkID, RXtimeoutmS, WAIT_RX);
  digitalWrite(LED1, HIGH);
  printSeconds();

  if (RXPacketL > 0)
  {
    //if the LT.receiveDT() returns a value > 0 for RXPacketL then packet was received OK
    //then only action payload if destinationNode = thisNode
    readHeaderDT();                      //get the basic header details into global variables RXPacketType etc
    processPacket(RXPacketType);         //process and act on the packet
    Serial.println();
    return true;
  }
  else
  {
    //if the LoRa.receiveDT() function detects an error RXOK is 0
    Serial.print(F(" PacketError"));
    RXErrors++;
    printPacketDetails();
    LoRa.printReliableStatus();
    LoRa.printIrqStatus();
    Serial.println();
    return false;
  }
  digitalWrite(LED1, LOW);
}


bool processPacket(uint8_t packettype)
{

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


bool processSegmentWrite()
{
  //***********************************************************************************************
  // Code for dealing with segment writes   - checks that the sequence of segment writes is correct
  //***********************************************************************************************

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
    digitalWrite(LED1, HIGH);
    LoRa.sendACKDT(DTheader, DTStartHeaderL, TXpower);
    digitalWrite(LED1, LOW);
    return false;
  }


  if (DTSegment == DTSegmentNext)
  {
    Serial.print(F(" Received segment, "));
    Serial.print(DTSegment);
    Serial.print(F(" Bytes, "));
    Serial.print(RXDataarrayL);
    printPacketDetails();

    DTSD_writeSegmentFile(DTdata, RXDataarrayL);
    DTSD_fileFlush();

    Serial.print(F(" Send ACK"));
    DTheader[0] = DTSegmentWriteACK;
    delay(ACKdelaymS);
    digitalWrite(LED1, HIGH);
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);
    digitalWrite(LED1, LOW);
    DTReceivedSegments++;
    DTSegmentLast = DTSegment;                  //so we can tell if sequece has been received twice
    DTSegmentNext = DTSegment + 1;
    return true;
  }

  if (DTSegment == DTSegmentLast)
  {
    Serial.print(DTSegment);
    Serial.print(F(" ERROR segment already received "));
    printPacketDetails();
    DTheader[0] = DTSegmentWriteACK;
    Serial.print(F(" Send ACK"));
    delay(ACKdelaymS);
    digitalWrite(LED1, HIGH);
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);
    digitalWrite(LED1, LOW);
    return true;
  }


  if (DTSegment != DTSegmentNext )
  {
    Serial.print(F(" ERROR Received Segment "));
    Serial.print(DTSegment);
    Serial.print(F(" expected "));
    Serial.print(DTSegmentNext);
    Serial.print(F(" "));
    printPacketDetails();
    DTheader[0] = DTSegmentWriteNACK;
    DTheader[4] = lowByte(DTSegmentNext);
    DTheader[5] = highByte(DTSegmentNext);

    Serial.print(F(" Send NACK for segment "));
    Serial.print(DTSegmentNext);
    delay(ACKdelaymS);
    digitalWrite(LED1, HIGH);

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

    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);
    digitalWrite(LED1, LOW);
    return false;
  }

  return true;
}


bool processFileOpen(uint8_t *buff, uint8_t filenamesize)
{

  beginarrayRW(DTheader, 4);                      //start buffer read at location 4
  DTRemoteFileLength = arrayReadUint32();         //load the file length of the remote file being sent
  DTRemoteFileCRC = arrayReadUint16();            //load the CRC of the the remote file being sent

  memset(DTfilenamebuff, 0, DTfilenamesize);      //clear DTfilenamebuff to all 0s
  memcpy(DTfilenamebuff, buff, filenamesize);     //copy received DTdata into DTfilenamebuff

  Serial.print(F("FileOpen request for "));
  Serial.print((char*) DTfilenamebuff);
  Serial.println();
  printRemoteFileDetails();

  if (DTSD_openNewFileWrite(DTfilenamebuff, 0))     //open file for write at beginning, delete if it exists
  {
    Serial.print((char*) DTfilenamebuff);
    Serial.println(F(" File Opened OK"));
    DTSegmentNext = 0;                           //since file is opened the next sequence should be the first
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

  DTStartmS = millis();
  delay(ACKdelaymS);
  Serial.println(F("Sending ACK"));

  DTheader[0] = DTFileOpenACK;                //set the ACK packet type
  digitalWrite(LED1, HIGH);
  LoRa.sendACKDT(DTheader, DTFileOpenHeaderL, TXpower);
  digitalWrite(LED1, LOW);
  DTSegmentNext = 0;                          //after file open, segment 0 is next

  return true;
}


bool processFileClose()
{

  Serial.print(F("Fileclose for "));
  Serial.println((char*) DTfilenamebuff);

  if (DTFileOpened)                                     //check if file has been opened, close it if it is
  {
    if (SD.exists(DTfilenamebuff))                      //check if file exists
    {
      DTSD_closeFile();
      DTLocalFileLength = DTSD_openFileRead(DTfilenamebuff);
      DTLocalFileCRC = DTSD_fileCRCCCITT(DTLocalFileLength);
      beginarrayRW(DTheader, 4);                       //start writing to array at location 12
      arrayWriteUint32(DTLocalFileLength);             //write local file length just written to ACK header
      arrayWriteUint16(DTLocalFileCRC);                //write local file CRC just written to ACK header
      //arrayWriteUint16(0x2301);                //write local file CRC just written to ACK header
      printLocalFileDetails();

      Serial.print(F("Transfer time "));
      Serial.print(millis() - DTStartmS);
      Serial.print(F("mS"));
      Serial.println();
      Serial.println(F("File now closed"));
      DTFileOpened = false;
    }
  }
  else
  {
    Serial.println(F("File already closed"));
  }

  delay(ACKdelaymS);
  Serial.println(F("Sending ACK"));
  DTheader[0] = DTFileCloseACK;

  digitalWrite(LED1, HIGH);
  LoRa.sendACKDT(DTheader, DTFileCloseHeaderL, TXpower);
  digitalWrite(LED1, LOW);

  Serial.println();
  DTSD_printDirectory();
  Serial.println();
  Serial.println();
  return true;
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
}


void printRemoteFileDetails()
{
  Serial.print(DTfilenamebuff);
  Serial.print(" RemoteFilelength is ");
  Serial.print(DTRemoteFileLength);
  Serial.println(" bytes");
  Serial.print(DTfilenamebuff);
  Serial.print(" RemoteFileCRC is 0x");
  Serial.println(DTRemoteFileCRC, HEX);
}


void readHeaderDT()
{
  //the first 6 bytes of the header contain the important stuff, so load it up
  //so we can decide what to do next.
  beginarrayRW(DTheader, 0);                      //start buffer read at location 0
  RXPacketType = arrayReadUint8();                //load the packet type
  RXFlags = arrayReadUint8();                     //initial DTflags byte, not used here
  RXHeaderL = arrayReadUint8();                   //load the header length
  RXDataarrayL = arrayReadUint8();                //load the datalength
  DTSegment = arrayReadUint16();                  //load the segment number
}


void printdata(uint8_t *buff, uint8_t buffsize)
{
  Serial.print(F("DataBytes, "));
  Serial.print(buffsize);
  Serial.print(F("  "));
  printarrayHEX((uint8_t *) buff, buffsize);
}


void printheader(uint8_t *buff, uint8_t buffsize)
{
  Serial.print(F("HeaderBytes, "));
  Serial.print(buffsize);
  Serial.print(F(" "));
  printarrayHEX(buff, buffsize);
}


void printPacketDetails()
{
  PacketRSSI = LoRa.readPacketRSSI();
  PacketSNR = LoRa.readPacketSNR();
  Serial.print(F(" RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dBm,RXOKCount,"));
  Serial.print(DTReceivedSegments);
  Serial.print(F(",RXErrs,"));
  Serial.print(RXErrors);

#ifdef DEBUG
  Serial.print(F(" RX"));
  printheader(DTheader, RXHeaderL);
#endif
}


void printSeconds()
{
  float secs;
  secs = ( (float) millis() / 1000);
  Serial.print(secs, 3);
  Serial.print(F(" "));
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


bool DTSD_openNewFileWrite(char *buff, uint32_t position)
{

  if (SD.exists(buff))
  {
    Serial.print(buff);
    Serial.println(" exists, deleted");
    SD.remove(buff);
  }

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


uint32_t DTSD_openFileRead(char *buff)
{
  uint32_t filesize;
  dataFile = SD.open(buff);
  filesize = dataFile.size();
  dataFile.seek(0);
  return filesize;
}

void DTSD_fileFlush()
{
  dataFile.flush();
}


void DTSD_closeFile()
{
  dataFile.close();                                      //close local file
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


void setup()
{
  pinMode(LED1, OUTPUT);                       //setup pin as output for indicator LED
  led_Flash(2, 125);                           //two quick LED flashes to indicate program start

  Serial.begin(115200);

  //while (!Serial);                           / wait for serial port to connect. Needed for native USB

  Serial.println();
  Serial.println(F("234_SDfile_Transfer_Receiver starting"));

  SPI.begin();

  if (LoRa.begin(NSS, NRESET, DIO0, LORA_DEVICE))
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

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  Serial.print("Initializing SD card...");

  if (DTSD_initSD(SDCS))
  {
    Serial.println("Card initialized.");
  }
  else
  {
    Serial.println("Card failed, or not present.");
    while (1) led_Flash(100, 50);
  }

  Serial.println();
  DTSD_printDirectory();
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

  DTSegmentNext = 0;
  DTFileOpened = false;
}
