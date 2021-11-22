/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/11/21
  
  The functions expect the calling sketch to create an instance called LoRa, so that functions
  are called like this; LoRa.getTXNetworkID().

  This code is supplied as is, it is up to the user of the program to decide if the program is suitable
  for the intended purpose and free from errors.
  
  This is a version of DTlibrary that will transfer an array in memory across to the receiver which saves
  the array as a file on its SD drive. This version is for use with the IRQ transmit and receive functions.
  Currently this library file only support the sending of an array.
  
  To receive the array use a program from the SX12XX LoRa library such as 234_SDfile_Transfer_Receiver or
  239_StuartCAM_LoRa_Receiver.
    
  There is a copy of this file in the SX12XX-LoRa library \src folder, but the file can be copied to the
  sketch folder and used locally. In this way its possible to carry out custom modifications. 
   
*******************************************************************************************************/

//Variables used on tranmitter
uint8_t RXPacketL;                         //length of received packet
uint8_t RXPacketType;                      //type of received packet, segment write, ACK, NACK etc
uint8_t RXHeaderL;                         //length of header
int16_t PacketRSSI;                        //stores RSSI of received packet
int8_t  PacketSNR;                         //stores signal to noise ratio of received packet
uint16_t AckCount;                         //keep a count of acks that are received within timeout period
uint16_t NoAckCount;                       //keep a count of acks not received within timeout period
uint16_t DTDestinationFileCRC;             //CRC of complete file received
uint16_t DTSourceArrayCRC;                  //CRC returned of the remote saved file
uint32_t DTDestinationFileLength;          //length of file written on the destination\receiver
uint32_t DTSourceFileLength;               //length of file at source\transmitter
uint32_t DTStartmS;                        //used for timeing transfers
uint16_t DTSegment = 0;                    //current segment number
char DTfilenamebuff[DTfilenamesize];       //global buffer to store current filename
uint8_t DTheader[16];                      //header array
int DTLED = -1;                            //pin number for indicator LED, if -1 then not used
uint8_t DTdata[245];                       //data/segment array

//Transmitter mode only variables
uint16_t TXNetworkID;                      //this is used to store the 'network' number, receiver must have the same networkID
uint16_t TXArrayCRC;                       //should contain CRC of data array transmitted
uint8_t  TXPacketL;                        //length of transmitted packet
uint16_t LocalPayloadCRC;                  //for calculating the local data array CRC
uint8_t DTLastSegmentSize;                 //size of the last segment
uint16_t DTNumberSegments;                 //number of segments for a file transfer
uint16_t DTSentSegments;                   //count of segments sent
bool DTFileTransferComplete;               //bool to flag file transfer complete
uint32_t DTSendmS;                         //used for timing transfers
float DTsendSecs;                          //seconds to transfer a file


//A global pointer to the array and a variable for its length and current location are used so that all routines 
//have access to the array to send without constantly passing the array pointer and variables between functions. 
uint8_t *ptrATsendarray;                   //create a global pointer to the array to send, so all functions have access
uint32_t ATArrayLength;                    //length of array to send
uint32_t ATarraylocation;                  //a global variable giving the location in the array last used


//Transmitter mode functions
bool sendFile(char *DTFileName, uint8_t namelength);
bool startFileTransfer(char *buff, uint8_t filenamesize, uint8_t attempts);
bool sendSegments();
bool sendFileSegment(uint16_t segnum, uint8_t segmentsize);
bool endFileTransfer(char *buff, uint8_t filenamesize);
void build_DTFileOpenHeader(uint8_t *header, uint8_t headersize, uint8_t datalength, uint32_t filelength, uint16_t filecrc, uint8_t segsize);
void build_DTSegmentHeader(uint8_t *header, uint8_t headersize, uint8_t datalen, uint16_t segnum);
void build_DTFileCloseHeader(uint8_t *header, uint8_t headersize, uint8_t datalength, uint32_t filelength, uint16_t filecrc, uint8_t segsize);
void printLocalFileDetails();
void printSeconds();
void printAckBrief();
void printAckReception();
void printACKdetail();
void printdata(uint8_t *dataarray, uint8_t arraysize);
void printPacketHex();
uint16_t getNumberSegments(uint32_t arraysize, uint8_t segmentsize);
uint8_t getLastSegmentSize(uint32_t arraysize, uint8_t segmentsize);

//Common functions
void setDTLED(int8_t pinnumber);
void printheader(uint8_t *hdr, uint8_t hdrsize);


//************************************************
//Transmit mode functions
//************************************************

bool sendArray(uint8_t *ptrarray, uint32_t arraylength, char *DTFileName, uint8_t namelength)
{
  // This routine allows the array transfer to be run with a function call of sendArray().
  memcpy(DTfilenamebuff, DTFileName, namelength);    //copy the name of destination file into global filename array for use outside this function
  ptrATsendarray = ptrarray;                         //set global pointer to array pointer passed
  ATArrayLength = arraylength;                       // the length of array to send
    
  do
  {
    NoAckCount = 0;
    DTStartmS = millis();

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
        Serial.println(F("************************************************"));
        Serial.println(DTFileName);
        Serial.println(F("ERROR remote file and array lengths do not match"));
        Serial.println(F("Restarting transfer"));
        Serial.println(F("************************************************"));
        DTFileTransferComplete = false;
        continue;
      }
      else
      {
        Serial.println(F("Remote file and array lengths match"));
      }
#ifdef ENABLEFILECRC
      DTDestinationFileCRC = arrayReadUint16();
      Serial.print(F("Acknowledged remote destination file CRC 0x"));
      Serial.println(DTDestinationFileCRC, HEX);

      if (DTDestinationFileCRC != DTSourceArrayCRC)
      {
        Serial.println(F("*********************************************"));
        Serial.println(DTFileName);
        Serial.println(F("ERROR remote file and array CRCs do not match"));
        Serial.println(F("Restarting transfer"));
        Serial.println(F("*********************************************"));
        DTFileTransferComplete = false;
        continue;
      }
      else
      {
        Serial.println(F("Remote file and array CRCs match"));
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
  Serial.print(F("Transmit rate "));
  Serial.print( (DTDestinationFileLength * 8) / (DTsendSecs), 0 );
  Serial.println(F("bps"));
  Serial.println(("Transfer finished"));
  return true;
}


bool startFileTransfer(char *buff, uint8_t filenamesize, uint8_t attempts)
{
  //Start transfer of array to remote file.

  uint8_t ValidACK;

  Serial.print(buff);
  Serial.println(F(" Start array transfer"));
  DTSourceFileLength = ATArrayLength;

  if (DTSourceFileLength == 0)
  {
    Serial.print(F("Error - array 0 bytes "));
    Serial.println(buff);
    return false;
  }

#ifdef ENABLEFILECRC
  DTSourceArrayCRC = LoRa.CRCCCITT((uint8_t *) ptrATsendarray, ATArrayLength, 0xFFFF);       //get array CRC from position 0 to end
#endif

  DTNumberSegments = getNumberSegments(DTSourceFileLength, DTSegmentSize);
  DTLastSegmentSize = getLastSegmentSize(DTSourceFileLength, DTSegmentSize);
  build_DTFileOpenHeader(DTheader, DTFileOpenHeaderL, filenamesize, DTSourceFileLength, DTSourceArrayCRC, DTSegmentSize);
  LocalPayloadCRC = LoRa.CRCCCITT((uint8_t *) buff, filenamesize, 0xFFFF);

  do
  {
    Serial.println(F("Send open remote file request"));

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    TXPacketL = LoRa.transmitDTIRQ(DTheader, DTFileOpenHeaderL, (uint8_t *) buff, filenamesize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
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
    ValidACK = LoRa.waitACKDTIRQ(DTheader, DTFileOpenHeaderL, ACKopentimeoutmS);
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
      Serial.print(F(" - No ACK "));
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


bool sendSegments()
{
  // Start the array transfer at segment 0
  DTSegment = 0;
  DTSentSegments = 0;

  ATarraylocation = 0;                      //start at first position in array

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

  Serial.println(F("Last segment"));

  if (!sendFileSegment(DTSegment, DTLastSegmentSize))
  {
    return false;
  }

  return true;
}


bool sendFileSegment(uint16_t segnum, uint8_t segmentsize)
{
  // Send array segment as payload in a DT packet

  uint8_t ValidACK;
  uint8_t index;
  uint8_t tempdata;
  
  for (index = 0; index < segmentsize; index++)
  {
    tempdata = ptrATsendarray[ATarraylocation];
    DTdata[index] = tempdata;
    ATarraylocation++;
  }

  build_DTSegmentHeader(DTheader, DTSegmentWriteHeaderL, segmentsize, segnum);

#ifdef PRINTSEGMENTNUM
  //Serial.print(F("Segment,"));
  Serial.println(segnum);
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

    TXPacketL = LoRa.transmitDTIRQ(DTheader, DTSegmentWriteHeaderL, (uint8_t *) DTdata, segmentsize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }

    if (TXPacketL == 0)                                     //if there has been an error TXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
    }

    ValidACK = LoRa.waitACKDTIRQ(DTheader, DTSegmentWriteHeaderL, ACKsegtimeoutmS);
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
        ATarraylocation = DTSegment * DTSegmentSize;
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
        //printAckReception()
#endif
        DTSegment++;                  //increase value for next segment
        return true;
      }
    }
    else
    {
      NoAckCount++;
      Serial.print(F("Error no ACK "));
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


bool endFileTransfer(char *buff, uint8_t filenamesize)
{
  //End array transfer close remote file

  uint8_t ValidACK;

  build_DTFileCloseHeader(DTheader, DTFileCloseHeaderL, filenamesize, DTSourceFileLength, DTSourceArrayCRC, DTSegmentSize);

  do
  {
    printSeconds();
    Serial.println(F(" Send close remote file"));

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    TXPacketL = LoRa.transmitDTIRQ(DTheader, DTFileCloseHeaderL, (uint8_t *) buff, filenamesize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
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

    ValidACK = LoRa.waitACKDTIRQ(DTheader, DTFileCloseHeaderL, ACKclosetimeoutmS);
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
      Serial.println(F("No ACK "));
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
  // This builds the header buffer for the filename receiver saves the array as

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
  //This builds the header buffer for a segment transmit

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
  //This builds the header buffer for the close remote file command

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
  Serial.print(F("Source array length "));
  Serial.print(DTSourceFileLength);
  Serial.println(F(" bytes"));
#ifdef ENABLEFILECRC
  Serial.print(F("Source array CRC is 0x"));
  Serial.println(DTSourceArrayCRC, HEX);
#endif
  Serial.print(F("Segment Size "));
  Serial.println(DTSegmentSize);
  Serial.print(F("Number segments "));
  Serial.println(DTNumberSegments);
  Serial.print(F("Last segment size "));
  Serial.println(DTLastSegmentSize);
}


void printSeconds()
{
  float secs;
  secs = ( (float) millis() / 1000);
  Serial.print(secs, 2);
  Serial.print(F(" "));
}


void printAckBrief()
{
  PacketRSSI = LoRa.readPacketRSSI();
  Serial.print(F(",AckRSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm"));
}


void printAckReception()
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


void printdata(uint8_t *dataarray, uint8_t arraysize)
{
  Serial.print(F("DataBytes,"));
  Serial.print(arraysize);
  Serial.print(F("  "));
  printarrayHEX((uint8_t *) dataarray, 16);             //There is a lot of data to print so only print first 16 bytes
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


uint16_t getNumberSegments(uint32_t arraysize, uint8_t segmentsize)
{
  uint16_t segments;
  segments = arraysize / segmentsize;

  if ((arraysize % segmentsize) > 0)
  {
    segments++;
  }
  return segments;
}


uint8_t getLastSegmentSize(uint32_t arraysize, uint8_t segmentsize)
{
  uint8_t lastsize;

  lastsize = arraysize % segmentsize;
  if (lastsize == 0)
  {
    lastsize = segmentsize;
  }
  return lastsize;
}

//************************************************
//Common functions
//************************************************

void setDTLED(int8_t pinnumber)
{
  //give the data transfer routines an LED to flash
  if (pinnumber >= 0)
  {
    DTLED = pinnumber;
    pinMode(pinnumber, OUTPUT);
  }
}


void printheader(uint8_t *hdr, uint8_t hdrsize)
{
  Serial.print(F("HeaderBytes,"));
  Serial.print(hdrsize);
  Serial.print(F(" "));
  printarrayHEX(hdr, hdrsize);
}
