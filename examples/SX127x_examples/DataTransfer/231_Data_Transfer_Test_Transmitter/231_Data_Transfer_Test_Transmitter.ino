/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 20/09/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a simulation test program for the use of a data transfer (DT) packet.

  DT packets can be used for transfering large amounts of data in a sequence of packets or segments,
  in a reliable and resiliant way. A segemnt will be transmitted until a valid acknowledge comes from the
  receiver. Use with the matching receiver program, 232_Data_Transfer_Test_Receiver.ino

  The purpose of this test program is to allow you to check the reliability of a set of LoRa modem
  parameters that might be used for file transfers, without the need to use SD card modules. Define the LoRa
  parameters in the settings.h file and also define a segment size. The transmitter will then transmit a
  numbered sequence of segments and the receiver uses the seqment numbers and number of received packet to
  provide running totals which tell you how many packets are being missed (and needed to be re-tranmitted)
  due to errors etc.

  Each DT packet contains a variable length header array and a variable length data array. On transmission
  the provided NetworkID and CRC of the entire packet are appended to the end of the packet. This ensures
  that the receiver can validate the packet to a high degree of certainty. The receiver will not accept
  packets that dont have the appropriate NetworkID or payload CRC at the end of the packet. You can test
  this by setting up a transmitter such as \SX127X_examples\Basics\3_LoRa_Transmitter.ino, the receiver
  should see these packets but will not accept them for processing as valid data.

  The transmitter sends a sequence of segments in order and the receiver keeps track of the sequence. If
  the sequence fails for some reason, the transmitter resets for instance, the receiver will return a NACK
  packet to the transmitter requesting the segment sequence it was expecting.

  Details of the packet identifiers, header and data lengths and format are in the file
  Data_transfer_packet_definitions.md in the \SX127X_examples\DataTransfer\ folder.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>

#include <SX127XLT.h>
#include <ProgramLT_Definitions.h>
#include "Settings.h"                             //LoRa settings etc.
#include <arrayRW.h>

SX127XLT LoRa;                                    //create an SX127XLT library instance called LoRa

uint8_t RXPacketType;                             //type of received packet, segment write, ACK, NACK etc
uint8_t RXFlags;                                  //DTflags byte in header, could be used to control actions in TX and RX
uint8_t RXHeaderL;                                //length of header
uint8_t RXDataarrayL;                             //length of data array\segment
uint16_t DTSegment = 0;                           //current segment number

int16_t PacketRSSI;                               //stores RSSI of received packet
int8_t  PacketSNR;                                //stores signal to noise ratio of received packet

uint16_t AckCount;                                //keep a track of acks that are received within timeout period
uint16_t NoAckCount;                              //keep a track of acks not received within timeout period

uint8_t DTheader[16];                             //header array
uint8_t DTdata[245];                              //data/segment array


void loop()
{
  sendDataSegment(DTSegment, DTSegmentSize);
  Serial.println();
  delay(packetdelaymS);
}


bool sendDataSegment(uint16_t segnum, uint8_t segmentsize)
{
  //****************************************************************
  //Send data segment
  //****************************************************************

  uint8_t ValidACK;
  uint8_t TXOK;

  build_DTSegmentHeader(DTheader, DTSegmentWriteHeaderL, segmentsize, segnum);

  Serial.print(F("Send Segment,"));
  Serial.print(segnum);
  Serial.print(F(" "));
  printheader(DTheader, DTSegmentWriteHeaderL);
  Serial.print(F(" "));
  printdata(DTdata, segmentsize);                         //print segment size of data array only
  Serial.println();


  do
  {
    digitalWrite(LED1, HIGH);
    TXOK = LoRa.transmitDT(DTheader, DTSegmentWriteHeaderL, (uint8_t *) DTdata, segmentsize, NetworkID, TXtimeoutmS, TXpower,  WAIT_TX);
    digitalWrite(LED1, LOW);

    if (TXOK == 0)                                        //if there has been a send and ack error, RXPacketL returns as 0
    {
      Serial.println(F("Transmit error"));
      return false;
    }

    ValidACK = LoRa.waitACKDT(DTheader, DTSegmentWriteHeaderL, ACKtimeoutDTmS);

    RXPacketType = DTheader[0];

    if (ValidACK == 0)
    {
      NoAckCount++;
      Serial.println(F("ERROR No Ack"));
      Serial.print(F("NoAckCount,"));
      Serial.print(NoAckCount);
      LoRa.printReliableStatus();
      Serial.println();
      return 0;
    }

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
      Serial.println(F("************************************"));
      Serial.println();
      Serial.flush();
    }

    //ack is valid, segment was acknowledged if here

    if (RXPacketType == DTSegmentWriteACK)
    {
      readACKHeader();
      AckCount++;
      printPacketDetails();
      DTSegment++;
    }


  }
  while (ValidACK == 0);

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
  Serial.print(F("Databytes,"));
  Serial.print(arraysize);
  Serial.print(F("  "));
  printarrayHEX((uint8_t *) dataarray, arraysize);
}


void printheader(uint8_t *hdr, uint8_t hdrsize)
{
  Serial.print(F("Headerbytes,"));
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
  pinMode(LED1, OUTPUT);                         //setup pin as output for indicator LED
  led_Flash(2, 125);                             //two quick LED flashes to indicate program start

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("231_Data_Transfer_Test_Transmitter starting"));

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

  //fill data array with test data
  for (uint8_t index = 0; index < DTSegmentSize; index++)
  {
    DTdata[index] = index;
  }

  Serial.println(F("Data Transfer Test ready"));
  Serial.println();
}
