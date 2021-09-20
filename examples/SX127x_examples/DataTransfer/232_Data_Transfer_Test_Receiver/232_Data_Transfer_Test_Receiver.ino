/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 20/09/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a simulation test program for the use of a data transfer (DT) packet.

  DT packets can be used for transfering large amounts of data in a sequence of packets or segments,
  in a reliable and resiliant way. A segemnt will be transmitted until a valid acknowledge comes from the
  receiver. Use with the matching receiver program, 231_Data_Transfer_Test_Transmitter.ino

  The purpose of this test program is to allow you to check the reliability of a set of LoRa modem
  parameters that might be used for file transfers, without the need to use SD card modules. Define the LoRa
  parameters in the settings.h file and also define a segment size. The transmitter will then transmit a
  numbered sequence of segments and the receiver uses the seqment numbers and number of received packet to
  provide running totals which tell you how many packets are being missed (and need to be re-tranmitted) due
  to errors etc.

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
#include "Settings.h"                      //LoRa settings etc.
#include <arrayRW.h>

SX127XLT LoRa;                             //create an SX127XLT library instance called LoRa

uint16_t DTReceivedSegments;               //count of segments received
uint16_t DTSegmentNext;                    //next segment to receive
uint16_t DTSegmentLast;                    //last segments received

uint8_t RXPacketType;                      //type of received packet
uint8_t RXFlags;                           ///DTflags can be used to control actions in TX and RX
uint8_t RXHeaderL;                         //length of received header
uint8_t RXDataarrayL;                      //length of received data array
uint16_t DTSegment;                        //current segment number

int16_t PacketRSSI;                        //stores RSSI of received packet
int8_t PacketSNR;                          //stores signal to noise ratio of received packet
uint16_t RXErrors;                         //count of packets received with error

uint8_t DTheader[16];                      //array for header
uint8_t DTdata[245];                       //array for data

//#define DEBUG                            //enable define to see more detail


void loop()
{
  receiveaPacketDT();
}


bool receiveaPacketDT()
{
  //****************************************************************
  //Receive file segment - receives the packet and reports details
  //****************************************************************

  uint8_t RXOK;

  RXOK = LoRa.receiveDT(DTheader, HeaderSizeMax, (uint8_t *) DTdata, DataSizeMax, NetworkID, RXtimeoutmS, WAIT_RX);

  digitalWrite(LED1, HIGH);
  printSeconds();

  if (RXOK > 0)
  {
    //if the LT.receiveDT() returns a value > 0 for PacketOK then packet was received OK
    //then only action payload if destinationNode = thisNode
    RXPacketType = DTheader[0];
    processPacket(RXPacketType);
    Serial.println();
    return true;
  }
  else
  {
    //if the LT.receiveReliable() function detects an error PacketOK is 0
    Serial.print(F(" Packet error"));
    RXErrors++;
    printPacketDetails();
    Serial.println();
    return false;
  }
}


bool processPacket(uint8_t packettype)
{
  readHeader();

  if (packettype == DTSegmentWrite)
  {
    processSegmentWrite();
    return true;
  }

  if (packettype == DTFileOpen)
  {
    processFileOpen();
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

  if (DTSegment == DTSegmentNext)
  {
    Serial.print(F(" Received segment,"));
    Serial.print(DTSegment);
    Serial.print(F(" Bytes,"));
    Serial.print(RXDataarrayL);
    printPacketDetails();
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
    Serial.println(F("****************************************"));
    Serial.print(F("Transmit restart request for segment "));
    Serial.println(DTSegmentNext);
    printheader(DTheader, RXHeaderL);
    Serial.println();
    Serial.println(F("****************************************"));
    Serial.println();
    Serial.flush();

    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);
    digitalWrite(LED1, LOW);
    return false;
  }

  return true;
}


bool processFileOpen()
{
  //not implemented in this simulation
  return false;
}


bool processFileClose()
{
  //not implemented in this simulation
  return false;
}


void readHeader()
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
  Serial.print(F("DataBytes,"));
  Serial.print(buffsize);
  Serial.print(F("  "));
  printarrayHEX((uint8_t *) buff, buffsize);
}


void printheader(uint8_t *buff, uint8_t buffsize)
{
  Serial.print(F("HeaderBytes,"));
  Serial.print(buffsize);
  Serial.print(F(" "));
  printarrayHEX(buff, buffsize);
}


void printPacketDetails()
{
  PacketRSSI = LoRa.readPacketRSSI();
  PacketSNR = LoRa.readPacketSNR();
  Serial.print(F("  RSSI,"));
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

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("232_Data_Transfer_Test_Receiver starting"));

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
      led_Flash(50, 50);                        //long fast speed flash indicates device error
    }
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  //fill data array with test data
  for (uint8_t index = 0; index < DataSizeMax; index++)
  {
    DTdata[index] = index;
  }

  Serial.println(F("Basic test ready"));
  Serial.println();

  DTSegmentNext = 0;
}
