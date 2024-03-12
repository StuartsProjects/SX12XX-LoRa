/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 15/05/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - This is a program that can be used to test the throughput of a LoRa transmitter in
  FLRC mode.

  Whilst the various LoRa calculators tell you the on air data rate, in practice the achievable data
  rate will be less than that due to the overhead of the software routines to load and send a packet
  and internal delays in the LoRa device itself.

  A buffer is filled with characters and that buffer is then transmitted. The total time for a number of
  transmissions is recorded and the bit rate calculated. The packet size (1 - 127 bytes) and the number of
  packets to send in the test are specified in the 'Settings.h' file, see the 'Setup packet parameters Here !'
  section. The setting file also has the lora settings to use. A lower spreading factors and higher
  bandwidths will result in higher bitrates.

  There is the option of turning on an a requirement for an acknowledgement from a remote receiver, before
  the transmitter sends the next packet, set this; 'const bool waitforACK = true;' definition in the
  settings file. The matching receiver program '43_FLRC_Data_Throughput_Acknowledge_Receiver' does then need
  to be configured with same FLRC settings as this transmitter. When this option is set, the program will
  keep running until the number of transmissions and acknowledgements has completed without any timeouts
  in order to produce a valid average.

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

#include <SPI.h>                      //the lora device is SPI based so load the SPI library                                         
#include <SX128XLT.h>                 //include the appropriate library  
#include <ProgramLT_Definitions.h>
#include "Settings.h"                 //include the setiings file, frequencies, LoRa settings etc   

SX128XLT LT;                          //create a library class instance called LT

uint32_t startmS, endmS, sendtimemS, bitspersecond, bitsPerpacket;
uint32_t TXPacketCount;
float averagePacketTime;
uint8_t packetNumber;
uint8_t RXPacketL;                    //length of received packet
uint8_t PacketType;                   //for packet addressing, identifies packet type received
uint32_t packetCheck;
bool loopFail = false;


void loop()
{
  uint16_t index, index2;
  uint8_t TXBUFFER[TXPacketL + 1];           //create buffer for transmitted packet
  loopFail = false;

  Serial.println(F("Start transmit test"));

  startmS =  millis();                       //start transmit timer

  for (index = 0; index < numberPackets; index++)
  {
    //fill the buffer
    for (index2 = 0; index2 < TXPacketL; index2++)
    {
      TXBUFFER[index2] = index2;
    }

    TXBUFFER[0] = TestPacket;                 //set first byte to identify this test packet
    TXBUFFER[1] = index;                      //put the index as packet number in second byte of packet
    Serial.print(index);                      //print number of packet sent

    if (waitforACK)
    {
      packetCheck = ( (uint32_t) TXBUFFER[4] << 24) + ( (uint32_t) TXBUFFER[3] << 16) + ( (uint32_t) TXBUFFER[2] << 8) + (uint32_t) TXBUFFER[1];
      Serial.print(F(",Checkvalue,"));
      Serial.print(packetCheck, HEX);
      Serial.print(F(","));
    }

    digitalWrite(LED1, HIGH);

    if (LT.transmit(TXBUFFER, TXPacketL, 10000, TXpower, WAIT_TX))   //will return 0 if transmit error
    {
      digitalWrite(LED1, LOW);

      if (waitforACK)
      {
        if (!waitAck(packetCheck))
        {
          Serial.print(F("NoACKreceived,"));
          loopFail = true;
          break;
        }
      }

    }
    else
    {
      packet_is_Error();                              //transmit packet returned 0, there was an error
      loopFail = true;
    }

    Serial.println();

  }

  if (!loopFail)
  {
    endmS = millis();                                  //all packets sent, note end time
    digitalWrite(LED1, LOW);
    Serial.println();
    LT.printModemSettings();
    sendtimemS = endmS - startmS;
    Serial.println();
    Serial.print(F("Total transmit time "));
    Serial.print(numberPackets);
    Serial.print(F(" packets = "));
    Serial.print(sendtimemS);
    Serial.println(F("mS"));

    averagePacketTime = (float) ((endmS - startmS) / numberPackets);

    if (waitforACK)
    {
      Serial.print(F("Average "));
      Serial.print(TXPacketL);
      Serial.print(F(" byte packet transmit and acknowledge time = "));
    }
    else
    {
      Serial.print(F("Average "));
      Serial.print(TXPacketL);
      Serial.print(F(" byte packet transmit time = "));
    }

    Serial.print(averagePacketTime, 2);
    Serial.println(F("mS"));
    Serial.print(F("Packets per second "));
    Serial.println((float) (1000 / averagePacketTime));
    bitsPerpacket = (uint32_t) (TXPacketL * 8);
    Serial.print(F("Bits per packet sent = "));
    Serial.println(bitsPerpacket);

    Serial.print(F("Data rate = "));
    Serial.print((bitsPerpacket / (averagePacketTime / 1000)), 0);
    Serial.print(F("bps"));
    Serial.println();
    Serial.println();
    delay(10000);                                     //have a delay between loops so we can see result
  }
  else
  {
    Serial.println(F("Transmit test failed, trying again"));
    Serial.println();
    delay(1000);
  }

}


bool waitAck(uint32_t TXnum)
{
  uint32_t RXnum;
  uint16_t IRQStatus;

  RXPacketL = LT.receiveSXBuffer(0, 1000, WAIT_RX);   //returns 0 if packet error of some sort

  IRQStatus = LT.readIrqStatus();                     //read the LoRa device IRQ status register

  if (IRQStatus & IRQ_RX_TIMEOUT)                     //check for an RX timeout
  {
    Serial.print(F("RXTimeout,"));
    return false;
  }
  else
  {
    Serial.print(F("ACKRX,"));
  }

  LT.startReadSXBuffer(0);
  PacketType = LT.readUint8();
  RXnum = LT.readUint32();
  RXPacketL = LT.endReadSXBuffer();

  if ( (PacketType != ACK) || (RXnum != TXnum))
  {
    Serial.print(F("NotValidACK,"));
    return false;
  }
  else
  {
    Serial.print(RXnum, HEX);
    Serial.print(F(",OK"));
    return true;
  }

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
  pinMode(LED1, OUTPUT);                                //setup pin as output for indicator LED
  led_Flash(2, 125);                                    //two quick LED flashes to indicate program start

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("42_FLRC_Data_Throughput_Test_Transmitter Starting"));

  SPI.begin();

  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
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

  //***************************************************************************************************
  //Setup FLRC
  //***************************************************************************************************
  LT.setMode(MODE_STDBY_RC);
  LT.setRegulatorMode(USE_LDO);
  LT.setPacketType(PACKET_TYPE_FLRC);
  LT.setRfFrequency(Frequency, Offset);
  LT.setBufferBaseAddress(0, 0);
  LT.setModulationParams(BandwidthBitRate, CodingRate, BT);
  LT.setPacketParams(PREAMBLE_LENGTH_32_BITS, FLRC_SYNC_WORD_LEN_P32S, RADIO_RX_MATCH_SYNCWORD_1, RADIO_PACKET_VARIABLE_LENGTH, 127, RADIO_CRC_3_BYTES, RADIO_WHITENING_OFF);
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);              //set for IRQ on TX done and timeout on DIO1
  LT.setSyncWord1(Sample_Syncword);
  //***************************************************************************************************

  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();

  Serial.print(F("Transmitter ready"));
  Serial.println();
}
