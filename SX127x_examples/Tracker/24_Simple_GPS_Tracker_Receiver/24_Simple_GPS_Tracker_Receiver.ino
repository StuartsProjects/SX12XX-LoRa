/*******************************************************************************************************
  LoRaTracker Programs for Arduino - Copyright of the author Stuart Robinson - 16/12/19

  http://www.LoRaTracker.uk

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This program is an basic receiver for the '23_Simple_GPS_Tracker_TX' program.
  The program reads the received packet from the tracker transmitter and displays the results on
  the serial monitor. The LoRa and frequency settings settings provided in the Settings.h file must
  match those used by the transmitter.

  The program receives direct from the LoRa devices internal buffer.

  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

#define Program_Version "V1.0"

#include <SPI.h>
#include <SX127XLT.h>

SX127XLT LT;

#include "Settings.h"
#include "Program_Definitions.h"


uint32_t RXpacketCount;
uint16_t errors;

uint8_t RXPacketL;             //length of received packet
int8_t PacketRSSI;             //RSSI of received packet
int8_t PacketSNR;              //signal to noise ratio of received packet
uint8_t PacketType;
uint8_t Destination;
uint8_t Source;
uint8_t TRStatus;
float TRLat;
float TRLon;
float TRAlt;
uint32_t TRHdop;
uint32_t TRGPSFixTime;
uint16_t TRVolts;
uint8_t TRSats;


void loop()
{
  RXPacketL = LT.receiveSXBuffer(0, 0, WAIT_RX);       //returns 0 if packet error of some sort

  digitalWrite(LED1, HIGH);

  if (BUZZER > 0)
  {
    digitalWrite(BUZZER, HIGH);
  }

  PacketRSSI = LT.readPacketRSSI();
  PacketSNR = LT.readPacketSNR();

  if (RXPacketL == 0)
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }

  digitalWrite(LED1, LOW);

  if (BUZZER > 0)
  {
    digitalWrite(BUZZER, LOW);
  }

  Serial.println();
}


void readPacketAddressing()
{
  //the transmitter is using packet addressing, so read in the details
  LT.startReadSXBuffer(0);
  PacketType = LT.readUint8();
  Destination = LT.readUint8();
  Source = LT.readUint8();
  LT.endReadSXBuffer();
}


void packet_is_OK()
{
  float tempHdop;

  RXpacketCount++;
  Serial.print(F("Packet OK > "));

  readPacketAddressing();

  if (PacketType == PowerUp)
  {
    LT.startReadSXBuffer(0);
    LT.readUint8();                              //read byte from FIFO, not used
    LT.readUint8();                              //read byte from FIFO, not used
    LT.readUint8();                              //read byte from FIFO, not used
    TRVolts = LT.readUint16();
    LT.endReadSXBuffer();
    Serial.print(F("Tracker transmitter powerup - battery "));
    Serial.print(TRVolts);
    Serial.print(F("mV"));
  }

  if (PacketType == LocationBinaryPacket)
  {
    //packet has been received, now read from the SX1280 FIFO in the correct order.
    LT.startReadSXBuffer(0);
    PacketType = LT.readUint8();
    Destination = LT.readUint8();
    Source = LT.readUint8();
    TRLat = LT.readFloat();
    TRLon = LT.readFloat();
    TRAlt = LT.readFloat();
    TRSats = LT.readUint8();
    TRHdop = LT.readUint32();
    TRStatus = LT.readUint8();
    TRGPSFixTime = LT.readUint32();
    TRVolts = LT.readUint16();
    RXPacketL = LT.endReadSXBuffer();

    tempHdop = ( (float) TRHdop / 100);           //need to convert Hdop read from GPS as uint32_t to a float for display

    Serial.write(PacketType);
    Serial.write(Destination);
    Serial.write(Source);
    Serial.print(F(","));
    Serial.print(TRLat, 5);
    Serial.print(F(","));
    Serial.print(TRLon, 5);
    Serial.print(F(","));
    Serial.print(TRAlt, 1);
    Serial.print(F("m,"));
    Serial.print(TRSats);
    Serial.print(F(","));
    Serial.print(tempHdop, 2);
    Serial.print(F(","));
    Serial.print(TRStatus);
    Serial.print(F(","));
    Serial.print(TRGPSFixTime);
    Serial.print(F("mS,"));
    Serial.print(TRVolts);
    Serial.print(F("mV"));
    printpacketDetails();
    return;
  }

  if ((PacketType != LocationBinaryPacket) && (PacketType != PowerUp))
  {
    Serial.print(F("Packet not recognised "));
    Serial.write(PacketType);
    Serial.write(Destination);
    Serial.write(Source);
    Serial.print(F(","));
    printpacketDetails();
  }

}


void printpacketDetails()
{
  uint16_t IRQStatus;
  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Packets,"));
  Serial.print(RXpacketCount);

  Serial.print(F(",Length,"));
  Serial.print(RXPacketL);
  IRQStatus = LT.readIrqStatus();
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);

}



void packet_is_Error()
{
  uint16_t IRQStatus;

  if (BUZZER > 0)
  {
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);
  }

  IRQStatus = LT.readIrqStatus();                    //get the IRQ status
  errors++;
  Serial.print(F("PacketError,RSSI"));

  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);

  Serial.print(F("dB,Length,"));
  Serial.print(LT.readRXPacketL());                  //get the real packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  LT.printIrqStatus();
  digitalWrite(LED1, LOW);

  if (BUZZER > 0)
  {
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);
  }
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
  pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
  led_Flash(2, 125);                            //two quick LED flashes to indicate program start

  Serial.begin(9600);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();

  Serial.println(F("24_Simple_GPS_Tracker_Receiver Starting"));

  if (BUZZER >= 0)
  {
    pinMode(BUZZER, OUTPUT);
    Serial.println(F("BUZZER Enabled"));
  }
  else
  {
    Serial.println(F("BUZZER Not Enabled"));
  }

  SPI.begin();

  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, DEVICE))
  {
    Serial.println(F("LoRa device found"));
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                             //long fast speed flash indicates device error
    }
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  Serial.println(F("Receiver ready"));
  Serial.println();
}



