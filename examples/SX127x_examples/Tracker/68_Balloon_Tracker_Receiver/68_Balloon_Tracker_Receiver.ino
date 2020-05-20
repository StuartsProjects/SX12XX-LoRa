/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 12/05/20
  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a LoRa tracker receiver intended to be used with a high altitude balloon
  (HAB) tracker. The program receives a standard format payload with LoRa that is compatible with the
  HABHUB online tracking system.

  The HAB payload sent by the tracker transmitter is assumed to be formatted like this;

  PayloadID,Sequence,Time,Lat,Lon,Alt,Satellites,Volts,Temperature,Resets,status,errors,Checksum
  Field 0      1      2    3   4   5      6        7       8         9      10     11      12

  The LoRa and frequency settings can be changed in the Settings.h file. There is the option of the transmitter
  sending out a much shorter Search mode binary location only payload. This is intended for ground based searching
  and locating. The frequency and LoRa settings of the Search mode packet can be different to the Tracker
  mode used by the HAB payload. To switch between standard tracker mode press the switch that is defined in the
  Settings.h file. This receiver cannot receive the transmitted FSK RTTY payload.

  There is a matching Tracker transmitter program.

  Serial monitor baud rate is set at 9600.

  ToDo:

*******************************************************************************************************/


#define Program_Version "V1.1"

#include <SPI.h>
#include <SX127XLT.h>
SX127XLT LT;

#include "Settings.h"
#include <ProgramLT_Definitions.h>

#include <U8x8lib.h>                                        //https://github.com/olikraus/u8g2 
U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);      //standard 0.96" SSD1306
//U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);     //1.3" OLED often sold as 1.3" SSD1306

#include <TinyGPS++.h>                                      //http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                            //create the TinyGPS++ object


#ifdef USESOFTSERIALGPS
//#include <NeoSWSerial.h>                                    //https://github.com/SlashDevin/NeoSWSerial
//NeoSWSerial GPSserial(RXpin, TXpin);                        //this library is more relaible with a GPS over SoftwareSerial
#include <SoftwareSerial.h>
SoftwareSerial GPSserial(RXpin, TXpin);
#else
#define GPSserial HARDWARESERIALPORT                        //hardware serial port (eg Serial1) is configured in the Settings.h file
#endif

#include <AFSKRTTY.h>

//**************************************************************************************************
// HAB tracker data - these are the variables transmitted in payload
//**************************************************************************************************
uint32_t TXSequence;              //sequence number of payload
uint8_t TXHours;                  //Hours
uint8_t TXMinutes;                //Minutes
uint8_t TXSeconds;                //Seconds
float TXLat;                      //latitude from GPS
float TXLon;                      //longitude from GPS
uint16_t TXAlt;                   //altitude from GPS
uint8_t TXSatellites;             //satellites used by GPS
uint16_t TXVolts;                 //measured tracker supply volts
int8_t TXTemperature;             //measured temperature
uint16_t TXResets;                //number of tracker resets
uint8_t TXStatus;                 //used to store current status flag bits
uint16_t TXErrors;                //number of tracker Errors
//**************************************************************************************************

float RXLat;                      //latitude of RX
float RXLon;                      //longitude of RX
float RXAlt;                      //altitude of RX

uint32_t RXpacketCount;           //count of received packets
uint8_t RXPacketL;                //length of received packet
int8_t  PacketRSSI;               //signal strength (RSSI) dBm of received packet
int8_t  PacketSNR;                //signal to noise ratio (SNR) dB of received packet
uint16_t RXerrors;
uint8_t PacketType;               //for packet addressing, identifies packet type
uint8_t Destination;              //for packet addressing, identifies the destination (receiving) node
uint8_t Source;                   //for packet addressing, identifies the source (transmiting) node
float TXdistance;                 //calculated distance to tracker
uint16_t TXdirection;             //calculated direction to tracker
uint16_t RXVolts;                 //supply\battery voltage of this receiver
uint32_t LastRXGPSfixCheck;       //used to record the time of the last GPS fix
bool TXLocation = false;          //set to true when at least one tracker location packet has been received
bool RXGPSfix = false;            //set to true if the local GPS has a recent fix

uint8_t FixCount = DisplayRate;   //used to keep track of number of GPS fixes before display updated

uint8_t RXBUFFER[RXBUFFER_SIZE];  //create the buffer that received packets are copied into
char FlightID[16];                //buffer for flight ID
uint8_t FlightIDlen;              //length of received flight ID

uint8_t modeNumber;               //mode receiver is in, 1 = Tracker, 2 = Search
//bool switch1Flag;                                 //flag to record switch press


void loop()
{
  RXPacketL = LT.receiveSXBuffer(0, 0, NO_WAIT);

  GPSserial.begin(GPSBaud);                       //startup GPS input
   
  while (!digitalRead(DIO0))
  {
    readGPS();                                     //If the DIO pin is low, no packet has arrived, so read the GPS

    if (!digitalRead(SWITCH1))
    {
      checkModeSwitch();
      break;
    }
  }

  if (digitalRead(DIO0))
  {
    //something has happened in receiver
    GPSserial.end();                              //stop GPS input to use SPI reliably
    digitalWrite(LED1, HIGH);

    RXPacketL = LT.readRXPacketL();
    PacketRSSI = LT.readPacketRSSI();
    PacketSNR = LT.readPacketSNR();

    if (LT.readIrqStatus() != (IRQ_RX_DONE + IRQ_HEADER_VALID))
    {
      packet_is_Error();
    }
    else
    {
      packet_is_OK();
    }

    digitalWrite(LED1, LOW);

    Serial.println();
  }
}


void readGPS()
{

  if (GPSserial.available() > 0)
  {
    gps.encode(GPSserial.read());
  }

  if ( millis() > (LastRXGPSfixCheck + NoRXGPSfixms))
  {
    RXGPSfix = false;
    LastRXGPSfixCheck = millis();
    dispscreen1();
  }

  if (gps.location.isUpdated() && gps.altitude.isUpdated())
  {
    RXGPSfix = true;
    RXLat = gps.location.lat();
    RXLon = gps.location.lng();
    RXAlt = gps.altitude.meters();
    LastRXGPSfixCheck = millis();

    if ( FixCount == 1)                           //update screen when FIXcoount counts down from DisplayRate to 1
    {
      FixCount = DisplayRate;
      dispscreen1();
    }
    FixCount--;
  }
}


void checkModeSwitch()
{
    digitalWrite(LED1, LOW);
    Serial.println();
    Serial.print(F("Listening in "));

    if (modeNumber == 1)
    {
      modeNumber = SearchMode;                     //used for receiver to tell whatmode it is in;
      setSearchMode();
      Serial.println(F("Search Mode"));
    }
    else
    {
      modeNumber = TrackerMode;
      setTrackerMode();
      Serial.println(F("Tracker Mode"));
    }

  dispscreen3();                                   //update receive mode to screen
  
  LT.printModemSettings();
  Serial.println();
  Serial.println();
  delay(1500);                                     //do a bit of switch de-bounce   
}


bool readTXStatus(uint8_t bitnum)
{
  return bitRead(TXStatus, bitnum);
}


void readPacketAddressing()
{
  LT.startReadSXBuffer(0);
  PacketType = LT.readUint8();
  Destination = LT.readUint8();
  Source = LT.readUint8();
  LT.endReadSXBuffer();
}


void packet_is_OK()
{
  uint16_t includedCRC, actualCRC;

  RXpacketCount++;

  printElapsedTime();                                   //print elapsed time to Serial Monitor
  readPacketAddressing();

  if (PacketType == PowerUp)
  {
    read_Command();
    Serial.print(F("TrackerPowerup,Battery,"));
    Serial.print(TXVolts);
    Serial.print(F("mV"));
    dispscreen2();
    return;
  }

  if (PacketType == HABPacket)
  {
    includedCRC = calcIncludedCRC();
    actualCRC = LT.CRCCCITTSX(2, RXPacketL - 7, 0xFFFF);

    if (actualCRC == includedCRC)
    {
      extractHABdata(0);
    }
    else
    {
      Serial.println();
      Serial.println();
      Serial.print(F("PayloadCRCError"));
      Serial.print(F(",includedCRC,"));
      Serial.print(includedCRC, HEX);
      Serial.print(F(",actualCRC,"));
      Serial.print(actualCRC, HEX);
      Serial.print(F(","));
      packet_is_Error();     //print the LoRa error flags
    }

    TXLocation = true;
    LT.printSXBufferASCII(0, (RXPacketL - 1));
    
    TXdistance = 0;
    TXdirection = 0;

    if (RXGPSfix)
    {
      doDistanceDirectionCalc();
    }

    printDistanceDirection();
    printpacketDetails();

    //Serial.println();
    //printHABdata();
    //Serial.println();
  
#ifdef UPLOADHABPACKET
    if (actualCRC == includedCRC)
    {
      Serial.println();
      uploadHABpacket();
    }
#endif

    return;
  }

  if ((PacketType == LocationBinaryPacket) && (Destination == '*') && (RXPacketL == 14))
  {
    //packet from tracker transmitter has been received, now read from the SX12XX FIFO in the correct order.
    TXLocation = true;
    LT.startReadSXBuffer(0);
    PacketType = LT.readUint8();
    Destination = LT.readUint8();
    Source = LT.readUint8();
    TXLat = LT.readFloat();
    TXLon = LT.readFloat();
    TXAlt = LT.readInt16();
    TXStatus = LT.readUint8();
    RXPacketL = LT.endReadSXBuffer();

    Serial.write(PacketType);
    Serial.write(Destination);
    Serial.write(Source);
    Serial.print(F(","));
    Serial.print(TXLat, 5);
    Serial.print(F(","));
    Serial.print(TXLon, 5);
    Serial.print(F(","));
    Serial.print(TXAlt, 0);
    Serial.print(F("m,"));
    Serial.print(TXStatus);

    TXdistance = 0;
    TXdirection = 0;

    if (RXGPSfix)
    {
      doDistanceDirectionCalc();
    }

    printDistanceDirection();

    printpacketDetails();
    dispscreen1();
    return;
  }

  if (PacketType == NoFix)
  {
    read_Command();
    Serial.write(Source);
    Serial.print(F(",NoTrackerGPSFix"));
    Serial.write(7);                             //send a BEL to serial terminal
    delay(250);
    Serial.write(7);
    return;
  }

  if (PacketType == NoGPS)
  {
    Serial.write(Source);
    read_Command();
    Serial.print(F(",GPSError"));
    return;
  }

  Serial.print(F("PacketNotRecognised"));
  printpacketDetails();
  printmorepacketDetails();
}


void uploadHABpacket()
{
  uint8_t index;
  uint8_t chartosend;

  Serial.print(F("Sending AFSK RTTY "));
  Serial.flush();

  startAFSKRTTY(AUDIOOUT, tonehighHz, leadinmS);
  sendAFSKRTTY(13, AUDIOOUT, CHECK, tonelowHz, tonehighHz, AFSKRTTYperiod);

  for (index = 0; index <= (RXPacketL - 2); index++)
  {
    chartosend = LT.getByteSXBuffer(index);
    sendAFSKRTTY(chartosend, AUDIOOUT, CHECK, tonelowHz, tonehighHz, AFSKRTTYperiod);
    Serial.write(chartosend);
    Serial.flush();
  }

  sendAFSKRTTY(13, AUDIOOUT, CHECK, tonelowHz, tonehighHz, AFSKRTTYperiod);
  sendAFSKRTTY(10, AUDIOOUT, CHECK, tonelowHz, tonehighHz, AFSKRTTYperiod);
  Serial.println();
  endAFSKRTTY(AUDIOOUT);
}


uint16_t calcIncludedCRC()
{
  uint8_t high, midhigh, midlow, low;
  uint16_t crc;
  high = LT.getByteSXBuffer(RXPacketL - 5);
  midhigh = LT.getByteSXBuffer(RXPacketL - 4);
  midlow = LT.getByteSXBuffer(RXPacketL - 3);
  low = LT.getByteSXBuffer(RXPacketL - 2);

  high = convertASCIIbyte(high);
  midhigh = convertASCIIbyte(midhigh);
  midlow = convertASCIIbyte(midlow);
  low = convertASCIIbyte(low);

  crc = (high * 4096) + (midhigh * 256) + (midlow * 16) + low;
  return (uint16_t) crc;
}


uint8_t convertASCIIbyte(uint8_t val)
{
  if (val > 0x40)
  {
    val = (val - 0x41) + 10;
    return val;
  }
  else
  {
    val = (val - 0x30);
    return val;
  }
}


void read_Command()
{
  LT.startReadSXBuffer(0);
  PacketType = LT.readUint8();
  Destination = LT.readUint8();
  Source = LT.readUint8();
  TXVolts = LT.readUint16();                           //read tracker transmitter voltage
  LT.endReadSXBuffer();
}


void printDistanceDirection()
{
  Serial.print(F(","));
  Serial.print(TXdistance, 0);
  Serial.print(F("m,"));
  Serial.print(TXdirection);
  Serial.print(F("d"));
}


void doDistanceDirectionCalc()
{
  TXdirection = (int16_t) TinyGPSPlus::courseTo(RXLat, RXLon, TXLat, TXLon);
  TXdistance = TinyGPSPlus::distanceBetween(RXLat, RXLon, TXLat, TXLon);
}


void printpacketDetails()
{
  int32_t hertzerror;
  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,FreqErr,"));
  hertzerror = LT.getFrequencyErrorHz();
  Serial.print(hertzerror);
  Serial.print(F("hz,PacketErrs,"));
  Serial.print(RXerrors);
  Serial.print(F(",PacketsOK,"));
  Serial.print(RXpacketCount);
}


void printmorepacketDetails()
{
  uint16_t IRQStatus;
  Serial.print(F(",Length,"));
  Serial.print(RXPacketL);
  IRQStatus = LT.readIrqStatus();
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
}


void packet_is_Error()
{
  uint16_t IRQStatus;

  IRQStatus = LT.readIrqStatus();                    //get the IRQ status
  RXerrors++;
  Serial.print(F("PacketError,RSSI"));

  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);

  Serial.print(F("dB,Length,"));
  Serial.print(LT.readRXPacketL());                  //get the real packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  LT.printIrqStatus();
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


void dispscreen1()
{
  //show received packet data on display
  uint8_t index;

  disp.clearLine(0);
  disp.setCursor(0, 0);

  if (PacketType == HABPacket)
  {
    for (index = 0; index < FlightIDlen; index++)
    {
      disp.write(FlightID[index]);
    }
  }

  if (PacketType == LocationBinaryPacket)
  {
    disp.print(Source);
  }

  disp.clearLine(1);
  disp.setCursor(0, 1);
  disp.print(F("Lat "));
  disp.print(TXLat, 5);
  disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(F("Lon "));
  disp.print(TXLon, 5);
  disp.clearLine(3);
  disp.setCursor(0, 3);
  disp.print(F("Alt "));
  disp.print(TXAlt);
  disp.print(F("m"));

  disp.clearLine(4);
  disp.setCursor(0, 4);
  disp.print(F("RSSI "));
  disp.print(PacketRSSI);
  disp.print(F("dBm"));
  disp.clearLine(5);
  disp.setCursor(0, 5);
  disp.print(F("SNR  "));

  if (PacketSNR > 0)
  {
    disp.print(F("+"));
  }

  if (PacketSNR == 0)
  {
    disp.print(F(" "));
  }

  if (PacketSNR < 0)
  {
    disp.print(F("-"));
  }

  disp.print(PacketSNR);
  disp.print(F("dB"));

  disp.clearLine(6);
  disp.setCursor(0, 6);
  disp.print(F("Packets "));
  disp.print(RXpacketCount);

  disp.clearLine(7);
  disp.setCursor(14, 1);

  if (RXGPSfix)
  {
    disp.print(F("RG"));
  }
  else
  {
    disp.setCursor(14, 1);
    disp.print(F("R?"));
    disp.setCursor(0, 7);
    disp.print(F("No Local Fix"));
  }

  dispscreen3();

  if (RXGPSfix && TXLocation)           //only display distance and direction if have received tracker packet and have local GPS fix
  {
    disp.clearLine(7);
    disp.setCursor(0, 7);
    disp.print(F("D&D "));
    disp.print(TXdistance, 0);
    disp.print(F("m "));
    disp.print(TXdirection);
    disp.print(F("d"));
  }

  disp.setCursor(14, 2);

  if (readTXStatus(NoGPSFix))
  {
    disp.print(F("T?"));
  }
  else
  {
    disp.print(F("TG"));
  }
}


void dispscreen2()
{
  //show tracker transmitter powerup data on display
  float tempfloat;
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("TrackerPowerup"));
  disp.setCursor(0, 1);
  disp.print(F(",Battery,"));
  tempfloat = ((float) TXVolts / 1000);
  disp.print(tempfloat, 2);
  disp.print(F("v"));
}


void dispscreen3()
{
  //show receive mode on display
  disp.setCursor(14, 0);

  if (modeNumber == TrackerMode)
  {
    disp.print(F("TR"));
  }
  else
  {
    disp.print(F("SE"));
  }
}


void extractHABdata(uint8_t startaddr)
{
  //extracts data from received HAB packets where first fields are lat,lon,alt
  //all varialbles are extracted, not all are used.

  uint8_t ptr = startaddr;                          //pointer to current location in SXbuffer
  uint8_t buffData;

  //Skip leading $
  do
  {
    buffData =  LT.getByteSXBuffer(ptr++);
  }
  while ( buffData == '$');

  ptr--;                                           //so ptr is at location of first non $

  FlightIDlen = extractBuffer(FlightID, sizeof(FlightID), ptr);  //extract flight ID

  ptr = nextComma(ptr);                            //step to next comma in SX buffer
  TXSequence = extractUint(ptr);                   //extract sequence

  ptr = nextComma(ptr);
  TXHours = (uint32_t) extractUint(ptr);

  ptr = nextComma(ptr);
  TXMinutes = (uint32_t) extractUint(ptr);

  ptr = nextComma(ptr);
  TXSeconds = (uint32_t) extractUint(ptr);

  //ptr = nextComma(ptr);
  ptr = nextComma(ptr);
  TXLat = extractFloat(ptr);

  ptr = nextComma(ptr);
  TXLon = extractFloat(ptr);

  ptr = nextComma(ptr);
  TXAlt = extractUint(ptr);

  ptr = nextComma(ptr);
  TXSatellites = extractUint(ptr);

  ptr = nextComma(ptr);
  TXVolts = extractUint(ptr);

  ptr = nextComma(ptr);
  TXTemperature = extractUint(ptr);

  ptr = nextComma(ptr);
  TXResets =  extractUint(ptr);

  ptr = nextComma(ptr);
  TXStatus = extractUint(ptr);

  ptr = nextComma(ptr);
  TXErrors = extractUint(ptr);
}


void printHABdata()
{
  printBuffer(FlightID, FlightIDlen);
  Serial.print(F(","));
  Serial.print(TXSequence);
  Serial.print(F(","));

  Serial.print(TXHours);
  Serial.print(F(","));
  Serial.print(TXMinutes);
  Serial.print(F(","));
  Serial.print(TXSeconds);
  Serial.print(F(","));

  Serial.print(TXLat, 5);
  Serial.print(F(","));
  Serial.print(TXLon, 5);
  Serial.print(F(","));
  Serial.print(TXAlt);
  Serial.print(F(","));
  Serial.print(TXSatellites);
  Serial.print(F(","));
  Serial.print(TXVolts);
  Serial.print(F(","));
  Serial.print(TXTemperature);
  Serial.print(F(","));
  Serial.print(TXResets);
  Serial.print(F(","));
  Serial.print(TXStatus);
  Serial.print(F(","));
  Serial.print(TXErrors);
  //Serial.print(F(","));
}


uint8_t extractBuffer(char *mybuffer, size_t bufferSize, uint8_t startptr)
{
  //extracts a character buffer in ASCII format from lora device RX buffer, returns the length of the buffer to , char
  uint16_t index;

  memset(mybuffer, 0, bufferSize);                   //clear array to 0s

  bufferSize--;                                      //last index location is one less than buffer size

  for (index = 0; index <= bufferSize; index++)
  {
    mybuffer[index] = LT.getByteSXBuffer(startptr++);
    if ((mybuffer[index] == ',') || (mybuffer[index] == '*'))
    {
      break;
    }
  }
  mybuffer[index] = 0;                               //it was a , so clear it
  return index++;                                    //buffer length is one more than index location
}


void printBuffer(char *buff, uint8_t len)
{
  //send buffer to serial terminal
  uint8_t index;

  for (index = 0; index < len; index++)
  {
    Serial.write(buff[index]);
  }

}


uint8_t nextComma(uint8_t localpointer)
{
  //skips through HAB packet (in SX device buffer) to next  comma
  uint8_t bufferdata;
  do
  {
    bufferdata =  LT.getByteSXBuffer(localpointer++);
  }
  while ((bufferdata != ',') && (bufferdata != ':') && (localpointer < RXPacketL));
  return localpointer;                              //note returns start of next field
}


uint8_t nextColon(uint8_t localpointer)
{
  //skips through HAB packet (in SX device buffer) to next  colon
  uint8_t bufferdata;
  do
  {
    bufferdata =  LT.getByteSXBuffer(localpointer++);
  }
  while ((bufferdata != ':') && (localpointer < RXPacketL));
  return localpointer;
}


int32_t extractUint(uint16_t localpointer)
{
  //extracts an unsigned int in ASCII format from buffer

  char temp[16];
  uint8_t tempptr = 0;
  uint8_t buffdata;
  uint32_t tempint;
  do
  {
    buffdata =  LT.getByteSXBuffer(localpointer++);;
    temp[tempptr++] = buffdata;
  }
  while ((buffdata != ',') && (buffdata != '*') && (localpointer < 256) );
  temp[tempptr] = 0;  //terminator for string
  tempint = (int32_t) atof(temp);
  return tempint;
}


float extractFloat(uint16_t localpointer)
{
  //extracts a float in ASCII format from buffer
  char temp[16];
  uint8_t tempptr = 0;
  uint8_t buffdata;
  float tempfloat;
  do
  {
    buffdata =  LT.getByteSXBuffer(localpointer++);;
    temp[tempptr++] = buffdata;
  }
  while ((buffdata != ',') && (buffdata != '*') && (localpointer < 256) );
  temp[tempptr] = 0;  //terminator for string
  tempfloat = (float)atof(temp);
  return tempfloat;
}


void setTrackerMode()
{
  LT.setupLoRa(TrackerFrequency, Offset, TrackerSpreadingFactor, TrackerBandwidth, TrackerCodeRate, TrackerOptimisation);
}


void setSearchMode()
{
  LT.setupLoRa(SearchFrequency, Offset, SearchSpreadingFactor, SearchBandwidth, SearchCodeRate, SearchOptimisation);
}


void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F(","));
}


void GPSPowerOn(int8_t pin, uint8_t state)
{
  if (pin >= 0)
  {
  digitalWrite(pin, state); 
  }
}


void setup()
{
  uint32_t endmS;

  pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
  led_Flash(2, 125);                            //two quick LED flashes to indicate program start
  pinMode(SWITCH1, INPUT_PULLUP);               //setup pin as switch input

  if (CHECK <= 0)
  {
    pinMode(CHECK, OUTPUT);
  }

  if (GPSPOWER >= 0)
  {
    pinMode(GPSPOWER, OUTPUT);
  }

  Serial.begin(9600);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();

  Serial.println(F("68_Balloon_Tracker_Receiver Starting"));

  SPI.begin();

  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);

  Serial.print(F("Checking LoRa device - "));         
  disp.setCursor(0, 0);

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))              //Initialize LoRa device
  {
    Serial.println(F("Receiver ready"));
    disp.print(F("Receiver ready"));
    led_Flash(2, 125);
    delay(1000);
  }
  else
  {
    Serial.println(F("No LoRa device responding"));
    disp.print(F("No LoRa device"));
    while (1)
    {
      led_Flash(50, 50);                                     //long fast speed flash indicates device error
    }
  }

  Serial.println();
  Serial.println(F("Startup GPS check"));
   
  endmS = millis() + 2000;

  //now startup GPS
  GPSPowerOn(GPSPOWER, GPSONSTATE);
  
  GPSserial.begin(GPSBaud);

  while (millis() < endmS)
  {
    while (GPSserial.available() > 0)
      Serial.write(GPSserial.read());
  }
  Serial.println();
  Serial.println(F("Done"));
  Serial.println();
  Serial.flush(); 

  GPSserial.end();                                                      //software serial interferes with SPI for LoRa device 
  setTrackerMode();

  modeNumber = TrackerMode;
  LT.printModemSettings();
  Serial.println();
  Serial.println(F("Listening in Tracker mode"));
  Serial.println();
  Serial.write(7);                                                      //send a BEL to serial terminal
  TXStatus = 4;                                                         //set default flag of no TX GPS fix 
}

