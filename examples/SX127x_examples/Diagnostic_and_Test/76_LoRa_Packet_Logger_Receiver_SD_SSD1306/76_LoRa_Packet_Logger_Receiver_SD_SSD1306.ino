/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 05/11/22

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - The program listens for incoming packets using the LoRa settings in the 'Settings.h'
  file. The pins to access the LoRa device need to be defined in the 'Settings.h' file also.

  There is a printout and save to SD card (if attached) of the valid packets received in HEX format. Thus
  the program can be used to receive and record non-ASCII packets. The LED will flash for each packet
  received. The measured frequency difference between the frequency used by the transmitter and the
  frequency used by the receiver is shown. If this frequency difference gets to 25% of the set LoRa
  bandwidth, packet reception will fail. The displayed error can be reduced by using the
  'offset' setting in the 'Settings.h' file.

  There will be a limit to how fast the logger can receive packets, mainly caused by the delay in writing
  to SD card, so at high packet rates, packets can be lost.

  If the SWITCH1 pin is connected to ground at startup the program will go into tracker logger mode. It
  will then display and log to SD the location packets sent by the GPS trackers that are examples in the
  SX12XX-LoRa library. Thus the program can be used as a check that the LoRa GPS trackers are working.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>
#define USESDFAT                               //select this define to use the SdFat library.

#ifdef USESDFAT
#include <SdFat.h>                           //get library here > https://github.com/greiman/SdFat
SdFat SD;
File logFile;
#else
#include <SD.h>
File logFile;
#endif

#include <SX127XLT.h>                                     //get the library here > https://github.com/StuartsProjects/SX12XX-LoRa                                  
SX127XLT LoRa;

#include "Settings.h"
#include <ProgramLT_Definitions.h>
#include <TimeLib.h>                                      //get the library here > https://github.com/PaulStoffregen/Time
time_t recordtime;                                        //used to record the current time, preventing displayed rollover on printing

#include <U8x8lib.h>                                      //get library here >  https://github.com/olikraus/u8g2 
U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    //use this line for standard 0.96" SSD1306
//U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);   //use this line for 1.3" OLED often sold as 1.3" SSD1306
#define DEFAULTFONT u8x8_font_chroma48medium8_r           //font for U8X8 Library

char filename[] = "Log000.txt";     //base name for SD logfile
bool SD_Found = false;              //set if SD card found at program startup
uint8_t lognumber;                  //lognumber used to generate logfile name
uint32_t RXpacketCount;             //count of good packets
uint32_t RXpacketErrors;            //count of packet errors
uint8_t RXPacketL;                  //stores length of packet received
uint8_t PacketType;                 //for packet addressing, identifies packet type
uint8_t Destination;                //for packet addressing, identifies the destination (receiving) node
uint8_t Source;                     //for packet addressing, identifies the source (transmiting) node
uint16_t TXVolts;                   //supply\battery voltage of tracker
float TXLat;                        //latitude
float TXLon;                        //longitude
float TXAlt;                        //altitude
uint8_t TXStatus;                   //status byte from tracker transmitter
uint8_t TXSats;                     //number of sattelites in use
uint32_t TXHdop;                    //HDOP, indication of fix quality, horizontal dilution of precision, low is good
uint32_t TXGPSFixTime;              //time in mS for fix
uint32_t TXupTimemS;                //up time of TX in mS

int16_t PacketRSSI;                 //stores RSSI of received packet
int8_t  PacketSNR;                  //stores signal to noise ratio of received packet
uint16_t IRQStatus;                 //used to read the IRQ status
int32_t FreqErrror;                 //frequency error of received packet, in hz
bool standardmode;                  //used to indicate standard logger mode or tracker logger mode


void loop()
{
  RXPacketL = LoRa.receiveSXBuffer(0, 60000, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout

  digitalWrite(LED1, HIGH);                            //something has happened
  recordtime = now();                                  //stop the time to be displayed rolling over
  printTime();

  if (SD_Found)
  {
    printTimeSD();
  }

  PacketRSSI = LoRa.readPacketRSSI();
  PacketSNR = LoRa.readPacketSNR();
  FreqErrror = LoRa.getFrequencyErrorHz();
  IRQStatus = LoRa.readIrqStatus();

  if (RXPacketL == 0)
  {
    packet_is_Error();
    if (SD_Found)
    {
      packet_is_ErrorSD();
    }
  }
  else
  {
    if (standardmode)
    {
      packet_is_OK();
      if (SD_Found)
      {
        packet_is_OKSD();
      }
    }
    else
    {
      if (isTrackerPacket())
      {
        processTrackerPacket();
      }
      else
      {
        packet_is_OK();
        if (SD_Found)
        {
          packet_is_OKSD();
        }
      }
    }
  }
  digitalWrite(LED1, LOW);

  Serial.println();
}


void packet_is_OK()
{

  RXpacketCount++;

  Serial.print(F(" FreqErrror,"));
  Serial.print(FreqErrror);
  Serial.print(F("hz  "));

  printHEXPacketSX(0, RXPacketL);            //print the packet in HEX starting at address 0 in SX buffer
  printpacketDetails();
  dispscreen1();
}


bool isTrackerPacket()
{
  LoRa.startReadSXBuffer(0);
  PacketType = LoRa.readUint8();
  Destination = LoRa.readUint8();
  Source = LoRa.readUint8();
  LoRa.endReadSXBuffer();

  if ((PacketType == PowerUp) && (RXPacketL == 5))
  {
    return true;
  }

  if ((PacketType == LocationPacket) && (RXPacketL == 31))
  {
    return true;
  }

  if ((PacketType == LocationBinaryPacket) && (RXPacketL == 14))
  {
    return true;
  }

  if ((PacketType == NoFix)  && (RXPacketL == 4))
  {
    return true;
  }
  return false;
}


void processTrackerPacket()
{
  float tempfloat;

  if (PacketType == PowerUp)
  {
    RXpacketCount++;
    LoRa.startReadSXBuffer(3);
    TXVolts = LoRa.readUint16();                           //read tracker transmitter voltage
    LoRa.endReadSXBuffer();
    printtime();
    Serial.print(F(" Tracker Powerup "));
    Serial.print(TXVolts);
    Serial.println(F("mV"));
    dispscreen5();
    if (SD_Found)
    {
      logFile.print(F("Tracker Powerup "));
      logFile.println(TXVolts);
    }
  }

  if (PacketType == LocationPacket)
  {
    //packet has been received, now read from the SX12XX FIFO in the correct order.
    RXpacketCount++;
    Serial.print(F("LocationPacket "));
    LoRa.startReadSXBuffer(3);                //start the read of received packet
    TXLat = LoRa.readFloat();                 //read in the tracker latitude
    TXLon = LoRa.readFloat();                 //read in the tracker longitude
    TXAlt = LoRa.readFloat();                 //read in the tracker altitude
    TXSats = LoRa.readUint8();                //read in the satellites in use by tracker GPS
    TXHdop = LoRa.readUint32();               //read in the HDOP of tracker GPS
    TXStatus = LoRa.readUint8();              //read in the tracker status byte
    TXGPSFixTime = LoRa.readUint32();         //read in the last fix time of tracker GPS
    TXVolts = LoRa.readUint16();              //read in the tracker supply\battery volts
    TXupTimemS = LoRa.readUint32();           //read in the TX uptime in mS
    RXPacketL = LoRa.endReadSXBuffer();       //end the read of received packet
    printtime();
    Serial.print(F(" "));
    Serial.write(PacketType);
    Serial.write(Destination);
    Serial.write(Source);
    Serial.print(F(","));
    Serial.print(TXLat, 6);
    Serial.print(F(","));
    Serial.print(TXLon, 6);
    Serial.print(F(","));
    Serial.print(TXAlt, 1);
    Serial.print(F(","));
    Serial.print(TXSats);
    Serial.print(F(","));

    tempfloat = ( (float) TXHdop / 100);           //need to convert Hdop read from GPS as uint32_t to a float for display
    Serial.print(tempfloat, 2);

    Serial.print(F(","));
    Serial.print(TXStatus);
    Serial.print(F(","));

    Serial.print(TXGPSFixTime);
    Serial.print(F("mS,"));
    Serial.print(TXVolts);
    Serial.print(F("mV,"));
    Serial.print((TXupTimemS / 1000));
    Serial.print(F("s,"));

    printpacketDetails();
    dispscreen4();                                  //and show the packet detail it on screen
    if (SD_Found)
    {
      logGPSfix();
    }
    return;
  }


  if (PacketType == LocationBinaryPacket)
  {
    //packet from locator has been received, now read from the SX12XX FIFO in the correct order.
    RXpacketCount++;
    Serial.print(F("LocationBinaryPacket "));
    LoRa.startReadSXBuffer(3);
    TXLat = LoRa.readFloat();
    TXLon = LoRa.readFloat();
    TXAlt = LoRa.readInt16();
    TXStatus = LoRa.readUint8();
    RXPacketL = LoRa.endReadSXBuffer();
    printtime();
    Serial.print(F(" "));
    Serial.write(PacketType);
    Serial.write(Destination);
    Serial.write(Source);
    Serial.print(F(","));
    Serial.print(TXLat, 6);
    Serial.print(F(","));
    Serial.print(TXLon, 6);
    Serial.print(F(","));
    Serial.print(TXAlt, 0);
    Serial.print(F("m,"));
    Serial.print(TXStatus);

    printpacketDetails();
    dispscreen4();
    if (SD_Found)
    {
      logGPSfix();
    }
    return;
  }

  if (PacketType == NoFix)
  {
    RXpacketCount++;
    printtime();
    Serial.print(F(" No tracker GPS fix"));
    if (SD_Found)
    {
      logFile.println(F(" No tracker GPS fix"));
    }
  }
}


void printtime()
{
  Serial.print(hour(recordtime));
  printDigits(minute(recordtime));
  printDigits(second(recordtime));
}


void printHEXPacketSX(uint8_t addr, uint8_t size)
{
  uint8_t index, bufferbyte;

  LoRa.startReadSXBuffer(addr);               //start SX buffer

  for (index = 0; index < size; index++)
  {
    bufferbyte = LoRa.readUint8();           //read in the byte from SX buffer
    printHEXByte(bufferbyte);                //print the byte
    Serial.print(F(" "));
  }

  LoRa.endReadSXBuffer();
}


void printHEXByte(uint8_t temp)
{
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void packet_is_Error()
{
  RXPacketL = LoRa.readRXPacketL();                    //get the real packet length

  if (IRQStatus & IRQ_RX_TIMEOUT)
  {
    Serial.print(F(" RXTimeout"));
    disp.clearLine(0);
    disp.setCursor(0, 0);
    disp.print(F("RXTimeout"));
  }
  else
  {
    RXpacketErrors++;
    Serial.print(F(" Packet Error"));
    printpacketDetails();
    dispscreen1();
    disp.clearLine(0);
    disp.setCursor(0, 0);
    disp.print(F("Packet Error"));
  }
}


void printpacketDetails()
{
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(RXpacketErrors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
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


void printDigits(int8_t digits)
{
  //utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(F(":"));
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void printTime()
{
  Serial.print(hour(recordtime));
  printDigits(minute(recordtime));
  printDigits(second(recordtime));
}


//*******************************************************************************
// Start SD card routines
//*******************************************************************************

void printtimeSD()
{
  logFile.print(hour(recordtime));
  printDigitsSD(minute(recordtime));
  printDigitsSD(second(recordtime));
}


void printModemSettingsSD()
{
  uint8_t regdata;
  uint8_t sf;
  uint32_t bandwidth;
  uint8_t cr;
  uint8_t opt;
  uint16_t syncword;
  uint8_t  invertIQ;
  uint16_t preamble;
  uint32_t freqint;

  if (LORA_DEVICE == DEVICE_SX1272)
  {
    regdata = (LoRa.readRegister(REG_MODEMCONFIG1) & READ_BW_AND_2);
  }
  else
  {
    regdata = (LoRa.readRegister(REG_MODEMCONFIG1) & READ_BW_AND_X);
  }

  //get al the data frome the lora device in one go to avoid swapping
  //devices on the SPI bus all the time

  if (LORA_DEVICE == DEVICE_SX1272)
  {
    regdata = (LoRa.readRegister(REG_MODEMCONFIG1) & READ_BW_AND_2);
  }
  else
  {
    regdata = (LoRa.readRegister(REG_MODEMCONFIG1) & READ_BW_AND_X);
  }

  bandwidth = LoRa.returnBandwidth(regdata);
  freqint = LoRa.getFreqInt();
  sf = LoRa.getLoRaSF();
  cr = LoRa.getLoRaCodingRate();
  opt = LoRa.getOptimisation();
  syncword = LoRa.getSyncWord();
  invertIQ = LoRa.getInvertIQ();
  preamble = LoRa.getPreamble();

  printDeviceSD();
  logFile.print(F(","));
  logFile.print(freqint);
  logFile.print(F("hz,SF"));
  logFile.print(sf);

  logFile.print(F(",BW"));
  logFile.print(bandwidth);

  logFile.print(F(",CR4:"));
  logFile.print(cr);
  logFile.print(F(",LDRO_"));

  if (opt)
  {
    logFile.print(F("On"));
  }
  else
  {
    logFile.print(F("Off"));
  }

  logFile.print(F(",SyncWord_0x"));
  logFile.print(syncword, HEX);

  if (invertIQ == LORA_IQ_INVERTED)
  {
    logFile.print(F(",IQInverted"));
  }
  else
  {
    logFile.print(F(",IQNormal"));
  }
  logFile.print(F(",Preamble_"));
  logFile.print(preamble);
  logFile.flush();
}


void printOperatingSettingsSD()
{
  uint8_t ver = LoRa.getVersion();
  uint8_t pm = LoRa.getPacketMode();
  uint8_t hm = LoRa.getHeaderMode();
  uint8_t crcm = LoRa.getCRCMode();
  uint8_t agc = LoRa.getAGC();
  uint8_t lnag = LoRa.getLNAgain();
  uint8_t boosthf = LoRa.getLNAboostHF();
  uint8_t boostlf = LoRa.getLNAboostLF();

  printDeviceSD();

  logFile.print(F(",Version_"));
  logFile.print(ver, HEX);

  logFile.print(F(",PacketMode_"));

  if (pm)
  {
    logFile.print(F("LoRa"));
  }
  else
  {
    logFile.print(F("FSK"));
  }

  if (hm)
  {
    logFile.print(F(",Implicit"));
  }
  else
  {
    logFile.print(F(",Explicit"));
  }

  logFile.print(F(",CRC_"));
  if (crcm)
  {
    logFile.print(F("On"));
  }
  else
  {
    logFile.print(F("Off"));
  }


  logFile.print(F(",AGCauto_"));
  if (agc)
  {
    logFile.print(F("On"));
  }
  else
  {
    logFile.print(F("Off"));
  }

  logFile.print(F(",LNAgain_"));
  logFile.print(lnag);

  logFile.print(F(",LNAboostHF_"));
  if (boosthf)
  {
    logFile.print(F("On"));
  }
  else
  {
    logFile.print(F("Off"));
  }

  logFile.print(F(",LNAboostLF_"));
  if (boostlf)
  {
    logFile.print(F("On"));
  }
  else
  {
    logFile.print(F("Off"));
  }
  logFile.flush();
}


uint8_t setup_SDLOG()
{
  //checks if the SD card is present and can be initialised
  //returns log number, 1-99, if OK, 0 if not

  uint8_t i;

  Serial.print(F("SD card..."));

  if (!SD.begin(SDCS))
  {
    Serial.println(F("ERROR SD card fail"));
    Serial.println();
    SD_Found = false;
    return 0;
  }

  Serial.print(F("Initialized OK - "));
  SD_Found = true;

  for (i = 1; i < 100; i++) {
    filename[4] = i / 10 + '0';
    filename[5] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logFile = SD.open(filename, FILE_WRITE);
      break;
    }
  }

  Serial.print(F("Writing to "));
  Serial.println(filename);
  return i;
}


void printDeviceSD()
{
  switch (LORA_DEVICE)
  {
    case DEVICE_SX1272:
      logFile.print(F("SX1272"));
      break;

    case DEVICE_SX1276:
      logFile.print(F("SX1276"));
      break;

    case DEVICE_SX1277:
      logFile.print(F("SX1277"));
      break;

    case DEVICE_SX1278:
      logFile.print(F("SX1278"));
      break;

    case DEVICE_SX1279:
      logFile.print(F("SX1279"));
      break;

    default:
      logFile.print(F("Unknown Device"));

  }
  logFile.flush();
}


void printHEXPacketSXSD(uint8_t addr, uint8_t size)
{
  uint8_t index, bufferbyte;
  uint8_t buffertemp[size];

  LoRa.startReadSXBuffer(addr);                     //start SX buffer

  for (index = 0; index < size; index++)
  {
    bufferbyte = LoRa.readUint8();                  //read in the byte from SX buffer
    buffertemp[index] = bufferbyte;
  }
  LoRa.endReadSXBuffer();

  for (index = 0; index < size; index++)
  {
    printHEXByteSD(buffertemp[index]);              //print the byte to SD
    logFile.print(F(" "));
  }
  logFile.flush();
}


void printHEXByteSD(uint8_t temp)
{
  if (temp < 0x10)
  {
    logFile.print(F("0"));
  }
  logFile.print(temp, HEX);
}


void packet_is_OKSD()
{
  IRQStatus = LoRa.readIrqStatus();
  logFile.print(F(" FreqErrror,"));
  logFile.print(LoRa.getFrequencyErrorHz());
  logFile.print(F("hz  "));

  printHEXPacketSXSD(0, RXPacketL);

  logFile.print(F(" RSSI,"));
  logFile.print(PacketRSSI);
  logFile.print(F("dBm,SNR,"));
  logFile.print(PacketSNR);
  logFile.print(F("dB,Length,"));
  logFile.print(RXPacketL);
  logFile.print(F(",Packets,"));
  logFile.print(RXpacketCount);
  logFile.print(F(",Errors,"));
  logFile.print(RXpacketErrors);
  logFile.print(F(",IRQreg,"));
  logFile.println(IRQStatus, HEX);
  logFile.flush();
}


void packet_is_ErrorSD()
{
  IRQStatus = LoRa.readIrqStatus();                    //get the IRQ status
  RXPacketL = LoRa.readRXPacketL();                    //get the real packet length

  if (IRQStatus & IRQ_RX_TIMEOUT)
  {
    logFile.print(F(" RXTimeout"));
  }
  else
  {
    logFile.print(F(" PacketError"));
    logFile.print(F(",RSSI,"));
    logFile.print(PacketRSSI);
    logFile.print(F("dBm,SNR,"));
    logFile.print(PacketSNR);
    logFile.print(F("dB,Length,"));
    logFile.print(RXPacketL);
    logFile.print(F(",Packets,"));
    logFile.print(RXpacketCount);
    logFile.print(F(",Errors,"));
    logFile.print(RXpacketErrors);
    logFile.print(F(",IRQreg,"));
    logFile.println(IRQStatus, HEX);
  }
  logFile.flush();
}


bool readTXStatus(byte bitnum)
{
  return bitRead(TXStatus, bitnum);
}


void printDigitsSD(int8_t digits)
{
  //utility function for digital clock display: prints preceding colon and leading 0
  logFile.print(F(":"));
  if (digits < 10)
    logFile.print('0');
  logFile.print(digits);
}


void printTimeSD()
{
  logFile.print(hour(recordtime));
  printDigitsSD(minute(recordtime));
  printDigitsSD(second(recordtime));
}


bool logGPSfix()
{
  float tempfloat;

  tempfloat = ( (float) TXHdop / 100);

  logFile.print(F(" "));
  logFile.print(PacketType);
  logFile.print(F(","));
  logFile.write(Destination);
  logFile.print(F(","));
  logFile.write(Source);
  logFile.print(F(","));

  logFile.print(TXLat, 6);
  logFile.print(F(","));
  logFile.print(TXLon, 6);
  logFile.print(F(","));
  logFile.print(TXAlt, 1);
  logFile.print(F(","));
  logFile.print(TXSats);
  logFile.print(F(","));
  logFile.print(tempfloat, 2);                    //hdop
  logFile.print(F(","));
  logFile.print(TXStatus, HEX);
  logFile.print(F(","));
  logFile.print(TXGPSFixTime);
  logFile.print(F(","));
  logFile.print(TXVolts);
  logFile.print(F(","));
  logFile.print(TXupTimemS);
  logFile.println();
  logFile.flush();
  return true;
}

//*******************************************************************************
// End SD card routines
//*******************************************************************************


//*******************************************************************************
// Start display routines
//*******************************************************************************

void dispscreen1()
{
  disp.clearLine(0);
  disp.setCursor(0, 0);
  disp.print(F("OK "));
  disp.print(RXpacketCount);
  disp.clearLine(1);
  disp.setCursor(0, 1);
  disp.print(F("Errors "));
  disp.print(RXpacketErrors);
  disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(F("Length "));
  disp.print(RXPacketL);
  disp.clearLine(3);
  disp.setCursor(0, 3);
  disp.print(F("RSSI "));
  disp.print(PacketRSSI);
  disp.print(F("dBm"));
  disp.clearLine(4);
  disp.setCursor(0, 4);

  disp.print(F("SNR "));

  if (PacketSNR > 0)
  {
    disp.print(F("+"));
  }

  disp.print(PacketSNR);
  disp.print(F("dB"));
}


void dispscreen2()
{
  disp.setCursor(0, 0);
  disp.print(Frequency);
  disp.println(F("hz"));
  disp.clearLine(1);
  disp.setCursor(0, 1);
  disp.print(F("BW "));
  disp.print(LoRa.returnBandwidth(Bandwidth));
  disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(F("SF "));
  disp.print(SpreadingFactor);
}


void dispscreen4()
{
  float tempfloat;
  disp.clearLine(0);
  disp.setCursor(0, 0);
  disp.print(TXLat, 6);

  disp.setCursor(14, 0);

  if (readTXStatus(GPSFix))
  {
    disp.print(F("T"));
    disp.write(Source);
  }
  else
  {
    disp.print(F("T?"));
  }

  disp.clearLine(1);
  disp.setCursor(0, 1);
  disp.print(TXLon, 6);

  disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(TXAlt, 0);
  disp.print(F("m"));
  disp.clearLine(3);
  disp.setCursor(0, 3);

  disp.print(F("RSSI "));
  disp.print(PacketRSSI);
  disp.print(F("dBm"));
  disp.clearLine(4);
  disp.setCursor(0, 4);
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

  if (PacketType == LocationPacket)
  {
    disp.clearLine(5);
    disp.setCursor(0, 5);
    tempfloat = ((float) TXVolts / 1000);
    disp.print(F("Batt "));
    disp.print(tempfloat, 2);
    disp.print(F("v"));
  }

  disp.clearLine(6);
  disp.setCursor(0, 6);
  disp.print(F("Packets "));
  disp.print(RXpacketCount);

}

void dispscreen5()
{
  //show tracker powerup data on display
  float tempfloat;
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("Tracker Powerup"));
  disp.setCursor(0, 1);
  disp.print(F("Battery "));
  tempfloat = ((float) TXVolts / 1000);
  disp.print(tempfloat, 2);
  disp.print(F("v"));
}

//*******************************************************************************
// End didplay routines
//*******************************************************************************

void setup()
{
  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("76_LoRa_Packet_Logger_Receiver_SD_SSD1306 Starting"));
  Serial.println();

  disp.begin();
  disp.setFont(DEFAULTFONT);

  disp.clear();
  disp.setCursor(0, 0);

  pinMode(SWITCH1, INPUT_PULLUP);

  if (digitalRead(SWITCH1))
  {
    standardmode = true;
    Serial.println(F("Standard logger mode"));
  }
  else
  {
    standardmode = false;
    Serial.println(F("Tracker logger mode"));
    disp.setCursor(0, 6);
    disp.println(F("Tracker logger"));
  }


  SPI.begin();

  if (LoRa.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    disp.print(F("LoRa OK"));
    Serial.println(F("LoRa device found"));
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("No device responding"));
    disp.println(F("No LoRa Device"));
    while (1)
    {
      led_Flash(50, 50);
    }
  }

  lognumber = setup_SDLOG() ;                   //setup SD card

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  Serial.println();
  LoRa.printModemSettings();
  Serial.println();
  LoRa.printOperatingSettings();
  Serial.println();

  dispscreen2();
  delay(1000);

  if (SD_Found)
  {
    printModemSettingsSD();
    logFile.println();
    printOperatingSettingsSD();
    logFile.println();
    disp.setCursor(0, 7);
    disp.print(filename);
  }
  else
  {
    disp.setCursor(0, 7);
    disp.print(F("No SD Card"));
  }
  Serial.println();

  printTime();

  Serial.println();
}
