/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 05/06/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - This is a GPS SD card logger intended for use in a high altitude balloon. The
  program  sets the chosen GPS, Ublox or Quectel, into balloon mode otherwise the GPSs would stop working
  at circa 12,000m. If the SD card fails the LED will flash rapidly for a couple of seconds. 

  The data logged to SD card looks like this;

  16:32:45,5/6/2020,51.482301,-3.181362,15,7,1.15
  
  Serial monitor baud rate is set at 115200
*******************************************************************************************************/

#define Program_Version "V1.0"

#include <Arduino.h>

#include "Settings.h"

//**************************************************************************************************
// HAB tracker data - these are the variables recorded to the SD card
//**************************************************************************************************
uint8_t TXHours;                                 //Hours
uint8_t TXMinutes;                               //Minutes
uint8_t TXSeconds;                               //Seconds
float TXLat;                                     //latitude from GPS
float TXLon;                                     //longitude from GPS
uint16_t TXAlt;                                  //altitude from GPS
uint8_t TXSatellites;                            //satellites used by GPS
uint8_t TXStatus = 0;                            //used to store current status flag bits
uint32_t TXGPSfixms;                             //fix time of GPS
uint32_t TXGPSHdop;                              //HDOP value of GPS
uint8_t hours, mins, secs, day, month;
uint16_t year;
//**************************************************************************************************


#include <SD.h>
#include <SPI.h>
File logFile;
char filename[] = "/Log0000.txt";                //filename used as base for creating logfile, 0000 replaced with numbers


#include <TinyGPS++.h>                           //http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                 //create the TinyGPS++ object

#ifdef USESOFTSERIALGPS
#include <NeoSWSerial.h>                       //https://github.com/SlashDevin/NeoSWSerial
NeoSWSerial GPSserial(RXpin, TXpin);           //this library is more relaible at GPS init than software serial
//#include <SoftwareSerial.h>
//SoftwareSerial GPSserial(RXpin, TXpin);
#endif

#ifndef USESOFTSERIALGPS
#define GPSserial HARDWARESERIALPORT
#endif

#include GPS_Library                             //include previously defined GPS Library 

uint32_t GPSstartms;                             //start time waiting for GPS to get a fix
uint32_t loopCount = 1;

void loop()
{
  Serial.print(loopCount++);
  Serial.println(F(" Start Loop"));

  GPSstartms = millis();

  if (!gpsWaitFix(WaitGPSFixSeconds))
  {
    GPS_OutputOff();
    delay(1000);                                 //give receiver enough time to report NoFix
  }
  Serial.println();

  Serial.print(F("Write fix to "));
  Serial.print(filename);
  
  digitalWrite(LED1, HIGH);
  logGPSfix(filename);                           //log fix data to SD card
  digitalWrite(LED1, LOW);
  
  Serial.println();
  Serial.println(F("Sleep"));
   
  Serial.flush();                                //make sure no serial output pending before goint to sleep

  delay(SleepTimesecs * 1000);

  Serial.println(F("Wake"));
  
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  //flash LED to show tracker is alive
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


//***********************************************************
// Start GPS Functions
//***********************************************************

void GPSTest()
{
  uint32_t endmS;

  endmS = millis() + 2000;                     //run GPS echo for 2000mS

  while (millis() < endmS)
  {
    while (GPSserial.available() > 0)
      Serial.write(GPSserial.read());
  }
  Serial.println();
  Serial.println();
  Serial.flush();
}


bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for good fix

  uint32_t endwaitmS, millistowait, currentmillis;
  uint8_t GPSchar;

  Serial.flush();

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.print(F("s "));
  Serial.flush();

  GPS_OutputOn();
  Serial.flush();

  currentmillis = millis();
  millistowait = waitSecs * 1000;
  endwaitmS = currentmillis + millistowait;

  while (GPSserial.read() >= 0);                  //clear the GPS serial input buffer

  while (millis() < endwaitmS)
  {

    if (GPSserial.available() > 0)
    {
      GPSchar = GPSserial.read();
      gps.encode(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      TXLat = gps.location.lat();
      TXLon = gps.location.lng();
      TXAlt = (uint16_t) gps.altitude.meters();

      //Altitude is used as an unsigned integer, so that the binary payload is as short as possible.
      //However gps.altitude.meters(); can return a negative value which converts to
      //65535 - Altitude, which we dont want. So we will assume any value over 60,000M is zero

      if (TXAlt > 60000)
      {
        TXAlt = 0;
      }

      TXHours = gps.time.hour(),
      TXMinutes = gps.time.minute(),
      TXSeconds = gps.time.second(),
      TXSatellites = gps.satellites.value();
      TXGPSHdop = gps.hdop.value();

      hours = gps.time.hour();
      mins = gps.time.minute();
      secs = gps.time.second();
      day = gps.date.day();
      month = gps.date.month();
      year = gps.date.year();

      TXGPSfixms = millis() - GPSstartms;

      Serial.print(F("Have GPS Fix "));
      Serial.print(TXGPSfixms);
      Serial.print(F("mS"));
      Serial.println();

      return true;
    }

  }

  //if here then there has been no fix and a timeout
  GPS_OutputOff();
  Serial.println(F("Error No GPS Fix"));
  return false;
}

//***********************************************************
// End GPS Functions
//***********************************************************

//*******************************************************************************
// Start SD card routines
//*******************************************************************************

void logGPSfix(char *buf )
{
  float tempfloat;

  

  logFile = SD.open(buf, FILE_WRITE);

  if (!logFile)
  {
  Serial.println();
  cardFail(2);
  return;
  }

  tempfloat = ( (float) TXGPSHdop / 100);

  if (hours < 10)
  {
    logFile.print(F("0"));
  }

  logFile.print(hours);
  logFile.print(F(":"));

  if (mins < 10)
  {
    logFile.print(F("0"));
  }

  logFile.print(mins);
  logFile.print(F(":"));

  if (secs < 10)
  {
    logFile.print(F("0"));
  }

  logFile.print(secs);
  logFile.print(F(","));

  logFile.print(day);
  logFile.print(F("/"));
  logFile.print(month);
  logFile.print(F("/"));
  logFile.print(year);

  logFile.print(F(","));
  logFile.print(TXLat, 6);
  logFile.print(F(","));
  logFile.print(TXLon, 6);
  logFile.print(F(","));
  logFile.print(TXAlt, 1);
  logFile.print(F(","));
  logFile.print(TXSatellites);
  logFile.print(F(","));
  logFile.print(tempfloat, 2);

  logFile.println();
  logFile.close();

  
}


uint8_t setupSDLOG(char *buf)
{
  //creats a new filename

  uint16_t index;

  for (index = 1; index <= 9999; index++) {
    buf[4] = index / 1000 + '0';
    buf[5] = ((index % 1000) / 100) + '0';
    buf[6] = ((index % 100) / 10) + '0';
    buf[7] = index % 10 + '0' ;
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logFile = SD.open(buf, FILE_WRITE);
      break;
    }
  }

  return index;                                   //return number of logfile created
}


void cardFail(uint8_t num)
{
    Serial.print(num);                                //so we can tell where crd failed
    Serial.println(F(" Card failed, or not present"));
    led_Flash(25, 25);
}

//*******************************************************************************
// End SD card routines
//*******************************************************************************




void setup()
{
  //uint32_t i;
  //uint16_t j;

  Serial.begin(115200);                     //Setup Serial console ouput

  pinMode(LED1, OUTPUT);                    //for indicator LED
  led_Flash(2, 500);

  Serial.println();
  Serial.println();
  Serial.println(F("76_Balloon_GPS_SDlogger Starting"));

  if (GPSPOWER >= 0)                        //if GPS needs power switching, turn it on
  {
    pinMode(GPSPOWER, OUTPUT);
    digitalWrite(GPSPOWER, GPSONSTATE);
  }

  if (BATVREADON >= 0)
  {
    pinMode(BATVREADON, OUTPUT);            //for MOSFET controlling battery volts resistor divider
  }

#ifdef QUECTELINUSE
  Serial.println(F("Quectel GPS library"));
#endif

#ifdef UBLOXINUSE
  Serial.println(F("UBLOX GPS library"));
#endif

  SPI.begin();                              //initialize SPI

  Serial.println();

  GPS_OutputOn();
  GPSTest();
  GPS_Setup();                                 //GPS should have had plenty of time to initialise by now

  delay(2000);

  if (GPS_CheckConfiguration())               //Check that GPS is configured for high altitude mode
  {
    Serial.println();
    GPS_OutputOff();                          //GPS interrupts cause problems with lora device, so turn off for now
    
    Serial.println(F("GPS Config OK"));        //check tone indicates navigation model 6 set
    Serial.println();
    Serial.flush();
  }
  else
  {
    Serial.println(F("GPS Error"));
    Serial.println();
    led_Flash(100, 25);                          //long very rapid flash for GPS error
  }

  GPSstartms = millis();

  while (!gpsWaitFix(5))                         //wait for the initial GPS fix, this could take a while
  {
   led_Flash(2, 50);                            //two short LED flashes to indicate GPS waiting for fix
  }

  GPS_OutputOn();
  delay(2000);                                   //GPS may be in software backup allow time for it to wakeup
  GPS_SetCyclicMode();                           //set this regardless of whether hot fix mode is enabled
  GPS_OutputOff();


  Serial.println();
  Serial.println(F("Initializing SD card"));

  if (!SD.begin(SDCS))
  {
    cardFail(1);
  }

  Serial.println(F("Card initialized"));
  setupSDLOG(filename);                            //setup logfile name for writing
  Serial.print(F("Writing to "));
  Serial.println(filename);

}

