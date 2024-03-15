/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 23/03/23

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation -  This program is a portable GPS checker. It reads the GPS for 5 seconds and copies
  the characters from the GPS to the serial monitor.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <TinyGPS++.h>                             //get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                   //create the TinyGPS++ object

#define RXpin A3                                   //pin number for GPS RX input into Arduino - TX from GPS
#define TXpin A2                                   //pin number for GPS TX output from Arduino- RX into GPS
#define LED1 8                                     //pin number for LED, turns on when printing Fix data 

#include <SoftwareSerial.h>
SoftwareSerial GPSserial(RXpin, TXpin);


float GPSLat;                                      //Latitude from GPS
float GPSLon;                                      //Longitude from GPS
float GPSAlt;                                      //Altitude from GPS
uint8_t GPSSats;                                   //number of GPS satellites in use
uint32_t GPSHdop;                                  //HDOP from GPS
uint8_t hours, mins, secs, day, month;
uint16_t year;
uint32_t startGetFixmS;
uint32_t endFixmS;

//GPS test co-ordinates to use for distance and direction calculatio
const float TestLatitude  = 51.50807;              //Tower of London
const float TestLongitude  = -0.07606;


void loop()
{
  if (gpsWaitFix(5))
  {
    digitalWrite(LED1, HIGH);                      //LED on to indicate fix
    Serial.println();
    Serial.println();
    Serial.print(F("Fix time "));
    Serial.print(endFixmS - startGetFixmS);
    Serial.println(F("mS"));

    GPSLat = gps.location.lat();
    GPSLon = gps.location.lng();
    GPSAlt = gps.altitude.meters();
    GPSSats = gps.satellites.value();
    GPSHdop = gps.hdop.value();

    hours = gps.time.hour();
    mins = gps.time.minute();
    secs = gps.time.second();
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();

    printGPSfix();
    startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
    digitalWrite(LED1, LOW);
  }
  else
  {
    Serial.println();
    Serial.println();
    Serial.print(F("Timeout - No GPS Fix "));
    Serial.print( (millis() - startGetFixmS) / 1000 );
    Serial.println(F("s"));
  }
}


bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for updated fix

  uint32_t startmS, waitmS;
  uint8_t GPSchar;

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" seconds"));

  waitmS = waitSecs * 1000;                                   //convert seconds wait into mS

  startmS = millis();

  while ( (uint32_t) (millis() - startmS) < waitmS)           //allows for millis() overflow
  {
    if (GPSserial.available() > 0)
    {
      GPSchar = GPSserial.read();
      gps.encode(GPSchar);
      Serial.write(GPSchar);
    }

    if (gps.speed.isUpdated() && gps.satellites.isUpdated()) //ensures that GGA and RMC sentences have been received
    {
      endFixmS = millis();                                   //record the time when we got a GPS fix
      return true;
    }
  }
  return false;
}



void printGPSfix()
{
  float tempfloat;
  uint32_t distance;
  uint16_t direction;

  Serial.print(F("New GPS Fix "));

  tempfloat = ( (float) GPSHdop / 100);

  Serial.print(F("Lat,"));
  Serial.print(GPSLat, 6);
  Serial.print(F(",Lon,"));
  Serial.print(GPSLon, 6);
  Serial.print(F(",Alt,"));
  Serial.print(GPSAlt, 1);
  Serial.print(F("m,Sats,"));
  Serial.print(GPSSats);
  Serial.print(F(",HDOP,"));
  Serial.print(tempfloat, 2);
  Serial.print(F(",Time,"));

  if (hours < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(hours);
  Serial.print(F(":"));

  if (mins < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(mins);
  Serial.print(F(":"));

  if (secs < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(secs);
  Serial.print(F(",Date,"));

  Serial.print(day);
  Serial.print(F("/"));
  Serial.print(month);
  Serial.print(F("/"));
  Serial.print(year);

  distance = gps.distanceBetween(GPSLat, GPSLon, TestLatitude, TestLongitude);
  direction = gps.courseTo(GPSLat, GPSLon, TestLatitude, TestLongitude);

  Serial.println();
  Serial.print(F("Distance to Test Location ("));
  Serial.print(TestLatitude, 6);
  Serial.print((","));
  Serial.print(TestLongitude, 6);
  Serial.print((") "));
  Serial.print(distance);
  Serial.print(("m"));
  Serial.println();
  Serial.print(F("Direction to Test Location ("));
  Serial.print(TestLatitude, 6);
  Serial.print((","));
  Serial.print(TestLongitude, 6);
  Serial.print((") "));
  Serial.print(direction);
  Serial.print(("d"));
  Serial.println();
  Serial.println();
}


void setup()
{
  pinMode(LED1, OUTPUT);

  GPSserial.begin(9600);

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("28_GPS_Checker Starting"));
  Serial.println();

  startGetFixmS = millis();
}
