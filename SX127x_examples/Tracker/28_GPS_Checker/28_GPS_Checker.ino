/*******************************************************************************************************
  LoRaTracker Programs for Arduino - Copyright of the author Stuart Robinson - 17/12/19

  http://www.LoRaTracker.uk

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation -  This program is a GPS checker. At startup the program starts checking the data 
  coming from the GPS for a valid fix. It checks for 5 seconds and if there is no fix, prints a message
  on the serial monitor. During this time the data coming from the GPS is copied to the serial monitor also. 
  
  When the program detects that the GPS has a fix, it prints the Latitude, Longitude, Altitude, Number
  of satellites in use and the HDOP value to the serial monitor.
  
  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define Program_Version "V1.0"
#define authorname "Stuart Robinson"

#include <Arduino.h>

#include <TinyGPS++.h>                             //get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                   //create the TinyGPS++ object

#define RXpin A3                                   //pin number for GPS RX input into Arduino - TX from GPS
#define TXpin A2                                   //pin number for GPS TX output from Arduino- RX into GPS
#define GPSON 4

#include <SoftwareSerial.h>                        
SoftwareSerial GPSserial(RXpin, TXpin);            

float GPSLat;                                      //Latitude from GPS
float GPSLon;                                      //Longitude from GPS
float GPSAlt;                                      //Altitude from GPS
uint8_t GPSSats;                                   //number of GPS satellites in use
uint32_t GPSHdop;                                  //HDOP from GPS


void loop()
{
  if (gpsWaitFix(5))
  {
  displayGPSfix(); 
  }
  else
  {
  Serial.println();
  Serial.println();
  Serial.println(F("Timeout - No GPS Fix")); 
  Serial.println();
  }
}


bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for good fix
  
  uint32_t endwaitmS;
  uint8_t GPSchar;

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" seconds"));

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS)
  {
    if (GPSserial.available() > 0)
    {
      GPSchar = GPSserial.read();
      gps.encode(GPSchar);
      Serial.write(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
    return true;  
    }
  }

  return false;
}


void displayGPSfix()
{
      float tempfloat;

      Serial.println();
      Serial.println();
      Serial.print(F("New GPS Fix "));

      GPSLat = gps.location.lat();
      GPSLon = gps.location.lng();
      GPSAlt = gps.altitude.meters();
      GPSSats = gps.satellites.value();
      GPSHdop = gps.hdop.value();
      tempfloat = ( (float) GPSHdop / 100);

      Serial.print(F("Lat,"));
      Serial.print(GPSLat, 6);
      Serial.print(F(",Lon,"));
      Serial.print(GPSLon, 6);
      Serial.print(F(",Alt,"));
      Serial.print(GPSAlt,1);
      Serial.print(F("m,Sats,"));
      Serial.print(GPSSats);
      Serial.print(F(",HDOP,"));
      Serial.print(tempfloat, 2);
      
      Serial.println();
      Serial.println();
}


void setup()
{
  GPSserial.begin(9600);

  Serial.begin(115200);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();

  Serial.println(F("28_GPS_Checker Starting"));
  Serial.println();
}
