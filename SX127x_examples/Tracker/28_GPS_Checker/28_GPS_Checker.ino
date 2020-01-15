/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 17/12/19

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation -  This program is a GPS checker. At startup the program starts checking the data
  coming from the GPS for a valid fix. The GPS data is printed to the Serial monitor. When the program
  detects that the GPS has a fix, it prints the GPS fix time, Latitude, Longitude, Altitude, Number of
  satellites in use and the HDOP value to the serial monitor.

  Serial monitor baud rate is set at 115200, GPS baud rate to 9600, both are configured in setup().
*******************************************************************************************************/

#include <TinyGPS++.h>                            //get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                  //create the TinyGPS++ object

#define RXpin A3                                  //pin number for GPS RX input into Arduino - TX from GPS
#define TXpin A2                                  //pin number for GPS TX output from Arduino- RX into GPS

#include <SoftwareSerial.h>
SoftwareSerial GPSserial(RXpin, TXpin);

float GPSLat;                                     //Latitude from GPS
float GPSLon;                                     //Longitude from GPS
float GPSAlt;                                     //Altitude from GPS
uint8_t GPSSats;                                  //number of GPS satellites in use
uint32_t GPSHdop;                                 //HDOP from GPS
uint32_t GPSstarttimemS;                          //time in mS when GPS is scanned fo fix
uint32_t GPSendtimemS;                            //time in mS when GPS got a fix


void loop()
{
  gpsWaitFix();
  displayGPSfix();
}


void gpsWaitFix()
{
  //waits till the GPS gets an updated fix

  uint8_t GPSchar;

  Serial.println(F("Wait for updated GPS Fix"));
  GPSstarttimemS = millis();

  while (1)
  {
    if (GPSserial.available() > 0)
    {
      GPSchar = GPSserial.read();
      gps.encode(GPSchar);
      Serial.write(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      GPSendtimemS = millis();
      break;
    }
  }
}


void displayGPSfix()
{
  float tempfloat;

  Serial.println();
  Serial.println();
  Serial.print(F("New GPS Fix "));
  Serial.print(GPSendtimemS - GPSstarttimemS);
  Serial.print(F("mS  "));

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
  Serial.print(GPSAlt, 1);
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

  Serial.println(F("28_GPS_Checker Starting"));
  Serial.println();
}
