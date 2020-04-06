/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 05/04/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation -  This program is a portable GPS checker with display option. It uses an SSD1306 or
  SH1106 128x64 I2C OLED display. It reads the GPS for 5 seconds and copies the characters from the GPS
  to the serial monitor, this is an example printout from a working GPS that has just been powered on;
   
  28_GPS_Checker Starting
  Wait GPS Fix 5 seconds
  Timeout - No GPS Fix 5s
  Wait GPS Fix 5 seconds
  $PGACK,103*40
  $PGACK,105*46
  $PMTK011,MTKGPS*08
  $PMTK010,001*2E
  $PMTK010,00æ*2D
  $GPGGA,235942.800,,,,,0,0,,,M,,M,,*4B
  $GPGSA,A,1,,,,,,,,,,,,,,,*1E
  $GPRMC,235942.800,V,,,,,0.00,0.00,050180,,,N*42
  $GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32
  $GPGSV,1,1,03,30,,,43,07,,,43,05,,,38*70

  Timeout - No GPS Fix 5s
  Wait GPS Fix 5 seconds

  That printout is from a Meadiatek GPS, the Ublox ones are similar. The data from the GPS is also fed into
  the TinyGPS++ library and if there is no fix a message is printed on the serial monitor.

  When the program detects that the GPS has a fix, it prints the Latitude, Longitude, Altitude, Number
  of satellites in use, the HDOP value, time and date to the serial monitor. If the I2C OLED display is
  attached that is updated as well. Display is assumed to be on I2C address 0x3C.

  The program has the option of using a pin to control the power to the GPS, if the GPS module being used
  has this feature. To use the option change the define; '#define GPSPOWER -1' from -1 to the pin number
  being used. Also set the GPSONSTATE and GPSOFFSTATE to the appropriate logic levels.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/



/*******************************************************************************************************
  Program Operation -  This program is a portable GPS checker and display. It uses an SSD1306 or SH1106
  128x64 I2C OLED display. At startup the program starts checking the data coming from the GPS for a
  valid fix. It reads the GPS for 5 seconds and if there is no fix, prints a message on the serial monitor
  and updates the seconds without a fix on the display. During this time the data coming from the GPS is
  copied to the serial monitor also.

  When the program detects that the GPS has a fix, it prints the Latitude, Longitude, Altitude, Number
  of satellites in use, the HDOP value, time and date to the serial monitor. If the I2C OLED display is
  attached that is updated as well. Display is assumed to be on I2C address 0x3C.

  The program has the option of using a pin to control the power to the GPS, if the GPS module being used
  has this feature. To use the option change the define; '#define GPSPOWER -1' from -1 to the pin number
  being used. Also set the GPSONSTATE and GPSOFFSTATE to the appropriate logic levels.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define Program_Version "V1.1"
#define authorname "Stuart Robinson"

#include <TinyGPS++.h>                             //get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                   //create the TinyGPS++ object

#define RXpin A3                                   //pin number for GPS RX input into Arduino - TX from GPS
#define TXpin A2                                   //pin number for GPS TX output from Arduino- RX into GPS

#define GPSPOWER -1                                //Pin that controls power to GPS, set to -1 if not used
#define GPSONSTATE HIGH                            //logic level to turn GPS on via pin GPSPOWER 
#define GPSOFFSTATE LOW                            //logic level to turn GPS off via pin GPSPOWER 

#include <SoftwareSerial.h>
SoftwareSerial GPSserial(RXpin, TXpin);


#include <U8x8lib.h>                                      //get library here >  https://github.com/olikraus/u8g2 
U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    //use this line for standard 0.96" SSD1306
//U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);   //use this line for 1.3" OLED often sold as 1.3" SSD1306


float GPSLat;                                      //Latitude from GPS
float GPSLon;                                      //Longitude from GPS
float GPSAlt;                                      //Altitude from GPS
uint8_t GPSSats;                                   //number of GPS satellites in use
uint32_t GPSHdop;                                  //HDOP from GPS
uint8_t hours, mins, secs, day, month;
uint16_t year;
uint32_t startGetFixmS;
uint32_t endFixmS;

void loop()
{
  if (gpsWaitFix(5))
  {
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
    displayscreen1();
    startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
  }
  else
  {
    disp.clearLine(0);
    disp.setCursor(0, 0);
    disp.print(F("No GPS Fix "));
    disp.print( (millis() - startGetFixmS) / 1000 );
    Serial.println();
    Serial.println();
    Serial.print(F("Timeout - No GPS Fix "));
    Serial.print( (millis() - startGetFixmS) / 1000 );
    Serial.println(F("s"));
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

    if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.date.isUpdated())
    {
      endFixmS = millis();                                //record the time when we got a GPS fix
      return true;
    }
  }

  return false;
}


void printGPSfix()
{
  float tempfloat;

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

  Serial.println();
  Serial.println();
}


void displayscreen1()
{
  //show GPS data on display
  float tempfloat;
  tempfloat = ( (float) GPSHdop / 100);

  disp.clearLine(0);
  disp.clearLine(1);
  disp.setCursor(0, 1);
  disp.print(GPSLat, 6);
  disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(GPSLon, 6);
  disp.clearLine(3);
  disp.setCursor(0, 3);
  disp.print(GPSAlt);
  disp.print(F("m"));
  disp.clearLine(4);
  disp.setCursor(0, 4);
  disp.print(F("Sats "));
  disp.print(GPSSats);
  disp.clearLine(5);
  disp.setCursor(0, 5);
  disp.print(F("HDOP "));
  disp.print(tempfloat);

  disp.clearLine(6);
  disp.setCursor(0, 6);

  if (hours < 10)
  {
    disp.print(F("0"));
  }

  disp.print(hours);
  disp.print(F(":"));

  if (mins < 10)
  {
    disp.print(F("0"));
  }

  disp.print(mins);
  disp.print(F(":"));

  if (secs < 10)
  {
    disp.print(F("0"));
  }

  disp.print(secs);
  disp.print(F("  "));

  disp.clearLine(7);
  disp.setCursor(0, 7);

  disp.print(day);
  disp.print(F("/"));
  disp.print(month);
  disp.print(F("/"));
  disp.print(year);
}


void GPSON()
{
  if (GPSPOWER)
  {
    digitalWrite(GPSPOWER, GPSONSTATE);                         //power up GPS
  }
}


void GPSOFF()
{
  if (GPSPOWER)
  {
    digitalWrite(GPSPOWER, GPSOFFSTATE);                        //power off GPS
  }
}


void setup()
{
  if (GPSPOWER >= 0)
  {
    pinMode(GPSPOWER, OUTPUT);
    GPSON();
  }

  GPSserial.begin(9600);

  Serial.begin(115200);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();

  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("Display Ready"));

  Serial.println(F("29_GPS_Checker_Display Starting"));
  Serial.println();

  startGetFixmS = millis();
}
