/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 04/06/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation -  This program is a portable GPS checker with display and SD card logging. It uses
  an SSD1306 or SH1106 128x64 I2C OLED display. It reads the GPS for 5 seconds and copies the characters
  from the GPS to the serial monitor, this is an example printout from a working GPS that has just been
  powered on;
   
  74_GPS_Checker_With_Display_SDlogger_ESP32 Starting
  Initializing SD card
  Card initialized
  Writing to /LOG0028.TXT
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
  being used. Also set the GPSONSTATE and GPSOFFSTATE to the appropriate logic levels. As well as defining
  the CS pin for the SD card below, you also need to connect the card SCK to pin 18, MISO to pin 19 and
  MOSI to pin 23.

  At startup the program creates a new logfile on the SD card. In the example above the program had found
  log files Log0001.txt to Log0021.txt already existed on the SD card, so created a next in sequence file
  for logging with the name Log0022.txt.

  Removing and re-inserting the SD card while the program is running can stop the program working properly. 
  

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define Program_Version "V1.0"
#define authorname "Stuart Robinson"

#include <SPI.h>
#include <SD.h>

File logFile;
char filename[] = "/LOG0000.TXT";                  //filename used as base for creating logfile, 0000 replaced with numbers 

//pin definitions. You also need to connect the card SCK to pin 18, MISO to pin 19 and MOSI to pin 23
#define LED1 2                                     //pin number for LED
#define SDCS 13                                    //pin number for device select on SD card module

#include <TinyGPS++.h>                             //get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                   //create the TinyGPS++ object

#define GPSserial Serial2                          //define GPSserial as ESP32 Serial2 
#define GPSPOWER -1                                //Pin that controls power to GPS, set to -1 if not used
#define GPSONSTATE HIGH                            //logic level to turn GPS on via pin GPSPOWER 
#define GPSOFFSTATE LOW                            //logic level to turn GPS off via pin GPSPOWER 


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

    printGPSfix();               //print fix data to serial monitor   
    displayscreen1();            //put fix data on display
    digitalWrite(LED1, HIGH);

    Serial.print(F("Logging data to SD - "));
    
    if (!logGPSfix(filename))   //log fix data to SD card   
    {
    Serial.println(F("Failed"));
    cardFail(3);                //logging data to card failed   
    }
    else
    {
    Serial.println(F("OK"));  
    }
    
    digitalWrite(LED1, LOW);
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

  tempfloat = ( (float) GPSHdop / 100);

  Serial.print(F("Time,"));

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

  Serial.print(F(",Lat,"));
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

//*******************************************************************************
// SD card routines
//*******************************************************************************

bool logGPSfix(char *buf )
{
  float tempfloat;

  logFile = SD.open(buf, FILE_APPEND);

  if (!logFile)
  {
  return false;  
  }

  tempfloat = ( (float) GPSHdop / 100);

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
  logFile.print(GPSLat, 6);
  logFile.print(F(","));
  logFile.print(GPSLon, 6);
  logFile.print(F(","));
  logFile.print(GPSAlt, 1);
  logFile.print(F(","));
  logFile.print(GPSSats);
  logFile.print(F(","));
  logFile.print(tempfloat, 2);
  
  logFile.println();
  logFile.close();

  return true;
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

  if (!logFile)
  {
    return 0;
  }

  return index;                                   //return number of logfile created
}


void cardFail(uint8_t num)
{
    Serial.print(num);                            //so we can tell where card failed
    Serial.println(" Card failed, or not present");
    led_Flash(25, 25);
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  unsigned int index;

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
  pinMode(LED1, OUTPUT);                                //for PCB LED
  led_Flash(4, 125);
  
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

  Serial.println(F("74_GPS_Checker_With_Display_SDlogger_ESP32 Starting"));
  Serial.println();

  startGetFixmS = millis();

  Serial.println();
  Serial.println("Initializing SD card");

  if (!SD.begin(SDCS))
  {
    cardFail(1);
  }
  else
  {
  Serial.println("Card initialized");
  }
  
  if (!setupSDLOG(filename))                      //setup logfile name for writing
  {
  cardFail(2);  
  }
  else
  {
  Serial.print(F("Writing to "));
  Serial.println(filename);
  }
  
}
