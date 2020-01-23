/*
*****************************************************************************************************************************
  LoRaTracker TTN Tracker program

  Copyright of the author Stuart Robinson - 05/10/2019

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit
  permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose
  and free from errors.
*****************************************************************************************************************************
*/

/*
********************************************************************************************************************************
  Program operation

  At reset the program reads the attched GPS and waits for a fix. When a fix is obtained, the display is updated with the
  location and the packet sent to TTN using TinyLoRa. The payload count is saved in EEPROM and read each time the tracker
  starts. After a short delay the GPS and display are turned off and the board goes into deep sleep. To re-run the tracker
  press reset. A DS18B20 one wire temperature sensor is read on pin ONE-WIRE-BUS.

  Deep sleep current measured at 13uA with the GPS in backup mode.

  The LED is on whilst the program waits for the GPS to get a fix. 

  The circuit requires a GPS that is powered off via a logic low on the GPSPOWER pin.

  The LoRa device is checked at startup, if its not found there will be a message on the display and the LED will
  flash very quickly. 

  The pin definitions for the board you are using are defined in the file "Arduino_Pin_Definitions.h"

********************************************************************************************************************************
*/

/*
*************************************************************************************************************
Sketch size with all working and deep sleep
Sketch uses 247656 bytes (18%) of program storage space. Maximum is 1310720 bytes.
Global variables use 16640 bytes (5%) of dynamic memory, leaving 311040 bytes for local variables. Maximum is 327680 bytes.
*************************************************************************************************************
*/


//Include a file here that has the pin definitions defined ........
#include "ESP32_LoRa_Micro_Node.h"                 //Arduino pin definitions for board in use

#include "I2CFRAM_MB85RC16PNF.h"                     
#include <Wire.h>
const uint16_t Counter_Address = 0x0000;            //location in FRAM where Frame counter is stored
//#define CLEARRFRAMECOUNTER                        //the frame counter is saved in EEPROM, enable this define to set to zero


#include <TinyLoRa2.h>                             //https://github.com/adafruit/TinyLoRa  
#include <SPI.h>
TinyLoRa lora = TinyLoRa(lora_DIO0, lora_NSS, lora_NReset);

#define DATARATE SF7BW125                         //Set spreading factor and bandwidth, choices are;
//SF7BW125, SF7BW250, SF8BW125, SF9BW125, SF10BW125, SF10BW125
//SF11BW125, SF12BW125

#include <TinyGPS++.h>                            //https://github.com/mikalhart/TinyGPSPlus
TinyGPSPlus gps;                                  //define GPS instance
#define GPSserial Serial2                           
static const uint16_t GPSBaud = 9600;             //GPS baud rate

#include <OneWire.h>                              //https://github.com/PaulStoffregen/OneWire
OneWire oneWire(ONE_WIRE_BUS);                    //define onwire instance  
#include <DallasTemperature.h>                    //https://github.com/milesburton/Arduino-Temperature-Control-Library
DallasTemperature sensor(&oneWire);               //define Dallas instance 



#include <U8x8lib.h>                                      //https://github.com/olikraus/u8g2 
//U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);    //standard 0.96" SSD1306
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);   //1.3" OLED often sold as 1.3" SSD1306

#define LPP_GPS 136                                //CayenneLPP data type for GPS   
#define LPP_TEMPERATURE 103                        //CayenneLPP data type temperature    

uint8_t loraData[15];                              //this is the buffer where the payload is built for sending
uint16_t stored_frame_count;                       //the TTN frame count is stored in EEPROM
float TRLat, TRLon, TRAlt, TRhdopGPS, celsius;;    //these are the variables for storing lat,lon,alt and temperature.
float GPSfixtime;
uint8_t hours, minutes, seconds;


//***********************************************************************
//Keys: Application: stuarts_ttn_mapper  Device: ttn_mapper_1
//***********************************************************************
// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x82, 0x6E, 0x06, 0xDE, 0x17, 0xE9, 0x92, 0x43, 0xAA, 0xF0, 0xEB, 0x7E, 0xF7, 0x0B, 0x84, 0x57 };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x23, 0x0A, 0x63, 0x3C, 0x88, 0xB5, 0xDA, 0x86, 0x3F, 0xC6, 0x51, 0x2D, 0xB2, 0xF1, 0x58, 0xCE };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x26, 0x01, 0x10, 0xF6 };

#define Serial_Monitor_Baud 115200                 //this is the serial monitor baud rate 


#define uS_TO_S_FACTOR 1000000              // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  86400                //Time ESP32 will go to sleep (in seconds) 
#define Serial_Monitor_Baud 115200          //this is baud rate used for the Arduino IDE Serial Monitor
                             
RTC_DATA_ATTR int16_t bootCount = 0;
RTC_DATA_ATTR uint16_t sleepcount = 0;

void loop()
{
  Serial.println(F("Wait GPS fix"));
  
  u8x8.clear();                                    //clear the display   
  u8x8.setCursor(0, 0);
  u8x8.println(F("Wait GPS fix"));

  build_payload();                                 //fill buffer with GPS data

  Serial.println(F("Payload ready to send"));

  delay(1000);
  
  screen1();

  stored_frame_count = Memory_ReadUInt(Counter_Address);
  Serial.print(F("Frame sent currently "));
  Serial.println(stored_frame_count);
  stored_frame_count++;                              //increment frame counter ready for next send

  Serial.println(F("Sending payload..."));

  lora.sendData(loraData, sizeof(loraData), stored_frame_count);
  Memory_WriteUInt(Counter_Address, stored_frame_count);
   
  Serial.print(F("Sent frames now "));
  Serial.println(stored_frame_count);
  u8x8.setCursor(0, 6);
  u8x8.print(F("Sent frames "));
  u8x8.println(stored_frame_count);

  delay(10000);                                      //leave display up for a while

  u8x8.setPowerSave(1);                              //power save display
  GPSserial.end();                                   //stop GPS interrupts to prevent power up
  digitalWrite(GPSPOWER, HIGH);                      //turn off GPS power
  digitalWrite(VCCPOWER, HIGH);                      //turn off VCCOUT 
  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println(F("Start Sleep"));
  Serial.flush();
  sleepcount++;
  esp_deep_sleep_start();
  Serial.println();
  Serial.println();
  Serial.println(F("Awake !"));
}


float gpsWaitFix()
{
  uint8_t character;
  uint32_t startfixms;
  float temp;

  while (1)
  {
    if (GPSserial.available() > 0)
    {
      character = GPSserial.read();
      gps.encode(character);
      Serial.write(character);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      temp = (millis() - startfixms);
      return (temp / 1000);
    }
  }
  GPSserial.flush();                             //clear out the receive buffer
}


void build_payload ()
{
  uint16_t temp;

  digitalWrite(LED1, HIGH);                            //LED on to indicate start of GPS check

  Serial.println();
  Serial.println(F("Wait for GPS fix"));
  GPSfixtime = gpsWaitFix();
  digitalWrite(LED1, LOW);                             //finished with GPS, turn LED off
  Serial.println();
  Serial.println(F("Have GPS fix"));

  TRLat = gps.location.lat();
  TRLon = gps.location.lng();
  TRAlt = gps.altitude.meters();
 
  hours = gps.time.hour();
  minutes = gps.time.minute();
  seconds = gps.time.second();

  Serial.println();
  Serial.print(F("Latitude "));
  Serial.println(TRLat, 5);
  Serial.print(F("Longitude "));
  Serial.println(TRLon, 5);
  Serial.print(F("Altitude "));
  Serial.print(TRAlt, 1);
  Serial.println(F("m"));
  Serial.print(F("GPS fix time "));
  Serial.print(GPSfixtime);
  Serial.println(F("s"));
  if (hours < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(hours);
  Serial.print(F(":"));

  if (minutes < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(minutes);
  Serial.print(F(":"));

  if (seconds < 10)
  {
    Serial.print(F("0"));
  }
  Serial.println(seconds);


  //convert position information for LoRaWAN transmit
  int32_t lat = TRLat * 10000;
  int32_t lon = TRLon * 10000;
  int32_t altitudeGPS = TRAlt * 100;

  sensor.requestTemperatures();              //read the DS18B20
  celsius = sensor.getTempCByIndex(0);
  Serial.print(F("Temperature "));
  Serial.print(celsius, 2);
  Serial.println(F("c"));
  temp = (celsius * 10);

  //channel 1                                //device 1 is GPS
  loraData[0] = 0x01;
  loraData[1] = LPP_GPS;
  loraData[2] = lat >> 16;
  loraData[3] = lat >> 8;
  loraData[4] = lat;
  loraData[5] = lon >> 16;
  loraData[6] = lon >> 8;
  loraData[7] = lon;
  loraData[8] = altitudeGPS >> 16;
  loraData[9] = altitudeGPS >> 8;
  loraData[10] = altitudeGPS;

  //channel 2                                 //device 2 is tewmperature sensor  
  loraData[11] = 0x02;
  loraData[12] = LPP_TEMPERATURE;
  loraData[13] = temp >> 8;
  loraData[14] = temp;
}


void screen1()
{
  //show payload data on display
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print(F("Lat: "));
  u8x8.print(TRLat, 5);
  u8x8.setCursor(0, 1);
  u8x8.print(F("Lon: "));
  u8x8.print(TRLon, 5);
  u8x8.setCursor(0, 2);
  u8x8.print(F("Alt: "));
  u8x8.print(TRAlt, 0);
  u8x8.print(F("m"));
  u8x8.setCursor(0, 3);
  u8x8.print(F("Temp: "));
  u8x8.print(celsius, 1);
  u8x8.print(F("c"));
  u8x8.setCursor(0, 4);

  //and show the time as well
  
  hours = gps.time.hour();
  if (hours < 10)
  {
    u8x8.print(F("0"));
  }
  u8x8.print(hours);
  u8x8.print(F(":"));

  minutes = gps.time.minute();
  if (minutes < 10)
  {
    u8x8.print(F("0"));
  }
  u8x8.print(minutes);
  u8x8.print(F(":"));

  seconds = gps.time.second();
  if (seconds < 10)
  {
    u8x8.print(F("0"));
  }
  u8x8.print(seconds);

  u8x8.setCursor(0, 5);
  u8x8.print(F("Fix time "));
  u8x8.print(GPSfixtime, 2);
  u8x8.print(F("s"));
}



void setup()
{
  pinMode(LED1, OUTPUT);
  pinMode(VCCPOWER, OUTPUT);
  digitalWrite(VCCPOWER, LOW);                        //turn on devices                        
  pinMode(GPSPOWER, OUTPUT);
  digitalWrite(GPSPOWER, LOW);                        //turn on GPS 
  
  GPSserial.begin(GPSBaud, SERIAL_8N1, GPSTX, GPSRX);           //this works but full format not needed
                                                             //format is baud, mode, UART RX data, UART TX data

  Serial.begin(Serial_Monitor_Baud);
  Serial.println(F("16_TTN_GPS_Tracker_SX1276 Starting"));
  Serial.println();
  
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  Serial.print(F("Checking LoRa device - "));         //Initialize LoRa
  u8x8.setCursor(0, 0);
  u8x8.print(F("Check LoRa "));                       //check LoRa device

  lora.setChannel(MULTI);                             //define multi-channel sending
  lora.setDatarate(DATARATE);                         //set datarate
  
  if (!lora.begin())
  {
    u8x8.print(F("fail"));
    Serial.println(F("fail"));
    while (true)
    {
      digitalWrite(LED1, HIGH);
      delay(50);
      digitalWrite(LED1, LOW);
      delay(50);
    }
  }

  u8x8.print(F("OK"));
  Serial.println(F("OK"));

  sensor.begin();
  sensor.requestTemperatures();                          //do an initial dummy read of temperature sensor
  celsius = sensor.getTempCByIndex(0);

#ifdef CLEARRFRAMECOUNTER
  Memory_WriteUInt(Counter_Address, 0);                  //set  the stored frame counter to 1
  Serial.println(F("Frame counter is cleared"));
#endif

  Wire.begin(SDA,SCL);                                   //start I2C interface

  delay(1500);
}


