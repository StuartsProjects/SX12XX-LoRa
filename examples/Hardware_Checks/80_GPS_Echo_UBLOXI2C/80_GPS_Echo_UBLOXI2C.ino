/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 27/06/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - The purpose of this program is to check that an I2C connected UBLOX GPS is working.
  The program reads characters from the GPS over its I2C interface and sends them (echoes) to the IDE
  serial monitor.

  Serial monitor baud rate is set at 115200.

*******************************************************************************************************/

#include <Wire.h>

#define LED1 8                                   //pin for LED    

#define GPSI2CAddress 0x42                       //I2C address of GPS


void loop()
{
  uint8_t GPSchar;

  Wire.beginTransmission(GPSI2CAddress);
  Wire.write(0xFF);                                        //read data stream register

  while (1)
  {
    Wire.requestFrom(GPSI2CAddress, 1);
    GPSchar = Wire.read();
    if (GPSchar != 0xFF)                                   //if returned character is 0xFF, I2C buffer is empty
    {
      Serial.write(GPSchar);
    }
  }
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
  pinMode(LED1, OUTPUT);
  led_Flash(2, 500);

  Serial.begin(115200);
  Serial.println(F("80_GPS_Echo_UBLOXI2C Starting"));

  Wire.begin();
}


