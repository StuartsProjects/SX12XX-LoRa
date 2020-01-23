/*
*****************************************************************************************************************************
LoRaTracker Test Programs

Copyright of the author Stuart Robinson - 06/11/2019

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.
*****************************************************************************************************************************
*/

/*
********************************************************************************************************************************
Program operation

The program reads a single DS18B20 temperature sensor and prints the result to the serial monitor.

********************************************************************************************************************************
*/

#define programversion "V1.0"
#include <Arduino.h>

#define ONE_WIRE_BUS 33                           //pin the DS18b20 is connected to
#define LED1         2                            //LED, on when reading temperature  
#define VCCPOWER 14                               //controls power to external devices 

#include <OneWire.h>
OneWire oneWire(ONE_WIRE_BUS);
#include <DallasTemperature.h>
DallasTemperature sensor(&oneWire);


void loop()
{
float DS18B20temperature;

digitalWrite(LED1, HIGH);
sensor.requestTemperatures();
digitalWrite(LED1, LOW);

DS18B20temperature = sensor.getTempCByIndex(0);
Serial.print(F("DS18B20 Temperature "));
Serial.print(DS18B20temperature,2);
Serial.println(F("c"));
delay(5000);
}



void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, LOW);
    delay(delaymS);
    digitalWrite(LED1, HIGH);
    delay(delaymS);
  }
}


void setup()
{
  Serial.begin(9600);              
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println();
  Serial.println("DS18B20_Test Starting");

  pinMode(LED1, OUTPUT);                   
  pinMode(VCCPOWER, OUTPUT);
  digitalWrite(VCCPOWER, LOW);               //VCCOUT on, DS18B20 on
  
  
  led_Flash(4,125);                           //one second of flashes
 
  sensor.begin();
  sensor.requestTemperatures();               //do a null temperature read
}



