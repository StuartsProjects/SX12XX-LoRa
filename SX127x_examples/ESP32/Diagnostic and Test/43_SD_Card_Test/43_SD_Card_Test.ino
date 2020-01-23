#define programname "SD_Card_Test_ESP32_Shield"
#define programversion "V1.0"
#define dateproduced "04/05/2018"
#define aurthorname "Stuart Robinson"
#include <Arduino.h>


/*
*****************************************************************************************************************************
LoRaTracker Test Programs for ESP32 Mikrobus Shield

Copyright of the author Stuart Robinson - 04/05/2018

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

This test program has been written to check that a connected SD card adapter, Micro or standard, is funtional. To use 
the program first copy the file (in this programs directory) called testfile.txt to the root directory of the SD card. 

When the progrma runs it will attempt to open 'testfile.txt' and spool the contents to the Arduino IDE serial monitor. 
The testfile is part of the source code for the Apollo 11 Lunar Lander navigation and guidance computer. There are LED
flashes at power up or reset, then at start of every loop of the test. The LED is on whilst the testfile is being read. 
If the LED flashes very rapidly then there is a problem accessin the SD card.
********************************************************************************************************************************
*/

/*
********************************************************************************************************************************
Connections

The program uses the hardware SPI interface on the ESP32 to connect to the SD card module, so the SPI SCK, MOSI and MISO pins are
assumed to be connected. The test program needs a minimum of one extra pin connected to act as chip select. 
You can explicitly define the required pins below by removing the two // characters in front of the #defines 
********************************************************************************************************************************
*/


/*
********************************************************************************************************************************
Notes:

190318 Working, select pin for SD card can be changed.

********************************************************************************************************************************
*/


#include "ESP32_LoRa_Micro_Node.h"    

#include "SD.h"
#include "SPI.h"
  
  

void loop()
{
  Serial.println(F("LED Flash"));
  Serial.println();
  led_Flash(2,50);
  dump_card();
  delay(1000);
}


void dump_card()
{

  Serial.println("Opening File");

  File dataFile = SD.open("/testfile.txt");        //open the test file note that only one file can be open at a time,

  if (dataFile)                                   //if the file is available, read from it
  {
    digitalWrite(LED1, HIGH);
    while (dataFile.available())
    {
      Serial.write(dataFile.read());
    }
    dataFile.close();
    Serial.println("Finished File Dump");
    digitalWrite(LED1, LOW);
  }

  else
  {
    Serial.println("Error opening testfile.txt");   //if the file isn't open print error
    led_Flash(100,25);
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
  pinMode(VCCPOWER, OUTPUT);
  digitalWrite(VCCPOWER, LOW);                         //VCCOUT on, SD card on
  
  pinMode(LED1, OUTPUT);                               //for PCB LED
  led_Flash(4,125);
  
  Serial.begin(115200);
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));;

  Serial.print("Initializing SD card...");

  if (!SD.begin(SDCS)) {
    Serial.println("Card failed, or not present.");
    led_Flash(100,25);
    return;                                              //loop if no card found
  }

  Serial.println("Card initialized.");
  

}


