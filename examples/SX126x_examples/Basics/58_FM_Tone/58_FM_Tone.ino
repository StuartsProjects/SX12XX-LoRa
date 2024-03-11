/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 23/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors. 
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - Transmits a FM tone using the LoRa device that can be picked up on an FM UHF 
  handheld receiver. The tones are not true FM but the UHF receiver does not know that. 
  
  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

#include <SPI.h>                                               //the lora device is SPI based so load the SPI library                                         
#include <SX126XLT.h>                                          //include the appropriate library  
#include "Settings.h"                                          //include the setiings file, frequencies, LoRa settings etc   

SX126XLT LT;                                                   //create a library class instance called LT


void loop()
{
  Serial.print(TXpower);                                       //print the transmit power defined
  Serial.print(F("dBm "));
  Serial.println(F("PlayTone> "));
  Serial.println();

  digitalWrite(LED1, HIGH);
  LT.toneFM(1000, 1000, deviation, adjustfreq, TXpower);  
  digitalWrite(LED1, LOW);
  
  delay(1000);
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
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
  pinMode(LED1, OUTPUT);                                   //setup pin as output for indicator LED
  led_Flash(2, 125);                                       //two quick LED flashes to indicate program start
  pinMode(4, OUTPUT);
  
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("58_FM_Tone Starting"));

  SPI.begin();

  //SPI beginTranscation is normally part of library routines, but if it is disabled in library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);                                   //two further quick LED flashes to indicate device found
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                 //long fast speed LED flash indicates device error
    }
  }
  
  LT.setupDirect(Frequency, Offset);
    
  Serial.print(F("Tone Transmitter ready"));
  Serial.println();
}
