/*
*****************************************************************************************************************************
LoRaTracker Test Programs

Copyright of the author Stuart Robinson - 10/10/2019

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.
*****************************************************************************************************************************
*/

/*
********************************************************************************************************************************
Program operation - The program flashes a LED connected to the pin defined by LED1, and puts the ESP32 to deep_sleep for a
period determined by TIME_TO_SLEEP (in seconds).

Current in deep_sleep for a bare bones ESp32 with a lora device connected, but powered off was 26.9uA.
********************************************************************************************************************************
*/


#include "ESP32_LoRa_Micro_Node.h"


#define uS_TO_S_FACTOR 1000000              // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  100                   //Time ESP32 will go to sleep (in seconds) 
#define Serial_Monitor_Baud 9600          //this is baud rate used for the Arduino IDE Serial Monitor

                             
RTC_DATA_ATTR int16_t bootCount = 0;
RTC_DATA_ATTR uint16_t sleepcount = 0;

void loop()
{
  Serial.print(F("Bootcount "));
  Serial.println(bootCount);
  Serial.print(F("Sleepcount "));
  Serial.println(sleepcount);
  Serial.println(F("LED Flash"));
  led_Flash(4,125);
  Serial.println(F("LED On"));
  digitalWrite(LED1, HIGH);
  delay(2500);
  Serial.println(F("LED Off"));
  digitalWrite(LED1, LOW);
  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println(F("Start Sleep"));
  Serial.flush();
  sleepcount++;
  esp_deep_sleep_start();
  Serial.println();
  Serial.println();
  Serial.println(F("Awake !"));
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  //flash LED to show tracker is alive
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
  Serial.begin(Serial_Monitor_Baud);          //setup Serial console ouput
  Serial.println(F("ESP32_DeepSleep_Timed_Wakeup - Starting"));

  if(bootCount == 0) //Run this only the first time
  {
      bootCount = bootCount+1;
  }
  
  pinMode(LED1, OUTPUT);                      //for PCB LED                              
  pinMode(GPSPOWER, OUTPUT);                    
  pinMode(VCCPOWER, OUTPUT);
  digitalWrite(GPSPOWER, HIGH);               //GPS off
  digitalWrite(VCCPOWER, HIGH);               //VCCOUT off. lora device off

  pinMode(lora_DIO2, INPUT);                  //if this is not set to an input, sleep current rises by 73uA.   
                          
}

