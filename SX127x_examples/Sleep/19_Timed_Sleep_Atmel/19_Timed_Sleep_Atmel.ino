/*******************************************************************************************************
  LoRaTracker Programs for Arduino - Copyright of the author Stuart Robinson - 07/12/19

  http://www.LoRaTracker.uk

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This program is used to check the amount of time  the processor is awake during a loop which
  uses the Atmel sleep timer. The Atmel watchdog timer is limited to a maximum of 8 seconds (approx) sleep time, 
  so to sleep for longer you need to use the watchdog timer in a loop, so performing a number of sleeps to achieve
  the desired sleep time.

  In this example the sleep timer is set for 8 seconds and the program calls the funtion to sleep for x units
  of eight seconds. An LED is switched off a the begining of the LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  command and back on again when the processor wakes up. Thus the LED is powered on for the active part
  of the for loop, when the processor is actually running. This time was checked with a scope and the active time
  of the for loop was 7.8uS. 
  
  The time the processor takes to wake and start running the loop code run the code is a bit harder to measure,
  but I see a power pulses of circa 2.4mS when monitoring the supply rail with a resistor. So assuming a powered
  time of 2.4mS every 8 seconds (1/3333th of the time) and a processor run current of maybe 6mA, thats an average
  of 1.8uA. Not bad for a freebie sleep timer.
  
  Tested on an ATmega328P board.

  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

#include <avr/wdt.h>                            //watchdog timer library, integral to Arduino IDE
#include <LowPower.h>                           //get the library here; https://github.com/rocketscream/Low-Power

#define LED1 A3


void loop()
{
  Serial.println(F("Sleeping zzzzz...."));
  Serial.flush();

  sleep_timed(120);                             //goto sleep for 15 seconds

  Serial.println(F("Awake !"));
  Serial.flush();
}


void sleep_timed(uint32_t seconds)
{
  uint32_t index;

  for (index = 1; index <= seconds; index++)
  {
    digitalWrite(LED1, LOW);
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);         //sleep 1 seconds
    //LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);         //sleep 8 seconds
    digitalWrite(LED1, HIGH);
  }
}


void setup()
{
  pinMode(LED1, OUTPUT);                                     //setup pin as output for indicator LED
  Serial.begin(9600);
}

