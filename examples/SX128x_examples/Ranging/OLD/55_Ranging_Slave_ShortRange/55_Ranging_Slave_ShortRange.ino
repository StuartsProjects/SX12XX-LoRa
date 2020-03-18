/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 13/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

#define programversion "V1.0"

#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"

SX128XLT LT;

uint32_t endwaitmS;
uint16_t timeoutErrors;
uint16_t IrqStatus;
uint32_t sentOKcount;

void loop()
{
  Serial.print(sentOKcount);
  //LT.resetDevice();
  //LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, 0, RANGING_SLAVE);
  if (LT.receiveRanging(ranging_address, 10000, TXpower, WAIT_RX))
  {
    //have received a RANGING_SLAVE_REQUEST_VALID interrupt
    sentOKcount++;
    Serial.println(F(" Ranging packet sent"));;
    led_Flash(1, 10);                                 //single flash to indicate ranging reply
  }
  else
  {
    Serial.print(F(" Error"));
    LT.printIrqStatus();
    Serial.println();
    led_Flash(2, 50);                                 //double flash to indicate ranging error
  }

  delay(100);
}

void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  //flash LED to show board is alive
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
  Serial.begin(9600);                         //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println(F("Stuart Robinson"));
  Serial.println();

  Serial.println("55_Ranging_Slave Starting");

  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);

  SPI.begin();

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("Device found"));
    led_Flash(2, 125);
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                            //long fast speed flash indicates device error
    }
  }

  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Calibration, RANGING_SLAVE);
  
  Serial.print(F("Address  "));
  Serial.println(ranging_address);
  Serial.println(F("Ranging slave ready"));
}

