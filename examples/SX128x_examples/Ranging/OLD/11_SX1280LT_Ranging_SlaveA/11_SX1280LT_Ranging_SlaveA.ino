/*
******************************************************************************************************

  Copyright of the author Stuart Robinson 24/10/19

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  Changes:

  To Do:

******************************************************************************************************
*/


#define programversion "V1.0"
#define Serial_Monitor_Baud 9600


#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"

SX128XLT LT;

uint32_t endwaitmS;
uint16_t timeoutErrors;
uint16_t IrqStatus;
uint32_t response_sent;

void loop()
{
  LT.setMode(MODE_STDBY_RC);
  LT.setRx(PERIOBASE_01_MS, 0);                             //this waits for the the packet to be received

  endwaitmS = millis() + rangingRXTimeoutmS;

  while (!digitalRead(DIO1) && (millis() <= endwaitmS));          //wait for Ranging valid or timeout

  if (millis() >= endwaitmS)
  {
    Serial.println("Error - Ranging Receive Timeout!!");
    led_Flash(2, 100);                                             //single flash to indicate timeout
  }


  else
  {
    IrqStatus = LT.readIrqStatus();
    digitalWrite(LED1, HIGH);
    digitalWrite(BUZZER, HIGH);

    if (IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
    {
      response_sent++;
      Serial.print(response_sent);
      Serial.print(" Response sent");
    }
    else
    {
      Serial.print("Slave error,");
      Serial.print(",Irq,");
      Serial.print(IrqStatus, HEX);
      LT.printIrqStatus();
    }
    digitalWrite(LED1, LOW);
    digitalWrite(BUZZER, LOW);
    Serial.println();
  }

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


void setup_RangingRX()
{
  uint8_t tempreg;
  LT.setMode(MODE_STDBY_RC);
  LT.setPacketType(PACKET_TYPE_RANGING);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  LT.setRfFrequency(Frequency, Offset);
  LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
  tempreg = LT.readRegister(REG_LR_RANGINGIDCHECKLENGTH);
  LT.writeRegister(REG_LR_RANGINGIDCHECKLENGTH, (0xC0 & tempreg)); //check low 8 bits of range address only
  LT.setRangingSlaveAddress(RXaddress);
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RANGING_SLAVE_RESPONSE_DONE + IRQ_RANGING_SLAVE_REQUEST_DISCARDED), 0, 0);              //set for IRQ on RX done
  LT.setRangingCalibration(CalibrationSF8BW800);
  LT.setRangingRole(RANGING_SLAVE);
  LT.setHighSensitivity();                    //set high sensitivity RX mode
}


void setup()
{
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
  delay(2000);                                  //allow time for openlog to start

  Serial.begin(Serial_Monitor_Baud);            //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println(F("Stuart Robinson"));
  Serial.println();

  Serial.println("11_LT Ranging Slave Starting");

  led_Flash(2, 125);

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("Device found"));
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                            //long fast speed flash indicates device error
    }
  }

  setup_RangingRX();
  Serial.println("Ranging Listen");
}

