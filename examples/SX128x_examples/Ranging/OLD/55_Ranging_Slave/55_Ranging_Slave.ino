/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 18/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - 

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

#define programversion "V1.0"

#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"

SX128XLT LT;

uint32_t endwaitmS;
uint16_t rangeing_valid_count, timeoutErrors;
uint16_t IrqStatus, calvalue;

void loop()
{
  uint16_t regdata;
  Serial.print(F("RangingListen,"));
  LT.receiveRanging(RangingAddress, RXtimeoutmS, TXpower, NO_WAIT);

  while (!digitalRead(DIO1));           //wait for Ranging valid or timeout

  regdata = LT.readIrqStatus();
  Serial.print(F("IRQ,"));
  Serial.print(regdata,HEX);

  if (regdata & IRQ_RANGING_SLAVE_RESPONSE_DONE)
  {
    digitalWrite(LED1, HIGH);
    rangeing_valid_count++;
    Serial.print(F(",RangingOK,"));
    Serial.print(rangeing_valid_count);
  }

  if (regdata & IRQ_RX_TX_TIMEOUT)
  {
    timeoutErrors++;
    led_Flash(2, 50);                                               
  }
  
    Serial.print(F(",Timeouts,")); 
    Serial.print(timeoutErrors);
    LT.printIrqStatus();
    
    Serial.println();
    digitalWrite(LED1, LOW);
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
  Serial.begin(9600);            //setup Serial console ouput
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
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

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
      led_Flash(50, 50);                                 //long fast speed flash indicates device error
    }
  }

  //The function call list below shows the complete setup for the LoRa device for ranging using the information
  //defined in the Settings.h file.
  //The 'Setup LoRa device for Ranging' list below can be replaced with a single function call, note that 
  //the calibration value will be loaded automatically from the table in the library;
  //LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RangingRole);

  //***************************************************************************************************
  //Setup LoRa device for Ranging Slave
  //***************************************************************************************************
  LT.setMode(MODE_STDBY_RC);
  LT.setPacketType(PACKET_TYPE_RANGING);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  LT.setRfFrequency(Frequency, Offset);
  LT.setTxParams(TXpower, RADIO_RAMP_02_US);
  LT.setRangingMasterAddress(RangingAddress);
  LT.setRangingSlaveAddress(RangingAddress);
  LT.setRangingCalibration(LT.lookupCalibrationValue(SpreadingFactor, Bandwidth));
  LT.setRangingRole(RANGING_SLAVE);
  LT.writeRegister(REG_RANGING_FILTER_WINDOW_SIZE, 8); //set up window size for ranging averaging
  LT.setHighSensitivity();
  //***************************************************************************************************

  //LT.setRangingCalibration(Calibration);               //override automatic lookup of calibration value from library table

  Serial.print(F("Calibration,"));
  Serial.println(LT.getSetCalibrationValue());
  delay(2000);
}

