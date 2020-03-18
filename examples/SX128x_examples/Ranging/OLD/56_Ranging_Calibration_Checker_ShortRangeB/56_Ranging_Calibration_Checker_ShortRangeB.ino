/*****************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 13/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


#define programversion "V1.0"
#define Serial_Monitor_Baud 9600

#include <SPI.h>
#include <SX128XLT.h>
SX128XLT LT;
#include "Settings.h"

//boolean ranging_error;
uint8_t distance_negative_count;
uint16_t Calvalue;
uint16_t CalibrationStart, CalibrationEnd;
uint16_t rangeing_error_count, rangeing_valid_count;
uint16_t timeoutErrors, IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result;
float distance;


void loop()
{
  uint16_t index, regdata;
  distance_negative_count = 0;

  for (index = CalibrationStart; index <= CalibrationEnd; index = index + 10)
  {
    LT.setRangingCalibration(index);
    //endwaitmS = millis() + waittimemS;
    //startrangingmS = millis();

    LT.transmitRanging(RangingAddress, rangingTXTimeoutmS, RangingTXPower, NO_WAIT);

    while (!digitalRead(DIO1));           //wait for Ranging valid or timeout

    delay(10);                            //allow time for IRQ update
    regdata = LT.readIrqStatus();

    if (regdata & IRQ_RX_TX_TIMEOUT)
    {
      timeoutErrors++;
      Serial.print(",Timeout");
      led_Flash(2, 50);
    }

    if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
    {
      rangeing_valid_count++;
      digitalWrite(LED1, HIGH);
      Serial.print("Valid");
      range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
      Serial.print(",RAW,");
      Serial.print(range_result, HEX);

      distance = LT.getRangingDistance(RANGING_RESULT_RAW, 1);

      Serial.print(",Calibration,");
      Serial.print(index);
      Serial.print(",Distance,");
      Serial.print(distance, 1);
      Serial.print("m");
      Serial.print(",Time,");
      Serial.print(millis() - startrangingmS);
      Serial.print("mS");
      Serial.print(",Valid,");
      Serial.print(rangeing_valid_count);
      Serial.print(",Errors,");
      Serial.print(rangeing_error_count);

      if (distance < 0)
      {
        Serial.print(",Distance is negative !!!");
        distance_negative_count++;
      }

      if (distance_negative_count >= 3)
      {
        delay(5000);
        break;
      }

      digitalWrite(LED1, LOW);
    }
    else
    {
      rangeing_error_count++;
      distance = 0;
      range_result = 0;
      Serial.print("NotValid");
      Serial.print(",Irq,");
      Serial.print(IrqStatus, HEX);
    }

    Serial.println();
    delay(packet_delaymS);
  }

  Serial.println();
  Serial.println();
  Serial.println();
}




void led_Flash(uint16_t flashes, uint16_t delaymS)
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
  uint16_t Remainder;

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

  Serial.println("56_Ranging_Calibration_Checker_ShortRange Starting");

  pinMode(LED1, OUTPUT);

  led_Flash(2, 125);

  Serial.println("Checking device");

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
      led_Flash(50, 50);                                            //long fast speed flash indicates device error
    }
  }

  Calvalue = LT.getRangingCalibrationValue(SpreadingFactor, Bandwidth);

  Serial.print(F("CalibrationMidValue,"));
  Serial.print(Calvalue);

  Remainder = Calvalue / 10;

  CalibrationStart = (Remainder * 10) - 1000;
  CalibrationEnd = (Remainder * 10) + 1000;
  Serial.print(F(",Start,"));
  Serial.print(CalibrationStart);
  Serial.print(F(",End,"));
  Serial.println(CalibrationEnd);

  //The function call list below shows the complete setup for the LoRa device for ranging using the information
  //defined in the Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RangingRole);


  /*
    //***************************************************************************************************
    //Setup LoRa device for Ranging
    //***************************************************************************************************
    LT.setMode(MODE_STDBY_RC);
    LT.setPacketType(PACKET_TYPE_RANGING);
    LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
    LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
    LT.setRfFrequency(Frequency, Offset);
    LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
    LT.setRangingMasterAddress(RangingAddress);
    LT.setRangingSlaveAddress(RangingAddress);
    LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RANGING_MASTER_RESULT_VALID + IRQ_RANGING_MASTER_RESULT_TIMEOUT), 0, 0);              //set for IRQ on RX done
    LT.setRangingCalibration(Calvalue);
    LT.setRangingRole(RANGING_MASTER);
    LT.writeRegister(REG_RANGING_FILTER_WINDOW_SIZE, 8); //set up window size for ranging averaging
    LT.setHighSensitivity();
    //LT.setLowPowerRX();
    //***************************************************************************************************
  */
}

