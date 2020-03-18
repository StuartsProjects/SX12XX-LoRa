/*****************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 13/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


#define programversion "V1.0"
#define Serial_Monitor_Baud 9600

#include <SPI.h>
#include <SX128XLT.h>
SX128XLT SX1280LT;
#include "Settings.h"


uint16_t rangeing_error_count, rangeing_valid_count;
uint16_t timeoutErrors, IrqStatus, Calibration;
uint32_t endwaitmS, startrangingmS, range_result;
float distance; //, total_distance, adjusted_distance;
boolean ranging_error;
uint8_t distance_negative_count;
uint16_t Calvalue;
uint16_t CalibrationStart, CalibrationEnd; 


void loop()
{
  uint16_t index;
  distance_negative_count = 0;

  for (index = CalibrationStart; index <= CalibrationEnd; index = index + 10)
  {
    //Calibration = index;

    //setup_RangingMaster(Calibration);
    SX1280LT.setRangingCalibration(index);

    startrangingmS = millis();
    SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
    SX1280LT.setTx(rangingTXTimeoutmS);              //this sends the ranging packet
    endwaitmS = millis() + waittimemS;

    while (!(digitalRead(DIO1)) && (millis() < endwaitmS));           //wait for Ranging valid or timeout

    if (millis() > endwaitmS)
    {
      Serial.print(",Timout");
      rangeing_error_count++;

      range_result = 0;

    }
    else
    {
      IrqStatus = SX1280LT.readIrqStatus();

      if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
      {
        rangeing_valid_count++;
        ranging_error = false;
        digitalWrite(LED1, HIGH);
        Serial.print("Valid");
        range_result = SX1280LT.getRangingResultRegValue(RANGING_RESULT_RAW);
        Serial.print(",RAW,");
        Serial.print(range_result, HEX);

        distance = SX1280LT.getRangingDistance(RANGING_RESULT_RAW, 1);

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

        //delay(packet_delaymS);
        digitalWrite(LED1, LOW);
      }
      else
      {
        ranging_error = true;
        rangeing_error_count++;
        distance = 0;
        range_result = 0;
        Serial.print("NotValid");
        Serial.print(",Irq,");
        Serial.print(IrqStatus, HEX);
      }
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


void setup_RangingMaster(uint16_t cal)
{
  SX1280LT.setMode(MODE_STDBY_RC);
  SX1280LT.setPacketType(PACKET_TYPE_RANGING);
  SX1280LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setRfFrequency(Frequency, Offset);
  SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
  SX1280LT.setRangingRequestAddress(TXaddress);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RANGING_MASTER_RESULT_VALID + IRQ_RANGING_MASTER_RESULT_TIMEOUT), 0, 0);              //set for IRQ on RX done
  SX1280LT.setRangingCalibration(cal);
  SX1280LT.setRangingRole(RANGING_MASTER);
  SX1280LT.setHighSensitivity();
  SX1280LT.writeRegister(REG_RANGING_FILTER_WINDOW_SIZE, 8); //set up window size for ranging averaging
}


void setup()
{
  //uint32_t regval;
  //uint32_t num = 2;
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

  if (SX1280LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
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

  Calvalue = SX1280LT.getRangingCalibrationValue(Bandwidth, SpreadingFactor);
  setup_RangingMaster(Calvalue);

  Serial.print(F("CalibrationMidValue,"));
  Serial.println(Calvalue);
  CalibrationStart = Calvalue - 1000;
  CalibrationEnd = Calvalue + 1000;
  Serial.print(F(",Start,"));
  Serial.print(CalibrationStart);
  Serial.print(F(",End,"));
  Serial.println(CalibrationEnd);

}

