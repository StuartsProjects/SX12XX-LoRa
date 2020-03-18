/*
******************************************************************************************************

  lora Programs for Arduino

  Copyright of the author Stuart Robinson 24/10/19

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  This program is a ranging requestor. It transmitts a ranging request which a receiver should here and
  respond to. The requestor times the round trip of the received response and calculates the distance. 
  The results are shown on the serial monitor and a I2C addressable I2C display if connected.
  
  The ranging parameters are in the in the 'Settings.h' file. The pins to access the SX1280 need to be
  defined in the 'Settings.h' file also.

  Changes:

  To Do:

******************************************************************************************************/


#define programversion "V1.0"
#define Serial_Monitor_Baud 115200

#include "Settings.h"
#include <SPI.h>
#include "SX1280LT.h"

SX1280Class SX1280LT;

#ifdef ENABLEDISPLAY
#include <Wire.h>
#include <U8x8lib.h>                                        //https://github.com/olikraus/u8g2 
//U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    //standard 0.96" SSD1306
U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);       //1.3" OLED often sold as 1.3" SSD1306
#endif

uint16_t rangeing_error_count, rangeing_valid_count, rangeing_valid_results;
uint16_t IrqStatus;
uint32_t endwaitmS, range_result, range_result_sum, range_result_average;
float distance, distance_sum, distance_average, distance_average_adjusted;


void loop()
{
  uint8_t index;
  distance_sum = 0;
  range_result_sum = 0;
  rangeing_valid_count = 0;

  setup_RangingTX();

  for (index = 1; index <= range_count; index++)
  {

    SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
    SX1280LT.setTx(PERIOBASE_01_MS, rangingTXTimeoutmS);             //this sends the ranging packet

    endwaitmS = millis() + rangingTXTimeoutmS;

    while (!(digitalRead(DIO1)) && (millis() < endwaitmS));          //wait for Ranging valid or timeout

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
        rangeing_valid_results++;
        rangeing_valid_count++;
        digitalWrite(BUZZER, HIGH);
        digitalWrite(LED1, HIGH);
        Serial.print("Valid");
        range_result = SX1280LT.getRangingResultRegValue(RANGING_RESULT_RAW);
        Serial.print(",Reg_RAW,");
        Serial.print(range_result, HEX);
        
        if (range_result > 800000)
        {
        range_result = 0; 
        }
        range_result_sum = range_result_sum + range_result;

        distance = SX1280LT.getRangingResult(range_result, RANGING_RESULT_RAW);
        distance_sum = distance_sum + distance;
        Serial.print(",Distance,");
        Serial.print(distance, 1);
        Serial.print("m");
        Serial.print(",Valid,");
        Serial.print(rangeing_valid_results);
        Serial.print(",Errors,");
        Serial.print(rangeing_error_count);
        digitalWrite(BUZZER, LOW);
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
        Serial.print(",Errors,");
        Serial.print(rangeing_error_count);
      }
    }
    Serial.println();
    delay(packet_delaymS);
  }
  
  range_result_average = (range_result_sum / rangeing_valid_count);

  if (rangeing_valid_count == 0)
  {
  distance_average = 0; 
  distance_average_adjusted = 0;
  }
  else
  {
  distance_average = (distance_sum / rangeing_valid_count);
  distance_average_adjusted = (distance_average * distance_adjustment);  
  }
    
  Serial.println();
  Serial.print("Average Register Result ");
  Serial.print(range_result_average, HEX);
  Serial.println();
  Serial.print("Average distance ");
  Serial.print(distance_average, 1);
  Serial.print("m");
  Serial.println();
  Serial.print("Adjusted average distance ");
  Serial.print(distance_average_adjusted, 1);
  Serial.print("m");
  Serial.println();
  Serial.println();
  #ifdef ENABLEDISPLAY
  display_screen1();
  #endif
  delay(1000);
}


#ifdef ENABLEDISPLAY
void display_screen1()
{
  disp.clear();
  disp.setCursor(0,0);
  disp.print("Distance ");
  disp.print(distance_average,0);
  disp.print("m");
  //disp.setCursor(0,1);
  //disp.print("Adjusted ");
  //disp.print(distance_average_adjusted,0);
  //disp.print("m");
  disp.setCursor(0,2);
  disp.print("OK,");
  disp.print(rangeing_valid_results);
  disp.print(",Err,");
  disp.print(rangeing_error_count);
  disp.setCursor(0,3);
  disp.print("RegRaw ");
  disp.print(range_result_average, HEX);
}
#endif


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


void setup_RangingTX()
{
  SX1280LT.setStandby(MODE_STDBY_RC);
  SX1280LT.setPacketType(PACKET_TYPE_RANGING);
  SX1280LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setRfFrequency(Frequency, Offset);
  SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
  SX1280LT.setRangingRequestAddress(TXaddress);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RANGING_MASTER_RESULT_VALID + IRQ_RANGING_MASTER_RESULT_TIMEOUT), 0, 0);              //set for IRQ on RX done
  SX1280LT.setRangingCalibration(Calibration);
  SX1280LT.setRangingRole(RADIO_RANGING_ROLE_MASTER);
  SX1280LT.setHighSensitivity();                    //set high sensitivity RX mode
}


void setup()
{
  Serial.begin(9600);                        //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println(F("Stuart Robinson"));
  Serial.println();
  Serial.println("10_SX1280LT_Ranging_Master_RAW Starting");

  pinMode(LED1, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  led_Flash(2, 125);

  Serial.println("Checking device");

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  if (SX1280LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3))
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
      led_Flash(50, 50);                                            //long fast flash indicates device error
    }
  }

#ifdef ENABLEDISPLAY
  Serial.println("Display Enabled");
  Wire.begin();
  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.setCursor(0, 0);
  disp.print("Ranging Ready");
  disp.setCursor(0, 1);
  disp.print("Power ");
  disp.print(RangingTXPower);
  disp.print("dBm");
  disp.setCursor(0, 2);
  disp.print("Cal ");
  disp.print(Calibration);
  disp.setCursor(0, 3);
  disp.print("Adjust ");
  disp.print(distance_adjustment,4);
#endif

   Serial.print("Power ");
   Serial.print(RangingTXPower);
   Serial.print("dBm");
   Serial.print(",Calibration,");
   Serial.println(Calibration);
   Serial.print("Distance adjust ");
   Serial.print(distance_adjustment,4);
   Serial.println();
   delay(1000);
}

