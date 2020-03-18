/*****************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 13/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

#define Program_Version "V1.0"

#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"

SX128XLT LT;

#ifdef ENABLEDISPLAY                                        //check to see if diaplay option required
#include <Wire.h>
#include <U8x8lib.h>                                        //get library here >  https://github.com/olikraus/u8g2
//U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);    //standard 0.96" SSD1306
U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);       //1.3" OLED often sold as 1.3" SSD1306
#endif


uint16_t rangeing_error_count, rangeing_valid_count;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result, range_result_sum, range_result_average;
uint32_t total_ranging_OK;
float distance, distance_sum, distance_average;
bool ranging_error;


void loop()
{
  uint8_t index;
  //distance_sum = 0;
  //range_result_sum = 0;
  //rangeing_valid_count = 0;

  LT.transmitRanging(RangingAddress, TXtimeoutmS, TXpower, WAIT_TX);

  IrqStatus = LT.readIrqStatus();

  if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
  {
    led_Flash(1, 10);
    total_ranging_OK++;
    rangeing_valid_count++;
    ranging_error = false;
    Serial.print(F("RangingValid,Irq,"));
    Serial.print(IrqStatus, HEX);
    range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
    //range_result_sum = range_result_sum + range_result;
    Serial.print(F(",RAW,"));
    Serial.print(range_result, HEX);
    distance = LT.getRangingDistance(RANGING_RESULT_RAW, distance_adjustment);
    //distance_sum = distance_sum + distance;
    Serial.print(F(",Distance,"));
    Serial.print(distance, 1);
    Serial.print(F("m"));
    //Serial.print(F(",RangingTime,"));
    //Serial.print(millis() - startrangingmS);
    //Serial.print(F("mS"));
    Serial.print(F(",Valid,"));
    Serial.print(rangeing_valid_count);
    Serial.print(F(",Errors,"));
    Serial.print(rangeing_error_count);
  }
  else
  {
    led_Flash(2, 50);                        //double flash to indicate ranging error
    ranging_error = true;
    rangeing_error_count++;
    distance = 0;
    range_result = 0;
    Serial.print(F("RangingNotValid"));
    Serial.print(F(",Irq,"));
    Serial.print(IrqStatus, HEX);
  }

  Serial.println();
  Serial.print(F("RAW Result "));
  Serial.print(range_result, HEX);
  Serial.println();
  Serial.print(F("Distance "));
  Serial.print(distance, 1);
  Serial.print(F("m"));
  Serial.println();
  Serial.println();

#ifdef ENABLEDISPLAY
  display_screen1();
#endif
 delay(packet_delaymS);
}


#ifdef ENABLEDISPLAY
void display_screen1()
{
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("Address  "));
  disp.print(RangingAddress);
  disp.setCursor(0, 2);
  disp.print(F("OK Count "));
  disp.print(total_ranging_OK);
  disp.setCursor(0, 3);
  disp.print(F("Errors   "));
  disp.print(rangeing_error_count);
  disp.setCursor(0, 4);
  disp.print(F("CalValue "));
  disp.println(LT.getSetCalibrationValue());
  disp.setCursor(0, 5);
  disp.print(F("RAW      "));
  disp.print(range_result, HEX);
  disp.setCursor(0, 6);
  disp.print(F("Distance "));
  disp.print(distance, 0);
  disp.print(F("m"));
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


void setup()
{
  pinMode(LED1, OUTPUT);                                   //setup pin as output for indicator LED
  led_Flash(2, 125);                                       //two quick LED flashes to indicate program start

  Serial.begin(9600);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();
  Serial.println(F("54_Ranging_Master Starting"));

  SPI.begin();

  led_Flash(2, 125);

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
      led_Flash(50, 50);                                 //long fast flash indicates device error
    }
  }

  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RangingRole);

  //LT.setRangingCalibration(Calibration);               //override automatic lookup of calibration value from library table

#ifdef ENABLEDISPLAY
  Serial.println(F("Display Enabled"));
  Wire.begin();
  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.setCursor(0, 0);
  disp.print(F("Ranging Ready"));
#endif

  Serial.print(F("Address "));
  Serial.println(RangingAddress);
  Serial.print(F("CalibrationValue "));
  Serial.println(LT.getSetCalibrationValue());
  Serial.println(F("Ranging master ready"));



  delay(2000);
}

