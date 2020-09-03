/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 07/06/19

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This program is a simple test program for the LIL9341 TFT display that comes in a 
  range of sizes. The program prints a short message on each line, pauses, clears the screen, and starts
  again. The displayILI9341 library is used so that sketches and writes to the screen for the SSD1306
  OLED have the same structure of function calls. Thus it should not be necessary to re-write an application
  to use a different display. The displayILI9341D library in turn uses the ILI9341_due.h library (which 
  does work on Atmel!) that can be downloaded from here https://github.com/marekburiak/ILI9341_due 

  Screen write on Arduino DUE 61mS

  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

#include <SPI.h>

int8_t DISPCS = 43;                            //we need to pass the pin definitions across to dispILI9341.h
int8_t DISPRESET = 42;
int8_t DISPDC = 45;

#include <displayILI9341D.h>                   //this library uses the DUE version of the ILI9341 library
displayILI9341D disp;                          //create a library class instance called disp for the main sketch

#include "ScreensILI9341.h" 

uint16_t writecount;
uint32_t startwritemS, endwritemS, timemS;


void loop()
{
  writecount++;
  Serial.print(writecount);
  Serial.print(F(" Writing to display"));

  startwritemS = millis();
  disp.clear();
  screen1();
  endwritemS = millis();
  timemS = endwritemS - startwritemS;
  disp.setCursor(8, 4);
  disp.print(timemS);
  disp.print(F("mS"));

  Serial.print(F(" - done "));
  Serial.print(timemS);
  Serial.println(F("mS"));

  delay(5000);

}  


void setup()
{
  Serial.begin(9600);
  Serial.println(F("77_ILI9341_TFT_Checker starting"));
  SPI.begin();
  disp.begin();
}

