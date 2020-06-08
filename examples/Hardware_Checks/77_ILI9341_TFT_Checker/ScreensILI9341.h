/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 08/06/20
  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


void screen1()
{
  disp.setCursor(0, 0);
  disp.print(F("Hello World !"));
  
  disp.setCursor(0, 1);
  disp.print(F("Line 1"));
  
  disp.setCursor(0, 2);
  disp.print(F("Line 2"));
  
  disp.setCursor(0, 3);
  disp.print(F("Line 3"));
  
  disp.setCursor(0, 4);
  disp.print(F("Line 4"));
  
  disp.setCursor(0, 5);
  disp.print(F("Line 5"));
  
  disp.setCursor(0, 6);
  disp.print(F("Line 6"));
  
  disp.setCursor(0, 7);
  disp.print(F("Line 7"));
  
  disp.setCursor(0, 8);
  disp.print(F("Line 8"));
  
  disp.setCursor(0, 9);
  disp.print(F("Line 9"));

  disp.setCursor(0, 10);
  disp.print(F("Line 10"));
  
  disp.setCursor(0, 11);
  disp.print(F("01234567890123456789012"));                         //display is 12 lines x 23 charaters
}

