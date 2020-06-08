/*
  Copyright 2020 - Stuart Robinson
  Licensed under a MIT license displayed at the bottom of this document.
  08/06/20
*/

#define LTUNUSED(v) (void) (v)    //add LTUNUSED(variable); to avoid compiler warnings 


#include <SPI.h>
#include "dispILI9341.h"

ILI9341_due disp1 = ILI9341_due(DISPCS, DISPDC, DISPRESET);


dispILI9341::dispILI9341()
{
}


bool dispILI9341::begin()
{

  disp1.begin();
  //disp1.setFont(Arial_bold_14);
  disp1.setFont(SystemFont5x7);
  disp1.fillScreen(ILI9341_BLACK);
  disp1.setRotation(iliRotation90);
  disp1.cursorToXY(0, 0);
  disp1.setTextColor(ILI9341_WHITE);
  _textscale = default_textscale;
  disp1.setTextScale(_textscale);
  return true;
}


size_t dispILI9341::write(uint8_t _lcharacter)
{
  disp1.write(_lcharacter);
  return 1;
}


void dispILI9341::clear()
{
  disp1.fillScreen(ILI9341_BLACK);
  disp1.setTextColor(ILI9341_WHITE);
}


void dispILI9341::setCursor(uint8_t lcol, uint8_t lrow)
{
  disp1.cursorToXY((lcol * 6 * _textscale), (lrow * 10 * _textscale));
}


void dispILI9341::clearLine(uint8_t linetoclear)
{

  uint8_t index, lcols;

  switch (_textscale)
  {
    case 1:
      lcols = 53;
      break;

    case 2:
      lcols = 26;
      break;

    case 3:
      lcols = 19;
      break;

    case 4:
      lcols = 14;
      break;

    case 5:
      lcols = 12;
      break;

    case 6:
      lcols = 10;
      break;

    case 7:
      lcols = 9;
      break;

    case 8:
      lcols = 8;
      break;

    case 9:
      lcols = 7;
      break;

    case 10:
      lcols = 6;
      break;

    default:
      lcols = 19;
  }

  setCursor(0, linetoclear);

  for (index = 1; index <= lcols; index++)
  {
    disp1.write(0x20);          //write a space to display
  }
}

void dispILI9341::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour)
{
  disp1.drawLine(x0, y0, x1, y1, colour);
}

/*
  void dispILI9341::setFont(uint8_t font)
  {

  }
*/


/*
  MIT license

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions
  of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.
*/





