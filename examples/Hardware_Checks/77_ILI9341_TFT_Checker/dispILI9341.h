/*
  Copyright 2020 - Stuart Robinson
  Licensed under a MIT license displayed at the bottom of this document.
  Original published 08/06/20
*/

extern int8_t DISPCS;
extern int8_t DISPRESET;
extern int8_t DISPDC;


#include <ILI9341_due.h>                            //https://github.com/marekburiak/ILI9341_due
#include <ILI9341_due_config.h>
#include "fonts\SystemFont5x7.h"
#include "fonts\Arial_bold_14.h"

#ifndef dispILI9341_h
#define dispILI9341_h

#define default_displayrotation iliRotation90        //default rotation iliRotation0,iliRotation90,iliRotation180,iliRotation270   
#define default_fontstyle 1                          //default font style - not yet implemented
#define default_backgroundcolour ILI9341_BLACK       //default background colour
#define default_textcolour ILI9341_WHITE             //default textcolour 
#define default_textscale 2                          //default text size


class dispILI9341 : public Print
{

  public:

  dispILI9341();
  bool begin();
  virtual size_t write(uint8_t _lcharacter);
  void clear();
  void setCursor(uint8_t lcol, uint8_t lrow);
  void clearLine(uint8_t linetoclear);
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour);
  
  private:
  
  uint8_t _textscale;

  
};
#endif


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
