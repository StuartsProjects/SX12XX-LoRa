/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 12/05/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors. 
*******************************************************************************************************/


/*******************************************************************************************************
Program Operation - This test program has been written to check that hardware for sending AFSK RTTY on 
a LoRa receiver board has been connected correctly. AFSKRTTY can be used to upload packets received from 
a high altitude balloon tracker as LoRa into DL-FLDIGI running on a PC and from there uploaded to Internet
connected tracking system.
    
The audio output is connected to a PC sound cards microphone input. Do use a small capacitor and resistor
is series with the output pin to limit the input to the PC sound card. A 0.1uF capacitor and 47k resistor
are suitable. The AFSK RTTY is sent as ASCII 7 bit, 2 Stop bit, no parity, 300 baud. Tones in the example
are 635Hz for a 0 bit and 1000hz for 1 bit. A screenshot of the FLDIGI settings used in in the folder
containing the program; 'FLDIGI Settings.jpg'

The program uses 2 pins on the Arduino, AUDIOOUT and LED1. You need to define the pins these outputs 
are on. Uses the tone() library which is not avaialable for all processors supported by the Arduino IDE.

Only works on Arduinos that support the tone() function.
  
Serial monitor baud rate is set at 9600
*******************************************************************************************************/

const int8_t AUDIOOUT = 4;                    //Pin used to output Audio tones  
const int8_t CHECK = A3;                      //This pin is toggled inside the AFSKRTTY library, high for logic 1, low for logic 0, so it can be used to check the timing.

const uint16_t AFSKRTTYperiod = 3333;         //period in uS for 1 bit at chosen baud rate, e.g. 10000 for 100baud, 3333 for 300baud
const uint16_t leadinmS = 500;                //number of ms for AFSK constant lead in tone
const uint16_t tonehighHz = 1000;             //high tone in Hertz 
const uint16_t tonelowHz = 635;               //low tone in Hertz   

#include <AFSKRTTY.h>

//Choose whichever test pattern taks your fancy
//uint8_t testBuffer[] = "0123456789* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *";               //This string is sent as AFSK RTTY, 7 bit, 2 Stop bit, no parity, 300 baud.
uint8_t testBuffer[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ 0123456789";               
//uint8_t testBuffer[] = "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU";
//uint8_t testBuffer[] = "$$MyFlight1,2213,14:54:37,51.48230,-3.18136,15,6,3680,23,66,3,0*2935";


void loop()
{
  uint8_t index;
  uint8_t chartosend;
  uint8_t len = sizeof(testBuffer);
  
  Serial.print(F("Sending AFSK RTTY "));
  Serial.flush();
 
  start_AFSK_RTTY(AUDIOOUT,tonehighHz,leadinmS);
  
  for (index = 0; index < len; index++)
  {
    chartosend = testBuffer[index];
    SendAFSKRTTY(chartosend,AUDIOOUT,CHECK,tonelowHz,tonehighHz,AFSKRTTYperiod);
    Serial.write(chartosend);
    Serial.flush();
  }

  SendAFSKRTTY(13,AUDIOOUT,CHECK,tonelowHz,tonehighHz,AFSKRTTYperiod);
  SendAFSKRTTY(10,AUDIOOUT,CHECK,tonelowHz,tonehighHz,AFSKRTTYperiod);
  
  //end_AFSK_RTTY(AUDIOOUT);                                //optional, if enabled prevents noise appearing on FLDIGI decode
  
  digitalWrite(CHECK, LOW);
  Serial.println();
  delay(1000);
}


void setup()
{
  pinMode(CHECK, OUTPUT);                                   //setup pin as output for indicator LED

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("70_AFSKRTTY_Upload_Test"));
  Serial.println();
}


