/*******************************************************************************************************
  LoRaTracker Programs for Arduino - Copyright of the author Stuart Robinson - 16/12/19

  http://www.LoRaTracker.uk

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This program checks that a SX127X LoRa device can be accessed. There should be two
  short LED flashes at startup. If the SX127X is detected there will be two more LED flashes and the
  contents of the registers from 0x00 to 0x7F are printed, this is a copy of a typical printout below.
  Note that the read back changed frequency may be different to the programmed frequency, there is a
  rounding error due to the use of floats to calculate the frequency. 

  If the SX127X is not detected LED1 will flashes rapidly.

  Frequency at reset 434000000
  Registers at reset
  Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
  0x00  00 09 1A 0B 00 52 6C 80 00 4F 09 2B 20 08 02 0A
  0x10  FF 70 15 0B 28 0C 12 47 32 3E 00 00 00 00 00 40
  0x20  00 00 00 00 05 00 03 93 55 55 55 55 55 55 55 55
  0x30  90 40 40 00 00 0F 00 00 00 F5 20 82 03 02 80 40
  0x40  00 00 12 24 2D 00 03 00 04 23 00 09 05 84 32 2B
  0x50  14 00 00 12 00 00 00 0F E0 00 0C 03 08 00 5C 78
  0x60  00 19 0C 4B CC 0F 41 20 04 47 AF 3F D4 00 53 0B
  0x70  D0 01 10 00 00 00 00 00 00 00 00 00 00 00 00 00

  Changed Frequency 434099968
  Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
  0x00  00 09 1A 0B 00 52 6C 86 66 4F 09 2B 20 08 02 0A
  0x10  FF 70 15 0B 28 0C 12 47 32 3E 00 00 00 00 00 40
  0x20  00 00 00 00 05 00 03 93 55 55 55 55 55 55 55 55
  0x30  90 40 40 00 00 0F 00 00 00 F5 20 82 03 02 80 40
  0x40  00 00 12 24 2D 00 03 00 04 23 00 09 05 84 32 2B
  0x50  14 00 00 12 00 00 00 0F E0 00 0C 03 08 00 5C 78
  0x60  00 19 0C 4B CC 0F 41 20 04 47 AF 3F D4 00 53 0B
  0x70  D0 01 10 00 00 00 00 00 00 00 00 00 00 00 00 00


  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

//These are the pin definitions for one of the LoRaTracker boards, be sure to change them to match your
//own setup. You will also need to connect up the pins for the SPI bus, which on an Arduino Pro Mini are
//SCK pin 13, MISO pin 12, and MOSI pin 11.

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using 

#define NSS 10                                  //SX127X device select
#define NRESET 9                                //SX127X reset pin
#define LED1 8                                  //for on board LED, put high for on

#define Program_Version "V1.0"

#include <SPI.h>
#include <SX127XLT.h>                           //load the appropriate library   

SX127XLT LT;                                    //create a library class instance called LT


void loop()
{
  uint32_t frequency;
  digitalWrite(LED1, HIGH);

  frequency = LT.getFreqInt();                  //read the set frequency following a reset
  Serial.print(F("Frequency at reset "));
  Serial.println(frequency);

  Serial.println(F("Registers at reset"));      //show the registers after reset
  LT.printRegisters(0x00, 0x7F);

  Serial.println();
  Serial.println();

  LT.setRfFrequency(434100000, 0);              //change the set frequency, in hertz
  frequency = LT.getFreqInt();                  //read back the changed frequency
  Serial.print(F("Changed Frequency "));
  Serial.println(frequency);                    //print the changed frequency, did the write work (allow for rounding errors) ?
  LT.printRegisters(0x00, 0x7F);                //show the registers after frequncy change
  Serial.println();
  digitalWrite(LED1, LOW);
  delay(5000);
  LT.resetDevice();                             //reset the device and start again
}


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
  pinMode(LED1, OUTPUT);                                  //setup pin as output for indicator LED
  led_Flash(2, 125);                                      //two quick LED flashes to indicate program start

  Serial.begin(9600);
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(Program_Version));
  Serial.println();

  Serial.println(F("2_Register_Test Starting"));

  SPI.begin();

  //SPI beginTranscation is normally part of library routines, but if it is disabled in library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, -1, -1, -1, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);                                    //two more quick LED flashes to indicate device found
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                  //long fast speed flash indicates device error
    }
  }
  Serial.println();
}

