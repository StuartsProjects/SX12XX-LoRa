/*******************************************************************************************************
  
  One of a series of test programs for LoRa devices on Arduino, the full set of programs and accompanying
  library can be found here;

  https://github.com/LoRaTracker/SX12XX-LoRa
    
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 16/12/19

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This program is stand alone, it is not necessary to install the SX12XX-LoRa library
  to use it.
    
  The program checks that a SX1272 LoRa device can be accessed by doing a test register write and read.
  If there is no device found a message is printed on the serial monitor. The contents of the registers
  from 0x00 to 0x7F are printed, there is a copy of a typical printout below. Note that the read back
  changed frequency may be different to the programmed frequency, there is a rounding error due to the
  use of floats to calculate the frequency. 

2_Register_Test Starting
SX1272 Selected
LoRa Device found

Frequency at reset 915000000
Registers at reset
Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
0x00  00 01 1A 0B 00 52 E4 C0 00 0F 19 2B 20 08 02 0A 
0x10  FF 63 15 0B 28 0C 12 47 32 3E 00 00 00 00 00 40 
0x20  00 00 00 00 05 00 03 93 55 55 55 55 55 55 55 55 
0x30  90 40 40 00 00 0F 00 00 00 F5 20 82 01 02 80 40 
0x40  00 00 22 13 0E 5B DB 24 0E 7F 3A 2E 00 03 00 00 
0x50  00 00 04 23 00 BD 00 09 09 05 84 0B D0 0B D0 32 
0x60  2B 14 00 00 10 00 00 00 0F E0 00 0C 01 14 25 07 
0x70  00 5C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 


Changed Frequency 434099968
Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
0x00  00 01 1A 0B 00 52 6C 86 66 0F 19 2B 20 08 02 0A 
0x10  FF 63 15 0B 28 0C 12 47 32 3E 00 00 00 00 00 40 
0x20  00 00 00 00 05 00 03 93 55 55 55 55 55 55 55 55 
0x30  90 40 40 00 00 0F 00 00 00 F5 20 82 01 02 80 40 
0x40  00 00 22 13 0E 5B DB 24 0E 7F 3A 2E 00 03 00 00 
0x50  00 00 04 23 00 BD 00 09 09 05 84 0B D0 0B D0 32 
0x60  2B 14 00 00 10 00 00 00 0F E0 00 0C 01 14 25 07 
0x70  00 5C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 



  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

//These are the pin definitions for one of the LoRaTracker boards, be sure to change them to match your
//own setup. You will also need to connect up the pins for the SPI bus, which on an Arduino Pro Mini are
//SCK pin 13, MISO pin 12, and MOSI pin 11.

#define NSS 10                                  //SX127X device select
#define NRESET 9                                //SX127X reset pin

#define USE_SX1272                              //enable this define if the SX12672 is used

const uint8_t REG_FRMSB = 0x06;                  //register numbers for setting setting and reading frequency 
const uint8_t REG_FRMID = 0x07;
const uint8_t REG_FRLSB = 0x08;
const uint8_t REG_VERSION = 0x42;                //version number of device

#include <SPI.h>


void loop()
{
  uint32_t frequency;

  frequency = getFreqInt();                     //read the set frequency following a reset
  Serial.print(F("Frequency at reset "));
  Serial.println(frequency);

  Serial.println(F("Registers at reset"));      //show the registers after reset
  printRegisters(0x00, 0x7F);

  Serial.println();
  Serial.println();

  setRfFrequency(434100000, 0);              //change the set frequency, in hertz
  frequency = getFreqInt();                  //read back the changed frequency
  Serial.print(F("Changed Frequency "));
  Serial.println(frequency);                 //print the changed frequency, did the write work (allow for rounding errors) ?
  printRegisters(0x00, 0x7F);                //show the registers after frequency change
  Serial.println();
  delay(5000);
  resetDevice();                             //reset the device and start again
}


uint8_t readRegister(uint8_t address)
{
  uint8_t regdata;
  digitalWrite(NSS, LOW);          //set NSS low
  SPI.transfer(address & 0x7F);    //mask address for read
  regdata = SPI.transfer(0);       //read the byte
  digitalWrite(NSS, HIGH);         //set NSS high
  return regdata;
}


void writeRegister(uint8_t address, uint8_t value)
{
  digitalWrite(NSS, LOW);           //set NSS low
  SPI.transfer(address | 0x80);     //mask address for write
  SPI.transfer(value);              //write the byte
  digitalWrite(NSS, HIGH);          //set NSS high
}


uint32_t getFreqInt()
{
  //get the current set LoRa device frequency, return as long integer
  uint8_t Msb, Mid, Lsb;
  uint32_t uinttemp;
  float floattemp;
  Msb = readRegister(REG_FRMSB);
  Mid = readRegister(REG_FRMID);
  Lsb = readRegister(REG_FRLSB);
  floattemp = ((Msb * 0x10000ul) + (Mid * 0x100ul) + Lsb);
  floattemp = ((floattemp * 61.03515625) / 1000000ul);
  uinttemp = (uint32_t)(floattemp * 1000000);
  return uinttemp;
}


void printRegisters(uint16_t Start, uint16_t End)
{
  //prints the contents of SX127x registers to serial monitor

  uint16_t Loopv1, Loopv2, RegData;

  Serial.print(F("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  for (Loopv1 = Start; Loopv1 <= End;)           //32 lines
  {
    Serial.print(F("0x"));
    if (Loopv1 < 0x10)
    {
      Serial.print(F("0"));
    }
    Serial.print((Loopv1), HEX);                 //print the register number
    Serial.print(F("  "));
    for (Loopv2 = 0; Loopv2 <= 15; Loopv2++)
    {
      RegData = readRegister(Loopv1);
      if (RegData < 0x10)
      {
        Serial.print(F("0"));
      }
      Serial.print(RegData, HEX);                //print the register number
      Serial.print(F(" "));
      Loopv1++;
    }
    Serial.println();
  }
}


void setRfFrequency(uint64_t freq64, int32_t offset)
{
  freq64 = freq64 + offset;
  freq64 = ((uint64_t)freq64 << 19) / 32000000;
  writeRegister(REG_FRMSB, (uint8_t)(freq64 >> 16));
  writeRegister(REG_FRMID, (uint8_t)(freq64 >> 8));
  writeRegister(REG_FRLSB, (uint8_t)(freq64 >> 0));
}


void resetDevice()
{
 #ifdef USE_SX1272
    digitalWrite(NRESET, HIGH);
    delay(2);
    digitalWrite(NRESET, LOW);
    delay(20);
    Serial.println(F("SX1272 Selected"));
 #else
    digitalWrite(NRESET, LOW);
    delay(2);
    digitalWrite(NRESET, HIGH);
    delay(20);
    Serial.println(F("SX1276-79 Selected"));
  #endif 
 }


bool begin(int8_t pinNSS, int8_t pinNRESET)
{
  pinMode(pinNSS, OUTPUT);
  digitalWrite(pinNSS, HIGH);
  pinMode(pinNRESET, OUTPUT);
  digitalWrite(pinNRESET, LOW);

  resetDevice();

  if (checkDevice())
  {
    return true;
  }

  return false;
}


bool checkDevice()
{
  //check there is a device out there, writes a register and reads back

  uint8_t Regdata1, Regdata2;
  Regdata1 = readRegister(REG_FRMID);               //low byte of frequency setting
  writeRegister(REG_FRMID, (Regdata1 + 1));
  Regdata2 = readRegister(REG_FRMID);               //read changed value back
  writeRegister(REG_FRMID, Regdata1);               //restore register to original value

  if (Regdata2 == (Regdata1 + 1))
  {
    return true;
  }
  else
  {
    return false;
  }
}


void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);

  Serial.println(F("2_Register_Test_SX1272 Starting"));

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
  if (begin(NSS, NRESET))
  {
    Serial.println(F("LoRa Device found"));
  }
  else
  {
    Serial.println(F("No LoRa device responding"));
  }
  
  Serial.println();
}

