/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 19/12/22

  This program is supplied as is, it is up to the user of the program to decide if the program is suitable
  for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This program tests that an MB85RC16PNF or FM24CL64 FRAM is working and retaining
  memory contents when power is removed. FRAMs are non-voltatile memory devices with a typical write
  endurance of from 10,000,000,000 (MB85RC16PNF) to 100,000,000,000,000 (FM24CL64B) write cycles.

  When the program starts the serial monitor will first display the results of an I2C Device scan which
  should list 8 devices found from 0x50 to 0x57 for a MB85RC16PNF and 0x50 for a FM24CL64 with the default
  pin connections. The MB85RC16PNF has eight 256 byte pages, FM24CL64 is one page of 8kbytes. The program
  then will print the contents of the memory. Next all the variable types will be written to successive
  addresses and the program will then print the area of memory, then read back each variable from memory
  and print them out and check the values were read OK.

  To check if the FRAM is reating memory you could make some changes to the variables written, run the
  program and then power off. On restart the changed variables should be displayed.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

//#include "FRAM_FM24CL64.h"              //SX12xx library file for FM24CL64 FRAM, 64kbit, 8kbyte, , I2C addresse 0x50
#include <FRAM_MB85RC16PNF.h>             //SX12xx library file for MB85RC16PNF FRAM, 16kbit, 2kbyte, I2C addresses 0x50 to 0x57

#include <Wire.h>
#include "I2C_Scanner.h"

int16_t Memory_Address = 0x50;            //default start I2C address of MB85RC16PNF and FM24CL64 FRAM
uint32_t counter, errors;


void loop()
{
  while (true)
  {
    counter++;

    Serial.println(F("Printing memory"));
    printMemory(0, 0x3F);
    Serial.println();
    Serial.flush();

    Serial.println();
    writeMemory();
    Serial.println();
    readMemory();
    Serial.println();
    delay(2000);
  }
}


void writeMemory()
{
  Serial.println(F("Writing variables"));
  Serial.flush();

  Serial.println(F("Write char     A"));
  writeMemoryChar(0, 'A');

  Serial.println(F("Write int8_t   0x55"));
  writeMemoryInt8(1, 0x55);

  Serial.println(F("Write uint8_t  0xAA"));
  writeMemoryUint8(2, 0xAA);

  Serial.println(F("Write int16_t  0x1234"));
  writeMemoryInt16(3, 0x1234);

  Serial.println(F("Write uint16_t 0x5678"));
  writeMemoryUint16(5, 0x5678);

  Serial.println(F("Write int32_t  0x01234567"));
  writeMemoryInt32(7, 0x01234567);

  Serial.println(F("Write float    0.123456"));
  writeMemoryFloat(11, 0.123456);

  Serial.print(F("Write uint32_t (counter) "));
  Serial.println(counter);
  writeMemoryUint32(16, counter);
}


void readMemory()
{
  char var1;
  int8_t var2;
  uint8_t var3;
  int16_t var4;
  uint16_t var5;
  int32_t var6;
  uint32_t var7;
  float var8;
  char buf[9];

  Serial.println(F("Reading variables"));
  Serial.flush();

  var1 = readMemoryChar(0);
  Serial.print(F("Read char      "));
  Serial.write(var1);
  if (var1 == 'A')
  {
    Serial.println(F(" OK"));
  }
  else
  {
    errors++;
    Serial.println(F(" ERROR"));
  }

  var2 = readMemoryInt8(1);
  Serial.print(F("Read int8_t    0x"));
  Serial.print(var2, HEX);
  if (var2 == 0x55)
  {
    Serial.println(F(" OK"));
  }
  else
  {
    errors++;
    Serial.println(F(" ERROR"));
  }

  var3 = readMemoryUint8(2);
  Serial.print(F("Read uint8_t   0x"));
  Serial.print(var3, HEX);
  if (var3 == 0xAA)
  {
    Serial.println(F(" OK"));
  }
  else
  {
    errors++;
    Serial.println(F(" ERROR"));
  }

  var4 = readMemoryInt16(3);
  Serial.print(F("Read int16_t   0x"));
  sprintf(buf, "%04X", var4);
  Serial.print(buf);
  if (var4 == 0x1234)
  {
    Serial.println(F(" OK"));
  }
  else
  {
    errors++;
    Serial.println(F(" ERROR"));
  }

  var5 = readMemoryUint16(5);
  Serial.print(F("Read uint16_t  0x"));
  Serial.print(var5, HEX);
  if (var5 == 0x5678)
  {
    Serial.println(F(" OK"));
  }
  else
  {
    errors++;
    Serial.println(F(" ERROR"));
  }

  var6 = readMemoryInt32(7);
  Serial.print(F("Read int32_t   0x"));
  Serial.print(var6, HEX);
  if (var6 == 0x01234567)
  {
    Serial.println(F(" OK"));
  }
  else
  {
    errors++;
    Serial.println(F(" ERROR"));
  }

  var8 = readMemoryFloat(11);
  Serial.print(F("Read float     "));
  Serial.print(var8, 6);
  if (var8 == 0.123456)
  {
    Serial.println(F(" OK"));
  }
  else
  {
    errors++;
    Serial.println(F(" ERROR"));
  }
  var7 = readMemoryUint32(16);
  Serial.print(F("Read uint32_t  (counter) "));
  Serial.println(var7);
  Serial.print(F("Errors "));
  Serial.println(errors);
}


void setup()
{
  Serial.begin(115200);
  Serial.print(F(__FILE__));
  Serial.println();

  setup_I2CScan();
  run_I2CScan();
  delay(2000);

  memoryStart(Memory_Address);                        //optional command to start memory with I2C address, if required
  fillMemory(0, 0x1F, 0);
  fillMemory(0x20, 0x2F, 0x55);
  fillMemory(0x30, 0x3F, 0xAA);
}
