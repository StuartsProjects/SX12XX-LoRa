/*
  Copyright 2019 - Stuart Robinson 
  Licensed under a MIT license displayed at the bottom of this document.
  17/12/19
*/

#include <SX127XLT.h>
#include <SPI.h>

#define UNUSED(v) (void) (v)       //add UNUSED(variable); in functions to avoid compiler warnings 
#define USE_SPI_TRANSACTION        //this is the standard behaviour of library, use SPI Transaction switching

//#define SX127XDEBUG1             //enable level 1 debug messages
//#define DEBUGPHANTOM             //used to set bebuging for Phantom packets


SX127XLT::SX127XLT()
{

}


bool SX127XLT::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinDIO0, int8_t pinDIO1, int8_t pinDIO2, uint8_t device)
{
#ifdef SX127XDEBUG1
  Serial.println(F("begin()"));
#endif

  //assign the passed pins to the class private variabled
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _DIO0 = pinDIO0;
  _DIO1 = pinDIO1;
  _DIO2 = pinDIO2;
  _Device = device;            //device type needs to be assigned before reset
  _TXDonePin = pinDIO0;        //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO0;        //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);

  if (_DIO0 >= 0)
  {
    pinMode( _DIO0, INPUT);
  }

  if (_DIO1 >= 0)
  {
    pinMode( _DIO1,  INPUT);
  }

  if (_DIO2 >= 0)
  {
    pinMode( _DIO2,  INPUT);
  }

  resetDevice();

  if (checkDevice())
  {
    return true;
  }

  return false;
}


void SX127XLT::resetDevice()
{
#ifdef SX127XDEBUG1
  Serial.println(F("resetDevice()"));
#endif

  if (_Device == DEVICE_SX1272)
  {
    digitalWrite(_NRESET, HIGH);
    delay(2);
    digitalWrite(_NRESET, LOW);
    delay(20);
  }
  else
  {
    digitalWrite(_NRESET, LOW);
    delay(2);
    digitalWrite(_NRESET, HIGH);
    delay(20);
  }
}


void SX127XLT::setMode(uint8_t modeconfig)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setMode()"));
#endif

  uint8_t regdata;

  regdata = modeconfig + _PACKET_TYPE;
  writeRegister(REG_OPMODE, regdata);
}


void SX127XLT::setSleep(uint8_t sleepconfig)
{
  //settings passed via sleepconfig are ignored, feature retained for compatibility with SX1280,SX126x

#ifdef SX127XDEBUG1
  Serial.println(F("setSleep()"));
#endif

  UNUSED(sleepconfig);

  uint8_t regdata;

  regdata = readRegister(REG_OPMODE);
  writeRegister(REG_OPMODE, (regdata & 0xF8));       //clear bits 0,1,2 to set sleepmode
  delay(1);                                         //allow time for shutdown
}


bool SX127XLT::checkDevice()
{
  //check there is a device out there, writes a register and reads back

#ifdef SX127XDEBUG1
  Serial.println(F("checkDevice()"));
#endif

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


void SX127XLT::wake()
{
#ifdef SX127XDEBUG1
  Serial.println(F("wake()"));
#endif
  uint8_t regdata;

  regdata = readRegister(REG_OPMODE);
  writeRegister(REG_OPMODE, (regdata | 0x01));       //set bit 0 to goto stdby mode
}


void SX127XLT::calibrateImage(uint8_t null)
{
#ifdef SX127XDEBUG1
  Serial.println(F("calibrateImage()"));
#endif

  UNUSED(null);

  uint8_t regdata, savedmode;
  savedmode = readRegister(REG_OPMODE);
  writeRegister(REG_OPMODE, MODE_SLEEP);
  writeRegister(REG_OPMODE, MODE_STDBY);
  regdata = (readRegister(REG_IMAGECAL) | 0x40);
  writeRegister(REG_IMAGECAL, regdata);
  writeRegister(REG_OPMODE, MODE_SLEEP);
  writeRegister(REG_OPMODE, savedmode);
  delay(10);                              //calibration time 10mS
}



uint16_t SX127XLT::CRCCCITT(uint8_t *buffer, uint8_t size, uint16_t startvalue)
{
#ifdef SX127XDEBUG1
  Serial.println(F("CRCCCITT()"));
#endif

  uint16_t index, CRC;
  uint8_t j;

  CRC = startvalue;                                  //start value for CRC16

  for (index = 0; index < size; index++)
  {
    CRC ^= (((uint16_t)buffer[index]) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  return CRC;
}


uint16_t SX127XLT::CRCCCITTSX(uint8_t startadd, uint8_t endadd, uint16_t startvalue)
{
  //genrates a CRC of an area of the internal SX buffer

#ifdef SX127XDEBUG1
  Serial.println(F("CRCCCITTSX()"));
#endif


  uint16_t index, CRC;
  uint8_t j;

  CRC = startvalue;                                  //start value for CRC16

  startReadSXBuffer(startadd);                       //begin the buffer read

  for (index = startadd; index <= endadd; index++)
  {
    CRC ^= (((uint16_t) readUint8() ) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  return CRC;
}


void SX127XLT::setDevice(uint8_t type)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setDevice()"));
#endif

  _Device = type;
}


void SX127XLT::printDevice()
{
#ifdef SX127XDEBUG1
  Serial.println(F("printDevice()"));
#endif

  switch (_Device)
  {
    case DEVICE_SX1272:
      Serial.print(F("SX1272"));
      break;

    case DEVICE_SX1276:
      Serial.print(F("SX1276"));
      break;

    case DEVICE_SX1277:
      Serial.print(F("SX1277"));
      break;

    case DEVICE_SX1278:
      Serial.print(F("SX1278"));
      break;

    case DEVICE_SX1279:
      Serial.print(F("SX1279"));
      break;

    default:
      Serial.print(F("Unknown Device"));

  }
}


uint8_t SX127XLT::getOperatingMode()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getOperatingMode()"));
#endif

  return readRegister(REG_OPMODE);
}


bool SX127XLT::isReceiveDone()
{
#ifdef SX127XDEBUG1
  Serial.println(F("isReceiveDone()"));
#endif

  return digitalRead(_RXDonePin);
}


bool SX127XLT::isTransmitDone()
{
#ifdef SX127XDEBUG1
  Serial.println(F("isTransmitDone()"));
#endif

  return digitalRead(_TXDonePin);
}


void SX127XLT::writeRegister(uint8_t address, uint8_t value)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeRegister()"));
#endif

#ifdef USE_SPI_TRANSACTION                  //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                  //set NSS low
  SPI.transfer(address | 0x80);             //mask address for write
  SPI.transfer(value);                      //write the byte
  digitalWrite(_NSS, HIGH);                 //set NSS high

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

#ifdef SX127XDEBUG2
  Serial.print(F("Write register "));
  printHEXByte0x(address);
  Serial.print(F(" "));
  printHEXByte0x(value);
  Serial.println();
  Serial.flush();
#endif
}


uint8_t SX127XLT::readRegister(uint8_t address)
{
#ifdef SX127XDEBUG1
  Serial.println(F("readRegister()"));
#endif

  uint8_t regdata;

#ifdef USE_SPI_TRANSACTION         //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);         //set NSS low
  SPI.transfer(address & 0x7F);    //mask address for read
  regdata = SPI.transfer(0);       //read the byte
  digitalWrite(_NSS, HIGH);        //set NSS high

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

#ifdef SX127XDEBUG2
  Serial.print(F("Read register "));
  printHEXByte0x(address);
  Serial.print(F(" "));
  printHEXByte0x(regdata);
  Serial.println();
  Serial.flush();
#endif

  return regdata;
}



void SX127XLT::printRegisters(uint16_t Start, uint16_t End)
{
  //prints the contents of SX127x registers to serial monitor

#ifdef SX127XDEBUG
  Serial.println(F("printRegisters()"));
#endif

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


void SX127XLT::printOperatingMode()
{
#ifdef SX127XDEBUG1
  Serial.println(F("printOperatingMode()"));
#endif

  uint8_t regdata;

  regdata = getOpmode();

  switch (regdata)
  {
    case 0:
      Serial.print(F("SLEEP"));
      break;

    case 1:
      Serial.print(F("STDBY"));
      break;

    case 2:
      Serial.print(F("FSTX"));
      break;

    case 3:
      Serial.print(F("TX"));
      break;

    case 4:
      Serial.print(F("FSRX"));
      break;

    case 5:
      Serial.print(F("RXCONTINUOUS"));
      break;

    case 6:
      Serial.print(F("RXSINGLE"));
      break;

    case 7:
      Serial.print(F("CAD"));
      break;

    default:
      Serial.print(F("NOIDEA"));
      break;
  }
}


void SX127XLT::printOperatingSettings()
{
#ifdef SX127XDEBUG1
  Serial.println(F("printOperatingSettings()"));
#endif

  printDevice();
  Serial.print(F(","));

  printOperatingMode();

  Serial.print(F(",Version_"));
  Serial.print(getVersion(), HEX);

  Serial.print(F(",PacketMode_"));

  if (getPacketMode())
  {
    Serial.print(F("LoRa"));
  }
  else
  {
    Serial.print(F("FSK"));
  }

  if (getHeaderMode())
  {
    Serial.print(F(",Implicit"));
  }
  else
  {
    Serial.print(F(",Explicit"));
  }

  Serial.print(F(",CRC_"));
  if (getCRCMode())
  {
    Serial.print(F("On"));
  }
  else
  {
    Serial.print(F("Off"));
  }


  Serial.print(F(",AGCauto_"));
  if (getAGC())
  {
    Serial.print(F("On"));
  }
  else
  {
    Serial.print(F("Off"));
  }

  Serial.print(F(",LNAgain_"));
  Serial.print(getLNAgain());

  Serial.print(F(",LNAboostHF_"));
  if (getLNAboostHF())
  {
    Serial.print(F("On"));
  }
  else
  {
    Serial.print(F("Off"));
  }

  Serial.print(F(",LNAboostLF_"));
  if (getLNAboostLF())
  {
    Serial.print(F("On"));
  }
  else
  {
    Serial.print(F("Off"));
  }

}


void SX127XLT::setTxParams(int8_t txPower, uint8_t rampTime)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setTxParams()"));
#endif

  uint8_t param1, param2;

  writeRegister(REG_PARAMP, rampTime);       //Reg 0x0A this is same for SX1272 and SX1278
  
  if (txPower > 20)
  {
   txPower = 20;
  }
  
  if (txPower < 2)
  {
   txPower = 2;
  }
  
  if (txPower > 17)
  {
    writeRegister(REG_PADAC, 0x87);          //Reg 0x4D this is same for SX1272 and SX1278
  }
  else
  {
    writeRegister(REG_PADAC, 0x84);          //Reg 0x4D this is same for SX1272 and SX1278
  }

//now the device specifif settings
  
  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272
    
    if (txPower > 17)
    {
      param1 = (txPower + 0xEB);
      param2 = OCP_TRIM_130MA;
    }

    if (txPower <= 17)
    {
      param1 = txPower + 0xEE;
      param2 = OCP_TRIM_100MA;
    }

    if (txPower <= 5)
    {
      param1 = txPower + 0xEE;
      param2 = OCP_TRIM_80MA;
    }
    
  }
  else
  {
    //for device SX1272

    if (txPower > 17)
    {
      param1 = (txPower + 0x7F);
      param2 = OCP_TRIM_130MA;
    }

    if (txPower <= 17)
    {
      param1 = txPower + 0x82;
      param2 = OCP_TRIM_100MA;
    }

    if (txPower <= 5)
    {
      param1 = txPower + 0x82;
      param2 = OCP_TRIM_80MA;
    }
    
  }

  writeRegister(REG_PACONFIG, param1);       //Reg 0x09 this changes for SX1272 and SX1278
  writeRegister(REG_OCP, param2);            //Reg 0x0C this is same for SX1272 and SX1278
}

/*
void SX127XLT::setTxParams2(int8_t txPower, uint8_t rampTime)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setTxParams()"));
#endif

  uint8_t param1, param2;

  writeRegister(REG_PARAMP, rampTime);       //Reg 0x0A this is same for SX1272 and SX1278

  if (txPower >= 17)
  {
    writeRegister(REG_PADAC, 0x87);          //Reg 0x4D this is same for SX1272 and SX1278
  }
  else
  {
    writeRegister(REG_PADAC, 0x84);          //Reg 0x4D this is same for SX1272 and SX1278
  }

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272

    if (txPower >= 17)
    {
      param1 = (txPower + 0xEB);                  //
      param2 = OCP_TRIM_130MA;
    }

    if (txPower >= 10)
    {
      param1 = txPower + 0xEE;
      param2 = OCP_TRIM_100MA;
    }

    if (txPower >= 2)
    {
      param1 = txPower + 0xEE;
      param2 = OCP_TRIM_80MA;
    }

    if (txPower < 2)
    {
      param1 = 2 + 0xEE;
      param2 = OCP_TRIM_80MA;
    }
  }
  else
  {
    //for device SX1272

    if (txPower >= 17)
    {
      param1 = (txPower + 0x7F);
      param2 = OCP_TRIM_130MA;
    }

    if (txPower >= 10)
    {
      param1 = txPower + 0x82;
      param2 = OCP_TRIM_100MA;
    }

    if (txPower >= 2)
    {
      param1 = txPower + 0x82;
      param2 = OCP_TRIM_80MA;
    }

    if (txPower < 2)
    {
      param1 = 0x84;
      param2 = OCP_TRIM_80MA;
    }
  }

  writeRegister(REG_PACONFIG, param1);       //Reg 0x09 this changes for SX1272 and SX1278
  writeRegister(REG_OCP, param2);            //Reg 0x0C this is same for SX1272 and SX1278
}
*/


void SX127XLT::setPacketParams(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5)
{
  //format is PreambleLength, Fixed\Variable length packets, Packetlength, CRC mode, IQ mode

#ifdef SX127XDEBUG1
  Serial.println(F("SetPacketParams()"));
#endif

  uint8_t preambleMSB, preambleLSB, regdata;


  //*******************************************************
  //These changes are the same for SX1272 and SX127X
  //PreambleLength reg 0x20, 0x21
  preambleMSB = packetParam1 >> 8;
  preambleLSB = packetParam1 & 0xFF;
  writeRegister(REG_PREAMBLEMSB, preambleMSB);
  writeRegister(REG_PREAMBLELSB, preambleLSB);

  //TX Packetlength reg 0x22
  writeRegister(REG_PAYLOADLENGTH, packetParam3);                //when in implicit mode, this is used as receive length also
  
  //IQ mode reg 0x33
  regdata = ( (readRegister(REG_INVERTIQ)) & 0xBF );             //mask off invertIQ bit 6
  writeRegister(REG_INVERTIQ, (regdata + packetParam5));
  //*******************************************************


  //CRC mode
  _UseCRC = packetParam4;                                       //save CRC status

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272
    //Fixed\Variable length packets
    regdata = ( (readRegister(REG_MODEMCONFIG1)) & (~READ_IMPLCIT_AND_X)); //mask off bit 0
    writeRegister(REG_MODEMCONFIG1, (regdata + packetParam2));             //write out with bit 0 set appropriatly

    //CRC on payload
    regdata = ( (readRegister(REG_MODEMCONFIG2)) & (~READ_HASCRC_AND_X));  //mask off all bits bar CRC on - bit 2
    writeRegister(REG_MODEMCONFIG2, (regdata + (packetParam4 << 2)));      //write out with CRC bit 2 set appropriatly
  }
  else
  {
    //for SX1272
    //Fixed\Variable length packets
    regdata = ( (readRegister(REG_MODEMCONFIG1)) & (~READ_IMPLCIT_AND_2)); //mask off bit 2
    writeRegister(REG_MODEMCONFIG1, (regdata + (packetParam2 << 2)));      //write out with bit 2 set appropriatly

    //CRC on payload
    regdata = ( (readRegister(REG_MODEMCONFIG1)) & (~READ_HASCRC_AND_2));  //mask of all bits bar CRC on - bit 1
    writeRegister(REG_MODEMCONFIG1, (regdata + (packetParam4 << 1)));      //write out with CRC bit 1 set appropriatly
  }


}


void SX127XLT::setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4)
{
  //order is SpreadingFactor, Bandwidth, CodeRate, Optimisation

#ifdef SX127XDEBUG1
  Serial.println(F("setModulationParams()"));
#endif

  uint8_t regdata, bw;

  //Spreading factor - same for SX1272 and SX127X - reg 0x1D
  regdata = (readRegister(REG_MODEMCONFIG2) & (~READ_SF_AND_X));
  writeRegister(REG_MODEMCONFIG2, (regdata + (modParam1 << 4)));

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272

    //bandwidth
    regdata = (readRegister(REG_MODEMCONFIG1) & (~READ_BW_AND_X));
    writeRegister(REG_MODEMCONFIG1, (regdata + modParam2));

    //Coding rate
    regdata = (readRegister(REG_MODEMCONFIG1) & (~READ_CR_AND_X));
    writeRegister(REG_MODEMCONFIG1, (regdata + (modParam3)));

    //Optimisation
    if (modParam4 == LDRO_AUTO)
    {
      modParam4 = returnOptimisation(modParam2, modParam1);
    }

    regdata = (readRegister(REG_MODEMCONFIG3) & (~READ_LDRO_AND_X));
    writeRegister(REG_MODEMCONFIG3, (regdata + (modParam4 << 3)));

  }
  else
  {
    //for SX1272
    //bandwidth
    regdata = (readRegister(REG_MODEMCONFIG1) & (~READ_BW_AND_2));   //value will be LORA_BW_500 128, LORA_BW_250 64, LORA_BW_125 0

    switch (modParam2)
    {
      case LORA_BW_125:
        bw = 0x00;
        break;

      case LORA_BW_500:
        bw = 0x80;
        break;

      case LORA_BW_250:
        bw = 0x40;
        break;

      default:
        bw = 0x00;                       //defaults to LORA_BW_125
    }

    writeRegister(REG_MODEMCONFIG1, (regdata + bw));

    //Coding rate
    regdata = (readRegister(REG_MODEMCONFIG1) & (~READ_CR_AND_2));
    writeRegister(REG_MODEMCONFIG1, (regdata + (modParam3 << 2)));

    //Optimisation
    if (modParam4 == LDRO_AUTO)
    {
      modParam4 = returnOptimisation(modParam2, modParam1);
    }

    regdata = (readRegister(REG_MODEMCONFIG1) & (~READ_LDRO_AND_2));
    writeRegister(REG_MODEMCONFIG1, (regdata + modParam4));

  }
}


void SX127XLT::setRfFrequency(uint64_t freq64, int32_t offset)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setRfFrequency()"));
#endif

  freq64 = freq64 + offset;
  freq64 = ((uint64_t)freq64 << 19) / 32000000;
  writeRegister(REG_FRMSB, (uint8_t)(freq64 >> 16));
  writeRegister(REG_FRMID, (uint8_t)(freq64 >> 8));
  writeRegister(REG_FRLSB, (uint8_t)(freq64 >> 0));
}


uint32_t SX127XLT::getFreqInt()
{
  //get the current set LoRa device frequency, return as long integer
#ifdef SX127XDEBUG1
  Serial.println(F("getFreqInt()"));
#endif

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


int32_t SX127XLT::getFrequencyErrorRegValue()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getFrequencyErrorRegValue()"));
#endif

  int32_t FrequencyError;
  uint32_t msb, mid, lsb;
  uint32_t allreg;

  setMode(MODE_STDBY_RC);

  msb = readRegister(REG_FEIMSB);
  mid = readRegister(REG_FEIMID);
  lsb = readRegister(REG_FEILSB);

#ifdef SX127XDEBUG1
  Serial.println();
  Serial.print(F("Registers "));
  Serial.print(msb, HEX);
  Serial.print(F(" "));
  Serial.print(mid, HEX);
  Serial.print(F(" "));
  Serial.println(lsb, HEX);
#endif

  allreg = (uint32_t) ( msb << 16 ) | ( mid << 8 ) | lsb;

  if (allreg & 0x80000)
  {
    FrequencyError = (0xFFFFF - allreg) * -1;
  }
  else
  {
    FrequencyError = allreg;
  }

  return FrequencyError;
}


int32_t SX127XLT::getFrequencyErrorHz()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getFrequencyErrorHz()"));
#endif

  uint16_t msb, mid, lsb;
  int16_t freqerr;
  uint8_t bw;
  float bwconst;

  msb = readRegister(REG_FEIMSB);
  mid = readRegister(REG_FEIMID);
  lsb = readRegister(REG_FEILSB);

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272
    bw = (readRegister(REG_MODEMCONFIG1)) & (0xF0);

    switch (bw)
    {
      case LORA_BW_125:           //ordered with most commonly used first
        bwconst = 2.097;
        break;

      case LORA_BW_062:
        bwconst = 1.049;
        break;

      case LORA_BW_041:
        bwconst = 0.6996;
        break;

      case LORA_BW_007:
        bwconst = 0.1309;
        break;

      case LORA_BW_010:
        bwconst = 0.1745;
        break;

      case LORA_BW_015:
        bwconst = 0.2617;
        break;

      case LORA_BW_020:
        bwconst = 0.3490;
        break;

      case LORA_BW_031:
        bwconst = 0.5234;
        break;

      case LORA_BW_250:
        bwconst = 4.194;
        break;

      case LORA_BW_500:
        bwconst = 8.389;
        break;

      default:
        bwconst = 0x00;
    }
  }
  else
  {
    //for the SX1272
    bw = (readRegister(REG_MODEMCONFIG1)) & (0xF0);

    switch (bw)
    {
      case 0:                   //this is LORA_BW_125
        bwconst = 2.097;
        break;

      case 64:                  //this is LORA_BW_250
        bwconst = 4.194;
        break;

      case 128:                 //this is LORA_BW_250
        bwconst = 8.389;
        break;

      default:
        bwconst = 0x00;
    }


  }

  freqerr = msb << 12;          //shift lower 4 bits of msb into high 4 bits of freqerr
  mid = (mid << 8) + lsb;
  mid = (mid >> 4);
  freqerr = freqerr + mid;

  freqerr = (int16_t) (freqerr * bwconst);

  return freqerr;
}


void SX127XLT::setTx(uint32_t timeout)
{
  //There is no TX timeout function for SX127X, the value passed is ignored

#ifdef SX127XDEBUG1
  Serial.println(F("setTx()"));
#endif

  UNUSED(timeout);

  clearIrqStatus(IRQ_RADIO_ALL);

  /* This function not used on current SX127x
    if (_rxtxpinmode)
    {
    rxEnable();
    }
  */

  writeRegister(REG_OPMODE, (MODE_TX + 0x88));    //TX on LoRa mode
}


void SX127XLT::setRx(uint32_t timeout)
{
  //no timeout in this routine, left in for compatibility
#ifdef SX127XDEBUG1
  Serial.println(F("setRx()"));
#endif

  UNUSED(timeout);

  clearIrqStatus(IRQ_RADIO_ALL);

  /* This function not used on current SX127x
    if (_rxtxpinmode)
    {
    rxEnable();
    }
  */

  writeRegister(REG_OPMODE, (MODE_RXCONTINUOUS + 0x80));    //RX on LoRa mode
}


bool SX127XLT::readTXIRQ()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readTXIRQ()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_TX_DONE)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX127XLT::readRXIRQ()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readRXIRQ()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_RX_DONE)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX127XLT::setLowPowerReceive()
{
  //set RX power saving mode

#ifdef SX127XDEBUG1
  Serial.println(F("setLowPowerReceive()"));
#endif

  writeRegister(REG_LNA, 0x20 ); //at HF 100% LNA current
}


void SX127XLT::setHighSensitivity()
{
  //set Boosted LNA for HF mode

#ifdef SX127XDEBUG1
  Serial.println(F("setHighSensitivity()"));
#endif

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272
    writeRegister(REG_LNA, 0x3B ); //at HF 150% LNA current.
  }
  else
  {
    //for SX1272
    writeRegister(REG_LNA, 0x23 ); //at HF 10% LNA current.
  }
}


void SX127XLT::setRXGain(uint8_t config)
{
  //set RX power saving mode

#ifdef SX127XDEBUG1
  Serial.println(F("setRXGain( )"));
#endif

  uint8_t regdata;

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272
    regdata = readRegister(REG_MODEMCONFIG3);
    writeRegister(REG_MODEMCONFIG3, (regdata &  (~READ_AGCAUTO_AND_X))); //clear bit AgcAutoOn (2) to ensure RegLNA is controlling gain
    writeRegister(REG_LNA, config);
  }
  else
  {
    regdata = readRegister(REG_MODEMCONFIG2);
    writeRegister(REG_MODEMCONFIG2, (regdata & (~READ_AGCAUTO_AND_2))); //clear bit AgcAutoOn (2) to ensure RegLNA is controlling gain
    writeRegister(REG_LNA, config);
  }
}



uint8_t SX127XLT::getAGC()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getAGC()"));
#endif

  uint8_t regdata;

  if (_Device != DEVICE_SX1272)
  {
    regdata = readRegister(REG_MODEMCONFIG3);
    regdata = (regdata & READ_AGCAUTO_AND_X);
  }
  else
  {
    regdata = readRegister(REG_MODEMCONFIG2);
    regdata = (regdata & READ_AGCAUTO_AND_2);
  }

  return (regdata >> 2);
}


uint8_t SX127XLT::getLNAgain()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getLNAgain()"));
#endif

  uint8_t regdata;
  regdata = readRegister(REG_LNA);
  regdata = (regdata & READ_LNAGAIN_AND_X);
  return (regdata >> 5);
}


uint8_t SX127XLT::getCRCMode()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getCRCMode()"));
#endif

  uint8_t regdata;

  regdata = readRegister(REG_MODEMCONFIG2);

  if (_Device != DEVICE_SX1272)
  {
    regdata = readRegister(REG_MODEMCONFIG2);
    regdata = ((regdata & READ_HASCRC_AND_X) >> 2);
  }
  else
  {
    regdata = readRegister(REG_MODEMCONFIG1);
    regdata = ((regdata & READ_HASCRC_AND_2) >> 1);
  }

  return regdata;
}


uint8_t SX127XLT::getHeaderMode()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getHeaderMode()"));
#endif

  uint8_t regdata;

  regdata = readRegister(REG_MODEMCONFIG1);

  if (_Device != DEVICE_SX1272)
  {
    regdata = (regdata & READ_IMPLCIT_AND_X);
  }
  else
  {
    regdata = ((regdata & READ_IMPLCIT_AND_2) >> 2);
  }

  return regdata;
}


uint8_t SX127XLT::getLNAboostLF()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getLNAboostLF"));
#endif

  uint8_t regdata;

  regdata = readRegister(REG_LNA);

  if (_Device != DEVICE_SX1272)
  {
    regdata = (regdata & READ_LNABOOSTLF_AND_X);
  }
  else
  {
    regdata = (regdata & READ_LNABOOSTLF_AND_X);
  }

  return (regdata >> 3);
}


uint8_t SX127XLT::getLNAboostHF()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getLNAboostHF()"));
#endif

  uint8_t regdata;

  regdata = readRegister(REG_LNA);

  if (_Device != DEVICE_SX1272)
  {
    regdata = (regdata & READ_LNABOOSTHF_AND_X);
  }
  else
  {
    regdata = (regdata & READ_LNABOOSTHF_AND_X);
  }

  return regdata;
}


uint8_t SX127XLT::getOpmode()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getOpmode()"));
#endif

  uint8_t regdata;

  regdata = (readRegister(REG_OPMODE) & READ_OPMODE_AND_X);

  return regdata;
}


uint8_t SX127XLT::getPacketMode()
{
  //its either LoRa or FSK

#ifdef SX127XDEBUG1
  Serial.println(F("getPacketMode()"));
#endif

  uint8_t regdata;

  regdata = (readRegister(REG_OPMODE) & READ_RANGEMODE_AND_X);

  return (regdata >> 7);
}


uint8_t SX127XLT::readRXPacketL()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readRXPacketL()"));
#endif

  _RXPacketL = readRegister(REG_RXNBBYTES);
  return _RXPacketL;
}


uint8_t SX127XLT::readPacketRSSI()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readPacketRSSI()"));
#endif

  uint8_t regdata;
  regdata = readRegister(REG_PKTRSSIVALUE);
  _PacketRSSI = (157 - regdata) * (-1);
  return _PacketRSSI;
}


uint8_t SX127XLT::readPacketSNR()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readPacketSNR()"));
#endif

  uint8_t regdata;
  regdata = readRegister(REG_PKTSNRVALUE);

  if (regdata > 127)
  {
    _PacketSNR  =  ((255 - regdata) / 4) * (-1);
  }
  else
  {
    _PacketSNR  = regdata / 4;
  }

  return _PacketSNR;
}


bool SX127XLT::readPacketCRCError()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readPacketCRCError()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_CRC_ERROR)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX127XLT::readPacketHeaderValid()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readPacketHeaderValid()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_HEADER_VALID)
  {
    return true;
  }
  else
  {
    return false;
  }
}


uint8_t SX127XLT::packetOK()
{
#ifdef SX127XDEBUG1
  Serial.println(F("packetOK("));
#endif

  bool packetHasCRC;

  packetHasCRC = (readRegister(REG_HOPCHANNEL) & 0x40);                      //read the packet has CRC bit in RegHopChannel

#ifdef DEBUGPHANTOM
  Serial.print(F("PacketHasCRC = "));
  Serial.println(packetHasCRC);
  Serial.print(F("_UseCRC = "));
  Serial.println(_UseCRC);
#endif

  if ( !packetHasCRC && _UseCRC )
  {
    _IRQmsb = _IRQmsb + IRQ_NO_PACKET_CRC;                              //flag the phantom packet
    return 0;
  }

  if ( readIrqStatus() != (IRQ_RX_DONE + IRQ_HEADER_VALID) )
  {
    return 0;                                                           //no RX done and header valid only, could be CRC error
  }

  return 1;                                                             //packet is OK
}


uint8_t SX127XLT::readRXPacketType()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readRXPacketType()"));
#endif

  return _RXPacketType;
}


uint8_t SX127XLT::readRXDestination()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readRXDestination()"));
#endif
  return _RXDestination;
}


uint8_t SX127XLT::readRXSource()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readRXSource()"));
#endif

  return _RXSource;
}



void SX127XLT::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setBufferBaseAddress()"));
#endif

  writeRegister(REG_FIFOTXBASEADDR, txBaseAddress);
  writeRegister(REG_FIFORXBASEADDR, rxBaseAddress);
}


void SX127XLT::setPacketType(uint8_t packettype )
{
#ifdef SX127XDEBUG1
  Serial.println(F("setPacketType()"));
#endif
  uint8_t regdata;

  regdata = (readRegister(REG_OPMODE) & 0x7F);           //save all register bits bar the LoRa\FSK bit 7

  if (packettype == PACKET_TYPE_LORA)
  {
    _PACKET_TYPE = PACKET_TYPE_LORA;
    writeRegister(REG_OPMODE, 0x80);                     //REG_OPMODE, need to set to sleep mode before configure for LoRa mode
    writeRegister(REG_OPMODE, (regdata + 0x80));         //back to original standby mode with LoRa set
  }
  else
  {
    _PACKET_TYPE = PACKET_TYPE_GFSK;
    writeRegister(REG_OPMODE, 0x00);                     //REG_OPMODE, need to set to sleep mode before configure for FSK
    writeRegister(REG_OPMODE, regdata);                  //back to original standby mode with LoRa set
  }

}


void SX127XLT::clearIrqStatus(uint16_t irqMask)
{
#ifdef SX127XDEBUG1
  Serial.println(F("clearIrqStatus()"));
#endif

  uint8_t masklsb;
  uint16_t maskmsb;
  _IRQmsb = _IRQmsb & 0xFF00;                                 //make sure _IRQmsb does not have LSB bits set.
  masklsb = (irqMask & 0xFF);
  maskmsb = (irqMask & 0xFF00);
  writeRegister(REG_IRQFLAGS, masklsb);                        //clear standard IRQs
  _IRQmsb = (_IRQmsb & (~maskmsb));                               //only want top bits set.
}


uint16_t SX127XLT::readIrqStatus()
{
#ifdef SX127XDEBUG1
  Serial.print(F("readIrqStatus()"));
#endif

  bool packetHasCRC;

  packetHasCRC = (readRegister(REG_HOPCHANNEL) & 0x40);                      //read the packet has CRC bit in RegHopChannel

#ifdef DEBUGPHANTOM
  Serial.print(F("PacketHasCRC = "));
  Serial.println(packetHasCRC);
  Serial.print(F("_UseCRC = "));
  Serial.println(_UseCRC);
#endif

  if ( !packetHasCRC && _UseCRC )                                          //check if packet header indicates no CRC on packet, byt use CRC set
  {
    _IRQmsb = _IRQmsb + IRQ_NO_PACKET_CRC;                                  //flag the phantom packet
  }

  return (readRegister(REG_IRQFLAGS) + _IRQmsb);
}


void SX127XLT::setDioIrqParams(uint16_t irqMask, uint16_t dio0Mask, uint16_t dio1Mask, uint16_t dio2Mask)
{
  //note the irqmask contains the bit values of the interrupts that are allowed, so for all interrupts value is 0xFFFF

#ifdef SX127XDEBUG1
  Serial.println(F("setDioIrqParams()"));
#endif

  uint8_t mask0, mask1, mask2;

  UNUSED(dio2Mask);                      //variable left in call for compatibility with other libraries


  switch (dio0Mask)
  {
    case IRQ_RX_DONE:
      mask0 = 0;
      break;

    case IRQ_TX_DONE:
      mask0 = 0x40;
      break;

    case IRQ_CAD_DONE:
      mask0 = 0x80;
      break;

    default:
      mask0 = 0x0C;
  }

  switch (dio1Mask)               //for compatibility with other libraries IRQ_RX_TIMEOUT = IRQ_RX_TX_TIMEOUT
  {
    case IRQ_RX_TIMEOUT:
      mask1 = 0;
      break;

    case IRQ_FSHS_CHANGE_CHANNEL:
      mask1 = 0x10;
      break;

    case IRQ_CAD_ACTIVITY_DETECTED:
      mask1 = 0x20;
      break;

    default:
      mask1 = 0x30;
  }

  mask2 = 0x00;                  //is always IRQ_FSHS_CHANGE_CHANNEL

  writeRegister(REG_IRQFLAGSMASK, ~irqMask);
  writeRegister(REG_DIOMAPPING1, (mask0 + mask1 + mask2));
}



void SX127XLT::printIrqStatus()
{
#ifdef SX127XDEBUG1
  Serial.println(F("printIrqStatus()"));
#endif

  uint8_t _IrqStatus;
  _IrqStatus = readIrqStatus();

  //0x01
  if (_IrqStatus & IRQ_CAD_ACTIVITY_DETECTED)
  {
    Serial.print(F(",IRQ_CAD_ACTIVITY_DETECTED"));
  }

  //0x02
  if (_IrqStatus & IRQ_FSHS_CHANGE_CHANNEL)
  {
    Serial.print(F(",IRQ_FSHS_CHANGE_CHANNEL"));
  }

  //0x04
  if (_IrqStatus & IRQ_CAD_DONE)
  {
    Serial.print(F(",IRQ_CAD_DONE"));
  }

  //0x08
  if (_IrqStatus & IRQ_TX_DONE)
  {
    Serial.print(F(",IRQ_TX_DONE"));
  }

  //0x10
  if (_IrqStatus & IRQ_HEADER_VALID)
  {
    Serial.print(F(",IRQ_HEADER_VALID"));
  }

  //0x20
  if (_IrqStatus & IRQ_CRC_ERROR)
  {
    Serial.print(F(",IRQ_CRC_ERROR"));
  }

  //0x40
  if (_IrqStatus & IRQ_RX_DONE)
  {
    Serial.print(F(",IRQ_RX_DONE"));
  }

  //0x80
  if (_IrqStatus & IRQ_RX_TIMEOUT )
  {
    Serial.print(F(",IRQ_RX_TIMEOUT "));
  }

  if (_IRQmsb & IRQ_TX_TIMEOUT  )
  {
    Serial.print(F(",TX_TIMEOUT"));
  }

  if (_IRQmsb & IRQ_RX_TIMEOUT  )
  {
    Serial.print(F(",RX_TIMEOUT"));
  }

  if (_IRQmsb & IRQ_NO_PACKET_CRC  )
  {
    Serial.print(F(",NO_PACKET_CRC"));
  }

}


void SX127XLT::printASCIIPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX127XDEBUG1
  Serial.println(F("printASCIIPacket()"));
#endif

  uint8_t index;

  for (index = 0; index < size; index++)
  {
    Serial.write(buffer[index]);
  }

}


void SX127XLT::printHEXPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX127XDEBUG1
  Serial.println(F("printHEXPacket()"));
#endif

  uint8_t index;

  for (index = 0; index < size; index++)
  {
    printHEXByte(buffer[index]);
    Serial.print(F(" "));
  }
}




void SX127XLT::printASCIIorHEX(uint8_t temp)
{
  //prints as ASCII if within range, otherwise as HEX.
#ifdef SX127XDEBUG1
  Serial.println(F("printASCIIorHEX()"));
#endif

  if ((temp < 0x10) || (temp > 0x7E))
  {
    Serial.print(F(" ("));
    printHEXByte(temp);
    Serial.print(F(") "));
  }
  else
  {
    Serial.write(temp);
  }
}


void SX127XLT::printHEXByte(uint8_t temp)
{
#ifdef SX127XDEBUG1
  Serial.println(F("printHEXByte()"));
#endif

  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void SX127XLT::printHEXByte0x(uint8_t temp)
{
  //print a byte, adding 0x

#ifdef SX127XDEBUG1
  Serial.println(F("printHEXByte0x()"));
#endif

  Serial.print(F("0x"));
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


bool SX127XLT::isRXdone()
{
#ifdef SX127XDEBUG1
  Serial.println(F("isRXdone()"));
#endif

  return digitalRead(_DIO0);
}

bool SX127XLT::isTXdone()
{
#ifdef SX127XDEBUG1
  Serial.println(F("isTXdone()"));
#endif

  return digitalRead(_DIO0);
}


bool SX127XLT::isRXdoneIRQ()
{
#ifdef SX127XDEBUG1
  Serial.println(F("isRXdoneIRQ()"));
#endif

  return (readRegister(REG_IRQFLAGS) & IRQ_RX_DONE);
}


bool SX127XLT::isTXdoneIRQ()
{
#ifdef SX127XDEBUG1
  Serial.println(F("isTXdoneIRQ()"));
#endif

  return (readRegister(REG_IRQFLAGS) & IRQ_TX_DONE);
}


void SX127XLT::setTXDonePin(uint8_t pin)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setTXDonePin()"));
#endif

  _TXDonePin = pin;
}


void SX127XLT:: setRXDonePin(uint8_t pin)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setRXDonePin()"));
#endif

  _RXDonePin = pin;
}



uint8_t SX127XLT::receive(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait )
{
#ifdef SX127XDEBUG1
  Serial.println(F("receive()"));
#endif

  uint16_t index;
  uint32_t endtimeoutmS;
  uint8_t regdata;

  setMode(MODE_STDBY_RC);
  regdata = readRegister(REG_FIFORXBASEADDR);                               //retrieve the RXbase address pointer
  writeRegister(REG_FIFOADDRPTR, regdata);                                  //and save in FIFO access ptr

  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_HEADER_VALID), 0, 0);  //set for IRQ on RX done
  setRx(0);                                                                //no actual RX timeout in this function

  if (!wait)
  {
    return 0;                                                              //not wait requested so no packet length to pass
  }

  if (rxtimeout == 0)
  {
    while (!digitalRead(_RXDonePin));                                      //Wait for DIO0 to go high, no timeout, RX DONE
  }
  else
  {
    endtimeoutmS = millis() + rxtimeout;
    while (!digitalRead(_RXDonePin) && (millis() < endtimeoutmS));
  }

  setMode(MODE_STDBY_RC);                                                   //ensure to stop further packet reception

  if (!digitalRead(_RXDonePin))                                             //check if DIO still low, is so must be RX timeout
  {
    _IRQmsb = IRQ_RX_TIMEOUT;
    return 0;
  }


  if ( readIrqStatus() != (IRQ_RX_DONE + IRQ_HEADER_VALID) )
  {
    return 0;                                                                //no RX done and header valid only, could be CRC error
  }

  _RXPacketL = readRegister(REG_RXNBBYTES);

  if (_RXPacketL > size)                      //check passed buffer is big enough for packet
  {
    _RXPacketL = size;                        //truncate packet if not enough space in passed buffer
  }

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                    //start the burst read
  SPI.transfer(REG_FIFO);

  for (index = 0; index < _RXPacketL; index++)
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _RXPacketL;                           //so we can check for packet having enough buffer space
}


uint8_t SX127XLT::receiveAddressed(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait)
{
#ifdef SX127XDEBUG1
  Serial.println(F("receiveAddressed()"));
#endif

  uint16_t index;
  uint32_t endtimeoutmS;
  uint8_t regdata;

  setMode(MODE_STDBY_RC);
  regdata = readRegister(REG_FIFORXBASEADDR);                                //retrieve the RXbase address pointer
  writeRegister(REG_FIFOADDRPTR, regdata);                                   //and save in FIFO access ptr

  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_HEADER_VALID), 0, 0);   //set for IRQ on RX done
  setRx(0);                                                                 //no actual RX timeout in this function

  if (!wait)
  {
    return 0;
  }

  if (rxtimeout == 0)
  {
    while (!digitalRead(_RXDonePin));                                      //Wait for DIO0 to go high, no timeout, RX DONE
  }
  else
  {
    endtimeoutmS = millis() + rxtimeout;
    while (!digitalRead(_RXDonePin) && (millis() < endtimeoutmS));
  }

  setMode(MODE_STDBY_RC);                                          //ensure to stop further packet reception

  if (!digitalRead(_RXDonePin))                                         //check if not DIO still low, is so must be RX timeout
  {
    _IRQmsb = IRQ_RX_TIMEOUT;
    return 0;
  }

  if ( readIrqStatus() != (IRQ_RX_DONE + IRQ_HEADER_VALID) )
  {
    return 0;                       //no RX done and header valid only, could be CRC error
  }

  _RXPacketL = readRegister(REG_RXNBBYTES);

  if (_RXPacketL > size)                      //check passed buffer is big enough for packet
  {
    _RXPacketL = size;                          //truncate packet if not enough space in passed buffer
  }

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                    //start the burst read
  SPI.transfer(REG_FIFO);

  _RXPacketType = SPI.transfer(0);
  _RXDestination = SPI.transfer(0);
  _RXSource = SPI.transfer(0);

  for (index = 0; index < _RXPacketL; index++)
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _RXPacketL;                           //so we can check for packet having enough buffer space
}



uint8_t SX127XLT::readPacket(uint8_t *rxbuffer, uint8_t size)
{
#ifdef SX127XDEBUG1
  Serial.println(F("readPacket()"));
#endif

  uint8_t index, regdata;

  if ( readIrqStatus() != (IRQ_RX_DONE + IRQ_HEADER_VALID) )
  {
    return 0;                                 //no RX done and header valid only, could be CRC error
  }

  setMode(MODE_STDBY_RC);
  regdata = readRegister(REG_FIFORXBASEADDR);  //retrieve the RXbase address pointer
  writeRegister(REG_FIFOADDRPTR, regdata);     //and save in FIFO access ptr

  _RXPacketL = readRegister(REG_RXNBBYTES);

  if (_RXPacketL > size)                      //check passed buffer is big enough for packet
  {
    _RXPacketL = size;                          //truncate packet if not enough space
  }

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                    //start the burst read
  SPI.transfer(REG_FIFO);

  for (index = 0; index < _RXPacketL; index++)
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _RXPacketL;                           //so we can check for packet having enough buffer space
}


uint8_t SX127XLT::readPacketAddressed(uint8_t *rxbuffer, uint8_t size)
{
#ifdef SX127XDEBUG1
  Serial.println(F("readPacketAddressed()"));
#endif

  uint8_t index, regdata;

  setMode(MODE_STDBY_RC);
  regdata = readRegister(REG_FIFORXBASEADDR);  //retrieve the RXbase address pointer
  writeRegister(REG_FIFOADDRPTR, regdata);     //and save in FIFO access ptr

  _RXPacketL = readRegister(REG_RXNBBYTES);

  if (_RXPacketL > size)                      //check passed buffer is big enough for packet
  {
    _RXPacketL = size;                          //truncate packet if not enough space
  }

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                    //start the burst read
  SPI.transfer(REG_FIFO);
  _RXPacketType = SPI.transfer(0);
  _RXDestination = SPI.transfer(0);
  _RXSource = SPI.transfer(0);

  for (index = 0; index < _RXPacketL; index++)
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _RXPacketL;                           //so we can check for packet having enough buffer space
}


uint8_t SX127XLT::transmit(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait)
{
#ifdef SX127XDEBUG1
  Serial.println(F("transmit()"));
#endif

  uint8_t index, ptr;
  uint8_t bufferdata;
  uint32_t endtimeoutmS;

  if (size == 0)
  {
    return false;
  }

  setMode(MODE_STDBY_RC);
  ptr = readRegister(REG_FIFOTXBASEADDR);       //retrieve the TXbase address pointer
  writeRegister(REG_FIFOADDRPTR, ptr);          //and save in FIFO access ptr

#ifdef USE_SPI_TRANSACTION                   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(WREG_FIFO);

  for (index = 0; index < size; index++)
  {
    bufferdata = txbuffer[index];
    SPI.transfer(bufferdata);
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  _TXPacketL = size;
  writeRegister(REG_PAYLOADLENGTH, _TXPacketL);

  setTxParams(txpower, RADIO_RAMP_DEFAULT);             //TX power and ramp time

  setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);   //set for IRQ on TX done on first DIO pin
  setTx(0);                                            //TX timeout is not handled in setTX()

  if (!wait)
  {
    return _TXPacketL;
  }

  if (txtimeout == 0)
  {
    while (!digitalRead(_TXDonePin));                  //Wait for pin to go high, TX finished
  }
  else
  {
    endtimeoutmS = (millis() + txtimeout);
    while (!digitalRead(_TXDonePin) && (millis() < endtimeoutmS));
  }

  setMode(MODE_STDBY_RC);                                            //ensure we leave function with TX off

  if (!digitalRead(_TXDonePin))
  {
    _IRQmsb = IRQ_TX_TIMEOUT;
    return 0;
  }

  return _TXPacketL;                                                     //no timeout, so TXdone must have been set
}



uint8_t SX127XLT::transmitAddressed(uint8_t *txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, uint32_t txtimeout, int8_t txpower, uint8_t wait )
{
#ifdef SX127XDEBUG1
  Serial.println(F("transmitAddressed()"));
#endif

  uint8_t index, ptr;
  uint8_t bufferdata;
  uint32_t endtimeoutmS;

  if (size == 0)
  {
    return false;
  }

  setMode(MODE_STDBY_RC);
  ptr = readRegister(REG_FIFOTXBASEADDR);         //retrieve the TXbase address pointer
  writeRegister(REG_FIFOADDRPTR, ptr);            //and save in FIFO access ptr

#ifdef USE_SPI_TRANSACTION                     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(WREG_FIFO);
  SPI.transfer(txpackettype);                     //Write the packet type
  SPI.transfer(txdestination);                    //Destination node
  SPI.transfer(txsource);                         //Source node
  _TXPacketL = 3 + size;                          //we have added 3 header bytes to size

  for (index = 0; index < size; index++)
  {
    bufferdata = txbuffer[index];
    SPI.transfer(bufferdata);
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  writeRegister(REG_PAYLOADLENGTH, _TXPacketL);

  setTxParams(txpower, RADIO_RAMP_DEFAULT);             //TX power and ramp time

  setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);   //set for IRQ on TX done
  setTx(0);                                            //TX timeout is not handled in setTX()

  if (!wait)
  {
    return _TXPacketL;
  }

  endtimeoutmS = (millis() + txtimeout);

  if (txtimeout == 0)
  {
    while (!digitalRead(_TXDonePin));                                      //Wait for DIO0 to go high, TX finished
  }
  else
  {
    //endtimeoutmS = (millis() + txtimeout);
    while (!digitalRead(_TXDonePin) && (millis() < endtimeoutmS));
  }

  setMode(MODE_STDBY_RC);                                             //ensure we leave function with TX off

  if (millis() >= endtimeoutmS)                                         //flag if TX timeout
  {
    _IRQmsb = IRQ_TX_TIMEOUT;
    return 0;
  }

  return _TXPacketL;                                                     //no timeout, so TXdone must have been set

}



void SX127XLT::setupLoRa(uint32_t Frequency, int32_t Offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t modParam4)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setupLoRa()"));
#endif

  setMode(MODE_STDBY_RC);                            //go into standby mode to configure device
  setPacketType(PACKET_TYPE_LORA);                   //use LoRa packets
  setRfFrequency(Frequency, Offset);                 //set the operating frequncy
  calibrateImage(0);                                 //run calibration after setting frequency
  setModulationParams(modParam1, modParam2, modParam3, modParam4);
  setBufferBaseAddress(0x00, 0x00);                  //set bases addresses for TX and RX packets in SX buffer
  setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL); //set the packet options
  setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);            //syncword to use, 0x12 for standard private, 0x34 for public (LORAWAN)
  setHighSensitivity();
}



uint32_t SX127XLT::getLoRaBandwidth()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getLoRaBandwidth()"));
#endif

  uint8_t regdata;


  if (_Device == DEVICE_SX1272)
  {
    regdata = (readRegister(REG_MODEMCONFIG1) & READ_BW_AND_2);
    switch (regdata)
    {
      case 0:
        return 125000;

      case 64:
        return 250000;

      case 128:
        return 500000;

      default:
        return 0xFF;                      //so that a bandwidth invalid entry can be identified ?
    }
  }
  else
  {

    regdata = (readRegister(REG_MODEMCONFIG1) & READ_BW_AND_X);

    switch (regdata)
    {
      case 0:
        return 7800;

      case 16:
        return 10400;

      case 32:
        return 15600;

      case 48:
        return 20800;

      case 64:
        return 31200;

      case 80:
        return 41700;

      case 96:
        return 62500;

      case 112:
        return 125000;

      case 128:
        return 250000;

      case 144:
        return 500000;

      default:
        return 0xFF;                      //so that a bandwidth invalid entry can be identified ?
    }

  }
}


uint8_t SX127XLT::getLoRaSF()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getLoRaSF()"));
#endif

  uint8_t regdata;
  regdata = readRegister(REG_MODEMCONFIG2);
  regdata = ((regdata & READ_SF_AND_X) >> 4);       //SX1272 and SX1276 store SF in same place

  return regdata;
}


uint8_t SX127XLT::getLoRaCodingRate()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getLoRaCodingRate"));
#endif

  uint8_t regdata;
  regdata = readRegister(REG_MODEMCONFIG1);

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272
    regdata = (((regdata & READ_CR_AND_X) >> 1) + 4);
  }
  else
  {
    regdata = (((regdata & READ_CR_AND_2) >> 3) + 4);
  }

  return regdata;
}


uint8_t SX127XLT::getOptimisation()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getOptimisation"));
#endif

  uint8_t regdata;

  if (_Device != DEVICE_SX1272)
  {
    //for all devices apart from SX1272
    regdata = readRegister(REG_MODEMCONFIG3);
    regdata = ((regdata & READ_LDRO_AND_X) >> 3);
  }
  else
  {
    regdata = readRegister(REG_MODEMCONFIG1);
    regdata = (regdata & READ_LDRO_AND_2);
  }

  return regdata;
}


uint8_t SX127XLT::getSyncWord()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getSyncWord"));
#endif

  return readRegister(REG_SYNCWORD);
}


uint8_t SX127XLT::getInvertIQ()
{
  //IQ mode reg 0x33

#ifdef SX127XDEBUG1
  Serial.println(F("getInvertIQ"));
#endif

  uint8_t regdata;
  regdata =  ( (readRegister(REG_INVERTIQ)) & 0x40 );
  return regdata;
}


uint8_t SX127XLT::getVersion()
{
  //IQ mode reg 0x33

#ifdef SX127XDEBUG1
  Serial.println(F("getVersion"));
#endif

  return readRegister(REG_VERSION);
}


uint16_t SX127XLT::getPreamble()
{
#ifdef SX127XDEBUG1
  Serial.println(F("getPreamble"));
#endif

  uint16_t regdata;
  regdata =  ( (readRegister(REG_PREAMBLEMSB) << 8) + readRegister(REG_PREAMBLELSB) );
  return regdata;
}



uint32_t SX127XLT::returnBandwidth(byte BWregvalue)
{
#ifdef SX127XDEBUG1
  Serial.println(F("returnBandwidth()"));
#endif

  switch (BWregvalue)
  {
    case 0:
      return 7800;

    case 16:
      return 10400;

    case 32:
      return 15600;

    case 48:
      return 20800;

    case 64:
      return 31200;

    case 80:
      return 41700;

    case 96:
      return 62500;

    case 112:
      return 125000;

    case 128:
      return 250000;

    case 144:
      return 500000;

    default:
      break;
  }
  return 0xFFFF;                      //so that a bandwidth not set can be identified
}


uint8_t SX127XLT::returnOptimisation(uint8_t Bandwidth, uint8_t SpreadingFactor)
{
  //from the passed bandwidth (bandwidth) and spreading factor this routine
  //calculates whether low data rate optimisation should be on or off

#ifdef SX127XDEBUG1
  Serial.println(F("returnOptimisation()"));
#endif

  uint32_t tempBandwidth;
  float symbolTime;
  tempBandwidth = returnBandwidth(Bandwidth);
  symbolTime = calcSymbolTime(tempBandwidth, SpreadingFactor);

#ifdef SX127XDEBUG1
  Serial.print(F("Symbol Time "));
  Serial.print(symbolTime, 3);
  Serial.println(F("mS"));
#endif

  if (symbolTime > 16)
  {
#ifdef SX127XDEBUG1
    Serial.println(F("LDR Opt on"));
#endif
    return LDRO_ON;
  }
  else
  {
#ifdef SX127XDEBUG1
    Serial.println(F("LDR Opt off"));
#endif
    return LDRO_OFF;
  }
}


float SX127XLT::calcSymbolTime(float Bandwidth, uint8_t SpreadingFactor)
{
  //calculates symbol time from passed bandwidth (lbandwidth) and Spreading factor (lSF)and returns in mS

#ifdef SX127XDEBUG1
  Serial.println(F("calcSymbolTime()"));
#endif

  float symbolTimemS;
  symbolTimemS = (Bandwidth / pow(2, SpreadingFactor));
  symbolTimemS = (1000 / symbolTimemS);
  return symbolTimemS;
}


void SX127XLT::printLoraSettings()
{
#ifdef SX127XDEBUG1
  Serial.println(F("printLoraSettings()"));
#endif

  printDevice();
  Serial.print(F(","));
  Serial.print(getFreqInt());
  Serial.print(F("hz,SF"));
  Serial.print(getLoRaSF());
  Serial.print(F(",BW"));
  Serial.print(getLoRaBandwidth());
  Serial.print(F(",CR4:"));
  Serial.print(getLoRaCodingRate());
  Serial.print(F(",LDRO_"));

  if (getOptimisation())
  {
    Serial.print(F("On"));
  }
  else
  {
    Serial.print(F("Off"));
  }

  Serial.print(F(",SyncWord_0x"));
  Serial.print(getSyncWord(), HEX);
  if (getInvertIQ() == LORA_IQ_INVERTED)
  {
    Serial.print(F(",IQInverted"));
  }
  else
  {
    Serial.print(F(",IQNormal"));
  }
  Serial.print(F(",Preamble_"));
  Serial.print(getPreamble());
}


void SX127XLT::setSyncWord(uint8_t syncword)
{
#ifdef SX127XDEBUG1
  Serial.println(F("setSyncWord()"));
#endif

  writeRegister(REG_SYNCWORD, syncword);
}


uint8_t SX127XLT::receiveSXBuffer(uint8_t startaddr, uint32_t rxtimeout, uint8_t wait )
{
#ifdef SX127XDEBUG1
  Serial.println(F("receiveSXBuffer()"));
#endif

  uint32_t endtimeoutmS;

  setMode(MODE_STDBY_RC);
  writeRegister(REG_FIFORXBASEADDR, startaddr);          //set start address of RX packet in buffer
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_HEADER_VALID), 0, 0);   //set for IRQ on RX done
  setRx(0);                                                                 //no actual RX timeout in this function

  if (!wait)
  {
    return 0;
  }

  if (rxtimeout == 0)
  {
    while (!digitalRead(_RXDonePin));                                    //Wait for DIO0 to go high, no timeout, RX DONE
  }
  else
  {
    endtimeoutmS = millis() + rxtimeout;
    while (!digitalRead(_RXDonePin) && (millis() < endtimeoutmS));
  }

  setMode(MODE_STDBY_RC);                                          //ensure to stop further packet reception

  if (!digitalRead(_RXDonePin))                                         //check if not DIO still low, is so must be RX timeout
  {
    _IRQmsb = IRQ_RX_TIMEOUT;
    return 0;
  }

  if ( readIrqStatus() != (IRQ_RX_DONE + IRQ_HEADER_VALID) )
  {
    return 0;                       //no RX done and header valid only, could be CRC error
  }

  _RXPacketL = readRegister(REG_RXNBBYTES);

  return _RXPacketL;                           //so we can check for packet having enough buffer space
}



uint8_t SX127XLT::transmitSXBuffer(uint8_t startaddr, uint8_t length, uint32_t txtimeout, int8_t txpower, uint8_t wait)
{
#ifdef SX127XDEBUG1
  Serial.println(F("transmitSXBuffer()"));
#endif

  uint32_t endtimeoutmS = 0;

  setMode(MODE_STDBY_RC);

  writeRegister(REG_FIFOTXBASEADDR, startaddr);          //set start address of packet in buffer
  writeRegister(REG_PAYLOADLENGTH, length);

  setTxParams(txpower, RADIO_RAMP_DEFAULT);             //TX power and ramp time

  setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);    //set for IRQ on TX done
  setTx(0);                                             //TX timeout is not handled in setTX()

  if (!wait)
  {
    return length;
  }

  if (txtimeout == 0)
  {
    while (!digitalRead(_TXDonePin));                    //Wait for DIO0 to go high, TX finished
  }
  else
  {
    endtimeoutmS = (millis() + txtimeout);
    while (!digitalRead(_TXDonePin) && (millis() < endtimeoutmS));
  }

  setMode(MODE_STDBY_RC);                                 //ensure we leave function with TX off


  if (millis() >= endtimeoutmS)                           //flag if TX timeout
  {
    _IRQmsb = IRQ_TX_TIMEOUT;

    return 0;
  }

  return _TXPacketL;                                                     //no timeout, so TXdone must have been set
}


void SX127XLT::printSXBufferHEX(uint8_t start, uint8_t end)
{
#ifdef SX127XDEBUG1
  Serial.println(F("printSXBufferHEX()"));
#endif

  uint8_t index, regdata;

  setMode(MODE_STDBY_RC);
  writeRegister(REG_FIFOADDRPTR, start);          //set FIFO access ptr to start

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                       //start the burst read
  SPI.transfer(REG_FIFO);

  for (index = start; index <= end; index++)
  {
    regdata = SPI.transfer(0);
    printHEXByte(regdata);
    Serial.print(F(" "));

  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

}


void SX127XLT::printSXBufferASCII(uint8_t start, uint8_t end)
{
#ifdef SX127XDEBUG1
  Serial.println(F("printSXBufferASCII)"));
#endif

  uint8_t index, regdata;
  setMode(MODE_STDBY_RC);

  writeRegister(REG_FIFOADDRPTR, start);     //and save in FIFO access ptr

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                    //start the burst read
  SPI.transfer(REG_FIFO);

  for (index = 0; index <= end; index++)
  {
    regdata = SPI.transfer(0);
    Serial.write(regdata);
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif
 
}


void SX127XLT::fillSXBuffer(uint8_t startaddress, uint8_t size, uint8_t character)
{
#ifdef SX127XDEBUG1
  Serial.println(F("fillSXBuffer()"));
#endif
  uint8_t index;

  setMode(MODE_STDBY_RC);
  writeRegister(REG_FIFOADDRPTR, startaddress);     //and save in FIFO access ptr

#ifdef USE_SPI_TRANSACTION                       //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                         //start the burst write
  SPI.transfer(WREG_FIFO);

  for (index = 0; index < size; index++)
  {
    SPI.transfer(character);
  }

  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

}


uint8_t SX127XLT::getByteSXBuffer(uint8_t addr)
{
#ifdef SX127XDEBUG1
  Serial.println(F("getByteSXBuffer()"));
#endif

  uint8_t regdata;
  setMode(MODE_STDBY_RC);                     //this is needed to ensure we can read from buffer OK.

  writeRegister(REG_FIFOADDRPTR, addr);        //set FIFO access ptr to location

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                    //start the burst read
  SPI.transfer(REG_FIFO);
  regdata = SPI.transfer(0);
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return regdata;
}


void SX127XLT::writeByteSXBuffer(uint8_t addr, uint8_t regdata)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeByteSXBuffer"));
#endif

  setMode(MODE_STDBY_RC);                 //this is needed to ensure we can write to buffer OK.

  writeRegister(REG_FIFOADDRPTR, addr);    //set FIFO access ptr to location

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                //start the burst read
  SPI.transfer(WREG_FIFO);
  SPI.transfer(regdata);
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

}



void SX127XLT::startWriteSXBuffer(uint8_t ptr)
{
#ifdef SX127XDEBUG1
  Serial.println(F("startWriteSXBuffer()"));
#endif

  setMode(MODE_STDBY_RC);

  _TXPacketL = 0;                               //this variable used to keep track of bytes written
  writeRegister(REG_FIFOADDRPTR, ptr);           //set buffer access ptr

#ifdef USE_SPI_TRANSACTION                      //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(WREG_FIFO);
}


uint8_t SX127XLT::endWriteSXBuffer()
{
#ifdef SX127XDEBUG1
  Serial.println(F("endWriteSXBuffer()"));
#endif

  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _TXPacketL;
}


void SX127XLT::startReadSXBuffer(uint8_t ptr)
{
#ifdef SX127XDEBUG1
  Serial.println(F("startReadSXBuffer()"));
#endif

  setMode(MODE_STDBY_RC);
  _RXPacketL = 0;
  writeRegister(REG_FIFOADDRPTR, ptr);           //set buffer access ptr

#ifdef USE_SPI_TRANSACTION   //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);               //start the burst read
  SPI.transfer(REG_FIFO);

  //next line would be data = SPI.transfer(0);
  //SPI interface ready for byte to read from
}


uint8_t SX127XLT::endReadSXBuffer()
{
#ifdef SX127XDEBUG1
  Serial.println(F("endReadSXBuffer()"));
#endif

  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _RXPacketL;

}


void SX127XLT::writeUint8(uint8_t x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeUint8()"));
#endif

  SPI.transfer(x);

  _TXPacketL++;                     //increment count of bytes written
}


uint8_t SX127XLT::readUint8()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readUint8()"));
#endif

  uint8_t x;

  x = SPI.transfer(0);

  _RXPacketL++;                      //increment count of bytes read
  return (x);
}


void SX127XLT::writeInt8(int8_t x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeInt8()"));
#endif

  SPI.transfer(x);

  _TXPacketL++;                     //increment count of bytes written
}


int8_t SX127XLT::readInt8()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readInt8()"));
#endif

  int8_t x;

  x = SPI.transfer(0);

  _RXPacketL++;                      //increment count of bytes read
  return (x);
}


void SX127XLT::writeChar(char x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeChar()"));
#endif

  SPI.transfer(x);

  _TXPacketL++;                     //increment count of bytes written
}


char SX127XLT::readChar()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readChar()"));
#endif

  char x;

  x = SPI.transfer(0);

  _RXPacketL++;                      //increment count of bytes read
  return (x);
}


void SX127XLT::writeUint16(uint16_t x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeUint16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXPacketL = _TXPacketL + 2;         //increment count of bytes written
}


uint16_t SX127XLT::readUint16()
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeUint16()"));
#endif

  uint8_t lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXPacketL = _RXPacketL + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX127XLT::writeInt16(int16_t x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeInt16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXPacketL = _TXPacketL + 2;         //increment count of bytes written
}


int16_t SX127XLT::readInt16()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readInt16()"));
#endif

  uint8_t lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXPacketL = _RXPacketL + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX127XLT::writeUint32(uint32_t x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeUint32()"));
#endif

  uint8_t i, j;

  union
  {
    uint8_t b[4];
    uint32_t f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXPacketL = _TXPacketL + 4;         //increment count of bytes written
}


uint32_t SX127XLT::readUint32()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readUint32()"));
#endif

  uint8_t i, j;

  union
  {
    uint8_t b[4];
    uint32_t f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXPacketL = _RXPacketL + 4;         //increment count of bytes read
  return readdata.f;
}


void SX127XLT::writeInt32(int32_t x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeInt32()"));
#endif

  uint8_t i, j;

  union
  {
    uint8_t b[4];
    int32_t f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXPacketL = _TXPacketL + 4;         //increment count of bytes written
}


int32_t SX127XLT::readInt32()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readInt32()"));
#endif

  uint8_t i, j;

  union
  {
    uint8_t b[4];
    int32_t f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXPacketL = _RXPacketL + 4;         //increment count of bytes read
  return readdata.f;
}


void SX127XLT::writeFloat(float x)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeFloat()"));
#endif

  uint8_t i, j;

  union
  {
    uint8_t b[4];
    float f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXPacketL = _TXPacketL + 4;         //increment count of bytes written
}


float SX127XLT::readFloat()
{
#ifdef SX127XDEBUG1
  Serial.println(F("readFloat()"));
#endif

  uint8_t i, j;

  union
  {
    uint8_t b[4];
    float f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXPacketL = _RXPacketL + 4;         //increment count of bytes read
  return readdata.f;
}


void SX127XLT::writeBuffer(uint8_t *txbuffer, uint8_t size)
{
#ifdef SX127XDEBUG1
  Serial.println(F("writeBuffer()"));
#endif

  uint8_t index, regdata;

  _TXPacketL = _TXPacketL + size;      //these are the number of bytes that will be added

  size--;                              //loose one byte from size, the last byte written MUST be a 0

  for (index = 0; index < size; index++)
  {
    regdata = txbuffer[index];
    SPI.transfer(regdata);
  }

  SPI.transfer(0);                     //this ensures last byte of buffer writen really is a null (0)

}


uint8_t SX127XLT::readBuffer(uint8_t *rxbuffer)
{
#ifdef SX127XDEBUG1
  Serial.println(F("readBuffer()"));
#endif

  uint8_t index = 0, regdata;

  do                                     //need to find the size of the buffer first
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;             //fill the buffer.
    index++;
  } while (regdata != 0);                //keep reading until we have reached the null (0) at the buffer end
  //or exceeded size of buffer allowed
  return index;                          //return the actual size of the buffer, till the null (0) detected

}



void SX127XLT::rxtxInit(int8_t pinRXEN, int8_t pinTXEN)
{
  //not used on current SX127x modules

#ifdef SX127XDEBUG1
  Serial.println(F("rxtxInit()"));
#endif

  //_rxtxpinmode = true;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;

  pinMode(pinRXEN, OUTPUT);
  digitalWrite(pinRXEN, LOW);           //pins needed for E28-2G4M20S
  pinMode(pinTXEN, OUTPUT);
  digitalWrite(pinTXEN, LOW);           //pins needed for E28-2G4M20S
}


void SX127XLT::rxEnable()
{
#ifdef SX127XDEBUG1
  Serial.println(F("rxEnable()"));
#endif

  digitalWrite(_RXEN, HIGH);
  digitalWrite(_TXEN, LOW);
}


void SX127XLT::txEnable()
{
#ifdef SX127XDEBUG1
  Serial.println(F("txEnable()"));
#endif

  digitalWrite(_RXEN, LOW);
  digitalWrite(_TXEN, HIGH);
}








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
