/*
  Copyright 2019 - Stuart Robinson
  Licensed under a MIT license displayed at the bottom of this document.
  17/12/19
*/

/*
Parts of code Copyright (c) 2013, SEMTECH S.A.
See LICENSE.TXT file included in the library
*/


#include <SX126XLT.h>
#include <SPI.h>

#define LTUNUSED(v) (void) (v)       //add LTUNUSED(variable); to avoid compiler warnings 
#define USE_SPI_TRANSACTION

//#define DEBUGBUSY
//#define SX126XDEBUG               //enable debug messages


/*
****************************************************************************
  To Do:


****************************************************************************
*/

SX126XLT::SX126XLT()
{
  //Anything you need when instantiating your object goes here
}


bool SX126XLT::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, int8_t pinSW, uint8_t device)
{

  //assign the passed pins to the class private variables
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = pinDIO2;
  _DIO3 = pinDIO3;
  _SW = pinSW;
  _Device = device;
  _TXDonePin = pinDIO1;        //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;        //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);


#ifdef SX126XDEBUG
  Serial.println(F("begin()"));
  Serial.println(F("SX126XLT constructor instantiated successfully"));
  Serial.print(F("NSS "));
  Serial.println(_NSS);
  Serial.print(F("NRESET "));
  Serial.println(_NRESET);
  Serial.print(F("RFBUSY "));
  Serial.println(_RFBUSY);
  Serial.print(F("DIO1 "));
  Serial.println(_DIO1);
  Serial.print(F("DIO2 "));
  Serial.println(_DIO2);
  Serial.print(F("DIO3 "));
  Serial.println(_DIO3);
  Serial.print(F("SW "));
  Serial.println(_SW);
#endif



  if (_DIO1 >= 0)
  {
    pinMode( _DIO1, INPUT);
  }
  else
  {
    //Serial.println(F("DIO1 not used"));
  }

  if (_DIO2 >= 0)
  {
    pinMode( _DIO2, INPUT);
  }
  else
  {
    //Serial.println(F("DIO2 not used"));
  }

  if (_DIO3 >= 0)
  {
    pinMode( _DIO3, INPUT);
  }
  else
  {
    //Serial.println(F("DIO3 not used"));
  }

  if (_SW >= 0)
  {
    pinMode( _SW, OUTPUT);                     //Dorji devices have an RW pin that needs to be set high to power antenna switch
    digitalWrite(_SW, HIGH);
  }
  else
  {
    //Serial.println(F("RW not used"));
  }

  resetDevice();
  if (checkDevice())
  {
    return true;
  }

  return false;
}


void SX126XLT::checkBusy()
{
#ifdef SX126XDEBUG
  //Serial.println(F("checkBusy()"));
#endif

  uint8_t busy_timeout_cnt;
  busy_timeout_cnt = 0;

  while (digitalRead(_RFBUSY))
  {
    delay(1);
    busy_timeout_cnt++;

    if (busy_timeout_cnt > 10) //wait 10mS for busy to complete
    {
      busy_timeout_cnt = 0;
#ifdef DEBUGBUSY
      Serial.println(F("ERROR - Busy Timeout!"));
#endif
      resetDevice();          //reset device
      setMode(MODE_STDBY_RC);
      config();               //re-run saved config
      break;
    }
  }
}


void SX126XLT::writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX126XDEBUG
  //Serial.println(F("writeCommand()"));
#endif

  uint8_t index;
  checkBusy();

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);

  for (index = 0; index < size; index++)
  {
    SPI.transfer(buffer[index]);
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  if (Opcode != RADIO_SET_SLEEP)
  {
    checkBusy();
  }
}


void SX126XLT::readCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX126XDEBUG
  //Serial.println(F("readCommand()"));
#endif

  uint8_t i;
  checkBusy();

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);
  SPI.transfer(0xFF);

  for ( i = 0; i < size; i++ )
  {

    *(buffer + i) = SPI.transfer(0xFF);
  }
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

}


void SX126XLT::writeRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX126XDEBUG
  //Serial.println(F("writeRegisters()"));
#endif
  uint8_t addr_l, addr_h;
  uint8_t i;

  addr_l = address & 0xff;
  addr_h = address >> 8;
  checkBusy();

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_REGISTER);
  SPI.transfer(addr_h);   //MSB
  SPI.transfer(addr_l);   //LSB

  for (i = 0; i < size; i++)
  {
    SPI.transfer(buffer[i]);
  }

  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif
}


void SX126XLT::writeRegister(uint16_t address, uint8_t value)
{
#ifdef SX126XDEBUG
  //Serial.println(F("writeRegisters()"));
#endif

  writeRegisters( address, &value, 1 );
}

void SX126XLT::readRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX126XDEBUG
  //Serial.println(F("readRegisters()"));
#endif

  uint16_t index;
  uint8_t addr_l, addr_h;

  addr_h = address >> 8;
  addr_l = address & 0x00FF;
  checkBusy();

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_READ_REGISTER);
  SPI.transfer(addr_h);               //MSB
  SPI.transfer(addr_l);               //LSB
  SPI.transfer(0xFF);
  for (index = 0; index < size; index++)
  {
    *(buffer + index) = SPI.transfer(0xFF);
  }

  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

}


uint8_t SX126XLT::readRegister(uint16_t address)
{
#ifdef SX126XDEBUG
  //Serial.println(F("readRegister()"));
#endif

  uint8_t data;

  readRegisters(address, &data, 1);
  return data;
}

void SX126XLT::resetDevice()
{
#ifdef SX126XDEBUG
  Serial.println(F("resetDevice()"));
#endif

  delay(10);
  digitalWrite(_NRESET, LOW);
  delay(2);
  digitalWrite(_NRESET, HIGH);
  delay(25);
  checkBusy();
}



bool SX126XLT::checkDevice()
{
  //check there is a device out there, writes a register and reads back
#ifdef SX126XDEBUG
  Serial.println(F("checkDevice()"));
#endif

  uint8_t Regdata1, Regdata2;
  Regdata1 = readRegister(0x88e);               //low byte of frequency setting
  writeRegister(0x88e, (Regdata1 + 1));
  Regdata2 = readRegister(0x88e);               //read changed value back
  writeRegister(0x88e, Regdata1);               //restore register to original value

  if (Regdata2 == (Regdata1 + 1))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void SX126XLT::setupLoRa(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t modParam4)
{
  //order of passed parameters is, frequency, offset, spreadingfactor, bandwidth, coderate, optimisation

#ifdef SX126XDEBUG
  Serial.println(F("setupLoRa()"));
#endif
  setMode(MODE_STDBY_RC);
  setRegulatorMode(USE_DCDC);
  setPaConfig(0x04, PAAUTO, _Device);         //use _Device, saved by begin.
  setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);
  calibrateDevice(ALLDevices);                //is required after setting TCXO
  calibrateImage(frequency);
  setDIO2AsRfSwitchCtrl();
  setPacketType(PACKET_TYPE_LORA);
  setRfFrequency(frequency, offset);
  setModulationParams(modParam1, modParam2, modParam3, modParam4);
  setBufferBaseAddress(0, 0);
  setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  setHighSensitivity();  //set for maximum gain
  setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
}


void SX126XLT::setMode(uint8_t modeconfig)
{
#ifdef SX126XDEBUG
  Serial.println(F("setMode()"));
#endif

  checkBusy();

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_SET_STANDBY);
  SPI.transfer(modeconfig);
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  _OperatingMode = modeconfig;
}


void SX126XLT::setRegulatorMode(uint8_t mode)
{
#ifdef SX126XDEBUG
  Serial.println(F("setRegulatorMode()"));
#endif

  savedRegulatorMode = mode;

  writeCommand(RADIO_SET_REGULATORMODE, &mode, 1);
}


void SX126XLT::setPaConfig(uint8_t dutycycle, uint8_t hpMax, uint8_t device)
{
#ifdef SX126XDEBUG
  Serial.println(F("setPaConfig()"));
#endif


  uint8_t buffer[4];

  if (hpMax == PAAUTO)
  {
    if (device == DEVICE_SX1261)
    {
      hpMax = 0x00;
    }
    if (device == DEVICE_SX1262)
    {
      hpMax = 0x07;
    }
    if (device == DEVICE_SX1268)
    {
      hpMax = 0x07;
    }
  }

  if (_Device == DEVICE_SX1261)
  {
    device = 1;
  }
  else
  {
    device = 0;
  }

  buffer[0] = dutycycle;   //paDutyCycle
  buffer[1] = hpMax;       //hpMax:0x00~0x07; 7 for =22dbm
  buffer[2] = device;      //deviceSel: 0 = SX1262; 1 = SX1261; 0 = SX1268;
  buffer[3] = 0x01;        //reserved, always 0x01

  writeCommand(RADIO_SET_PACONFIG, buffer, 4);
}


void SX126XLT::setDIO3AsTCXOCtrl(uint8_t tcxoVoltage)
{
#ifdef SX126XDEBUG
  Serial.println(F("setDIO3AsTCXOCtrl()"));
#endif

  uint8_t buffer[4];

  buffer[0] = tcxoVoltage;
  buffer[1] = 0x00;
  buffer[2] = 0x00;
  buffer[3] = 0x64;

  writeCommand(RADIO_SET_TCXOMODE, buffer, 4);
}


void SX126XLT::calibrateDevice(uint8_t devices)
{
#ifdef SX126XDEBUG
  Serial.println(F("calibrateDevice()"));
#endif

  writeCommand(RADIO_CALIBRATE, &devices, 1);
  delay(5);                              //calibration time for all devices is 3.5mS, SX126x

}


void SX126XLT::calibrateImage(uint32_t freq)
{
#ifdef SX126XDEBUG
  Serial.println(F("calibrateImage()"));
#endif

  uint8_t calFreq[2];

  if ( freq > 900000000 )
  {
    calFreq[0] = 0xE1;
    calFreq[1] = 0xE9;
  }
  else if ( freq > 850000000 )
  {
    calFreq[0] = 0xD7;
    calFreq[1] = 0xD8;
  }
  else if ( freq > 770000000 )
  {
    calFreq[0] = 0xC1;
    calFreq[1] = 0xC5;
  }
  else if ( freq > 460000000 )
  {
    calFreq[0] = 0x75;
    calFreq[1] = 0x81;
  }
  else if ( freq > 425000000 )
  {
    calFreq[0] = 0x6B;
    calFreq[1] = 0x6F;
  }
  writeCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}


void SX126XLT::setDIO2AsRfSwitchCtrl()
{
#ifdef SX126XDEBUG
  Serial.println(F("setDIO2AsRfSwitchCtrl()"));
#endif

  uint8_t mode = 0x01;

  writeCommand(RADIO_SET_RFSWITCHMODE, &mode, 1);
}


void SX126XLT::setPacketType(uint8_t packettype )
{
#ifdef SX126XDEBUG
  Serial.println(F("setPacketType()"));
#endif
  savedPacketType = packettype;

  writeCommand(RADIO_SET_PACKETTYPE, &packettype, 1);
}


void SX126XLT::setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4)
{
  //order for LoRa is spreading factor, bandwidth, code rate, optimisation

#ifdef SX126XDEBUG
  Serial.println(F("setModulationParams()"));
#endif
  uint8_t regvalue;
  uint8_t buffer[4];

  regvalue = readRegister(REG_TX_MODULATION);

  savedModParam1 = modParam1;
  savedModParam2 = modParam2;
  savedModParam3 = modParam3;

  if (modParam2 == LORA_BW_500)
  {
    writeRegister(REG_TX_MODULATION, (regvalue & 0xFB));         //if bandwidth is 500k set bit 2 to 0, see datasheet 15.1.1
  }
  else
  {
    writeRegister(REG_TX_MODULATION, (regvalue | 0x04));         //if bandwidth is < 500k set bit 2 to 0 see datasheet 15.1.1
  }

  if (modParam4 == LDRO_AUTO)
  {
    modParam4 = returnOptimisation(modParam1, modParam2);        //pass Spreading factor then bandwidth to optimisation calc
  }

  savedModParam4 = modParam4;

  buffer[0] = modParam1;
  buffer[1] = modParam2;
  buffer[2] = modParam3;
  buffer[3] = modParam4;

  writeCommand(RADIO_SET_MODULATIONPARAMS, buffer, 4);
}


uint8_t SX126XLT::returnOptimisation(uint8_t SpreadingFactor, uint8_t Bandwidth)
{
  //from the passed bandwidth (bandwidth) and spreading factor this routine
  //calculates whether low data rate optimisation should be on or off

#ifdef SX126XDEBUG
  Serial.println(F("returnOptimisation()"));
#endif

  uint32_t tempBandwidth;
  float symbolTime;
  
  tempBandwidth = returnBandwidth(Bandwidth);
  
  symbolTime = calcSymbolTime(tempBandwidth, SpreadingFactor);

  if (symbolTime > 16)
  {
    return LDRO_ON;
  }
  else
  {
    return LDRO_OFF;
  }
}


uint32_t SX126XLT::returnBandwidth(uint8_t BWregvalue)
{

#ifdef SX126XDEBUG
  Serial.println(F("returnBandwidth()"));
#endif

  switch (BWregvalue)
  {
    case 0:
      return 7800;

    case 8:
      return 10400;

    case 1:
      return 15600;

    case 9:
      return 20800;

    case 2:
      return 31200;

    case 10:
      return 41700;

    case 3:
      return 62500;

    case 4:
      return 125000;

    case 5:
      return 250000;

    case 6:
      return 500000;

    default:
      break;
  }
  return 0xFFFF;                      //so that a bandwidth not set can be identified
}



float SX126XLT::calcSymbolTime(float Bandwidth, uint8_t SpreadingFactor)
{
  //calculates symbol time from passed bandwidth (lbandwidth) and Spreading factor (lSF)and returns in mS

#ifdef SX126XDEBUG
  Serial.println(F("calcSymbolTime()"));
#endif

  float symbolTimemS;
  symbolTimemS = (Bandwidth / pow(2, SpreadingFactor));
  symbolTimemS = (1000 / symbolTimemS);
  return symbolTimemS;
}


void SX126XLT::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
#ifdef SX126XDEBUG
  Serial.println(F("setBufferBaseAddress()"));
#endif

  uint8_t buffer[2];

  buffer[0] = txBaseAddress;
  buffer[1] = rxBaseAddress;
  writeCommand(RADIO_SET_BUFFERBASEADDRESS, buffer, 2);
}


void SX126XLT::setPacketParams(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5)
{
  //order is preamble, header type, packet length, CRC, IQ

#ifdef SX126XDEBUG
  Serial.println(F("SetPacketParams()"));
#endif

  uint8_t preambleMSB, preambleLSB;

  preambleMSB = packetParam1 >> 8;
  preambleLSB = packetParam1 & 0xFF;

  savedPacketParam1 = packetParam1;
  savedPacketParam2 = packetParam2;
  savedPacketParam3 = packetParam3;
  savedPacketParam4 = packetParam4;
  savedPacketParam5 = packetParam5;

  uint8_t buffer[9];
  buffer[0] = preambleMSB;
  buffer[1] = preambleLSB;
  buffer[2] = packetParam2;
  buffer[3] = packetParam3;
  buffer[4] = packetParam4;
  buffer[5] = packetParam5;
  buffer[6] = 0xFF;
  buffer[7] = 0xFF;
  buffer[8] = 0xFF;
  writeCommand(RADIO_SET_PACKETPARAMS, buffer, 9);
}


void SX126XLT::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
#ifdef SX126XDEBUG
  Serial.println(F("setDioIrqParams()"));
#endif

  savedIrqMask = irqMask;
  savedDio1Mask = dio1Mask;
  savedDio2Mask = dio2Mask;
  savedDio3Mask = dio3Mask;

  uint8_t buffer[8];

  buffer[0] = (uint8_t) (irqMask >> 8);
  buffer[1] = (uint8_t) (irqMask & 0xFF);
  buffer[2] = (uint8_t) (dio1Mask >> 8);
  buffer[3] = (uint8_t) (dio1Mask & 0xFF);
  buffer[4] = (uint8_t) (dio2Mask >> 8);
  buffer[5] = (uint8_t) (dio2Mask & 0xFF);
  buffer[6] = (uint8_t) (dio3Mask >> 8);
  buffer[7] = (uint8_t) (dio3Mask & 0xFF);
  writeCommand(RADIO_CFG_DIOIRQ, buffer, 8);
}


void SX126XLT::setHighSensitivity()
{
  //set RX Boosted gain mode
#ifdef SX126XDEBUG
  Serial.println(F("setHighSensitivity()"));
#endif
  writeRegister( REG_RX_GAIN, BOOSTED_GAIN ); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity
}

void SX126XLT::setLowPowerRX()
{
  //set RX power saving mode
#ifdef SX126XDEBUG
  Serial.println(F("setLowPowerRX()"));
#endif

  writeRegister( REG_RX_GAIN, POWER_SAVE_GAIN ); // min LNA gain, reduce current by 2mA for around 3dB loss in sensivity
}


void SX126XLT::setSyncWord(uint16_t syncword)
{
#ifdef SX126XDEBUG
  Serial.println(F("setSyncWord()"));
#endif
  writeRegister( REG_LR_SYNCWORD, ( syncword >> 8 ) & 0xFF );
  writeRegister( REG_LR_SYNCWORD + 1, syncword & 0xFF );
}


void SX126XLT::printModemSettings()
{
#ifdef SX126XDEBUG
  Serial.println(F("printModemSettings()"));
#endif

  printDevice();
  Serial.print(F(","));
  Serial.print(getFreqInt());
  Serial.print(F("hz,SF"));
  Serial.print(getLoRaSF());
  Serial.print(F(",BW"));
  Serial.print(returnBandwidth(savedModParam2));
  Serial.print(F(",CR4:"));
  Serial.print((getLoRaCodingRate() + 4));
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


uint32_t SX126XLT::getFreqInt()
{
  //get the current set device frequency from registers, return as long integer
#ifdef SX126XDEBUG
  Serial.println(F("getFreqInt()"));
#endif

  uint8_t MsbH, MsbL, Mid, Lsb;
  uint32_t uinttemp;
  float floattemp;
  MsbH = readRegister(REG_RFFrequency31_24);
  MsbL = readRegister(REG_RFFrequency23_16);
  Mid = readRegister(REG_RFFrequency15_8);
  Lsb = readRegister(REG_RFFrequency7_0);
  floattemp = ( (MsbH * 0x1000000ul) + (MsbL * 0x10000ul) + (Mid * 0x100ul) + Lsb);
  floattemp = ((floattemp * FREQ_STEP) / 1000000ul);
  uinttemp = (uint32_t)(floattemp * 1000000);
  return uinttemp;
}


uint8_t SX126XLT::getLoRaCodingRate()
{
#ifdef SX126XDEBUG
  Serial.println(F("getLoRaCodingRate"));
#endif

  return savedModParam3;
}


uint8_t SX126XLT::getOptimisation()
{
#ifdef SX126XDEBUG
  Serial.println(F("getOptimisation"));
#endif

  return savedModParam4;
}


uint16_t SX126XLT::getSyncWord()
{
#ifdef SX126XDEBUG
  Serial.println(F("getSyncWord"));
#endif

  uint8_t msb, lsb;
  uint16_t syncword;
  msb = readRegister(REG_LR_SYNCWORD);
  lsb = readRegister(REG_LR_SYNCWORD + 1);
  syncword = (msb << 8) + lsb;

  return syncword;
}


uint16_t SX126XLT::getPreamble()
{
#ifdef SX126XDEBUG
  Serial.println(F("getPreamble"));
#endif

  return savedPacketParam1;
}


void SX126XLT::printOperatingSettings()
{
#ifdef SX126XDEBUG
  Serial.println(F("printOperatingSettings()"));
#endif

  //uint8_t regdata;

  printDevice();

  Serial.print(F(",PacketMode_"));

  if (savedPacketType == PACKET_TYPE_LORA)
  {
    Serial.print(F("LoRa"));
  }
  
  if (savedPacketType == PACKET_TYPE_GFSK)
  {
    Serial.print(F("GFSK"));
  }
  
  if (getHeaderMode())
  {
    Serial.print(F(",Implicit"));
  }
  else
  {
    Serial.print(F(",Explicit"));
  }

   Serial.print(F(",LNAgain_"));

  if (getLNAgain() == BOOSTED_GAIN)
  {
    Serial.print(F("Boosted"));
  }
  else
  {
    Serial.print(F("Powersave"));
  }

}


uint8_t SX126XLT::getHeaderMode()
{
#ifdef SX126XDEBUG
  Serial.println(F("getHeaderMode"));
#endif

  return savedPacketParam2;
}


uint8_t SX126XLT::getLNAgain()
{
#ifdef SX126XDEBUG
  Serial.println(F("getLNAgain"));
#endif

  return readRegister(REG_RX_GAIN);
}


void SX126XLT::setTxParams(int8_t TXpower, uint8_t RampTime)
{

  //note this routine does not check if power levels are valid for the module in use
#ifdef SX126XDEBUG
  Serial.println(F("setTxParams()"));
#endif

  uint8_t buffer[2];

  savedTXPower = TXpower;

  buffer[0] = TXpower;
  buffer[1] = (uint8_t)RampTime;
  writeCommand(RADIO_SET_TXPARAMS, buffer, 2);
}


void SX126XLT::setTx(uint32_t timeout)
{
  //SX126x base timeout in units of 15.625 µs
  //Note: timeout passed to function is in mS

#ifdef SX126XDEBUG
  Serial.println(F("setTx()"));
#endif
  uint8_t buffer[3];

  clearIrqStatus(IRQ_RADIO_ALL);

  if (_rxtxpinmode)
  {
    txEnable();
  }

  timeout = timeout << 6;         //timeout passed in mS, convert to units of 15.625us

  buffer[0] = (timeout >> 16) & 0xFF;
  buffer[1] = (timeout >> 8) & 0xFF;
  buffer[2] = timeout & 0xFF;
  writeCommand(RADIO_SET_TX, buffer, 3 );
  _OperatingMode = MODE_TX;
}


void SX126XLT::clearIrqStatus(uint16_t irqMask)
{
#ifdef SX126XDEBUG
  Serial.println(F("clearIrqStatus()"));
#endif

  uint8_t buffer[2];

  buffer[0] = (uint8_t) (irqMask >> 8);
  buffer[1] = (uint8_t) (irqMask & 0xFF);
  writeCommand(RADIO_CLR_IRQSTATUS, buffer, 2);
}


uint16_t SX126XLT::readIrqStatus()
{
#ifdef SX126XDEBUG
  Serial.print(F("readIrqStatus()"));
#endif

  uint16_t temp;
  uint8_t buffer[2];

  readCommand(RADIO_GET_IRQSTATUS, buffer, 2);
  temp = ((buffer[0] << 8) + buffer[1]);
  return temp;
}


uint16_t SX126XLT::CRCCCITT(uint8_t *buffer, uint8_t size, uint16_t start)
{
#ifdef SX126XDEBUG
  Serial.println(F("CRCCCITT()"));
#endif

  uint16_t index, libraryCRC;
  uint8_t j;

  libraryCRC = start;                                  //start value for CRC16

  for (index = 0; index < size; index++)
  {
    libraryCRC ^= (((uint16_t)buffer[index]) << 8);
    for (j = 0; j < 8; j++)
    {
      if (libraryCRC & 0x8000)
        libraryCRC = (libraryCRC << 1) ^ 0x1021;
      else
        libraryCRC <<= 1;
    }
  }

  return libraryCRC;
}


uint8_t SX126XLT::transmit(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait)
{
#ifdef SX126XDEBUG
  Serial.println(F("transmit()"));
#endif
  uint8_t index;
  uint8_t bufferdata;

  if (size == 0)
  {
    return false;
  }

  setMode(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  
  checkBusy();

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);

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
  writeRegister(REG_LR_PAYLOADLENGTH, _TXPacketL);
  setTxParams(txpower, RAMP_TIME);
  
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  setTx(txtimeout);                                                //this starts the TX

  if (!wait)
  {
    return _TXPacketL;
  }

  while (!digitalRead(_TXDonePin));                                //Wait for DIO1 to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )                        //check for timeout
  {
    return 0;
  }
  else
  {
    return _TXPacketL;
  }
}


void SX126XLT::printIrqStatus()
{
#ifdef SX126XDEBUG
  Serial.println(F("printIrqStatus()"));
#endif

  uint16_t _IrqStatus;
  _IrqStatus = readIrqStatus();

  //0x0001
  if (_IrqStatus & IRQ_TX_DONE)
  {
    Serial.print(F(",IRQ_TX_DONE"));
  }

  //0x0002
  if (_IrqStatus & IRQ_RX_DONE)
  {
    Serial.print(F(",IRQ_RX_DONE"));
  }

  //0x0004
  if (_IrqStatus & IRQ_PREAMBLE_DETECTED)
  {
    Serial.print(F(",IRQ_PREAMBLE_DETECTED"));
  }

  //0x0008
  if (_IrqStatus & IRQ_SYNCWORD_VALID)
  {
    Serial.print(F(",IRQ_SYNCWORD_VALID"));
  }

  //0x0010
  if (_IrqStatus & IRQ_HEADER_VALID)
  {
    Serial.print(F(",IRQ_HEADER_VALID"));
  }

  //0x0020
  if (_IrqStatus & IRQ_HEADER_ERROR)
  {
    Serial.print(F(",IRQ_HEADER_ERROR"));
  }

  //0x0040
  if (_IrqStatus & IRQ_CRC_ERROR)
  {
    Serial.print(F(",IRQ_CRC_ERROR"));
  }

  //0x0080
  if (_IrqStatus & IRQ_CAD_DONE)
  {
    Serial.print(F(",IRQ_CAD_DONE"));
  }

  //0x0100
  if (_IrqStatus & IRQ_CAD_ACTIVITY_DETECTED)
  {
    Serial.print(",IRQ_CAD_ACTIVITY_DETECTED");
  }

  //0x0200
  if (_IrqStatus & IRQ_RX_TX_TIMEOUT)
  {
    Serial.print(F(",IRQ_RX_TX_TIMEOUT"));
  }

}


void SX126XLT::printRegisters(uint16_t Start, uint16_t End)
{
  //prints the contents of SX1262 registers to serial monitor

#ifdef SX126XDEBUG
  Serial.println(F("printRegisters()"));
#endif

  uint16_t Loopv1, Loopv2, RegData;

  Serial.print(F("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  for (Loopv1 = Start; Loopv1 <= End;)           //32 lines
  {
    Serial.print(F("0x"));
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


void SX126XLT::printDevice()
{
#ifdef SX126XDEBUG
  Serial.println(F("printDevice()"));
#endif


  switch (_Device)
  {
    case DEVICE_SX1261:
      Serial.print(F("SX1261"));
      break;

    case DEVICE_SX1262:
      Serial.print(F("SX1262"));
      break;

    case DEVICE_SX1268:
      Serial.print(F("SX1268"));
      break;

    default:
      Serial.print(F("Unknown Device"));

  }
}


bool SX126XLT::config()
{
#ifdef SX126XDEBUG
  Serial.println(F("config()"));
#endif

  resetDevice();
  setMode(MODE_STDBY_RC);
  setRegulatorMode(savedRegulatorMode);
  setPacketType(savedPacketType);
  setRfFrequency(savedFrequency, savedOffset);
  setModulationParams(savedModParam1, savedModParam2, savedModParam3, LDRO_ON);
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5);
  setDioIrqParams(savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask);       //set for IRQ on RX done on DIO1
  return true;
}


void SX126XLT::setRfFrequency( uint32_t frequency, int32_t offset )
{
  //Note RF_Freq = freq_reg*32M/(2^25)-----> freq_reg = (RF_Freq * (2^25))/32

#ifdef SX126XDEBUG
  Serial.println(F("setRfFrequency()"));
#endif

  uint8_t Rf_Freq[4];

  savedFrequency = frequency;
  savedOffset = offset;

  frequency = frequency + offset;

  frequency = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );

  Rf_Freq[0] = (frequency >> 24) & 0xFF; //MSB
  Rf_Freq[1] = (frequency >> 16) & 0xFF;
  Rf_Freq[2] = (frequency >> 8) & 0xFF;
  Rf_Freq[3] = frequency & 0xFF;//LSB

  writeCommand(RADIO_SET_RFFREQUENCY, Rf_Freq, 4);
}


uint8_t SX126XLT::getLoRaSF()
{
#ifdef SX126XDEBUG
  Serial.println(F("getLoRaSF()"));
#endif

  return savedModParam1;
}


uint8_t SX126XLT::getInvertIQ()
{
  //IQ mode reg 0x33
#ifdef SX126XDEBUG
  Serial.println(F("getInvertIQ"));
#endif

  return readRegister(REG_IQ_POLARITY_SETUP);
}


void SX126XLT::rxEnable()
{
#ifdef SX126XDEBUG
  Serial.println(F("rxEnable()"));
#endif

  digitalWrite(_RXEN, HIGH);
  digitalWrite(_TXEN, LOW);
}


void SX126XLT::txEnable()
{
#ifdef SX126XDEBUG
  Serial.println(F("txEnable()"));
#endif

  digitalWrite(_RXEN, LOW);
  digitalWrite(_TXEN, HIGH);
}

void SX126XLT::printASCIIPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX126XDEBUG
  Serial.println(F("printASCIIPacket()"));
#endif

  uint8_t index;

  for (index = 0; index < size; index++)
  {
    Serial.write(buffer[index]);
  }

}


uint8_t SX126XLT::receive(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait)
{
#ifdef SX126XDEBUG
  Serial.println(F("receive()"));
#endif

  uint8_t index, RXstart, RXend;
  uint16_t regdata;
  uint8_t buffer[2];

  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
  setRx(rxtimeout);

  if (!wait)
  {
    return 0;                                                                 //not wait requested so no packet length to pass
  }

  while (!digitalRead(_RXDonePin));                                         //Wait for DIO1 to go high

  setMode(MODE_STDBY_RC);                                                //ensure to stop further packet reception

  regdata = readIrqStatus();

  if ( (regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT ) ) //check if any of the preceding IRQs is set
  {
    //packet is errored somewhere so return 0
    return 0;
  }

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];

  if (_RXPacketL > size)               //check passed buffer is big enough for packet
  {
    _RXPacketL = size;                   //truncate packet if not enough space
  }

  RXstart = buffer[1];

  RXend = RXstart + _RXPacketL;

  checkBusy();
  
#ifdef USE_SPI_TRANSACTION           //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);             //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(RXstart);
  SPI.transfer(0xFF);

  for (index = RXstart; index < RXend; index++)
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;
  }

  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _RXPacketL;                     //so we can check for packet having enough buffer space
}


uint8_t SX126XLT::readPacketRSSI()
{
#ifdef SX126XDEBUG
  Serial.println(F("readPacketRSSI()"));
#endif

  //readPacketReceptionLoRa();
  
  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5) ;
  _PacketRSSI = -status[0] / 2;

  return _PacketRSSI;
}


uint8_t SX126XLT::readPacketSNR()
{
#ifdef SX126XDEBUG
  Serial.println(F("readPacketSNR()"));
#endif
  //readPacketReceptionLoRa();
  
  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5) ;

  if ( status[1] < 128 )
  {
    _PacketSNR = status[1] / 4 ;
  }
  else
  {
    _PacketSNR = (( status[1] - 256 ) / 4);
  }
  
  return _PacketSNR;
}

uint8_t SX126XLT::readRXPacketL()
{
#ifdef SX126XDEBUG
  Serial.println(F("readRXPacketL()"));
#endif

  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  return _RXPacketL;
}

/*
void SX126XLT::readPacketReceptionLoRa()
{
#ifdef SX126XDEBUG
  Serial.println(F("readPacketReceptionLoRa()"));
#endif

  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5) ;
  _PacketRSSI = -status[0] / 2;

  if ( status[1] < 128 )
  {
    _PacketSNR = status[1] / 4 ;
  }
  else
  {
    _PacketSNR = (( status[1] - 256 ) / 4);
  }

}
*/

void SX126XLT::setRx(uint32_t timeout)
{
  //SX126x base timeout in units of 15.625 µs
  //timeout passed to function in mS
  //range is 1mS to 262 seconds

#ifdef SX126XDEBUG
  Serial.println(F("setRx()"));
#endif
  uint8_t buffer[3];
  clearIrqStatus(IRQ_RADIO_ALL);

  if (_rxtxpinmode)
  {
    rxEnable();
  }

  timeout = timeout << 8;           //timeout passed in mS, multiply by 64 to convert units of 15.625us to 1mS

  buffer[0] = (timeout >> 16) & 0xFF;
  buffer[1] = (timeout >> 8) & 0xFF;
  buffer[2] = timeout & 0xFF;
  writeCommand(RADIO_SET_RX, buffer, 3 );
}

/***************************************************************************
//Start direct access SX buffer routines
***************************************************************************/

void SX126XLT::startWriteSXBuffer(uint8_t ptr)
{
#ifdef SX126XDEBUG
  Serial.println(F("startWriteSXBuffer()"));
#endif

  _TXPacketL = 0;                   //this variable used to keep track of bytes written
  setMode(MODE_STDBY_RC);
  setBufferBaseAddress(ptr, 0);     //TX,RX
  
  checkBusy();
  
  #ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif
  
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);
  //SPI interface ready for byte to write to buffer
}


uint8_t  SX126XLT::endWriteSXBuffer()
{
#ifdef SX126XDEBUG
  Serial.println(F("endWriteSXBuffer()"));
#endif

  digitalWrite(_NSS, HIGH);
  
  #ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return _TXPacketL;
  
}


void SX126XLT::startReadSXBuffer(uint8_t ptr)
{
#ifdef SX126XDEBUG
  Serial.println(F("startReadSXBuffer"));
#endif

  //uint8_t rxstart;
  //uint8_t buffer[2];

  _RXPacketL = 0;
  
  //setBufferBaseAddress(0, ptr);          //TX,RX
  
  checkBusy();
  
  #ifdef USE_SPI_TRANSACTION             //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
  #endif

  digitalWrite(_NSS, LOW);               //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(ptr);
  SPI.transfer(0xFF);

  //next line would be data = SPI.transfer(0);
  //SPI interface ready for byte to read from
}


uint8_t SX126XLT::endReadSXBuffer()
{
#ifdef SX126XDEBUG
  Serial.println(F("endReadSXBuffer()"));
#endif

  digitalWrite(_NSS, HIGH);
  
  #ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif
  
  return _RXPacketL;
}


void SX126XLT::writeUint8(uint8_t x)
{
#ifdef SX126XDEBUG
  Serial.println(F("writeUint8()"));
#endif

  SPI.transfer(x);

  _TXPacketL++;                     //increment count of bytes written
}

uint8_t SX126XLT::readUint8()
{
#ifdef SX126XDEBUG
  Serial.println(F("readUint8()"));
#endif
  byte x;

  x = SPI.transfer(0);

  _RXPacketL++;                      //increment count of bytes read
  return (x);
}


void SX126XLT::writeInt8(int8_t x)
{
#ifdef SX126XDEBUG
  Serial.println(F("writeInt8()"));
#endif

  SPI.transfer(x);

  _TXPacketL++;                      //increment count of bytes written
}


int8_t SX126XLT::readInt8()
{
#ifdef SX126XDEBUG
  Serial.println(F("readInt8()"));
#endif
  int8_t x;

  x = SPI.transfer(0);

  _RXPacketL++;                      //increment count of bytes read
  return (x);
}


void SX126XLT::writeInt16(int16_t x)
{
#ifdef SX126XDEBUG
  Serial.println(F("writeInt16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXPacketL = _TXPacketL + 2;         //increment count of bytes written
}


int16_t SX126XLT::readInt16()
{
#ifdef SX126XDEBUG
  Serial.println(F("readInt16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXPacketL = _RXPacketL + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX126XLT::writeUint16(uint16_t x)
{
#ifdef SX126XDEBUG
  Serial.println(F("writeUint16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXPacketL = _TXPacketL + 2;         //increment count of bytes written
}


uint16_t SX126XLT::readUint16()
{
#ifdef SX126XDEBUG
  Serial.println(F("writeUint16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXPacketL = _RXPacketL + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX126XLT::writeInt32(int32_t x)
{
#ifdef SX126XDEBUG
  Serial.println(F("writeInt32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
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


int32_t SX126XLT::readInt32()
{
#ifdef SX126XDEBUG
  Serial.println(F("readInt32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
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


void SX126XLT::writeUint32(uint32_t x)
{
#ifdef SX126XDEBUG
  Serial.println(F("writeUint32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
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


uint32_t SX126XLT::readUint32()
{
#ifdef SX126XDEBUG
  Serial.println(F("readUint32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
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


void SX126XLT::writeFloat(float x)
{
#ifdef SX126XDEBUG
  Serial.println(F("writeFloat()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
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


float SX126XLT::readFloat()
{
#ifdef SX126XDEBUG
  Serial.println(F("readFloat()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
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


uint8_t SX126XLT::transmitSXBuffer(uint8_t startaddr, uint8_t length, uint32_t txtimeout, int8_t txpower, uint8_t wait)
{
#ifdef SX126XDEBUG
  Serial.println(F("transmitSXBuffer()"));
#endif

  setBufferBaseAddress(startaddr, 0);          //TX, RX

  setPacketParams(savedPacketParam1, savedPacketParam2, length, savedPacketParam4, savedPacketParam5);
  setTxParams(txpower, RAMP_TIME);
  
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  setTx(txtimeout);                            //this starts the TX

  if (!wait)
  {
    return _TXPacketL;
  }

  while (!digitalRead(_TXDonePin));            //Wait for DIO1 to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )    //check for timeout
  {
    return 0;
  }
  else
  {
    return _TXPacketL;
  }
}


void SX126XLT::writeBuffer(uint8_t *txbuffer, uint8_t size)
{
#ifdef SX126XDEBUG1
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

  SPI.transfer(0);                     //this ensures last byte of buffer written really is a null (0)

}


uint8_t SX126XLT::receiveSXBuffer(uint8_t startaddr, uint32_t rxtimeout, uint8_t wait )
{
#ifdef SX127XDEBUG1
  Serial.println(F("receiveSXBuffer()"));
#endif

  uint16_t regdata;
  uint8_t buffer[2];

  setMode(MODE_STDBY_RC);
  
  setBufferBaseAddress(0, startaddr);               //order is TX RX
  
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
  setRx(rxtimeout);                                 //no actual RX timeout in this function

  if (!wait)
  {
    return 0;
  }
  
  while (!digitalRead(_RXDonePin));                  //Wait for DIO1 to go high 
  
  setMode(MODE_STDBY_RC);                            //ensure to stop further packet reception

  regdata = readIrqStatus();
  
  if ( (regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT ) )
  {
    return 0;                                        //no RX done and header valid only, could be CRC error
  }

   readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];

  return _RXPacketL;                           
}


uint8_t SX126XLT::readBuffer(uint8_t *rxbuffer)
{
#ifdef SX126XDEBUG1
  Serial.println(F("readBuffer()"));
#endif

  uint8_t index = 0, regdata;

  do                                     //need to find the size of the buffer first
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;           //fill the buffer.
    index++;
  } while (regdata != 0);                //keep reading until we have reached the null (0) at the buffer end
                                         //or exceeded size of buffer allowed
  return index;                          //return the actual size of the buffer, till the null (0) detected

}

/***************************************************************************
//End direct access SX buffer routines
***************************************************************************/






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
