/*
  Copyright 2019 - Stuart Robinson
  Licensed under a MIT license displayed at the bottom of this document.
  06/02/20
*/

/*
Parts of code Copyright (c) 2013, SEMTECH S.A.
See LICENSE.TXT file included in the library
*/

#include <SX128XLT.h>
#include <SPI.h>

#define LTUNUSED(v) (void) (v)    //add LTUNUSED(variable); to avoid compiler warnings 
#define USE_SPI_TRANSACTION

//#define SX128XDEBUG             //enable debug messages
//#define RANGINGDEBUG            //enable debug messages for ranging
//#define  SX128XDEBUGRXTX        //enable debug messages for RX TX switching
//#define SX128XDEBUGPINS           //enable pin allocation debug messages

SX128XLT::SX128XLT()
{
}

/* Formats for :begin
1 All pins > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, int8_t pinRXEN, int8_t pinTXEN, uint8_t device)
2 NiceRF   > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, uint8_t device)
3 Ebyte    > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinRXEN, int8_t pinTXEN, uint8_t device); 
*/


bool SX128XLT::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, int8_t pinRXEN, int8_t pinTXEN, uint8_t device)
{
  
  //format 1 pins, assign all available pins 
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = pinDIO2;
  _DIO3 = pinDIO3;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;
  _Device = device;
  _TXDonePin = pinDIO1;        //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;        //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);


#ifdef SX128XDEBUGPINS
  Serial.println(F("begin()"));
  Serial.println(F("SX128XLT constructor instantiated successfully"));
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
  Serial.print(F("RX_EN "));
  Serial.println(_RXEN);
  Serial.print(F("TX_EN "));
  Serial.println(_TXEN);
#endif

  if (_DIO1 >= 0)
  {
    pinMode( _DIO1, INPUT);
  }
 
  if (_DIO2 >= 0)
  {
    pinMode( _DIO2, INPUT);
  }
 
  if (_DIO3 >= 0)
  {
    pinMode( _DIO3, INPUT);
  }
   
  if ((_RXEN >= 0) && (_TXEN >= 0))
  {
   #ifdef SX128XDEBUGPINS
   Serial.println(F("RX_EN & TX_EN switching enabled"));
   #endif
   pinMode(_RXEN, OUTPUT);
   pinMode(_TXEN, OUTPUT);
   _rxtxpinmode = true;
  }
  else
  {
  #ifdef SX128XDEBUGPINS
  Serial.println(F("RX_EN & TX_EN switching disabled"));
  #endif
  _rxtxpinmode = false;
  }

  resetDevice();

  if (checkDevice())
  {
    return true;
  }

  return false;
}


bool SX128XLT::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, uint8_t device)
{
  //format 2 pins for NiceRF, NSS, NRESET, RFBUSY, DIO1
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = -1;
  _DIO3 = -1;
  _RXEN = -1;                  //not defined, so mark as unused
  _TXEN = -1;                  //not defined, so mark as unused
  _Device = device;
  _TXDonePin = pinDIO1;        //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;        //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

#ifdef SX128XDEBUGPINS
  Serial.println(F("format 2 NiceRF begin()"));
  Serial.println(F("SX128XLT constructor instantiated successfully"));
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
  Serial.print(F("RX_EN "));
  Serial.println(_RXEN);
  Serial.print(F("TX_EN "));
  Serial.println(_TXEN);
#endif

  if (_DIO1 >= 0)
  {
    pinMode( _DIO1, INPUT);
  }
  
  #ifdef SX128XDEBUGPINS
  Serial.println(F("RX_EN & TX_EN switching disabled"));
  #endif
  
  _rxtxpinmode = false;
  
  resetDevice();

  if (checkDevice())
  {
    return true;
  }

  return false;
}


bool SX128XLT::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinRXEN, int8_t pinTXEN, uint8_t device)
{
  //format 3 pins for Ebyte, NSS, NRESET, RFBUSY, DIO1, RX_EN, TX_EN 
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = -1;
  _DIO3 = -1;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;
  _Device = device;
  _TXDonePin = pinDIO1;        //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;        //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

#ifdef SX128XDEBUGPINS
  Serial.println(F("format 3 Ebyte begin()"));
  Serial.println(F("SX128XLT constructor instantiated successfully"));
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
  Serial.print(F("RX_EN "));
  Serial.println(_RXEN);
  Serial.print(F("TX_EN "));
  Serial.println(_TXEN);
#endif

  if (_DIO1 >= 0)
  {
    pinMode( _DIO1, INPUT);
  }
 
  if ((_RXEN >= 0) && (_TXEN >= 0))
  {
   #ifdef SX128XDEBUGPINS
   Serial.println(F("RX_EN & TX_EN switching enabled"));
   #endif
   pinMode(_RXEN, OUTPUT);
   pinMode(_TXEN, OUTPUT);
   _rxtxpinmode = true;
  }
  else
  {
  #ifdef SX128XDEBUGPINS
  Serial.println(F("RX_EN & TX_EN switching disabled"));
  #endif
  _rxtxpinmode = false;
  }

  resetDevice();

  if (checkDevice())
  {
    return true;
  }

  return false;
}


void SX128XLT::rxEnable()
{
  //Enable RX mode on device such as Ebyte E28-2G4M20S which have RX and TX enable pins
  #ifdef SX128XDEBUGRXTX
  Serial.println(F("rxEnable()"));
  #endif

  digitalWrite(_RXEN, HIGH);
  digitalWrite(_TXEN, LOW);
}


void SX128XLT::txEnable()
{
  //Enable RX mode on device such as Ebyte E28-2G4M20S which have RX and TX enable pins
#ifdef SX128XDEBUGRXTX
  Serial.println(F("txEnable()"));
#endif

  digitalWrite(_RXEN, LOW);
  digitalWrite(_TXEN, HIGH);
}



void SX128XLT::checkBusy()
{
#ifdef SX128XDEBUG
  //Serial.println(F("checkBusy()"));
#endif

  uint8_t busy_timeout_cnt;
  busy_timeout_cnt = 0;

  while (digitalRead(_RFBUSY))
  {
    delay(1);
    busy_timeout_cnt++;

    if (busy_timeout_cnt > 10)                     //wait 5mS for busy to complete
    {
      Serial.println(F("ERROR - Busy Timeout!"));
      resetDevice();
      setMode(MODE_STDBY_RC);
      config();                                   //re-run saved config
      break;
    }
  }
}


bool SX128XLT::config()
{
#ifdef SX128XDEBUG
  Serial.println(F("config()"));
#endif

  resetDevice();
  setMode(MODE_STDBY_RC);
  setRegulatorMode(savedRegulatorMode);
  setPacketType(savedPacketType);
  setRfFrequency(savedFrequency, savedOffset);
  setModulationParams(savedModParam1, savedModParam2, savedModParam3);
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5, savedPacketParam6, savedPacketParam7);
  setDioIrqParams(savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask);       //set for IRQ on RX done on DIO1
  return true;
}


void SX128XLT::readRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX128XDEBUG
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


uint8_t SX128XLT::readRegister(uint16_t address)
{
#ifdef SX128XDEBUG
  //Serial.println(F("readRegister()"));
#endif

  uint8_t data;

  readRegisters(address, &data, 1);
  return data;
}


void SX128XLT::writeRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX128XDEBUG
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

  //checkBusy();
}


void SX128XLT::writeRegister(uint16_t address, uint8_t value)
{
#ifdef SX128XDEBUG
  //Serial.println(F("writeRegister()"));
#endif

  writeRegisters( address, &value, 1 );
}


void SX128XLT::writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX128XDEBUG
  //Serial.print(F("writeCommand() "));
  //Serial.println(Opcode, HEX);
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


void SX128XLT::readCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX128XDEBUG
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
  //checkBusy();
}


void SX128XLT::resetDevice()
{
#ifdef SX128XDEBUG
  Serial.println(F("resetDevice()"));
#endif

  //timings taken from Semtech library
  delay(20);
  digitalWrite(_NRESET, LOW);
  delay(50);
  digitalWrite(_NRESET, HIGH);
  delay(20);
}


bool SX128XLT::checkDevice()
{
  //check there is a device out there, writes a register and reads back
#ifdef SX128XDEBUG
  Serial.println(F("checkDevice()"));
#endif

  uint8_t Regdata1, Regdata2;
  Regdata1 = readRegister(0x0908);               //low byte of frequency setting
  writeRegister(0x0908, (Regdata1 + 1));
  Regdata2 = readRegister(0x0908);               //read changed value back
  writeRegister(0x0908, Regdata1);               //restore register to original value

  if (Regdata2 == (Regdata1 + 1))
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX128XLT::setupLoRa(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3)
{
#ifdef SX128XDEBUG
  Serial.println(F("setupLoRa()"));
#endif

  setMode(MODE_STDBY_RC);
  setRegulatorMode(USE_LDO);
  setPacketType(PACKET_TYPE_LORA);
  setRfFrequency(frequency, offset);
  setBufferBaseAddress(0, 0);
  setModulationParams(modParam1, modParam2, modParam3);
  setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
}


void SX128XLT::setMode(uint8_t modeconfig)
{
#ifdef SX128XDEBUG
  Serial.println(F("setMode()"));
#endif

  uint8_t Opcode = 0x80;

  checkBusy();

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif


  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);
  SPI.transfer(modeconfig);
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  _OperatingMode = modeconfig;

}

void SX128XLT::setRegulatorMode(uint8_t mode)
{
#ifdef SX128XDEBUG
  Serial.println(F("setRegulatorMode()"));
#endif

  savedRegulatorMode = mode;

  writeCommand(RADIO_SET_REGULATORMODE, &mode, 1);
}

void SX128XLT::setPacketType(uint8_t packettype )
{
#ifdef SX128XDEBUG
  Serial.println(F("setPacketType()"));
#endif
  savedPacketType = packettype;

  writeCommand(RADIO_SET_PACKETTYPE, &packettype, 1);
}


void SX128XLT::setRfFrequency(uint32_t frequency, int32_t offset)
{
#ifdef SX128XDEBUG
  Serial.println(F("setRfFrequency()"));
#endif

  savedFrequency = frequency;
  savedOffset = offset;

  frequency = frequency + offset;
  uint8_t buffer[3];
  uint32_t freqtemp = 0;
  freqtemp = ( uint32_t )( (double) frequency / (double)FREQ_STEP);
  buffer[0] = ( uint8_t )( ( freqtemp >> 16 ) & 0xFF );
  buffer[1] = ( uint8_t )( ( freqtemp >> 8 ) & 0xFF );
  buffer[2] = ( uint8_t )( freqtemp & 0xFF );
  writeCommand(RADIO_SET_RFFREQUENCY, buffer, 3);
}

void SX128XLT::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
#ifdef SX128XDEBUG
  Serial.println(F("setBufferBaseAddress()"));
#endif

  uint8_t buffer[2];

  buffer[0] = txBaseAddress;
  buffer[1] = rxBaseAddress;
  writeCommand(RADIO_SET_BUFFERBASEADDRESS, buffer, 2);
}


void SX128XLT::setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3)
{
//sequence is spreading factor, bandwidth, coding rate.

#ifdef SX128XDEBUG
  Serial.println(F("setModulationParams()"));
#endif

  uint8_t buffer[3];

  savedModParam1 = modParam1;
  savedModParam2 = modParam2;
  savedModParam3 = modParam3;

  buffer[0] = modParam1;
  buffer[1] = modParam2;
  buffer[2] = modParam3;

  writeCommand(RADIO_SET_MODULATIONPARAMS, buffer, 3);
}


void SX128XLT::setPacketParams(uint8_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5, uint8_t packetParam6, uint8_t packetParam7)
{
//for LoRa order is PreambleLength, HeaderType, PayloadLength, CRC, InvertIQ/chirp invert, not used, not used
//for FLRC order is PreambleLength, SyncWordLength, SyncWordMatch, HeaderType, PayloadLength, CrcLength, Whitening

#ifdef SX128XDEBUG
  Serial.println(F("SetPacketParams()"));
#endif

  savedPacketParam1 = packetParam1;
  savedPacketParam2 = packetParam2;
  savedPacketParam3 = packetParam3;
  savedPacketParam4 = packetParam4;
  savedPacketParam5 = packetParam5;
  savedPacketParam6 = packetParam6;
  savedPacketParam7 = packetParam7;

  uint8_t buffer[7];
  buffer[0] = packetParam1;
  buffer[1] = packetParam2;
  buffer[2] = packetParam3;
  buffer[3] = packetParam4;
  buffer[4] = packetParam5;
  buffer[5] = packetParam6;
  buffer[6] = packetParam7;
  writeCommand(RADIO_SET_PACKETPARAMS, buffer, 7);

}


void SX128XLT::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
#ifdef SX128XDEBUG
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
  writeCommand(RADIO_SET_DIOIRQPARAMS, buffer, 8);
}


void SX128XLT::setHighSensitivity()
{
  //set bits 7,6 of REG_LNA_REGIME
#ifdef SX128XDEBUG
  Serial.println(F("setHighSensitivity()"));
#endif

  writeRegister(REG_LNA_REGIME, (readRegister(REG_LNA_REGIME) | 0xC0));
}

void SX128XLT::setLowPowerRX()
{
  //clear bits 7,6 of REG_LNA_REGIME
#ifdef SX128XDEBUG
  Serial.println(F("setLowPowerRX()"));
#endif

  writeRegister(REG_LNA_REGIME, (readRegister(REG_LNA_REGIME) & 0x3F));
}


void SX128XLT::printModemSettings()
{
#ifdef SX128XDEBUG
  Serial.println(F("printModemSettings()"));
#endif

  printDevice();
  Serial.print(F(","));
  Serial.print(getFreqInt());
  Serial.print(F("hz"));
  
  if (savedPacketType == PACKET_TYPE_LORA) 
  {
  Serial.print(F(",SF"));
  Serial.print(getLoRaSF());
  Serial.print(F(",BW"));
  Serial.print(returnBandwidth(savedModParam2));
  Serial.print(F(",CR4:"));
  Serial.print((getLoRaCodingRate() + 4));

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
  
  if (savedPacketType == PACKET_TYPE_FLRC) 
  {
  if (savedPacketParam1 == 0)
  {
   Serial.print(F(",No_Syncword"));
  }
  
  if (savedPacketParam1 == 4)
  {
   Serial.print(F(",32bit_Syncword"));
  }  
  
  
  switch (savedPacketParam3)
  {
    case RADIO_RX_MATCH_SYNCWORD_OFF:
      Serial.print(F(",SYNCWORD_OFF"));
      break;

    case RADIO_RX_MATCH_SYNCWORD_1:
      Serial.print(F(",SYNCWORD_1"));
      break;

    case RADIO_RX_MATCH_SYNCWORD_2:
      Serial.print(F(",SYNCWORD_2"));
      break;

    case RADIO_RX_MATCH_SYNCWORD_1_2:
      Serial.print(F(",SYNCWORD_1_2"));
      break;

    case RADIO_RX_MATCH_SYNCWORD_3:
      Serial.print(F(",SYNCWORD_3"));
      break;

    case RADIO_RX_MATCH_SYNCWORD_1_3:
      Serial.print(F(",SYNCWORD_1_3"));
      break;

    case RADIO_RX_MATCH_SYNCWORD_2_3:
      Serial.print(F(",SYNCWORD_2_3"));
      break;

    case RADIO_RX_MATCH_SYNCWORD_1_2_3:
      Serial.print(F(",SYNCWORD_1_2_3"));
      break;

    default:
      Serial.print(F("Unknown_SYNCWORD"));
  }
   
   if (savedPacketParam4 == RADIO_PACKET_FIXED_LENGTH)
  {
   Serial.print(F(",PACKET_FIXED_LENGTH"));
  }  
  
  if (savedPacketParam4 == RADIO_PACKET_VARIABLE_LENGTH)
  {
   Serial.print(F(",PACKET_VARIABLE_LENGTH"));
  }  
  
  switch (savedPacketParam6)
  {
    case RADIO_CRC_OFF:
      Serial.print(F(",CRC_OFF"));
      break;

    case RADIO_CRC_1_BYTES:
      Serial.print(F(",CRC_1_BYTES"));
      break;

    case RADIO_CRC_2_BYTES:
      Serial.print(F(",CRC_2_BYTES"));
      break;

    case RADIO_CRC_3_BYTES:
      Serial.print(F(",CRC_3_BYTES"));
      break;

    default:
      Serial.print(F(",Unknown_CRC"));
  }
  
  if (savedPacketParam7 == RADIO_WHITENING_ON)
  {
   Serial.print(F(",WHITENING_ON"));
  }  
  
  if (savedPacketParam7 == RADIO_WHITENING_OFF)
  {
   Serial.print(F(",WHITENING_OFF"));
  }  
  
  }

}


void SX128XLT::printDevice()
{
#ifdef SX128XDEBUG
  Serial.println(F("printDevice()"));
#endif


  switch (_Device)
  {
    case DEVICE_SX1280:
      Serial.print(F("SX1280"));
      break;

    case DEVICE_SX1281:
      Serial.print(F("SX1281"));
      break;

    default:
      Serial.print(F("Unknown Device"));

  }
}


uint32_t SX128XLT::getFreqInt()
{

#ifdef SX128XDEBUG
  Serial.println(F("getFreqInt"));
#endif

  //get the current set device frequency, return as long integer
  uint8_t Msb = 0;
  uint8_t Mid = 0;
  uint8_t Lsb = 0;

  uint32_t uinttemp;
  float floattemp;
  
  LTUNUSED(Msb);           //to prevent a compiler warning
  LTUNUSED(Mid);           //to prevent a compiler warning
  LTUNUSED(Lsb);           //to prevent a compiler warning
  
  if (savedPacketType == PACKET_TYPE_LORA)
  { 
  Msb = readRegister(REG_RFFrequency23_16);
  Mid = readRegister(REG_RFFrequency15_8);
  Lsb = readRegister(REG_RFFrequency7_0);
  }
  
  if (savedPacketType == PACKET_TYPE_FLRC)
  { 
  Msb = readRegister(REG_FLRC_RFFrequency23_16);
  Mid = readRegister(REG_FLRC_RFFrequency15_8);
  Lsb = readRegister(REG_FLRC_RFFrequency7_0);
  }
  
  floattemp = ((Msb * 0x10000ul) + (Mid * 0x100ul) + Lsb);
  floattemp = ((floattemp * FREQ_STEP) / 1000000ul);
  uinttemp = (uint32_t)(floattemp * 1000000);
  return uinttemp;
}


uint8_t SX128XLT::getLoRaSF()
{
#ifdef SX128XDEBUG
  Serial.println(F("getLoRaSF()"));
#endif
  return (savedModParam1 >> 4);
}


uint32_t SX128XLT::returnBandwidth(uint8_t data)
{
#ifdef SX128XDEBUG
  Serial.println(F("returnBandwidth()"));
#endif

  switch (data)
  {
    case LORA_BW_0200:
      return 203125;

    case LORA_BW_0400:
      return 406250;

    case LORA_BW_0800:
      return 812500;

    case LORA_BW_1600:
      return 1625000;

    default:
      break;
  }

  return 0x0;                      //so that a bandwidth not set can be identified
}


uint8_t SX128XLT::getLoRaCodingRate()
{
#ifdef SX128XDEBUG
  Serial.println(F("getLoRaCodingRate"));
#endif

  return savedModParam3;
}


uint8_t SX128XLT::getInvertIQ()
{
//IQ mode reg 0x33
#ifdef SX128XDEBUG
  Serial.println(F("getInvertIQ"));
#endif

  return savedPacketParam5;
}


uint16_t SX128XLT::getPreamble()
{
#ifdef SX128XDEBUG
  Serial.println(F("getPreamble"));
#endif

  return savedPacketParam1;
}

void SX128XLT::printOperatingSettings()
{
#ifdef SX128XDEBUG
  Serial.println(F("printOperatingSettings()"));
#endif

  printDevice();

  Serial.print(F(",PacketMode_"));

  switch (savedPacketType)
  {
    case PACKET_TYPE_GFSK:
      Serial.print(F("GFSK"));
      break;

    case PACKET_TYPE_LORA:
      Serial.print(F("LORA"));
      break;

    case PACKET_TYPE_RANGING:
      Serial.print(F("RANGING"));
      break;

    case PACKET_TYPE_FLRC:
      Serial.print(F("FLRC"));
      break;


    case PACKET_TYPE_BLE:
      Serial.print(F("BLE"));
      break;

    default:
      Serial.print(F("Unknown"));

  }

  switch (savedPacketParam2)
  {
    case LORA_PACKET_VARIABLE_LENGTH:
      Serial.print(F(",Explicit"));
      break;

    case LORA_PACKET_FIXED_LENGTH:
      Serial.print(F(",Implicit"));
      break;

    default:
      Serial.print(F(",Unknown"));
  }

  Serial.print(F(",LNAgain_"));


  if (getLNAgain() == 0xC0)
  {
    Serial.print(F("HighSensitivity"));
  }
  else
  {
    Serial.print(F("LowPowerRX"));
  }

}


uint8_t SX128XLT::getLNAgain()
{
#ifdef SX128XDEBUG
  Serial.println(F("getLNAgain"));
#endif

  return (readRegister(REG_LNA_REGIME) & 0xC0);
}



void SX128XLT::printRegisters(uint16_t Start, uint16_t End)
{
  //prints the contents of SX1280 registers to serial monitor

#ifdef SX128XDEBUG
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


void SX128XLT::printASCIIPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX128XDEBUG
  Serial.println(F("printASCIIPacket()"));
#endif

  uint8_t index;

  for (index = 0; index < size; index++)
  {
    Serial.write(buffer[index]);
  }

}


uint8_t SX128XLT::transmit(uint8_t *txbuffer, uint8_t size, uint16_t timeout, int8_t txpower, uint8_t wait)
{
#ifdef SX128XDEBUG
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

  if (savedPacketType == PACKET_TYPE_LORA)
  {
   writeRegister(REG_LR_PAYLOADLENGTH, _TXPacketL);                           //only seems to work for lora  
  }  
  else if (savedPacketType == PACKET_TYPE_FLRC)
  {
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, _TXPacketL, savedPacketParam6, savedPacketParam7);
  }

  setTxParams(txpower, RAMP_TIME);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  setTx(timeout);                                                          //this starts the TX
  
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


void SX128XLT::setTxParams(int8_t TXpower, uint8_t RampTime)
{
#ifdef SX128XDEBUG
  Serial.println(F("setTxParams()"));
#endif

  uint8_t buffer[2];

  savedTXPower = TXpower;

  //power register is set to 0 to 31 which is -18dBm to +12dBm
  buffer[0] = (TXpower + 18);
  buffer[1] = (uint8_t)RampTime;
  writeCommand(RADIO_SET_TXPARAMS, buffer, 2);
}


void SX128XLT::setTx(uint16_t timeout)
{

#ifdef SX128XDEBUG
  Serial.println(F("setTx()"));
#endif

  if (_rxtxpinmode)
  {
    txEnable();
  }

  //Serial.print(F("timeout ")); 
  //Serial.println(timeout);
  //Serial.print(F("_PERIODBASE ")); 
  //Serial.println(_PERIODBASE);
  
  uint8_t buffer[3];

  clearIrqStatus(IRQ_RADIO_ALL);                             //clear all interrupt flags
  buffer[0] = _PERIODBASE;
  buffer[1] = ( uint8_t )( ( timeout >> 8 ) & 0x00FF );
  buffer[2] = ( uint8_t )( timeout & 0x00FF );
  writeCommand(RADIO_SET_TX, buffer, 3 );
}


void SX128XLT::clearIrqStatus(uint16_t irqMask)
{
#ifdef SX128XDEBUG
  Serial.println(F("clearIrqStatus()"));
#endif

  uint8_t buffer[2];

  buffer[0] = (uint8_t) (irqMask >> 8);
  buffer[1] = (uint8_t) (irqMask & 0xFF);
  writeCommand(RADIO_CLR_IRQSTATUS, buffer, 2);
}


uint16_t SX128XLT::readIrqStatus()
{
#ifdef SX128XDEBUG
  Serial.print(F("readIrqStatus()"));
#endif

  uint16_t temp;
  uint8_t buffer[2];

  readCommand(RADIO_GET_IRQSTATUS, buffer, 2);
  temp = ((buffer[0] << 8) + buffer[1]);
  return temp;
}


void SX128XLT::printIrqStatus()
{
#ifdef SX128XDEBUG
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
  if (_IrqStatus & IRQ_SYNCWORD_VALID)
  {
    Serial.print(F(",IRQ_SYNCWORD_VALID"));
  }

  //0x0008
  if (_IrqStatus & IRQ_SYNCWORD_ERROR)
  {
    Serial.print(F(",IRQ_SYNCWORD_ERROR"));
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
  if (_IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
  {
    Serial.print(F(",IRQ_RANGING_SLAVE_RESPONSE_DONE"));
  }

  //0x0100
  if (_IrqStatus & IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
  {
    Serial.print(",IRQ_RANGING_SLAVE_REQUEST_DISCARDED");
  }

  //0x0200
  if (_IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
  {
    Serial.print(F(",IRQ_RANGING_MASTER_RESULT_VALID"));
  }

  //0x0400
  if (_IrqStatus & IRQ_RANGING_MASTER_RESULT_TIMEOUT)
  {
    Serial.print(F(",IRQ_RANGING_MASTER_RESULT_TIMEOUT"));
  }

  //0x0800
  if (_IrqStatus & IRQ_RANGING_SLAVE_REQUEST_VALID)
  {
    Serial.print(F(",IRQ_RANGING_SLAVE_REQUEST_VALID"));
  }

  //0x1000
  if (_IrqStatus & IRQ_CAD_DONE)
  {
    Serial.print(F(",IRQ_CAD_DONE"));
  }

  //0x2000
  if (_IrqStatus & IRQ_CAD_ACTIVITY_DETECTED)
  {
    Serial.print(F(",IRQ_CAD_ACTIVITY_DETECTED"));
  }

  //0x4000
  if (_IrqStatus & IRQ_RX_TX_TIMEOUT)
  {
    Serial.print(F(",IRQ_RX_TX_TIMEOUT"));
  }

  //0x8000
  if (_IrqStatus & IRQ_PREAMBLE_DETECTED)
  {
    Serial.print(F(",IRQ_PREAMBLE_DETECTED"));
  }
}


uint16_t SX128XLT::CRCCCITT(uint8_t *buffer, uint8_t size, uint16_t start)
{
#ifdef SX128XDEBUG
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


uint8_t SX128XLT::receive(uint8_t *rxbuffer, uint8_t size, uint16_t timeout, uint8_t wait)
{
#ifdef SX128XDEBUG
  Serial.println(F("receive()"));
#endif

  uint8_t index, RXstart, RXend;
  uint16_t regdata;
  uint8_t buffer[2];

  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
  setRx(timeout);

  if (!wait)
  {
    return 0;                                                               //not wait requested so no packet length to pass
  }

  while (!digitalRead(_RXDonePin));                                         //Wait for DIO1 to go high

  setMode(MODE_STDBY_RC);                                                   //ensure to stop further packet reception

  regdata = readIrqStatus();

  if ( (regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT ) ) //check if any of the preceding IRQs is set
  {
    return 0;                          //packet is errored somewhere so return 0
  }

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  
  if (_RXPacketL > size)               //check passed buffer is big enough for packet
  {
    _RXPacketL = size;                 //truncate packet if not enough space
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


uint8_t SX128XLT::readPacketRSSI()
{
#ifdef SX128XDEBUG
  Serial.println(F("readPacketRSSI()"));
#endif

  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5) ;
  _PacketRSSI = -status[0] / 2;

  return _PacketRSSI;
}


uint8_t SX128XLT::readPacketSNR()
{
#ifdef SX128XDEBUG
  Serial.println(F("readPacketSNR()"));
#endif

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


uint8_t SX128XLT::readRXPacketL()
{
#ifdef SX128XDEBUG
  Serial.println(F("readRXPacketL()"));
#endif

  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  return _RXPacketL;
}


void SX128XLT::setRx(uint16_t timeout)
{
#ifdef SX128XDEBUG
  Serial.println(F("setRx()"));
#endif

  if (_rxtxpinmode)
  {
    rxEnable();
  }

  uint8_t buffer[3];

  clearIrqStatus(IRQ_RADIO_ALL);                             //clear all interrupt flags
  buffer[0] = _PERIODBASE;                                   //use pre determined period base setting
  buffer[1] = ( uint8_t ) ((timeout >> 8 ) & 0x00FF);
  buffer[2] = ( uint8_t ) (timeout & 0x00FF);
  writeCommand(RADIO_SET_RX, buffer, 3);
}

/***************************************************************************
//Start direct access SX buffer routines
***************************************************************************/

void SX128XLT::startWriteSXBuffer(uint8_t ptr)
{
#ifdef SX128XDEBUG
  Serial.println(F("startWriteSXBuffer()"));
#endif

  _TXPacketL = 0;                   //this variable used to keep track of bytes written
  setMode(MODE_STDBY_RC);
  setBufferBaseAddress(ptr, 0);     //TX,RX
  checkBusy();
  
  #ifdef USE_SPI_TRANSACTION        //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif
  
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(ptr);                //address in SX buffer to write to     
  //SPI interface ready for byte to write to buffer
}


uint8_t  SX128XLT::endWriteSXBuffer()
{
#ifdef SX128XDEBUG
  Serial.println(F("endWriteSXBuffer()"));
#endif

  digitalWrite(_NSS, HIGH);
  
  #ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif
  
  return _TXPacketL;
  
}


void SX128XLT::startReadSXBuffer(uint8_t ptr)
{
#ifdef SX128XDEBUG
  Serial.println(F("startReadSXBuffer"));
#endif

  _RXPacketL = 0;
  
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


uint8_t SX128XLT::endReadSXBuffer()
{
#ifdef SX128XDEBUG
  Serial.println(F("endReadSXBuffer()"));
#endif

  digitalWrite(_NSS, HIGH);
  
  #ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif
  
  return _RXPacketL;
}


void SX128XLT::writeUint8(uint8_t x)
{
#ifdef SX128XDEBUG
  Serial.println(F("writeUint8()"));
#endif

  SPI.transfer(x);

  _TXPacketL++;                     //increment count of bytes written
}

uint8_t SX128XLT::readUint8()
{
#ifdef SX128XDEBUG
  Serial.println(F("readUint8()"));
#endif
  byte x;

  x = SPI.transfer(0);

  _RXPacketL++;                      //increment count of bytes read
  return (x);
}


void SX128XLT::writeInt8(int8_t x)
{
#ifdef SX128XDEBUG
  Serial.println(F("writeInt8()"));
#endif

  SPI.transfer(x);

  _TXPacketL++;                      //increment count of bytes written
}


int8_t SX128XLT::readInt8()
{
#ifdef SX128XDEBUG
  Serial.println(F("readInt8()"));
#endif
  int8_t x;

  x = SPI.transfer(0);

  _RXPacketL++;                      //increment count of bytes read
  return (x);
}


void SX128XLT::writeInt16(int16_t x)
{
#ifdef SX128XDEBUG
  Serial.println(F("writeInt16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXPacketL = _TXPacketL + 2;         //increment count of bytes written
}


int16_t SX128XLT::readInt16()
{
#ifdef SX128XDEBUG
  Serial.println(F("readInt16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXPacketL = _RXPacketL + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX128XLT::writeUint16(uint16_t x)
{
#ifdef SX128XDEBUG
  Serial.println(F("writeUint16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXPacketL = _TXPacketL + 2;         //increment count of bytes written
}


uint16_t SX128XLT::readUint16()
{
#ifdef SX128XDEBUG
  Serial.println(F("writeUint16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXPacketL = _RXPacketL + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX128XLT::writeInt32(int32_t x)
{
#ifdef SX128XDEBUG
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


int32_t SX128XLT::readInt32()
{
#ifdef SX128XDEBUG
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


void SX128XLT::writeUint32(uint32_t x)
{
#ifdef SX128XDEBUG
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


uint32_t SX128XLT::readUint32()
{
#ifdef SX128XDEBUG
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


void SX128XLT::writeFloat(float x)
{
#ifdef SX128XDEBUG
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


float SX128XLT::readFloat()
{
#ifdef SX128XDEBUG
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


uint8_t SX128XLT::transmitSXBuffer(uint8_t startaddr, uint8_t length, uint16_t timeout, int8_t txpower, uint8_t wait)
{
#ifdef SX128XDEBUG
  Serial.println(F("transmitSXBuffer()"));
#endif

  setBufferBaseAddress(startaddr, 0);          //TX, RX

  setPacketParams(savedPacketParam1, savedPacketParam2, length, savedPacketParam4, savedPacketParam5, savedPacketParam6, savedPacketParam7);
  setTxParams(txpower, RAMP_TIME);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  setTx(timeout);                            //this starts the TX

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


void SX128XLT::writeBuffer(uint8_t *txbuffer, uint8_t size)
{
#ifdef SX128XDEBUG1
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


uint8_t SX128XLT::receiveSXBuffer(uint8_t startaddr, uint16_t timeout, uint8_t wait )
{
#ifdef SX127XDEBUG1
  Serial.println(F("receiveSXBuffer()"));
#endif

  uint16_t regdata;
  uint8_t buffer[2];

  setMode(MODE_STDBY_RC);
  
  setBufferBaseAddress(0, startaddr);               //order is TX RX
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
  
  setRx(timeout);                                 //no actual RX timeout in this function

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



uint8_t SX128XLT::readBuffer(uint8_t *rxbuffer)
{
#ifdef SX128XDEBUG1
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
  _RXPacketL = _RXPacketL + index;       //increment count of bytes read
  return index;                          //return the actual size of the buffer, till the null (0) detected

}


void SX128XLT::setSyncWord1(uint32_t syncword)
{
#ifdef SX128XDEBUG1
  Serial.println(F("setSyncWord1()"));
#endif

  //For FLRC packet type, the SyncWord is one byte shorter and
  //the base address is shifted by one byte
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR, ( syncword >> 24 ) & 0x000000FF );
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR + 1, ( syncword >> 16 ) & 0x000000FF );
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR + 2, ( syncword >> 8 ) & 0x000000FF );
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR + 3, syncword & 0x000000FF );
}


/***************************************************************************
//End direct access SX buffer routines
***************************************************************************/



//*******************************************************************************
//Start Ranging routines
//*******************************************************************************


void SX128XLT::setRangingSlaveAddress(uint32_t address)
{
//sets address of ranging slave
#ifdef SX128XDEBUG
  Serial.println(F("SetRangingSlaveAddress()"));
#endif

  uint8_t buffer[4];

  buffer[0] = (address >> 24u ) & 0xFFu;
  buffer[1] = (address >> 16u) & 0xFFu;
  buffer[2] = (address >>  8u) & 0xFFu;
  buffer[3] = (address & 0xFFu);

  writeRegisters(0x916, buffer, 4 );
}


void SX128XLT::setRangingMasterAddress(uint32_t address)
{
//sets address of ranging master
#ifdef SX128XDEBUG
  Serial.println(F("SetRangingMasterAddress()"));
#endif

  uint8_t buffer[4];

  buffer[0] = (address >> 24u ) & 0xFFu;
  buffer[1] = (address >> 16u) & 0xFFu;
  buffer[2] = (address >>  8u) & 0xFFu;
  buffer[3] = (address & 0xFFu);

  writeRegisters(0x912, buffer, 4 );
}


void SX128XLT::setRangingCalibration(uint16_t cal)
{
   #ifdef SX128XDEBUG
  Serial.println(F("setRangingCalibration()"));
#endif
  savedCalibration = cal;
  writeRegister( REG_LR_RANGINGRERXTXDELAYCAL, ( uint8_t )( ( cal >> 8 ) & 0xFF ) );
  writeRegister( REG_LR_RANGINGRERXTXDELAYCAL + 1, ( uint8_t )( ( cal ) & 0xFF ) );
}


void SX128XLT::setRangingRole(uint8_t role)
{
#ifdef SX128XDEBUG
  Serial.println(F("setRangingRole()"));
#endif

  uint8_t buffer[1];

  buffer[0] = role;

  writeCommand(RADIO_SET_RANGING_ROLE, buffer, 1 );
}


uint32_t SX128XLT::getRangingResultRegValue(uint8_t resultType)
{
  uint32_t valLsb = 0;

  setMode(MODE_STDBY_XOSC);
  writeRegister( 0x97F, readRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
  writeRegister( REG_LR_RANGINGRESULTCONFIG, ( readRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
  valLsb = ( ( (uint32_t) readRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( (uint32_t) readRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( readRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
  setMode(MODE_STDBY_RC);
  return valLsb;
}


double SX128XLT::getRangingDistance(uint8_t resultType, int32_t regval, float adjust)
{
  float val = 0.0;

  if (regval >= 0x800000)                  //raw reg value at low distance can goto 0x800000 which is negative, set distance to zero if this happens
  {
    regval = 0;
  }

  // Conversion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280

  switch (resultType)
  {
    case RANGING_RESULT_RAW:
      // Convert the ranging LSB to distance in meter. The theoretical conversion from register value to distance [m] is given by:
      // distance [m] = ( complement2( register ) * 150 ) / ( 2^12 * bandwidth[MHz] ) ). The API provide BW in [Hz] so the implemented
      // formula is complement2( register ) / bandwidth[Hz] * A, where A = 150 / (2^12 / 1e6) = 36621.09
      val = ( double ) regval / ( double ) returnBandwidth(savedModParam2) * 36621.09375;
      break;

    case RANGING_RESULT_AVERAGED:
    case RANGING_RESULT_DEBIASED:
    case RANGING_RESULT_FILTERED:
      Serial.print(F("??"));
      val = ( double )regval * 20.0 / 100.0;
      break;
    default:
      val = 0.0;
      break;
  }
  
  val = val * adjust;
  return val;
}


bool SX128XLT::setupRanging(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint32_t address, uint8_t role)
{
 //sequence is frequency, offset, spreading factor, bandwidth, coding rate, calibration, role.  
 #ifdef SX128XDEBUG
  Serial.println(F("setupRanging()"));
#endif
 
  setMode(MODE_STDBY_RC);
  setPacketType(PACKET_TYPE_RANGING);
  setModulationParams(modParam1, modParam2, modParam3);  
  setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  setRfFrequency(frequency, offset);
  setRangingSlaveAddress(address);
  setRangingMasterAddress(address);
  setRangingCalibration(lookupCalibrationValue(modParam1, modParam2));
  setRangingRole(role);
  setHighSensitivity();
  return true;
}



bool SX128XLT::transmitRanging(uint32_t address, uint16_t timeout, int8_t txpower, uint8_t wait)
{
#ifdef SX128XDEBUG
  Serial.println(F("transmitRanging()"));
#endif

  if ((_RXEN >= 0) || (_TXEN >= 0))
  {
   return false;
  }    
  
  setMode(MODE_STDBY_RC);
  setRangingMasterAddress(address);
  setTxParams(txpower, RADIO_RAMP_02_US);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RANGING_MASTER_RESULT_VALID + IRQ_RANGING_MASTER_RESULT_TIMEOUT), 0, 0);
  setTx(timeout);                               //this sends the ranging packet
    
  if (!wait)
  {
    return true;
  }

  while (!digitalRead(_TXDonePin));                               //Wait for DIO1 to go high

  if (readIrqStatus() & IRQ_RANGING_MASTER_RESULT_VALID )       //check for timeout
  {
    return true;
  }
  else
  {
    return false;
  }
}


uint8_t SX128XLT::receiveRanging(uint32_t address, uint16_t timeout, int8_t txpower, uint8_t wait)
{
#ifdef SX128XDEBUG
  Serial.println(F("receiveRanging()"));
#endif
  
  setTxParams(txpower, RADIO_RAMP_02_US);
  setRangingSlaveAddress(address);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RANGING_SLAVE_RESPONSE_DONE + IRQ_RANGING_SLAVE_REQUEST_DISCARDED), 0, 0);
  setRx(timeout);

  if (!wait)
  {
    return NO_WAIT;                                                            //not wait requested so no packet length to pass
  }

  while (!digitalRead(_RXDonePin));
    
  setMode(MODE_STDBY_RC);                                                    //ensure to stop further packet reception
  
  if (readIrqStatus() & IRQ_RANGING_SLAVE_REQUEST_VALID)
  {
  return true; 
  }
  else
  { 
  return false;                                                               //so we can check for packet having enough buffer space
  }
}


uint16_t SX128XLT::lookupCalibrationValue(uint8_t spreadingfactor, uint8_t bandwidth)
{
//this looks up the calibration value from the table in SX128XLT_Definitions.hifdef SX128XDEBUG
#ifdef SX128XDEBUG
Serial.println(F("lookupCalibrationValue()"));
#endif


switch (bandwidth)
  {
    case LORA_BW_0400:
      savedCalibration = RNG_CALIB_0400[(spreadingfactor>>4)-5];
      return savedCalibration;
  
    case LORA_BW_0800:
      savedCalibration = RNG_CALIB_0800[(spreadingfactor>>4)-5];
      return savedCalibration;
  

    case LORA_BW_1600:
     savedCalibration = RNG_CALIB_1600[(spreadingfactor>>4)-5];
     return savedCalibration;

    default:
      return 0xFFFF;

  }
  
}


uint16_t SX128XLT::getSetCalibrationValue()
{
#ifdef SX128XDEBUG
  Serial.println(F("getCalibrationValue()"));
#endif

return savedCalibration;;
  
}


//*******************************************************************************
//End Ranging routines
//*******************************************************************************


void SX128XLT::setSleep(uint8_t sleepconfig)
{
#ifdef SX128XDEBUG
  Serial.println(F("setSleep()"));
#endif
  
  setMode(MODE_STDBY_RC);
  checkBusy();
  
  #ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  //need to save registers to device RAM first
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_SET_SAVECONTEXT);
  digitalWrite(_NSS, HIGH);
  
  checkBusy();
  
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_SET_SLEEP);
  SPI.transfer(sleepconfig);
  digitalWrite(_NSS, HIGH);
  
  #ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif
  delay(1);           //allow time for shutdown
}


uint16_t SX128XLT::CRCCCITTSX(uint8_t startadd, uint8_t endadd, uint16_t startvalue)
{
  //genrates a CRC of an area of the internal SX buffer

#ifdef SX126XDEBUG1
  Serial.println(F("CRCCCITTSX()"));
#endif


  uint16_t index, libraryCRC;
  uint8_t j;

  libraryCRC = startvalue;                                  //start value for CRC16

  startReadSXBuffer(startadd);                       //begin the buffer read

  for (index = startadd; index <= endadd; index++)
  {
    libraryCRC ^= (((uint16_t) readUint8() ) << 8);
    for (j = 0; j < 8; j++)
    {
      if (libraryCRC & 0x8000)
        libraryCRC = (libraryCRC << 1) ^ 0x1021;
      else
        libraryCRC <<= 1;
    }
  }
  
  endReadSXBuffer();                                 //end the buffer read

  return libraryCRC;
}



uint8_t SX128XLT::getByteSXBuffer(uint8_t addr)
{
#ifdef SX128XDEBUG1
  Serial.println(F("getByteSXBuffer()"));
#endif

  uint8_t regdata;
  setMode(MODE_STDBY_RC);                     //this is needed to ensure we can read from buffer OK.

#ifdef USE_SPI_TRANSACTION                    //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);             //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(addr);
  SPI.transfer(0xFF);
  regdata = SPI.transfer(0);
  digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
  SPI.endTransaction();
#endif

  return regdata;
}


void SX128XLT::printSXBufferHEX(uint8_t start, uint8_t end)
{
#ifdef SX128XDEBUG
  Serial.println(F("printSXBufferHEX()"));
#endif

  uint8_t index, regdata;

  setMode(MODE_STDBY_RC);

#ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif

  digitalWrite(_NSS, LOW);                       //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(start);
  SPI.transfer(0xFF);

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


void SX128XLT::printHEXByte(uint8_t temp)
{
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void SX128XLT::wake()
{
#ifdef SX128XDEBUG
  Serial.println(F("wake()"));
#endif

digitalWrite(_NSS, LOW);
delay(1);
digitalWrite(_NSS, HIGH);
delay(1);
}


int32_t SX128XLT::getFrequencyErrorRegValue()
{
  #ifdef SX128XDEBUG
  Serial.println(F("getFrequencyErrorRegValue()"));
#endif
  
  int32_t FrequencyError;
  uint32_t regmsb, regmid, reglsb, allreg;
  
  setMode(MODE_STDBY_XOSC);
  
  regmsb = readRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
  regmsb = regmsb & 0x0F;       //clear bit 20 which is always set
  
  regmid = readRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
  
  reglsb = readRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
  setMode(MODE_STDBY_RC);

  #ifdef LORADEBUG
  Serial.println();
  Serial.print(F("Registers "));
  Serial.print(regmsb,HEX);
  Serial.print(F(" "));
  Serial.print(regmid,HEX);
  Serial.print(F(" "));
  Serial.println(reglsb,HEX);
  #endif
    
  allreg = (uint32_t) ( regmsb << 16 ) | ( regmid << 8 ) | reglsb;

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


int32_t SX128XLT::getFrequencyErrorHz()
{
    #ifdef SX128XDEBUG
  Serial.println(F("getFrequencyErrorHz()"));
#endif
  
  int32_t error, regvalue;
  uint32_t bandwidth;
  float divider;

  bandwidth =   returnBandwidth(savedModParam2);                   //gets the last configured bandwidth
  
  divider = (float) 1625000 / bandwidth;                           //data sheet says 1600000, but bandwidth is 1625000
  regvalue = getFrequencyErrorRegValue();
  error = (FREQ_ERROR_CORRECTION * regvalue) / divider;

  return error;
}

uint8_t SX128XLT::transmitAddressed(uint8_t *txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, uint32_t timeout, int8_t txpower, uint8_t wait)
{
#ifdef SX128XDEBUG
  Serial.println(F("transmitAddressed()"));
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

  
  if (savedPacketType == PACKET_TYPE_LORA)
  {
   writeRegister(REG_LR_PAYLOADLENGTH, _TXPacketL);                           //only seems to work for lora  
  }  
  else if (savedPacketType == PACKET_TYPE_FLRC)
  {
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, _TXPacketL, savedPacketParam6, savedPacketParam7);
  }

  setTxParams(txpower, RAMP_TIME);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  setTx(timeout);                                                          //this starts the TX
  
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


uint8_t SX128XLT::receiveAddressed(uint8_t *rxbuffer, uint8_t size, uint16_t timeout, uint8_t wait)
{
#ifdef SX128XDEBUG
  Serial.println(F("receiveAddressed()"));
#endif

  uint8_t index, RXstart, RXend;
  uint16_t regdata;
  uint8_t buffer[2];

  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
  setRx(timeout);

  if (!wait)
  {
    return 0;                                                               //not wait requested so no packet length to pass
  }

  while (!digitalRead(_RXDonePin));                                         //Wait for DIO1 to go high

  setMode(MODE_STDBY_RC);                                                   //ensure to stop further packet reception

  regdata = readIrqStatus();

  if ( (regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT ) ) //check if any of the preceding IRQs is set
  {
    return 0;                          //packet is errored somewhere so return 0
  }

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  
  if (_RXPacketL > size)               //check passed buffer is big enough for packet
  {
    _RXPacketL = size;                 //truncate packet if not enough space
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
  
  _RXPacketType = SPI.transfer(0);
  _RXDestination = SPI.transfer(0);
  _RXSource = SPI.transfer(0);

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


uint8_t SX128XLT::readRXPacketType()
{
#ifdef SX128XDEBUG
  Serial.println(F("readRXPacketType()"));
#endif
return _RXPacketType;
}


uint8_t SX128XLT::readPacket(uint8_t *rxbuffer, uint8_t size)
{
#ifdef SX128XDEBUG
  Serial.println(F("readPacket()"));
#endif

  uint8_t index, regdata, RXstart, RXend;
  uint8_t buffer[2];
  
  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  
  if (_RXPacketL > size)               //check passed buffer is big enough for packet
  {
  _RXPacketL = size;                   //truncate packet if not enough space
  }
  
  RXstart = buffer[1];
  
  RXend = RXstart + _RXPacketL;

  #ifdef USE_SPI_TRANSACTION     //to use SPI_TRANSACTION enable define at beginning of CPP file 
  SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode));
#endif
  
  digitalWrite(_NSS, LOW);               //start the burst read
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


uint16_t SX128XLT::addCRC(uint8_t data, uint16_t libraryCRC)
{
  uint8_t j;

  libraryCRC ^= ((uint16_t)data << 8);
  for (j = 0; j < 8; j++)
  {
    if (libraryCRC & 0x8000)
      libraryCRC = (libraryCRC << 1) ^ 0x1021;
    else
      libraryCRC <<= 1;
  }
  return libraryCRC;
}


void SX128XLT::writeBufferChar(char *txbuffer, uint8_t size)
{
#ifdef SX128XDEBUG1
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


uint8_t SX128XLT::readBufferChar(char *rxbuffer)
{
#ifdef SX128XDEBUG1
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
  
  _RXPacketL = _RXPacketL + index;       //increment count of bytes read
  
  return index;                          //return the actual size of the buffer, till the null (0) detected

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





