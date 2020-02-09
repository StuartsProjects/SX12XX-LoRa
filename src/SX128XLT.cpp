/*
  Copyright 2019 - Stuart Robinson
  Licensed under a MIT license displayed at the bottom of this document.
  06/02/20
*/


#include <SX128XLT.h>
#include <SPI.h>

#define LTUNUSED(v) (void) (v)    //add LTUNUSED(variable); to avoid compiler warnings 
#define USE_SPI_TRANSACTION

//#define SX128XDEBUG             //enable debug messages
//#define RANGINGDEBUG            //enable debug messages

SX128XLT::SX128XLT()
{
}


bool SX128XLT::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, uint8_t device)
{

  //assign the passed pins to the class private variable
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = pinDIO2;
  _DIO3 = pinDIO3;
  _Device = device;
  _TXDonePin = pinDIO1;        //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;        //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);


#ifdef SX128XDEBUG
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

  resetDevice();

  if (checkDevice())
  {
    return true;
  }

  return false;
}


void SX128XLT::rxtxInit(int8_t pinRXEN, int8_t pinTXEN)
{
#ifdef SX128XDEBUG
  Serial.println(F("rxtxpinInit()"));
#endif
  _rxtxpinmode = true;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;

  pinMode(pinRXEN, OUTPUT);
  digitalWrite(pinRXEN, LOW);           //pins needed for E28-2G4M20S
  pinMode(pinTXEN, OUTPUT);
  digitalWrite(pinTXEN, LOW);           //pins needed for E28-2G4M20S
}


void SX128XLT::rxEnable()
{
  //Enable RX mode on device such as Ebyte E28-2G4M20S which have RX and TX enable pins
#ifdef SX128XDEBUG
  Serial.println(F("rxEnable()"));
#endif

  digitalWrite(_RXEN, HIGH);
  digitalWrite(_TXEN, LOW);
}



void SX128XLT::txEnable()
{
  //Enable RX mode on device such as Ebyte E28-2G4M20S which have RX and TX enable pins
#ifdef SX128XDEBUG
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

    if (busy_timeout_cnt > 5)                     //wait 5mS for busy to complete
    {
      Serial.println(F("ERROR - Busy Timeout!"));
      setMode(0);                              //0:STDBY_RC; 1:STDBY_XOSC
      resetDevice();
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

  checkBusy();
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

  checkBusy();
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
  checkBusy();
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


void SX128XLT::printLoraSettings()
{
#ifdef SX128XDEBUG
  Serial.println(F("printLoraSettings()"));
#endif

  printDevice();
  Serial.print(F(","));
  Serial.print(getFreqInt());
  Serial.print(F("hz,SF"));
  Serial.print(getLoRaSF());
  Serial.print(F(",BW"));
  //Serial.print(getLoRaBandwidth());
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
  uint8_t Msb, Mid, Lsb;
  uint32_t uinttemp;
  float floattemp;
  Msb = readRegister(REG_RFFrequency23_16);
  Mid = readRegister(REG_RFFrequency15_8);
  Lsb = readRegister(REG_RFFrequency7_0);
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


uint8_t SX128XLT::transmit(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait)
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

  checkBusy();
  _TXPacketL = size;
  writeRegister(REG_LR_PAYLOADLENGTH, _TXPacketL);
  setTxParams(txpower, RAMP_TIME);

  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  setTx(txtimeout);                                                          //this starts the TX

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


uint8_t SX128XLT::receive(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait)
{
#ifdef SX128XDEBUG
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

  return _PacketRSSI;
}


uint8_t SX128XLT::readPacketSNR()
{
#ifdef SX128XDEBUG
  Serial.println(F("readPacketSNR()"));
#endif


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





