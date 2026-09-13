/*
  Copyright 2026 - Stuart Robinson
  Licensed under a MIT license displayed at the bottom of this document.
  10/09/26
*/

/*
  Parts of code Copyright (c) 2013, SEMTECH S.A.
  See LICENSE.TXT file included in the library
  10/09/26
*/

/*
Enhanced version of the original SX126XLT library for the Semtech SX126X
See the SX126X.h file for the changes.
*/



#include "SX126X.h"

#define SPIUNUSED(v) (void)(v)  //add SPIUNUSED(variable); to avoid compiler warnings
#define USE_SPI_TRANSACTION     //enable use of SPI_TRANSACTION


//#define DEBUGBUSY                  //comment out if you do not want a busy timeout message
//#define SX126XDEBUG                //enable debug messages
//#define SX126XDEBUG3               //enable debug messages
//#define SX126XDEBUGPINS            //enable pin allocation debug messages
//#define DEBUGFSKRTTY               //enable for FSKRTTY debugging
//#define DEBUGRXDUTYCYCLE           //comment out to see RXDuty cycle parameters  

SX126X::SX126X()
  :                                    //Anything you need when instantiating the LoRa object goes here
    _spi(&LORA_DEFAULT_SPI),           //default defined in SX126X.h
    _streamRef(&LORA_DEFAULT_Serial),  //default defined in SX126X.h
    _spiSettings(SPIspeedMaximum, SPIdataOrder, SPIdataMode) {
}


void SX126X::setSPI(SPIClass &spi) {
  //tested on HSPI of ESP32S3
  _spi = &spi;
}


void SX126X::setSerial(Stream *streamObject) {
  _streamRef = streamObject;
}


void SX126X::sendText(char *someText) {
  _streamRef->println(someText);
}


void SX126X::setSPIFrequency(uint32_t frequency) {
  _spiSettings = SPISettings(frequency, SPIdataOrder, SPIdataMode);
}

/* Formats for begin()
  original > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, int8_t pinSW, uint8_t device);
  1 All pins > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, int8_t pinRXEN, int8_t pinTXEN, int8_t pinSW, uint8_t device)
  2 NiceRF   > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, uint8_t device)
  3 Dorji    > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinSW, uint8_t device)
  4 Ebyte    > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinRXEN, int8_t pinTXEN, uint8_t device)
  5 IRQ      > begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, uint8_t device)
*/


bool SX126X::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, int8_t pinRXEN, int8_t pinTXEN, int8_t pinSW, uint8_t device) {
  //format 1 pins, assign all available pins
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = pinDIO2;
  _DIO3 = pinDIO3;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;
  _SW = pinSW;
  _Device = device;

  _TXDonePin = pinDIO1;  //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;  //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);


#ifdef SX126XDEBUGPINS
  _streamRef->println(F("format 1 begin()"));
  _streamRef->println(F("SX126X constructor instantiated successfully"));
  _streamRef->print(F("NSS "));
  _streamRef->println(_NSS);
  _streamRef->print(F("NRESET "));
  _streamRef->println(_NRESET);
  _streamRef->print(F("RFBUSY "));
  _streamRef->println(_RFBUSY);
  _streamRef->print(F("DIO1 "));
  _streamRef->println(_DIO1);
  _streamRef->print(F("DIO2 "));
  _streamRef->println(_DIO2);
  _streamRef->print(F("DIO3 "));
  _streamRef->println(_DIO3);
  _streamRef->print(F("RX_EN "));
  _streamRef->println(_RXEN);
  _streamRef->print(F("TXEN "));
  _streamRef->println(_TXEN);
  _streamRef->print(F("SW "));
  _streamRef->println(_SW);
#endif

  if (_DIO1 >= 0) {
    pinMode(_DIO1, INPUT);
  }

  if (_DIO2 >= 0) {
    pinMode(_DIO2, INPUT);
  }

  if (_DIO3 >= 0) {
    pinMode(_DIO3, INPUT);
  }

  if ((_RXEN >= 0) && (_TXEN >= 0)) {
#ifdef SX126XDEBUGPINS
    _streamRef->println(F("RX_EN & TX_EN switching enabled"));
#endif
    pinMode(_RXEN, OUTPUT);
    pinMode(_TXEN, OUTPUT);
    _rxtxpinmode = true;
  } else {
#ifdef SX126XDEBUGPINS
    _streamRef->println(F("RX_EN & TX_EN not used"));
#endif
    _rxtxpinmode = false;
  }

  if (_SW >= 0) {
    pinMode(_SW, OUTPUT);  //Dorji devices have an RW pin that needs to be set high to power antenna switch
    digitalWrite(_SW, HIGH);
  }

  resetDevice();
  if (checkDevice()) {
    return true;
  }

  return false;
}


bool SX126X::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, uint8_t device) {
  //format 2 pins for NSS, NRESET, RFBUSY, DIO1
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = -1;
  _DIO3 = -1;
  _RXEN = -1;
  _TXEN = -1;
  _SW = -1;
  _Device = device;

  _TXDonePin = pinDIO1;  //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;  //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

#ifdef SX126XDEBUGPINS
  _streamRef->println(F("format 2 NiceRF begin()"));
  _streamRef->println(F("SX126X constructor instantiated successfully"));
  _streamRef->print(F("NSS "));
  _streamRef->println(_NSS);
  _streamRef->print(F("NRESET "));
  _streamRef->println(_NRESET);
  _streamRef->print(F("RFBUSY "));
  _streamRef->println(_RFBUSY);
  _streamRef->print(F("DIO1 "));
  _streamRef->println(_DIO1);
  _streamRef->print(F("DIO2 "));
  _streamRef->println(_DIO2);
  _streamRef->print(F("DIO3 "));
  _streamRef->println(_DIO3);
  _streamRef->print(F("RX_EN "));
  _streamRef->println(_RXEN);
  _streamRef->print(F("TX_EN "));
  _streamRef->println(_TXEN);
  _streamRef->print(F("SW "));
  _streamRef->println(_SW);
#endif

  if (_DIO1 >= 0) {
    pinMode(_DIO1, INPUT);
  }

#ifdef SX126XDEBUGPINS
  _streamRef->println(F("RX_EN & TX_EN switching disabled"));
#endif

  _rxtxpinmode = false;

  resetDevice();

  if (checkDevice()) {
    return true;
  }

  return false;
}


bool SX126X::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinSW, uint8_t device) {
  //format 3 pins for NSS, NRESET, RFBUSY, DIO1, SW
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = -1;
  _DIO3 = -1;
  _RXEN = -1;
  _TXEN = -1;
  _SW = pinSW;
  _Device = device;

  _TXDonePin = pinDIO1;  //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;  //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

#ifdef SX126XDEBUGPINS
  _streamRef->println(F("format 3 Dorji begin()"));
  _streamRef->println(F("SX126X constructor instantiated successfully"));
  _streamRef->print(F("NSS "));
  _streamRef->println(_NSS);
  _streamRef->print(F("NRESET "));
  _streamRef->println(_NRESET);
  _streamRef->print(F("RFBUSY "));
  _streamRef->println(_RFBUSY);
  _streamRef->print(F("DIO1 "));
  _streamRef->println(_DIO1);
  _streamRef->print(F("DIO2 "));
  _streamRef->println(_DIO2);
  _streamRef->print(F("DIO3 "));
  _streamRef->println(_DIO3);
  _streamRef->print(F("RX_EN "));
  _streamRef->println(_RXEN);
  _streamRef->print(F("TX_EN "));
  _streamRef->println(_TXEN);
  _streamRef->print(F("SW "));
  _streamRef->println(_SW);
#endif

  if (_DIO1 >= 0) {
    pinMode(_DIO1, INPUT);
  }

#ifdef SX126XDEBUGPINS
  _streamRef->println(F("RX_EN & TX_EN switching disabled"));
#endif

  _rxtxpinmode = false;

  if (_SW >= 0) {
    pinMode(_SW, OUTPUT);  //Dorji devices have an RW pin that needs to be set high to power antenna switch
    digitalWrite(_SW, HIGH);
  }

  resetDevice();

  if (checkDevice()) {
    return true;
  }

  return false;
}


bool SX126X::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinRXEN, int8_t pinTXEN, uint8_t device) {
  //format 4 pins, NSS, NRESET, RFBUSY, DIO1, RXEN, TXEN
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = -1;
  _DIO3 = -1;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;
  _SW = -1;
  _Device = device;

  _TXDonePin = pinDIO1;  //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;  //this is defalt pin for sensing RX done

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

#ifdef SX126XDEBUGPINS
  _streamRef->println(F("format 4 Ebyte begin()"));
  _streamRef->println(F("SX126X constructor instantiated successfully"));
  _streamRef->print(F("NSS "));
  _streamRef->println(_NSS);
  _streamRef->print(F("NRESET "));
  _streamRef->println(_NRESET);
  _streamRef->print(F("RFBUSY "));
  _streamRef->println(_RFBUSY);
  _streamRef->print(F("DIO1 "));
  _streamRef->println(_DIO1);
  _streamRef->print(F("DIO2 "));
  _streamRef->println(_DIO2);
  _streamRef->print(F("DIO3 "));
  _streamRef->println(_DIO3);
  _streamRef->print(F("RX_EN "));
  _streamRef->println(_RXEN);
  _streamRef->print(F("TX_EN "));
  _streamRef->println(_TXEN);
  _streamRef->print(F("SW "));
  _streamRef->println(_SW);
#endif

  if (_DIO1 >= 0) {
    pinMode(_DIO1, INPUT);
  }

  if ((_RXEN >= 0) && (_TXEN >= 0)) {
#ifdef SX126XDEBUGPINS
    _streamRef->println(F("RX_EN & TX_EN switching enabled"));
#endif
    pinMode(_RXEN, OUTPUT);
    pinMode(_TXEN, OUTPUT);
    _rxtxpinmode = true;
  } else {
#ifdef SX126XDEBUGPINS
    _streamRef->println(F("RX_EN & TX_EN switching disabled"));
#endif
    _rxtxpinmode = false;
  }

  if (_SW >= 0) {
    pinMode(_SW, OUTPUT);  //Dorji devices have an RW pin that needs to be set high to power antenna switch
    digitalWrite(_SW, HIGH);
  }

  resetDevice();
  if (checkDevice()) {
    return true;
  }

  return false;
}


bool SX126X::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, uint8_t device) {
  //format 5 pins for NSS, NRESET, RFBUSY with no DIO1
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = -1;
  _DIO2 = -1;
  _DIO3 = -1;
  _RXEN = -1;
  _TXEN = -1;
  _SW = -1;
  _Device = device;

  _TXDonePin = -1;
  _RXDonePin = -1;

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

#ifdef SX126XDEBUGPINS
  _streamRef->println(F("format 2 NiceRF begin()"));
  _streamRef->println(F("SX126X constructor instantiated successfully"));
  _streamRef->print(F("NSS "));
  _streamRef->println(_NSS);
  _streamRef->print(F("NRESET "));
  _streamRef->println(_NRESET);
  _streamRef->print(F("RFBUSY "));
  _streamRef->println(_RFBUSY);
  _streamRef->print(F("DIO1 "));
  _streamRef->println(_DIO1);
  _streamRef->print(F("DIO2 "));
  _streamRef->println(_DIO2);
  _streamRef->print(F("DIO3 "));
  _streamRef->println(_DIO3);
  _streamRef->print(F("RX_EN "));
  _streamRef->println(_RXEN);
  _streamRef->print(F("TX_EN "));
  _streamRef->println(_TXEN);
  _streamRef->print(F("SW "));
  _streamRef->println(_SW);
#endif

#ifdef SX126XDEBUGPINS
  _streamRef->println(F("RX_EN & TX_EN switching disabled"));
#endif

  _rxtxpinmode = false;

  resetDevice();

  if (checkDevice()) {
    return true;
  }

  return false;
}


bool SX126X::setPins(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinRXEN, int8_t pinTXEN, uint8_t device) {
  //format 2 pins for NSS, NRESET, RFBUSY, DIO1
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = -1;
  _DIO3 = -1;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;
  _SW = -1;
  _Device = device;
  _TXDonePin = pinDIO1;  //this is defalt pin for sensing TX done
  _RXDonePin = pinDIO1;  //this is defalt pin for sensing RX done

  if (_NSS >= 0) {
    pinMode(_NSS, OUTPUT);
  }

  if (_NRESET >= 0) {
    pinMode(_NRESET, OUTPUT);
  }

  if (_RFBUSY >= 0) {
    pinMode(_RFBUSY, OUTPUT);
  }

  if (_DIO1 >= 0) {
    pinMode(_DIO1, OUTPUT);
  }

  _rxtxpinmode = false;

  if (_RXEN >= 0) {
    pinMode(_RXEN, OUTPUT);
    _rxtxpinmode = true;
  }

  if (_TXEN >= 0) {
    pinMode(_TXEN, OUTPUT);
    _rxtxpinmode = true;
  }

#ifdef SX126XDEBUGPINS
  if (_rxtxpinmode)
	{
    _streamRef->println(F("RXEN & TXEN switching enabled"));
    }
    else 
	{
    _streamRef->println(F("RXEN & TXEN switching disabled"));
    }
#endif
	  
      return true;
}


    void SX126X::checkBusy() {
#ifdef SX126XDEBUG
      _streamRef->println(F("checkBusy()"));
#endif

      uint8_t busy_timeout_cnt;
      busy_timeout_cnt = 0;

      while (digitalRead(_RFBUSY)) {
        delay(1);
        busy_timeout_cnt++;

        //this function checks for a timeout on the busy pin
        //if there is a timeout the device is set back to the saved settings
        //the fuction is of limited benefit, since you cannot know at which stage of the
        //operation the timeout occurs, so operation could resume
        if (busy_timeout_cnt > 10)  //wait 10mS for busy to complete
        {
          busy_timeout_cnt = 0;
#ifdef DEBUGBUSY
          _streamRef->println(F("ERROR - Busy Timeout!"));
#endif
          resetDevice();  //reset device
          setMode(MODE_STDBY_RC);
          config();  //re-run saved config
          break;
        }
      }
    }


    void SX126X::writeCommand(uint8_t Opcode, uint8_t * buffer, uint16_t size) {
      uint8_t index;
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(Opcode);

      for (index = 0; index < size; index++) {
        _spi->transfer(buffer[index]);
      }
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      if (Opcode != RADIO_SET_SLEEP) {
        checkBusy();
      }
    }


    void SX126X::readCommand(uint8_t Opcode, uint8_t * buffer, uint16_t size) {
      uint8_t i;
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(Opcode);
      _spi->transfer(0xFF);

      for (i = 0; i < size; i++) {
        *(buffer + i) = _spi->transfer(0xFF);
      }
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif
    }


    void SX126X::writeRegisters(uint16_t address, uint8_t * buffer, uint16_t size) {
      uint8_t addr_l, addr_h;
      uint8_t i;

      addr_l = address & 0xff;
      addr_h = address >> 8;
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_REGISTER);
      _spi->transfer(addr_h);  //MSB
      _spi->transfer(addr_l);  //LSB

      for (i = 0; i < size; i++) {
        _spi->transfer(buffer[i]);
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif
    }


    void SX126X::writeRegister(uint16_t address, uint8_t value) {
      writeRegisters(address, &value, 1);
    }

    void SX126X::readRegisters(uint16_t address, uint8_t * buffer, uint16_t size) {
      uint16_t index;
      uint8_t addr_l, addr_h;

      addr_h = address >> 8;
      addr_l = address & 0x00FF;
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_READ_REGISTER);
      _spi->transfer(addr_h);  //MSB
      _spi->transfer(addr_l);  //LSB
      _spi->transfer(0xFF);
      for (index = 0; index < size; index++) {
        *(buffer + index) = _spi->transfer(0xFF);
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif
    }


    uint8_t SX126X::readRegister(uint16_t address) {
      uint8_t data;

      readRegisters(address, &data, 1);
      return data;
    }

    void SX126X::resetDevice() {
#ifdef SX126XDEBUG
      _streamRef->println(F("resetDevice()"));
#endif

      delay(10);
      digitalWrite(_NRESET, LOW);
      delay(2);
      digitalWrite(_NRESET, HIGH);
      delay(25);
      checkBusy();
    }


    bool SX126X::checkDevice() {
      //check there is a device out there, writes a register and reads back
#ifdef SX126XDEBUG
      _streamRef->println(F("checkDevice()"));
#endif

      uint8_t Regdata1, Regdata2;
      Regdata1 = readRegister(0x88e);  //low byte of frequency setting
      writeRegister(0x88e, (Regdata1 + 1));
      Regdata2 = readRegister(0x88e);  //read changed value back
      writeRegister(0x88e, Regdata1);  //restore register to original value

      if (Regdata2 == (Regdata1 + 1)) {
        return true;
      } else {
        return false;
      }
    }

    void SX126X::setupLoRa(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t modParam3, uint8_t modParam4) {
      //order of passed parameters is, frequency, offset, spreadingfactor, bandwidth, coderate, optimisation

#ifdef SX126XDEBUG
      _streamRef->println(F("setupLoRa()"));
#endif
	  setMode(MODE_STDBY_RC);
	  setRegulatorMode(USE_DCDC);
	  setPaConfig(0x04, PAAUTO, _Device);  //use _Device, saved by begin.
	  setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);   //if not using TCXO set to NO_TCXO
	  calibrateDevice(ALLDevices);         //is required after setting TCXO
	  calibrateImage(frequency);
	  setDIO2AsRfSwitchCtrl();
	  setPacketType(PACKET_TYPE_LORA);
	  setRfFrequency(frequency, offset);
	  setModulationParams(modParam1, modParam2, modParam3, modParam4);
	  setBufferBaseAddress(0, 0);
	  setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
	  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
	  setHighSensitivity();                                                     //set for maximum gain
	  setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
    }


    void SX126X::setupLoRa(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t modParam3, uint8_t modParam4, uint8_t tcxoVoltage, uint8_t rfswitch) {
      //order of passed parameters is, frequency, offset, spreadingfactor, bandwidth, coderate, optimisation, ,TCXO control, DIO2 as RF switch

#ifdef SX126XDEBUG
      _streamRef->println(F("setupLoRa()"));
#endif
      setMode(MODE_STDBY_RC);
      setRegulatorMode(USE_DCDC);
      setPaConfig(0x04, PAAUTO, _Device);  //use _Device, saved by begin.
      setDIO3AsTCXOCtrl(tcxoVoltage);      //if not using TCXO set to NO_TCXO
      calibrateDevice(ALLDevices);         //is required after setting TCXO
      calibrateImage(frequency);

      if (rfswitch == DIO2RFSWITCH) {
        setDIO2AsRfSwitchCtrl();
#ifdef SX126XDEBUG
        _streamRef->println(F("Using DIO2 As RF Switch Control"));
#endif
      } else {
#ifdef SX126XDEBUG
        _streamRef->println(F("Not using DIO2 As RF Switch Control"));
#endif
      }

      setPacketType(PACKET_TYPE_LORA);
      setRfFrequency(frequency, offset);
      setModulationParams(modParam1, modParam2, modParam3, modParam4);
      setBufferBaseAddress(0, 0);
      setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setHighSensitivity();                                                     //set for maximum gain
      setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
    }


    void SX126X::setMode(uint8_t modeconfig) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setMode()"));
#endif

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_SET_STANDBY);
      _spi->transfer(modeconfig);
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      _OperatingMode = modeconfig;
    }


    void SX126X::setRegulatorMode(uint8_t mode) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setRegulatorMode()"));
#endif

      savedRegulatorMode = mode;
      writeCommand(RADIO_SET_REGULATORMODE, &mode, 1);
    }


    void SX126X::setPaConfig(uint8_t dutycycle, uint8_t hpMax, uint8_t device) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setPaConfig()"));
#endif

      uint8_t buffer[4];

      if (hpMax == PAAUTO) {
        if (device == DEVICE_SX1261) {
          hpMax = 0x00;
        }
        if (device == DEVICE_SX1262) {
          hpMax = 0x07;
        }
        if (device == DEVICE_SX1268) {
          hpMax = 0x07;
        }
      }

      if (_Device == DEVICE_SX1261) {
        device = 1;
      } else {
        device = 0;
      }

      buffer[0] = dutycycle;  //paDutyCycle
      buffer[1] = hpMax;      //hpMax:0x00~0x07; 7 for =22dbm
      buffer[2] = device;     //deviceSel: 0 = SX1262; 1 = SX1261; 0 = SX1268;
      buffer[3] = 0x01;       //reserved, always 0x01

      writeCommand(RADIO_SET_PACONFIG, buffer, 4);
    }


    void SX126X::setDIO3AsTCXOCtrl(uint8_t tcxoVoltage) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setDIO3AsTCXOCtrl()"));
#endif

      if (tcxoVoltage != NO_TCXO) {
        uint8_t buffer[4];
        buffer[0] = tcxoVoltage;
        buffer[1] = 0x00;
        buffer[2] = 0x00;
        buffer[3] = 0x64;
        writeCommand(RADIO_SET_TCXOMODE, buffer, 4);
#ifdef SX126XDEBUG
        _streamRef->print(F("TCXO Reg "));
        _streamRef->println(tcxoVoltage);
#endif
      } else {
#ifdef SX126XDEBUG
        _streamRef->println(F("NO_TCXO"));
#endif
      }
    }


    void SX126X::calibrateDevice(uint8_t devices) {
#ifdef SX126XDEBUG
      _streamRef->println(F("calibrateDevice()"));
#endif

      writeCommand(RADIO_CALIBRATE, &devices, 1);
      delay(5);  //calibration time for all devices is 3.5mS, SX126x
    }


    void SX126X::calibrateImage(uint32_t freq) {
#ifdef SX126XDEBUG
      _streamRef->println(F("calibrateImage()"));
#endif

      uint8_t calFreq[2];

      if (freq > 900000000) {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
      } else if (freq > 850000000) {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xD8;
      } else if (freq > 770000000) {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
      } else if (freq > 460000000) {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
      } else if (freq > 425000000) {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
      }
      writeCommand(RADIO_CALIBRATEIMAGE, calFreq, 2);
    }


    void SX126X::setDIO2AsRfSwitchCtrl() {
#ifdef SX126XDEBUG
      _streamRef->println(F("setDIO2AsRfSwitchCtrl()"));
#endif

      uint8_t mode = 0x01;
      writeCommand(RADIO_SET_RFSWITCHMODE, &mode, 1);
    }


    void SX126X::setPacketType(uint8_t packettype) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setPacketType()"));
#endif
      savedPacketType = packettype;
      writeCommand(RADIO_SET_PACKETTYPE, &packettype, 1);
    }


    void SX126X::setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t modParam3, uint8_t modParam4) {
      //order for LoRa is spreading factor, bandwidth, code rate, optimisation

#ifdef SX126XDEBUG
      _streamRef->println(F("setModulationParams()"));
#endif
      uint8_t buffer[4];

      savedModParam1 = modParam1;
      savedModParam2 = modParam2;
      savedModParam3 = modParam3;

      if (modParam4 == LDRO_AUTO) {
        modParam4 = returnOptimisation(modParam1, modParam2);  //pass Spreading factor then bandwidth to optimisation calc
      }

      savedModParam4 = modParam4;

      buffer[0] = modParam1;
      buffer[1] = modParam2;
      buffer[2] = modParam3;
      buffer[3] = modParam4;

      writeCommand(RADIO_SET_MODULATIONPARAMS, buffer, 4);
    }


    void SX126X::set_REGTXMODULATION() {
      uint8_t regvalue = readRegister(REG_TX_MODULATION);

      if (savedModParam2 == LORA_BW_500) {
        writeRegister(REG_TX_MODULATION, (regvalue & 0xFB));  //if bandwidth is 500k set bit 2 to 0, see datasheet 15.1.1
      } else {
        writeRegister(REG_TX_MODULATION, (regvalue | 0x04));  //if bandwidth is < 500k set bit 2 to 0 see datasheet 15.1.1
      }
    }


    uint8_t SX126X::returnOptimisation(uint8_t SpreadingFactor, uint8_t Bandwidth) {
      //from the passed bandwidth (bandwidth) and spreading factor this routine
      //calculates whether low data rate optimisation should be on or off

#ifdef SX126XDEBUG
      _streamRef->println(F("returnOptimisation()"));
#endif

      uint32_t tempBandwidth;
      float symbolTime;

      tempBandwidth = returnBandwidth(Bandwidth);

      symbolTime = calcSymbolTime(tempBandwidth, SpreadingFactor);

      if (symbolTime > 16) {
        return LDRO_ON;
      } else {
        return LDRO_OFF;
      }
    }


    uint32_t SX126X::returnBandwidth(uint8_t BWregvalue) {

#ifdef SX126XDEBUG
      _streamRef->println(F("returnBandwidth()"));
#endif

      switch (BWregvalue) {
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
      return 0xFFFF;  //so that a bandwidth not set can be identified
    }


    float SX126X::calcSymbolTime(float Bandwidth, uint8_t SpreadingFactor) {
      //calculates symbol time from passed bandwidth (lbandwidth) and Spreading factor (lSF)and returns in mS

#ifdef SX126XDEBUG
      _streamRef->println(F("calcSymbolTime()"));
#endif

      float symbolTimemS;
      symbolTimemS = (Bandwidth / pow(2, SpreadingFactor));
      symbolTimemS = (1000 / symbolTimemS);
      return symbolTimemS;
    }


    void SX126X::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setBufferBaseAddress()"));
#endif

      uint8_t buffer[2];

      buffer[0] = txBaseAddress;
      buffer[1] = rxBaseAddress;
      writeCommand(RADIO_SET_BUFFERBASEADDRESS, buffer, 2);
    }


    void SX126X::setPacketParams(uint16_t packetParam1, uint8_t packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5) {
      //order is preamble, header type, packet length, CRC, IQ

#ifdef SX126XDEBUG
      _streamRef->println(F("SetPacketParams()"));
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


    void SX126X::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setDioIrqParams()"));
#endif

      savedIrqMask = irqMask;
      savedDio1Mask = dio1Mask;
      savedDio2Mask = dio2Mask;
      savedDio3Mask = dio3Mask;

      uint8_t buffer[8];

      buffer[0] = (uint8_t)(irqMask >> 8);
      buffer[1] = (uint8_t)(irqMask & 0xFF);
      buffer[2] = (uint8_t)(dio1Mask >> 8);
      buffer[3] = (uint8_t)(dio1Mask & 0xFF);
      buffer[4] = (uint8_t)(dio2Mask >> 8);
      buffer[5] = (uint8_t)(dio2Mask & 0xFF);
      buffer[6] = (uint8_t)(dio3Mask >> 8);
      buffer[7] = (uint8_t)(dio3Mask & 0xFF);
      writeCommand(RADIO_CFG_DIOIRQ, buffer, 8);
    }


    void SX126X::setHighSensitivity() {
      //set RX Boosted gain mode
#ifdef SX126XDEBUG
      _streamRef->println(F("setHighSensitivity()"));
#endif
      writeRegister(REG_RX_GAIN, BOOSTED_GAIN);  //max LNA gain, increase current by ~2mA for around ~3dB in sensivity
    }


    void SX126X::setLowPowerRX() {
      //set RX power saving mode
#ifdef SX126XDEBUG
      _streamRef->println(F("setLowPowerRX()"));
#endif

      writeRegister(REG_RX_GAIN, POWER_SAVE_GAIN);  // min LNA gain, reduce current by 2mA for around 3dB loss in sensivity
    }


    void SX126X::setSyncWord(uint16_t syncword) {
#ifdef SX126XDEBUG
      _streamRef->print(F("setSyncWord() 0x"));
	  _streamRef->println(syncword,HEX);
#endif
      writeRegister(REG_LR_SYNCWORD, (syncword >> 8) & 0xFF);
      writeRegister(REG_LR_SYNCWORD + 1, syncword & 0xFF);
    }


    void SX126X::printModemSettings() {
#ifdef SX126XDEBUG
      _streamRef->println(F("printModemSettings()"));
#endif

      printDevice();
      _streamRef->print(F(","));
      _streamRef->print(getFreqInt());
      _streamRef->print(F("hz,SF"));
      _streamRef->print(getLoRaSF());
      _streamRef->print(F(",BW"));
      _streamRef->print(returnBandwidth(savedModParam2));
      _streamRef->print(F(",CR4:"));
      _streamRef->print((getLoRaCodingRate() + 4));
      _streamRef->print(F(",LDRO_"));

      if (getOptimisation()) {
        _streamRef->print(F("On"));
      } else {
        _streamRef->print(F("Off"));
      }

      _streamRef->print(F(",SyncWord_0x"));
      _streamRef->print(getSyncWord(), HEX);
      if (getInvertIQ() == LORA_IQ_INVERTED) {
        _streamRef->print(F(",IQInverted"));
      } else {
        _streamRef->print(F(",IQNormal"));
      }
      _streamRef->print(F(",Preamble_"));
      _streamRef->print(getPreamble());
    }


    uint32_t SX126X::getFreqInt() {
      //get the current set device frequency from registers, return as long integer
#ifdef SX126XDEBUG
      _streamRef->println(F("getFreqInt()"));
#endif

      uint8_t MsbH, MsbL, Mid, Lsb;
      uint32_t uinttemp;
      float floattemp;
      MsbH = readRegister(REG_RFFrequency31_24);
      MsbL = readRegister(REG_RFFrequency23_16);
      Mid = readRegister(REG_RFFrequency15_8);
      Lsb = readRegister(REG_RFFrequency7_0);
      floattemp = ((MsbH * 0x1000000ul) + (MsbL * 0x10000ul) + (Mid * 0x100ul) + Lsb);
      floattemp = ((floattemp * FREQ_STEP) / 1000000ul);
      uinttemp = (uint32_t)(floattemp * 1000000);
      return uinttemp;
    }


    uint8_t SX126X::getLoRaCodingRate() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getLoRaCodingRate"));
#endif

      return savedModParam3;
    }


    uint8_t SX126X::getOptimisation() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getOptimisation"));
#endif

      return savedModParam4;
    }


    uint16_t SX126X::getSyncWord() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getSyncWord"));
#endif

      uint8_t msb, lsb;
      uint16_t syncword;
      msb = readRegister(REG_LR_SYNCWORD);
      lsb = readRegister(REG_LR_SYNCWORD + 1);
      syncword = (msb << 8) + lsb;

      return syncword;
    }


    uint16_t SX126X::getPreamble() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getPreamble"));
#endif

      return savedPacketParam1;
    }


    void SX126X::printOperatingSettings() {
#ifdef SX126XDEBUG
      _streamRef->println(F("printOperatingSettings()"));
#endif

      printDevice();

      _streamRef->print(F(",PacketMode_"));

      if (savedPacketType == PACKET_TYPE_LORA) {
        _streamRef->print(F("LoRa"));
      }

      if (savedPacketType == PACKET_TYPE_GFSK) {
        _streamRef->print(F("GFSK"));
      }

      if (getHeaderMode()) {
        _streamRef->print(F(",Implicit"));
      } else {
        _streamRef->print(F(",Explicit"));
      }

      _streamRef->print(F(",LNAgain_"));

      if (getLNAgain() == BOOSTED_GAIN) {
        _streamRef->print(F("Boosted"));
      } else {
        _streamRef->print(F("Powersave"));
      }
    }


    uint8_t SX126X::getHeaderMode() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getHeaderMode"));
#endif

      return savedPacketParam2;
    }


    uint8_t SX126X::getLNAgain() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getLNAgain"));
#endif

      return readRegister(REG_RX_GAIN);
    }


    void SX126X::setTxParams(int8_t TXpower, uint8_t RampTime) {

      //note this routine does not check if power levels are valid for the module in use
#ifdef SX126XDEBUG
      _streamRef->println(F("setTxParams()"));
#endif

      uint8_t buffer[2];
      savedTXPower = TXpower;

      buffer[0] = TXpower;
      buffer[1] = (uint8_t)RampTime;
      writeCommand(RADIO_SET_TXPARAMS, buffer, 2);
    }


    void SX126X::setTx(uint32_t timeout) {
      //SX126x base timeout in units of 15.625 µs
      //Note: timeout passed to function is in mS

#ifdef SX126XDEBUG
      _streamRef->println(F("setTx()"));
#endif
      uint8_t buffer[3];

      clearIrqStatus(IRQ_RADIO_ALL);

      if (_rxtxpinmode) {
        txEnable();
      }

      timeout = timeout << 6;  //timeout passed in mS, convert to units of 15.625us

      buffer[0] = (timeout >> 16) & 0xFF;
      buffer[1] = (timeout >> 8) & 0xFF;
      buffer[2] = timeout & 0xFF;

      //change required for data sheet addendum 15.1
      uint8_t regvalue = readRegister(REG_TX_MODULATION);
      if (savedModParam2 == LORA_BW_500) {
        writeRegister(REG_TX_MODULATION, (regvalue & 0xFB));  //if bandwidth is 500k set bit 2 to 0, see datasheet 15.1.1
      } else {
        writeRegister(REG_TX_MODULATION, (regvalue | 0x04));  //if bandwidth is < 500k set bit 2 to 0 see datasheet 15.1.1
      }

      writeCommand(RADIO_SET_TX, buffer, 3);
      _OperatingMode = MODE_TX;
    }


    void SX126X::clearIrqStatus(uint16_t irqMask) {
#ifdef SX126XDEBUG
      _streamRef->println(F("clearIrqStatus()"));
#endif

      uint8_t buffer[2];

      buffer[0] = (uint8_t)(irqMask >> 8);
      buffer[1] = (uint8_t)(irqMask & 0xFF);
      writeCommand(RADIO_CLR_IRQSTATUS, buffer, 2);
    }


    uint16_t SX126X::readIrqStatus() {
#ifdef SX126XDEBUG
      _streamRef->print(F("readIrqStatus()"));
#endif

      uint16_t temp;
      uint8_t buffer[2];

      readCommand(RADIO_GET_IRQSTATUS, buffer, 2);
      temp = ((buffer[0] << 8) + buffer[1]);
      return temp;
    }


    uint16_t SX126X::CRCCCITT(uint8_t * buffer, uint32_t size, uint16_t start) {
#ifdef SX126XDEBUG
      _streamRef->println(F("CRCCCITT()"));
#endif

      uint32_t index;
      uint16_t libraryCRC;
      uint8_t j;

      libraryCRC = start;  //start value for CRC16

      for (index = 0; index < size; index++) {
        libraryCRC ^= (((uint16_t)buffer[index]) << 8);
        for (j = 0; j < 8; j++) {
          if (libraryCRC & 0x8000)
            libraryCRC = (libraryCRC << 1) ^ 0x1021;
          else
            libraryCRC <<= 1;
        }
      }

      return libraryCRC;
    }


    uint8_t SX126X::transmit(uint8_t * txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("transmit()"));
#endif
      uint8_t index;
      uint8_t bufferdata;

      if (size == 0) {
        return false;
      }

      setMode(MODE_STDBY_RC);
      setBufferBaseAddress(0, 0);

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      for (index = 0; index < size; index++) {
        bufferdata = txbuffer[index];
        _spi->transfer(bufferdata);
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      _TXPacketL = size;
      writeRegister(REG_LR_PAYLOADLENGTH, _TXPacketL);
      setTxParams(txpower, RADIO_RAMP_200_US);

      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);                                                         //this starts the TX

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }
    }


    uint8_t SX126X::transmitIRQ(uint8_t * txbuffer, uint8_t size, uint16_t timeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("transmitIRQ()"));
#endif
      uint8_t index;
      uint8_t bufferdata;

      if (size == 0) {
        return false;
      }

      setMode(MODE_STDBY_RC);
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      for (index = 0; index < size; index++) {
        bufferdata = txbuffer[index];
        _spi->transfer(bufferdata);
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      _TXPacketL = size;
      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setTx(timeout);  //this starts the TX

      if (!wait) {
        return _TXPacketL;
      }

      //0x0201   = IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT
      while (!(readIrqStatus() & 0x0201))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }
    }


    void SX126X::printIrqStatus() {
#ifdef SX126XDEBUG
      _streamRef->println(F("printIrqStatus()"));
#endif

      uint16_t _IrqStatus;
      _IrqStatus = readIrqStatus();

      //0x0001
      if (_IrqStatus & IRQ_TX_DONE) {
        _streamRef->print(F(",IRQ_TX_DONE"));
      }

      //0x0002
      if (_IrqStatus & IRQ_RX_DONE) {
        _streamRef->print(F(",IRQ_RX_DONE"));
      }

      //0x0004
      if (_IrqStatus & IRQ_PREAMBLE_DETECTED) {
        _streamRef->print(F(",IRQ_PREAMBLE_DETECTED"));
      }

      //0x0008
      if (_IrqStatus & IRQ_SYNCWORD_VALID) {
        _streamRef->print(F(",IRQ_SYNCWORD_VALID"));
      }

      //0x0010
      if (_IrqStatus & IRQ_HEADER_VALID) {
        _streamRef->print(F(",IRQ_HEADER_VALID"));
      }

      //0x0020
      if (_IrqStatus & IRQ_HEADER_ERROR) {
        _streamRef->print(F(",IRQ_HEADER_ERROR"));
      }

      //0x0040
      if (_IrqStatus & IRQ_CRC_ERROR) {
        _streamRef->print(F(",IRQ_CRC_ERROR"));
      }

      //0x0080
      if (_IrqStatus & IRQ_CAD_DONE) {
        _streamRef->print(F(",IRQ_CAD_DONE"));
      }

      //0x0100
      if (_IrqStatus & IRQ_CAD_ACTIVITY_DETECTED) {
        _streamRef->print(",IRQ_CAD_ACTIVITY_DETECTED");
      }

      //0x0200
      if (_IrqStatus & IRQ_RX_TX_TIMEOUT) {
        _streamRef->print(F(",IRQ_RX_TX_TIMEOUT"));
      }
    }


    void SX126X::printRegisters(uint16_t Start, uint16_t End) {
      //prints the contents of SX1262 registers to serial monitor

#ifdef SX126XDEBUG
      _streamRef->println(F("printRegisters()"));
#endif

      uint16_t Loopv1, Loopv2, RegData;

      _streamRef->print(F("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
      _streamRef->println();

      for (Loopv1 = Start; Loopv1 <= End;)  //32 lines
      {
        _streamRef->print(F("0x"));
        _streamRef->print((Loopv1), HEX);  //print the register number
        _streamRef->print(F("  "));
        for (Loopv2 = 0; Loopv2 <= 15; Loopv2++) {
          RegData = readRegister(Loopv1);
          if (RegData < 0x10) {
            _streamRef->print(F("0"));
          }
          _streamRef->print(RegData, HEX);  //print the register number
          _streamRef->print(F(" "));
          Loopv1++;
        }
        _streamRef->println();
      }
    }


    void SX126X::printDevice() {
#ifdef SX126XDEBUG
      _streamRef->println(F("printDevice()"));
#endif

      switch (_Device) {
        case DEVICE_SX1261:
          _streamRef->print(F("SX1261"));
          break;

        case DEVICE_SX1262:
          _streamRef->print(F("SX1262"));
          break;

        case DEVICE_SX1268:
          _streamRef->print(F("SX1268"));
          break;

        default:
          _streamRef->print(F("Unknown Device"));
      }
    }


    bool SX126X::config() {
#ifdef SX126XDEBUG
      _streamRef->println(F("config()"));
#endif

      resetDevice();
      setMode(MODE_STDBY_RC);
      setRegulatorMode(savedRegulatorMode);
      setPacketType(savedPacketType);
      setRfFrequency(savedFrequency, savedOffset);
      setModulationParams(savedModParam1, savedModParam2, savedModParam3, LDRO_ON);
      setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5);
      setDioIrqParams(savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask);  //set for IRQ on RX done on DIO1
      _TXPacketL = 0;
      _RXPacketL = 0;
      return true;
    }


    void SX126X::setRfFrequency(uint32_t frequency, int32_t offset) {
      //Note RF_Freq = freq_reg*32M/(2^25)-----> freq_reg = (RF_Freq * (2^25))/32

#ifdef SX126XDEBUG
      _streamRef->print(F("setRfFrequency()  "));
      _streamRef->println(frequency + offset);
#endif

      uint8_t buffer[4];
      uint32_t localfrequencyRegs;

      savedFrequency = frequency;
      savedOffset = offset;

      localfrequencyRegs = frequency + offset;
      localfrequencyRegs = (uint32_t)((double)localfrequencyRegs / (double)FREQ_STEP);
      savedFrequencyReg = localfrequencyRegs;

      buffer[0] = (localfrequencyRegs >> 24) & 0xFF;  //MSB
      buffer[1] = (localfrequencyRegs >> 16) & 0xFF;
      buffer[2] = (localfrequencyRegs >> 8) & 0xFF;
      buffer[3] = localfrequencyRegs & 0xFF;  //LSB

      _freqregH = buffer[0];
      _freqregMH = buffer[1];
      _freqregML = buffer[2];
      _freqregL = buffer[3];

      writeCommand(RADIO_SET_RFFREQUENCY, buffer, 4);
    }


    uint8_t SX126X::getLoRaSF() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getLoRaSF()"));
#endif

      return savedModParam1;
    }


    uint8_t SX126X::getInvertIQ() {
      //IQ mode reg 0x33
#ifdef SX126XDEBUG
      _streamRef->println(F("getInvertIQ"));
#endif

      return readRegister(REG_IQ_POLARITY_SETUP);
    }


    void SX126X::rxEnable() {
#ifdef SX126XDEBUG
      _streamRef->println(F("rxEnable()"));
#endif

      digitalWrite(_RXEN, HIGH);
      digitalWrite(_TXEN, LOW);
    }


    void SX126X::txEnable() {
#ifdef SX126XDEBUGPINS
      _streamRef->println(F("txEnable()"));
#endif

      digitalWrite(_RXEN, LOW);
      digitalWrite(_TXEN, HIGH);
    }


    void SX126X::printASCIIPacket(uint8_t * buffer, uint8_t size) {
#ifdef SX126XDEBUG
      _streamRef->println(F("printASCIIPacket()"));
#endif

      uint8_t index;

      for (index = 0; index < size; index++) {
        _streamRef->write(buffer[index]);
      }
    }


    uint8_t SX126X::receive(uint8_t * rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("receive()"));
#endif

      uint8_t index, RXstart, RXend;
      uint16_t regdata;
      uint8_t buffer[2];

      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      regdata = readIrqStatus();

      if ((regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT))  //check if any of the preceding IRQs is set
      {
        //packet is errored somewhere so return 0
        return 0;
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL > size)  //check passed buffer is big enough for packet
      {
        _RXPacketL = size;  //truncate packet if not enough space
      }

      RXstart = buffer[1];

      RXend = RXstart + _RXPacketL;

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(RXstart);
      _spi->transfer(0xFF);

      for (index = RXstart; index < RXend; index++) {
        regdata = _spi->transfer(0);
        rxbuffer[index] = regdata;
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return _RXPacketL;  //so we can check for packet having enough buffer space
    }


uint8_t SX126X::receiveRxDutyCycle(uint8_t *rxbuffer, uint8_t size, uint8_t wait, uint32_t rxus, uint32_t sleepus) {
#ifdef SX126XDEBUG
      _streamRef->println(F("receive()"));
#endif

      uint8_t index, RXstart, RXend;
      uint16_t regdata;
      uint8_t buffer[2];

      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE), 0, 0);  //set for IRQ on RX done
      setRx(rxus, sleepus);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      regdata = readIrqStatus();

      if ((regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT))  //check if any of the preceding IRQs is set
      {
        //packet is errored somewhere so return 0
        return 0;
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL > size)  //check passed buffer is big enough for packet
      {
        _RXPacketL = size;  //truncate packet if not enough space
      }

      RXstart = buffer[1];

      RXend = RXstart + _RXPacketL;

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(RXstart);
      _spi->transfer(0xFF);

      for (index = RXstart; index < RXend; index++) {
        regdata = _spi->transfer(0);
        rxbuffer[index] = regdata;
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return _RXPacketL;  //so we can check for packet having enough buffer space
    }



    uint8_t SX126X::receiveIRQ(uint8_t * rxbuffer, uint8_t size, uint16_t timeout, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("receiveIRQ()"));
#endif

      uint8_t index, RXstart, RXend;
      uint16_t regdata;
      uint8_t buffer[2];

      setRx(timeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      //0x0202   = IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
      while (!(readIrqStatus() & 0x0202))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
        return 0;  //packet is errored somewhere so return 0
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL > size)  //check passed buffer is big enough for packet
      {
        _RXPacketL = size;  //truncate packet if not enough space
      }

      RXstart = buffer[1];
      RXend = RXstart + _RXPacketL;

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(RXstart);
      _spi->transfer(0xFF);

      for (index = RXstart; index < RXend; index++) {
        regdata = _spi->transfer(0);
        rxbuffer[index] = regdata;
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return _RXPacketL;
    }


    int16_t SX126X::readPacketRSSI() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readPacketRSSI()"));
#endif

      uint8_t status[5];
      int16_t rssi;

      readCommand(RADIO_GET_PACKETSTATUS, status, 5);
      rssi = -status[0] / 2;

      return rssi;
    }


    int8_t SX126X::readPacketSNR() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readPacketSNR()"));
#endif

      uint8_t status[5];
      int8_t snr;

      readCommand(RADIO_GET_PACKETSTATUS, status, 5);

      if (status[1] < 128) {
        snr = status[1] / 4;
      } else {
        snr = ((status[1] - 256) / 4);
      }

      return snr;
    }


    uint8_t SX126X::readRXPacketL() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readRXPacketL()"));
#endif

      uint8_t buffer[2];

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];
      return _RXPacketL;
    }


    void SX126X::setRx(uint32_t timeout) {
      //SX126x base timeout in units of 15.625 µs
      //timeout passed to function in mS
      //range is 1mS to 262 seconds

#ifdef SX126XDEBUG
      _streamRef->println(F("setRx()"));
#endif
      uint8_t buffer[3];
      clearIrqStatus(IRQ_RADIO_ALL);

      if (_rxtxpinmode) {
        rxEnable();
      }

      timeout = timeout << 6;  //timeout passed in mS, multiply by 64 to convert units of 15.625us to 1mS

      buffer[0] = (timeout >> 16) & 0xFF;
      buffer[1] = (timeout >> 8) & 0xFF;
      buffer[2] = timeout & 0xFF;
      writeCommand(RADIO_SET_RX, buffer, 3);
    }


    void SX126X::setRx(uint32_t rxlistenus, uint32_t rxsleepus) {
      //to set RXdutycycle mode
      //rxlistenus and rxsleepus values required passed in uS
      //listen and sleep timings written to SX126X in units of 15.625 µs

      uint8_t buffer[6];

      clearIrqStatus(IRQ_RADIO_ALL);

#ifdef DEBUGRXDUTYCYCLE
      _streamRef->println(F("setRx()"));
      _streamRef->print(F("rxlistenus "));
      _streamRef->print(rxlistenus);
      _streamRef->print(F("   rxsleepus "));
      _streamRef->println(rxsleepus);
#endif

      if (_rxtxpinmode) {
        rxEnable();
      }
	  
	  buffer[0] = 0;                                                     //ensure reive stays active one preamble detected
	  
	  #ifdef DEBUGRXDUTYCYCLE
      _streamRef->print("RADIO_SET_STOPRXTIMERONPREAMBLE ");
      _streamRef->println(buffer[0]);
      #endif
    
      writeCommand(RADIO_SET_STOPRXTIMERONPREAMBLE, buffer, 1 );         //does not seem to make a differance
	  delay(200);

      uint32_t rxlisten = (rxlistenus * 8) / 125;  // divide passed times by 15.625us
      uint32_t rxsleep = (rxsleepus * 8) / 125;

      buffer[0] = (uint8_t)((rxlisten >> 16) & 0xFF);
      buffer[1] = (uint8_t)((rxlisten >> 8) & 0xFF);
      buffer[2] = (uint8_t)(rxlisten & 0xFF);
      buffer[3] = (uint8_t)((rxsleep >> 16) & 0xFF);
      buffer[4] = (uint8_t)((rxsleep >> 8) & 0xFF);
      buffer[5] = (uint8_t)(rxsleep & 0xFF);

#ifdef DEBUGRXDUTYCYCLE
      _streamRef->println();
      _streamRef->println(F("RADIO_SET_RXDUTYCYCLE buffer  "));
      for (uint8_t index = 0; index <= 5; index++) {
        _streamRef->print(buffer[index], HEX);
        _streamRef->print(F("  "));
      }
      _streamRef->println();
#endif

      writeCommand(RADIO_SET_RXDUTYCYCLE, buffer, 6);
    }

    /***************************************************************************
  //Start direct access SX buffer routines
***************************************************************************/

    void SX126X::startWriteSXBuffer(uint8_t ptr) {
#ifdef SX126XDEBUG
      _streamRef->println(F("startWriteSXBuffer()"));
#endif

      _TXPacketL = 0;  //this variable used to keep track of bytes written
      setMode(MODE_STDBY_RC);
      setBufferBaseAddress(ptr, 0);

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(ptr);
      //SPI interface ready for byte to write to buffer
    }


    uint8_t SX126X::endWriteSXBuffer() {
#ifdef SX126XDEBUG
      _streamRef->println(F("endWriteSXBuffer()"));
#endif

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return _TXPacketL;
    }


    void SX126X::startReadSXBuffer(uint8_t ptr) {
#ifdef SX126XDEBUG
      _streamRef->println(F("startReadSXBuffer"));
#endif

      _RXPacketL = 0;
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(ptr);
      _spi->transfer(0xFF);

      //next line would be data = _spi->transfer(0);
      //SPI interface ready for byte to read from
    }


    uint8_t SX126X::endReadSXBuffer() {
#ifdef SX126XDEBUG
      _streamRef->println(F("endReadSXBuffer()"));
#endif

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return _RXPacketL;
    }


    void SX126X::writeUint8(uint8_t x) {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeUint8()"));
#endif

      _spi->transfer(x);
      _TXPacketL++;  //increment count of bytes written
    }

    uint8_t SX126X::readUint8() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readUint8()"));
#endif
      byte x;
      x = _spi->transfer(0);
      _RXPacketL++;  //increment count of bytes read
      return (x);
    }


    void SX126X::writeInt8(int8_t x) {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeInt8()"));
#endif

      _spi->transfer(x);
      _TXPacketL++;  //increment count of bytes written
    }


    int8_t SX126X::readInt8() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readInt8()"));
#endif
      int8_t x;
      x = _spi->transfer(0);
      _RXPacketL++;  //increment count of bytes read
      return (x);
    }


    void SX126X::writeInt16(int16_t x) {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeInt16()"));
#endif

      _spi->transfer(lowByte(x));
      _spi->transfer(highByte(x));
      _TXPacketL = _TXPacketL + 2;  //increment count of bytes written
    }


    int16_t SX126X::readInt16() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readInt16()"));
#endif
      byte lowbyte, highbyte;
      lowbyte = _spi->transfer(0);
      highbyte = _spi->transfer(0);
      _RXPacketL = _RXPacketL + 2;  //increment count of bytes read
      return ((highbyte << 8) + lowbyte);
    }


    void SX126X::writeUint16(uint16_t x) {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeUint16()"));
#endif

      _spi->transfer(lowByte(x));
      _spi->transfer(highByte(x));
      _TXPacketL = _TXPacketL + 2;  //increment count of bytes written
    }


    uint16_t SX126X::readUint16() {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeUint16()"));
#endif
      byte lowbyte, highbyte;

      lowbyte = _spi->transfer(0);
      highbyte = _spi->transfer(0);
      _RXPacketL = _RXPacketL + 2;  //increment count of bytes read
      return ((highbyte << 8) + lowbyte);
    }


    void SX126X::writeInt32(int32_t x) {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeInt32()"));
#endif

      byte i, j;

      union {
        byte b[4];
        int32_t f;
      } data;
      data.f = x;

      for (i = 0; i < 4; i++) {
        j = data.b[i];
        _spi->transfer(j);
      }

      _TXPacketL = _TXPacketL + 4;  //increment count of bytes written
    }


    int32_t SX126X::readInt32() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readInt32()"));
#endif

      byte i, j;

      union {
        byte b[4];
        int32_t f;
      } readdata;

      for (i = 0; i < 4; i++) {
        j = _spi->transfer(0);
        readdata.b[i] = j;
      }
      _RXPacketL = _RXPacketL + 4;  //increment count of bytes read
      return readdata.f;
    }


    void SX126X::writeUint32(uint32_t x) {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeUint32()"));
#endif

      byte i, j;

      union {
        byte b[4];
        uint32_t f;
      } data;
      data.f = x;

      for (i = 0; i < 4; i++) {
        j = data.b[i];
        _spi->transfer(j);
      }

      _TXPacketL = _TXPacketL + 4;  //increment count of bytes written
    }


    uint32_t SX126X::readUint32() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readUint32()"));
#endif

      byte i, j;

      union {
        byte b[4];
        uint32_t f;
      } readdata;

      for (i = 0; i < 4; i++) {
        j = _spi->transfer(0);
        readdata.b[i] = j;
      }
      _RXPacketL = _RXPacketL + 4;  //increment count of bytes read
      return readdata.f;
    }


    void SX126X::writeFloat(float x) {
#ifdef SX126XDEBUG
      _streamRef->println(F("writeFloat()"));
#endif

      byte i, j;

      union {
        byte b[4];
        float f;
      } data;
      data.f = x;

      for (i = 0; i < 4; i++) {
        j = data.b[i];
        _spi->transfer(j);
      }

      _TXPacketL = _TXPacketL + 4;  //increment count of bytes written
    }


    float SX126X::readFloat() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readFloat()"));
#endif

      byte i, j;

      union {
        byte b[4];
        float f;
      } readdata;

      for (i = 0; i < 4; i++) {
        j = _spi->transfer(0);
        readdata.b[i] = j;
      }
      _RXPacketL = _RXPacketL + 4;  //increment count of bytes read
      return readdata.f;
    }


    uint8_t SX126X::transmitSXBuffer(uint8_t startaddr, uint8_t length, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("transmitSXBuffer()"));
#endif

      setBufferBaseAddress(startaddr, 0);
      setPacketParams(savedPacketParam1, savedPacketParam2, length, savedPacketParam4, savedPacketParam5);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);                                                         //this starts the TX

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }
    }


    void SX126X::writeBuffer(uint8_t * txbuffer, uint8_t size) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("writeBuffer()"));
#endif

      uint8_t index, regdata;

      _TXPacketL = _TXPacketL + size;  //these are the number of bytes that will be added
      size--;                          //loose one byte from size, the last byte written MUST be a 0

      for (index = 0; index < size; index++) {
        regdata = txbuffer[index];
        _spi->transfer(regdata);
      }

      _spi->transfer(0);  //this ensures last byte of buffer written really is a null (0)
    }


    void SX126X::writeBufferChar(char *txbuffer, uint8_t size) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("writeBuffer()"));
#endif

      uint8_t index, regdata;

      _TXPacketL = _TXPacketL + size;  //these are the number of bytes that will be added
      size--;                          //loose one byte from size, the last byte written MUST be a 0

      for (index = 0; index < size; index++) {
        regdata = txbuffer[index];
        _spi->transfer(regdata);
      }

      _spi->transfer(0);  //this ensures last byte of buffer written really is a null (0)
    }


    uint8_t SX126X::receiveSXBuffer(uint8_t startaddr, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("receiveSXBuffer()"));
#endif

      uint16_t regdata;
      uint8_t buffer[2];

      setMode(MODE_STDBY_RC);
      setBufferBaseAddress(0, startaddr);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxtimeout);

      if (!wait) {
        return 0;
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      regdata = readIrqStatus();

      if ((regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT)) {
        return 0;  //no RX done and header valid only, could be CRC error
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      return _RXPacketL;
    }
	
	    uint8_t SX126X::receiveSXBufferRxDutyCycle(uint8_t startaddr, uint8_t wait, uint32_t rxus, uint32_t sleepus) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("receiveSXBufferRxDutyCycle()"));
#endif

      uint16_t regdata;
      uint8_t buffer[2];

      setMode(MODE_STDBY_RC);
      setBufferBaseAddress(0, startaddr);
      setDioIrqParams(0x0272, (IRQ_RX_DONE), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxus, sleepus);;

      if (!wait) {
        return 0;
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      regdata = readIrqStatus();

      if ((regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR)) {
        return 0;  //no RX done and header valid only, could be CRC error
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      return _RXPacketL;
    }
	

    uint8_t SX126X::readBuffer(uint8_t * rxbuffer) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("readBuffer()"));
#endif

      uint8_t index = 0, regdata;

      do  //need to find the size of the buffer first
      {
        regdata = _spi->transfer(0);
        rxbuffer[index] = regdata;  //fill the buffer.
        index++;
      } while (regdata != 0);  //keep reading until we have reached the null (0) at the buffer end or exceeded size of buffer allowed

      _RXPacketL = _RXPacketL + index;  //increment count of bytes read
      return index;                     //return the actual size of the buffer, till the null (0) detected
    }


    uint8_t SX126X::readBufferChar(char *rxbuffer) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("readBuffer()"));
#endif

      uint8_t index = 0, regdata;

      do  //need to find the size of the buffer first
      {
        regdata = _spi->transfer(0);
        rxbuffer[index] = regdata;  //fill the buffer.
        index++;
      } while (regdata != 0);  //keep reading until we have reached the null (0) at the buffer end or exceeded size of buffer allowed

      _RXPacketL = _RXPacketL + index;  //increment count of bytes read
      return index;                     //return the actual size of the buffer, till the null (0) detected
    }


      uint16_t SX126X::CRCCCITTSX(uint8_t startadd, uint8_t endadd, uint16_t startvalue) {
      //genrates a CRC of an area of the internal SX buffer

#ifdef SX126XDEBUG1
      _streamRef->println(F("CRCCCITTSX()"));
#endif

      uint16_t index, libraryCRC;
      uint8_t j;

      libraryCRC = startvalue;      //start value for CRC16
      startReadSXBuffer(startadd);  //begin the buffer read

      for (index = startadd; index <= endadd; index++) {
        libraryCRC ^= (((uint16_t)readUint8()) << 8);
        for (j = 0; j < 8; j++) {
          if (libraryCRC & 0x8000)
            libraryCRC = (libraryCRC << 1) ^ 0x1021;
          else
            libraryCRC <<= 1;
        }
      }

      endReadSXBuffer();  //end the buffer read
      return libraryCRC;
    } 
	
	
	   void SX126X::writeUint16SXBuffer(uint8_t addr, uint16_t regdata) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} writeUint16SXBuffer() 0x"));
      _streamRef->print(addr, HEX);
      _streamRef->print(F(" 0x"));
      _streamRef->println(regdata, HEX);
#endif

      setMode(MODE_STDBY_RC);  //this is needed to ensure we can write to buffer OK.
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(addr);
      _spi->transfer(lowByte(regdata));
      _spi->transfer(highByte(regdata));
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif
    }

    /***************************************************************************
  //End direct access SX buffer routines
***************************************************************************/

    void SX126X::setSleep(uint8_t sleepconfig) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setSleep()"));
#endif
      setMode(MODE_STDBY_RC);
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_SET_SLEEP);
      _spi->transfer(sleepconfig);
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      if (_SW >= 0) {
        digitalWrite(_SW, LOW);  //turn off antenna switch if SW pin in use, saves 9uA.
      }

      delay(1);  //allow time for shutdown
    }


    void SX126X::wake() {
#ifdef SX126XDEBUG
      _streamRef->println(F("wake()"));
#endif

      if (_SW >= 0) {
        digitalWrite(_SW, HIGH);  //turn on antenna switch if SW pin in use
      }

      digitalWrite(_NSS, LOW);
      delay(1);
      digitalWrite(_NSS, HIGH);
      delay(1);
    }


    void SX126X::setupDirect(uint32_t frequency, int32_t offset) {
      //setup LoRa device for direct modulation mode
#ifdef SX126XDEBUG1
      _streamRef->print(F("setupDirect()"));
#endif
      setMode(MODE_STDBY_RC);
      setRegulatorMode(USE_DCDC);
      setPaConfig(0x04, PAAUTO, _Device);  //use _Device, saved by begin.
      setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);
      calibrateDevice(ALLDevices);  //is required after setting TCXO
      calibrateImage(frequency);
      setDIO2AsRfSwitchCtrl();
      setRfFrequency(frequency, offset);
    }


    void SX126X::setTXDirect() {
      //turns on transmitter,in direct mode for FSK and audio  power level is from 2 to 17
#ifdef SX126XDEBUG1
      _streamRef->print(F("setTxFSK()"));
#endif
      writeCommand(RADIO_SET_TXCONTINUOUSWAVE, 0, 0);
    }


    void SX126X::toneFM(uint16_t frequency, uint32_t length, uint32_t deviation, float adjust, uint8_t txpower) {
#ifdef SX126XDEBUG1
      _streamRef->print(F("toneFM()"));
#endif
      uint16_t index;
      uint32_t ToneDelayus;
      uint32_t registershift;
      uint32_t shiftedfreqregH, shiftedfreqregL;
      uint32_t loopcount;

      registershift = deviation / FREQ_STEP;
      shiftedfreqregH = savedFrequencyReg + registershift;
      shiftedfreqregL = savedFrequencyReg - registershift;

      uint8_t HighShiftH = shiftedfreqregH >> 24;
      uint8_t HighShiftMH = shiftedfreqregH >> 16;
      uint8_t HighShiftML = shiftedfreqregH >> 8;
      uint8_t HighShiftL = shiftedfreqregH;
      uint8_t LowShiftH = shiftedfreqregL >> 24;
      uint8_t LowShiftMH = shiftedfreqregL >> 16;
      uint8_t LowShiftML = shiftedfreqregL >> 8;
      uint8_t LowShiftL = shiftedfreqregL;
      uint8_t freqregH = savedFrequencyReg >> 24;
      uint8_t freqregMH = savedFrequencyReg >> 16;
      uint8_t freqregML = savedFrequencyReg >> 8;
      uint8_t freqregL = savedFrequencyReg;

      ToneDelayus = ((500000 / frequency));
      loopcount = (length * 500) / (ToneDelayus);
      ToneDelayus = ToneDelayus * adjust;

#ifdef SX126XDEBUG3
      _streamRef->print(F("frequency "));
      _streamRef->println(frequency);
      _streamRef->print(F("length "));
      _streamRef->println(length);

      _streamRef->print(F("savedFrequencyReg "));
      _streamRef->println(savedFrequencyReg, HEX);
      _streamRef->print(F("registershift "));
      _streamRef->println(registershift);
      shiftedfreqregH = savedFrequencyReg + (registershift / 2);
      shiftedfreqregL = savedFrequencyReg - (registershift / 2);
      _streamRef->print(F("shiftedfreqregH "));
      _streamRef->println(shiftedfreqregH, HEX);
      _streamRef->print(F("shiftedfreqregL "));
      _streamRef->println(shiftedfreqregL, HEX);

      _streamRef->print(F("ShiftedHigh,"));
      _streamRef->print(HighShiftH, HEX);
      _streamRef->print(F(","));
      _streamRef->print(HighShiftMH, HEX);
      _streamRef->print(F(","));
      _streamRef->print(HighShiftML, HEX);
      _streamRef->print(F(","));
      _streamRef->println(HighShiftL, HEX);

      _streamRef->print(F("ShiftedLow,"));
      _streamRef->print(LowShiftH, HEX);
      _streamRef->print(F(","));
      _streamRef->print(LowShiftMH, HEX);
      _streamRef->print(F(","));
      _streamRef->print(LowShiftML, HEX);
      _streamRef->print(F(","));
      _streamRef->println(LowShiftL, HEX);
      _streamRef->print(F("ToneDelayus,"));
      _streamRef->println(ToneDelayus);
      _streamRef->print(F("loopcount,"));
      _streamRef->println(loopcount);
      _streamRef->println();
      _streamRef->println();
#endif

      setTxParams(txpower, RADIO_RAMP_200_US);
      setTXDirect();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      for (index = 1; index <= loopcount; index++) {
        digitalWrite(_NSS, LOW);
        _spi->transfer(RADIO_SET_RFFREQUENCY);
        _spi->transfer(HighShiftH);
        _spi->transfer(HighShiftMH);
        _spi->transfer(HighShiftML);
        _spi->transfer(HighShiftL);
        digitalWrite(_NSS, HIGH);

        delayMicroseconds(ToneDelayus);

        digitalWrite(_NSS, LOW);
        _spi->transfer(RADIO_SET_RFFREQUENCY);
        _spi->transfer(LowShiftH);
        _spi->transfer(LowShiftMH);
        _spi->transfer(LowShiftML);
        _spi->transfer(LowShiftL);
        digitalWrite(_NSS, HIGH);

        delayMicroseconds(ToneDelayus);
      }

      //now set the frequency registers back to centre
      digitalWrite(_NSS, LOW);  //set NSS low
      _spi->transfer(0x86);     //address for write to REG_FRMSB
      _spi->transfer(freqregH);
      _spi->transfer(freqregMH);
      _spi->transfer(freqregML);
      _spi->transfer(freqregL);
      digitalWrite(_NSS, HIGH);  //set NSS high

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setMode(MODE_STDBY_RC);  //turns off carrier
    }


    uint8_t SX126X::getByteSXBuffer(uint8_t addr) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("getByteSXBuffer()"));
#endif

      uint8_t regdata;
      setMode(MODE_STDBY_RC);  //this is needed to ensure we can read from buffer OK.

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(addr);
      _spi->transfer(0xFF);
      regdata = _spi->transfer(0);
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return regdata;
    }


    void SX126X::printSXBufferHEX(uint8_t start, uint8_t end) {
#ifdef SX126XDEBUG
      _streamRef->println(F("printSXBufferHEX()"));
#endif

      uint8_t index, regdata;

      setMode(MODE_STDBY_RC);

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(start);
      _spi->transfer(0xFF);

      for (index = start; index <= end; index++) {
        regdata = _spi->transfer(0);
        printHEXByte(regdata);
        _streamRef->print(F(" "));
      }
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif
    }


    int32_t SX126X::getFrequencyErrorHz() {
      //Note: Semtech appear to have stated that the frequency error function that this code uses,
      //is not supported for SX126X, for reasons that have not been given, so use at your own risk.
      //The fuctions here are a replication of the routines for the very similar SX128X

#ifdef SX126XDEBUG
      _streamRef->println(F("getFrequencyErrorHz()"));
#endif

      int32_t error, regvalue;
      uint32_t bandwidth;
      float divider;

      bandwidth = returnBandwidth(savedModParam2);  //gets the last configured bandwidth in hz
      divider = (float)1625000 / bandwidth;         //why the values from the SX1280 datasheet work I have no idea

      regvalue = getFrequencyErrorRegValue();

      error = (FREQ_ERROR_CORRECTION * regvalue) / divider;

      return error;
    }


    int32_t SX126X::getFrequencyErrorRegValue() {
#ifdef SX126XDEBUG
      _streamRef->println(F("getFrequencyErrorRegValue()"));
#endif

      int32_t FrequencyError;
      uint32_t regmsb, regmid, reglsb, allreg;

      setMode(MODE_STDBY_XOSC);

      regmsb = readRegister(REG_FREQUENCY_ERRORBASEADDR);
      regmsb = regmsb & 0x0F;  //clear bit 20 which is always set
      regmid = readRegister(REG_FREQUENCY_ERRORBASEADDR + 1);
      reglsb = readRegister(REG_FREQUENCY_ERRORBASEADDR + 2);

      setMode(MODE_STDBY_RC);

#ifdef SX126XDEBUG
      _streamRef->println();
      _streamRef->print(F("Registers "));
      _streamRef->print(regmsb, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(regmid, HEX);
      _streamRef->print(F(" "));
      _streamRef->println(reglsb, HEX);
#endif

      allreg = (uint32_t)(regmsb << 16) | (regmid << 8) | reglsb;

      if (allreg & 0x80000) {
        FrequencyError = (0xFFFFF - allreg) * -1;
      } else {
        FrequencyError = allreg;
      }

      return FrequencyError;
    }


    void SX126X::printHEXByte(uint8_t temp) {
      if (temp < 0x10) {
        _streamRef->print(F("0"));
      }
      _streamRef->print(temp, HEX);
    }


    uint8_t SX126X::transmitAddressed(uint8_t * txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("transmitAddressed()"));
#endif

      uint8_t index;
      uint8_t bufferdata;

      if (size == 0) {
        return false;
      }

      setMode(MODE_STDBY_RC);

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      _spi->transfer(txpackettype);   //Write the packet type
      _spi->transfer(txdestination);  //Destination node
      _spi->transfer(txsource);       //Source node
      _TXPacketL = 3 + size;          //we have added 3 header bytes to size

      for (index = 0; index < size; index++) {
        bufferdata = txbuffer[index];
        _spi->transfer(bufferdata);
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      writeRegister(REG_LR_PAYLOADLENGTH, _TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setTx(txtimeout);  //this starts the TX

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }
    }


    uint8_t SX126X::readRXPacketType() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readRXPacketType()"));
#endif
      return _RXPacketType;
    }


    uint8_t SX126X::readRXDestination() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readRXDestination()"));
#endif
      return _RXDestination;
    }


    uint8_t SX126X::readRXSource() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readRXSource()"));
#endif
      return _RXSource;
    }


    uint8_t SX126X::receiveAddressed(uint8_t * rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("receiveAddressed()"));
#endif

      uint8_t index, RXstart, RXend;
      uint16_t regdata;
      uint8_t buffer[2];

      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      regdata = readIrqStatus();

      if ((regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT))  //check if any of the preceding IRQs is set
      {
        //packet is errored somewhere so return 0
        return 0;
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL > size)  //check passed buffer is big enough for packet
      {
        _RXPacketL = size;  //truncate packet if not enough space
      }

      RXstart = buffer[1];

      RXend = RXstart + _RXPacketL;

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(RXstart);
      _spi->transfer(0xFF);

      _RXPacketType = _spi->transfer(0);
      _RXDestination = _spi->transfer(0);
      _RXSource = _spi->transfer(0);

      for (index = RXstart; index < RXend; index++) {
        regdata = _spi->transfer(0);
        rxbuffer[index] = regdata;
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return _RXPacketL;  //so we can check for packet having enough buffer space
    }


    void SX126X::clearDeviceErrors() {
#ifdef SX126XDEBUG
      _streamRef->println(F("clearDeviceErrors()"));
#endif

      uint8_t buffer[2];

      buffer[0] = 0x00;  //can only clear all errors
      buffer[1] = 0x00;
      writeCommand(RADIO_CLEAR_ERRORS, buffer, 2);
    }


    void SX126X::printDeviceErrors() {
#ifdef SX126XDEBUG
      _streamRef->println(F("printDeviceErrors()"));
#endif

      uint16_t errors;
      uint8_t buffer[2];

      readCommand(RADIO_GET_ERROR, buffer, 2);
      errors = (buffer[0] << 8) + buffer[1];

      //0x0001
      if (errors & RC64K_CALIB_ERR) {
        _streamRef->print(F(",RC64K_CALIB_ERR"));
      }

      //0x0002
      if (errors & RC13M_CALIB_ERR) {
        _streamRef->print(F(",RC13M_CALIB_ERR"));
      }

      //0x0004
      if (errors & PLL_CALIB_ERR) {
        _streamRef->print(F(",PLL_CALIB_ERR"));
      }

      //0x0008
      if (errors & ADC_CALIB_ERR) {
        _streamRef->print(F(",ADC_CALIB_ERR"));
      }

      //0x0010
      if (errors & IMG_CALIB_ERR) {
        _streamRef->print(F(",IMG_CALIB_ERR"));
      }

      //0x0020
      if (errors & XOSC_START_ERR) {
        _streamRef->print(F(",XOSC_START_ERR"));
      }

      //0x0040
      if (errors & PLL_LOCK_ERR) {
        _streamRef->print(F(",PLL_LOCK_ERR"));
      }

      //0x0080
      if (errors & RFU) {
        _streamRef->print(F(",RFU"));
      }

      //0x0100
      if (errors & PA_RAMP_ERR) {
        _streamRef->print(",PA_RAMP_ERR");
      }
    }


    void SX126X::printHEXPacket(uint8_t * buffer, uint8_t size) {
#ifdef SX126XDEBUG
      _streamRef->println(F("printHEXPacket()"));
#endif

      uint8_t index;

      for (index = 0; index < size; index++) {
        printHEXByte(buffer[index]);
        _streamRef->print(F(" "));
      }
    }


    void SX126X::printHEXByte0x(uint8_t temp) {
      //print a byte, adding 0x
      _streamRef->print(F("0x"));
      if (temp < 0x10) {
        _streamRef->print(F("0"));
      }
      _streamRef->print(temp, HEX);
    }


    uint8_t SX126X::readsavedModParam1() {
      //return previously set spreading factor
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedModParam1()"));
#endif
      return savedModParam1;
    }


    uint8_t SX126X::readsavedModParam2() {
      //return previously set bandwidth
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedModParam2()"));
#endif
      return savedModParam2;
    }


    uint8_t SX126X::readsavedModParam3() {
      //return previously set code rate
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedModParam3()"));
#endif
      return savedModParam3;
    }


    uint8_t SX126X::readsavedModParam4() {
      //return previously set optimisation
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedModParam4()"));
#endif
      return savedModParam4;
    }


    uint8_t SX126X::readsavedPower() {
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedPower()"));
#endif
      return savedTXPower;
    }


    uint8_t SX126X::getPacketMode() {
      //its either LoRa or FSK

#ifdef SX126XDEBUG
      _streamRef->println(F("getPacketMode()"));
#endif

      return savedPacketType;
    }


    uint8_t SX126X::readsavedPacketParam1() {
      //return previously set preamble
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedPacketParam1()"));
#endif
      return savedPacketParam1;
    }


    uint8_t SX126X::readsavedPacketParam2() {
      //return previously set header type
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedPacketParam2()"));
#endif
      return savedPacketParam2;
    }


    uint8_t SX126X::readsavedPacketParam3() {
      //return previously set packet length
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedPacketParam3()"));
#endif
      return savedPacketParam3;
    }


    uint8_t SX126X::readsavedPacketParam4() {
      //return previously set CRC
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedPacketParam4()"));
#endif
      return savedPacketParam4;
    }


    uint8_t SX126X::readsavedPacketParam5() {
      //return previously set IQ
#ifdef SX126XDEBUG
      _streamRef->println(F("readsavedPacketParam5()"));
#endif
      return savedPacketParam5;
    }


    uint8_t SX126X::getOpmode() {
      //return last saved opmode
#ifdef SX126XDEBUG
      _streamRef->println(F("getOpmode()"));
#endif
      return _OperatingMode;
    }


    uint8_t SX126X::getCRCMode() {
      //return last saved opmode
#ifdef SX126XDEBUG
      _streamRef->println(F("getCRCMode()"));
#endif
      return savedPacketParam4;
    }


    void SX126X::fillSXBuffer(uint8_t startaddress, uint8_t size, uint8_t character) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("fillSXBuffer()"));
#endif
      uint8_t index;

      setMode(MODE_STDBY_RC);

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(startaddress);

      for (index = 0; index < size; index++) {
        _spi->transfer(character);
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif
    }


    uint8_t SX126X::readPacket(uint8_t * rxbuffer, uint8_t size) {
#ifdef SX126XDEBUG
      _streamRef->println(F("readPacket()"));
#endif

      uint8_t index, regdata, RXstart, RXend;
      uint8_t buffer[2];

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL > size)  //check passed buffer is big enough for packet
      {
        _RXPacketL = size;  //truncate packet if not enough space
      }

      RXstart = buffer[1];

      RXend = RXstart + _RXPacketL;

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(RXstart);
      _spi->transfer(0xFF);

      for (index = RXstart; index < RXend; index++) {
        regdata = _spi->transfer(0);
        rxbuffer[index] = regdata;
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return _RXPacketL;  //so we can check for packet having enough buffer space
    }


    void SX126X::writeByteSXBuffer(uint8_t addr, uint8_t regdata) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("writeByteSXBuffer"));
#endif

      setMode(MODE_STDBY_RC);  //this is needed to ensure we can write to buffer OK.

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(addr);
      _spi->transfer(regdata);
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif
    }


    void SX126X::printSXBufferASCII(uint8_t start, uint8_t end) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("printSXBufferASCII)"));
#endif

      uint8_t index, regdata;
      setMode(MODE_STDBY_RC);

      for (index = start; index <= end; index++) {
        regdata = getByteSXBuffer(index);
        _streamRef->write(regdata);
      }
    }


    void SX126X::startFSKRTTY(uint32_t freqshift, uint8_t pips, uint16_t pipPeriodmS, uint16_t pipDelaymS, uint16_t leadinmS) {

#ifdef SX126XDEBUG1
      _streamRef->print(F("startFSKRTTY()"));
#endif

      uint32_t shiftedFrequencyRegisters;
      uint8_t index;
      uint32_t endmS;
      uint32_t calculatedRegShift;

      calculatedRegShift = (uint32_t)(freqshift / FREQ_STEP);
      shiftedFrequencyRegisters = savedFrequencyReg + calculatedRegShift;

      _ShiftfreqregH = (shiftedFrequencyRegisters >> 24) & 0xFF;  //MSB
      _ShiftfreqregMH = (shiftedFrequencyRegisters >> 16) & 0xFF;
      _ShiftfreqregML = (shiftedFrequencyRegisters >> 8) & 0xFF;
      _ShiftfreqregL = shiftedFrequencyRegisters & 0xFF;  //LSB

#ifdef DEBUGFSKRTTY
      _streamRef->print(F("NotShiftedFrequencyRegisters "));
      _streamRef->println(savedFrequencyReg, HEX);
      _streamRef->print(F("calculatedRegShift "));
      _streamRef->println(calculatedRegShift, HEX);
      _streamRef->print(F("ShiftedFrequencyRegisters "));
      _streamRef->print((uint32_t)shiftedFrequencyRegisters, HEX);
      _streamRef->print(F(" ("));
      _streamRef->print(_ShiftfreqregH, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(_ShiftfreqregMH, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(_ShiftfreqregML, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(_ShiftfreqregL, HEX);
      _streamRef->print(F(" )"));
      _streamRef->println();
#endif

      setTxParams(10, RADIO_RAMP_200_US);

      for (index = 1; index <= pips; index++) {
        setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency
        setTXDirect();                                                                           //turn on carrier
        delay(pipPeriodmS);
        setMode(MODE_STDBY_RC);  //turns off carrier
        delay(pipDelaymS);
      }

      setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency
      endmS = millis() + leadinmS;
      setTXDirect();  //turn on carrier
      while (millis() < endmS)
        ;  //leave leadin on
    }


    void SX126X::transmitFSKRTTY(uint8_t chartosend, uint8_t databits, uint8_t stopbits, uint8_t parity, uint16_t baudPerioduS, int8_t pin) {
      //micros() will rollover at 4294967295 or 71mins 35secs
      //assume slowest baud rate is 45 (baud period of 22222us) then with 11 bits max to send if routine starts
      //when micros() > (4294967295 - (22222 * 11) = 4294722855 = 0xFFFC4525 then it could overflow during send
      //Rather than deal with rolloever in the middle of a character lets wait till it overflows and then
      //start the character

#ifdef SX126XDEBUG1
      _streamRef->print(F("transmitFSKRTTY()"));
#endif

      uint8_t numbits;
      uint32_t enduS;
      uint8_t bitcount = 0;  //set when a bit is 1

      if (micros() > 0xFFFB6000)  //check if micros would overflow within circa 300mS, approx 1 char at 45baud
      {

#ifdef DEBUGFSKRTTY
        _streamRef->print(F("Overflow pending - micros() = "));
        _streamRef->println(micros(), HEX);
#endif

        while (micros() > 0xFFFB6000)
          ;  //wait a short while until micros overflows to 0

#ifdef DEBUGFSKRTTY
        _streamRef->print(F("Paused - micros() = "));
        _streamRef->println(micros(), HEX);
#endif
      }

      enduS = micros() + baudPerioduS;
      setRfFrequencyDirect(_freqregH, _freqregMH, _freqregML, _freqregL);  //set carrier frequency  (low)

      if (pin >= 0) {
        digitalWrite(pin, LOW);
      }

      while (micros() < enduS)
        ;  //start bit

      for (numbits = 1; numbits <= databits; numbits++)  //send bits, LSB first
      {
        enduS = micros() + baudPerioduS;  //start the timer
        if ((chartosend & 0x01) != 0)     //test for bit set, a 1
        {
          bitcount++;
          if (pin >= 0) {
            digitalWrite(pin, HIGH);
          }
          setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency for a 1
        } else {
          if (pin >= 0) {
            digitalWrite(pin, LOW);
          }
          setRfFrequencyDirect(_freqregH, _freqregMH, _freqregML, _freqregL);  //set carrier frequency for a 0
        }
        chartosend = (chartosend >> 1);  //get the next bit
        while (micros() < enduS)
          ;
      }

      enduS = micros() + baudPerioduS;  //start the timer for possible parity bit

      switch (parity) {
        case ParityNone:
          break;

        case ParityZero:
          setRfFrequencyDirect(_freqregH, _freqregMH, _freqregML, _freqregL);  //set carrier frequency for a 0
          while (micros() < enduS)
            ;
          break;

        case ParityOne:

          setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency for a 1
          while (micros() < enduS)
            ;
          break;

        case ParityOdd:
          if (bitRead(bitcount, 0)) {
            setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency for a 1
          } else {
            setRfFrequencyDirect(_freqregH, _freqregMH, _freqregML, _freqregL);  //set carrier frequency for a 0
          }
          while (micros() < enduS)
            ;
          break;

        case ParityEven:
          if (bitRead(bitcount, 0)) {
            setRfFrequencyDirect(_freqregH, _freqregMH, _freqregML, _freqregL);  //set carrier frequency for a 0
          } else {
            setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency for a 1
          }
          while (micros() < enduS)
            ;
          break;

        default:
          break;
      }

      //stop bits, normally 1 or 2
      enduS = micros() + (baudPerioduS * stopbits);

      if (pin >= 0) {
        digitalWrite(pin, HIGH);
      }

      setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency  for a 1

      while (micros() < enduS)
        ;
    }


    void SX126X::transmitFSKRTTY(uint8_t chartosend, uint16_t baudPerioduS, int8_t pin) {
      //micros() will rollover at 4294967295 or 71mins 35secs
      //assume slowest baud rate is 45 (baud period of 22222us) then with 11 bits max to send if routine starts
      //when micros() > (4294967295 - (22222 * 11) = 4294722855 = 0xFFFC4525 then it could overflow during send
      //Rather than deal with rolloever in the middle of a character lets wait till it overflows and then
      //start the character
      //This overloaded version of transmitFSKRTTY() uses 1 start bit, 7 data bits, no parity and 2 stop bits.


#ifdef SX126XDEBUG1
      _streamRef->print(F("transmitFSKRTTY()"));
#endif

      uint8_t numbits;
      uint32_t enduS;

      if (micros() > 0xFFFB6000)  //check if micros would overflow within circa 300mS, approx 1 char at 45baud
      {
#ifdef DEBUGFSKRTTY
        _streamRef->print(F("Overflow pending - micros() = "));
        _streamRef->println(micros(), HEX);
#endif
        while (micros() > 0xFFFB6000)
          ;  //wait a short while until micros overflows to 0
#ifdef DEBUGFSKRTTY
        _streamRef->print(F("Paused - micros() = "));
        _streamRef->println(micros(), HEX);
#endif
      }

      enduS = micros() + baudPerioduS;
      setRfFrequencyDirect(_freqregH, _freqregMH, _freqregML, _freqregL);  //set carrier frequency  (low)

      if (pin >= 0) {
        digitalWrite(pin, LOW);
      }

      while (micros() < enduS)
        ;  //start bit

      for (numbits = 1; numbits <= 7; numbits++)  //send bits, LSB first
      {
        enduS = micros() + baudPerioduS;  //start the timer
        if ((chartosend & 0x01) != 0)     //test for bit set, a 1
        {
          if (pin >= 0) {
            digitalWrite(pin, HIGH);
          }
          setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency for a 1
        } else {
          if (pin >= 0) {
            digitalWrite(pin, LOW);
          }
          setRfFrequencyDirect(_freqregH, _freqregMH, _freqregML, _freqregL);  //set carrier frequency for a 0
        }
        chartosend = (chartosend >> 1);  //get the next bit
        while (micros() < enduS)
          ;
      }

      //stop bits, normally 1 or 2
      enduS = micros() + (baudPerioduS * 2);

      if (pin >= 0) {
        digitalWrite(pin, HIGH);
      }

      setRfFrequencyDirect(_ShiftfreqregH, _ShiftfreqregMH, _ShiftfreqregML, _ShiftfreqregL);  //set carrier frequency

      while (micros() < enduS)
        ;
    }


    void SX126X::printRTTYregisters() {

#ifdef SX126XDEBUG1
      _streamRef->print(F("printRTTYregisters()"));
#endif

      _streamRef->print(F("NoShift Registers "));
      _streamRef->print(_freqregH, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(_freqregMH, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(_freqregML, HEX);
      _streamRef->print(F(" "));
      _streamRef->println(_freqregL, HEX);

      _streamRef->print(F("Shifted Registers "));
      _streamRef->print(_ShiftfreqregH, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(_ShiftfreqregMH, HEX);
      _streamRef->print(F(" "));
      _streamRef->print(_ShiftfreqregML, HEX);
      _streamRef->print(F(" "));
      _streamRef->println(_ShiftfreqregL, HEX);
    }


    void SX126X::endFSKRTTY() {
#ifdef SX126XDEBUG1
      _streamRef->print(F("endFSKRTTY()"));
#endif

      setMode(MODE_STDBY_RC);
    }


    void SX126X::getRfFrequencyRegisters(uint8_t * buff) {
      //returns the register values for the current set frequency

#ifdef SX126XDEBUG1
      _streamRef->print(F("getRfFrequencyRegisters()"));
#endif

      buff[0] = _freqregH;
      buff[1] = _freqregMH;
      buff[2] = _freqregML;
      buff[3] = _freqregL;
    }


    void SX126X::setRfFrequencyDirect(uint8_t high, uint8_t midhigh, uint8_t midlow, uint8_t low) {

#ifdef SX126XDEBUG1
      _streamRef->print(F("setRfFrequencyDirect()"));
#endif

      uint8_t buffer[4];

      buffer[0] = high;  //MSB
      buffer[1] = midhigh;
      buffer[2] = midlow;
      buffer[3] = low;  //LSB

      writeCommand(RADIO_SET_RFFREQUENCY, buffer, 4);
    }


    //**********************************************************************************************
    // Reliable packet routines - added November 2021
    // Routines assume that RX and TX buffer base addresses are set to 0 by setupLoRa()
    //**********************************************************************************************

    void SX126X::printASCIIArray(uint8_t * buffer, uint8_t size) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("printASCIIArray() "));
#endif

      uint8_t index;

      for (index = 0; index < size; index++) {
        _streamRef->write(buffer[index]);
      }
    }


    uint8_t SX126X::getReliableConfig(uint8_t bitread) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} getReliableConfig() "));
      _streamRef->println(_ReliableConfig);
#endif

      return bitRead(_ReliableConfig, bitread);
    }


    void SX126X::printReliableStatus() {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} printReliableStatus() "));
#endif

      //0x00
      if (bitRead(_ReliableErrors, ReliableCRCError)) {
        _streamRef->print(F(",ReliableCRCError"));
      }

      //0x01
      if (bitRead(_ReliableErrors, ReliableIDError)) {
        _streamRef->print(F(",ReliableIDError"));
      }

      //0x02
      if (bitRead(_ReliableErrors, ReliableSizeError)) {
        _streamRef->print(F(",ReliableSizeError"));
      }

      //0x03
      if (bitRead(_ReliableErrors, ReliableACKError)) {
        _streamRef->print(F(",NoReliableACK"));
      }

      //0x04
      if (bitRead(_ReliableErrors, ReliableTimeout)) {
        _streamRef->print(F(",ReliableTimeout"));
      }

      //0x00
      if (bitRead(_ReliableFlags, ReliableACKSent)) {
        _streamRef->print(F(",ACKsent"));
      }

      //0x01
      if (bitRead(_ReliableFlags, ReliableACKReceived)) {
        _streamRef->print(F(",ACKreceived"));
      }
    }


    uint8_t SX126X::transmitReliable(uint8_t * txbuffer, uint8_t size, uint16_t networkID, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} transmitRELIABLE() "));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
      _streamRef->print(F(" {RELIABLE} Payload length "));
      _streamRef->println(size);
#endif

      uint8_t index, tempdata;
      uint16_t payloadcrc;

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      if (size > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      setMode(MODE_STDBY_RC);
      _TXPacketL = size + 4;

      if (bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = 0;
      } else {
        payloadcrc = CRCCCITT(txbuffer, size, 0xFFFF);
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      for (index = 0; index < size; index++) {
        tempdata = txbuffer[index];
        _spi->transfer(tempdata);
      }

      _spi->transfer(lowByte(networkID));
      _spi->transfer(highByte(networkID));
      _spi->transfer(lowByte(payloadcrc));
      _spi->transfer(highByte(payloadcrc));

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      return _TXPacketL;
    }


    uint16_t SX126X::getTXPayloadCRC(uint8_t length) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} getTXPayloadCRC() "));
#endif

      return readUint16SXBuffer(length - 2);
    }


    uint16_t SX126X::readUint16SXBuffer(uint8_t addr) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} readUint16SXBuffer() 0x"));
      _streamRef->println(addr, HEX);
#endif

      uint8_t regdatalow, regdatahigh;
      setMode(MODE_STDBY_RC);  //this is needed to ensure we can read from buffer OK.

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(addr);
      _spi->transfer(0xFF);
      regdatalow = _spi->transfer(0);
      regdatahigh = _spi->transfer(0);
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return (regdatalow + (regdatahigh << 8));
    }


    void SX126X::setPayloadLength(uint8_t length) {
#ifdef SX126XDEBUG
      _streamRef->println(F("setPayloadLength()"));
#endif

#ifdef USEPAYLOADLENGTHREGISTER
      writeRegister(REG_LR_PAYLOADLENGTH, length);
#else
  setPacketParams(savedPacketParam1, savedPacketParam2, length, savedPacketParam4, savedPacketParam5);
#endif
    }


    uint8_t SX126X::receiveReliable(uint8_t * rxbuffer, uint8_t size, uint16_t networkID, uint32_t rxtimeout, uint8_t wait) {
      //Maximum total packet size is 255 bytes, so allowing for the 4 bytes appended to the end of a reliable
      //packet

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} receiveReliable()"));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t payloadcrc = 0, RXcrc, RXnetworkID = 0;
      uint8_t regdataL, regdataH;
      uint8_t index;
      uint8_t buffer[2];

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      if (size > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      setMode(MODE_STDBY_RC);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
        return 0;  //packet is errored somewhere so return 0
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL < 4)  //check received packet is 4 or more bytes long
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      if ((_RXPacketL - 4) > size)  //check if calculated payload size (_RXPacketL -4) fits in array
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(0);
      _spi->transfer(0xFF);

      for (index = 0; index < (_RXPacketL - 4); index++) {
        regdataL = _spi->transfer(0);
        rxbuffer[index] = regdataL;
      }

      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);
      RXnetworkID = ((uint16_t)regdataH << 8) + regdataL;
      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      if (!bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = CRCCCITT(rxbuffer, (_RXPacketL - 4), 0xFFFF);
        RXcrc = ((uint16_t)regdataH << 8) + regdataL;

        if (payloadcrc != RXcrc) {
          bitSet(_ReliableErrors, ReliableCRCError);
        }
      }

      if (RXnetworkID != networkID) {
        bitSet(_ReliableErrors, ReliableIDError);
      }

      if (_ReliableErrors)  //if there has been a reliable error return a RX fail
      {
        return 0;
      }

      return _RXPacketL;  //return and indicate RX OK.
    }


    uint16_t SX126X::getRXNetworkID(uint8_t length) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} getRXnetworkID() "));
#endif

      return readUint16SXBuffer(length - 4);
    }


    uint16_t SX126X::getRXPayloadCRC(uint8_t length) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} getRXPayloadCRC) "));
#endif

      return readUint16SXBuffer(length - 2);
    }


    uint8_t SX126X::transmitSXReliable(uint8_t startaddr, uint8_t length, uint16_t networkID, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} transmitSXReliable() "));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t payloadcrc;

      setMode(MODE_STDBY_RC);
      checkBusy();
      _ReliableErrors = 0;
      _ReliableFlags = 0;

      if (startaddr + length > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      _TXPacketL = length + 4;

      if (bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = 0;
      } else {
        payloadcrc = CRCCCITTReliable(startaddr, startaddr + length - 1, 0xFFFF);
      }

      writeUint16SXBuffer(startaddr + _TXPacketL - 4, networkID);
      writeUint16SXBuffer(startaddr + _TXPacketL - 2, payloadcrc);

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);                                                         //this starts the TX

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for pin to go high, TX finished

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      return _TXPacketL;
    }


    uint8_t SX126X::receiveSXReliable(uint8_t startaddr, uint16_t networkID, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} receiveSXReliable()"));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t payloadcrc = 0, RXcrc, RXnetworkID = 0;
      uint8_t buffer[2];

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      setMode(MODE_STDBY_RC);
      checkBusy();
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high, no timeout, RX DONE

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
        return 0;  //no RX done and header valid only, could be CRC error
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL < 4)  //check received packet is 4 or more bytes long
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      RXnetworkID = readUint16SXBuffer(startaddr + _RXPacketL - 4);

      if (RXnetworkID != networkID) {
        bitSet(_ReliableErrors, ReliableIDError);
      }

      if (!bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = CRCCCITTReliable(startaddr, (startaddr + _RXPacketL - 5), 0xFFFF);
        RXcrc = readUint16SXBuffer(startaddr + _RXPacketL - 2);

        if (payloadcrc != RXcrc) {
          bitSet(_ReliableErrors, ReliableCRCError);
        }
      }

      if (_ReliableErrors)  //if there has been a reliable error return a RX fail
      {
        return 0;
      }
      return _RXPacketL;  //return and RX OK.
    }


    uint16_t SX126X::CRCCCITTReliable(uint8_t startadd, uint8_t endadd, uint16_t startvalue) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} CRCCCITTReliable()"));
#endif

      //generates a CRC of bytes from the internal SX buffer, _RXPackletL and _TXPackletL are not affected

#ifdef SX126DEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} CRCCCITTReliable() "));
#endif

      uint16_t index, libraryCRC;
      uint8_t j, readSX;

      libraryCRC = startvalue;  //start value for CRC16
      setMode(MODE_STDBY_RC);
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(startadd);
      _spi->transfer(0xFF);

      for (index = startadd; index <= endadd; index++) {
        readSX = _spi->transfer(0);
        libraryCRC ^= (((uint16_t)readSX) << 8);
        for (j = 0; j < 8; j++) {
          if (libraryCRC & 0x8000)
            libraryCRC = (libraryCRC << 1) ^ 0x1021;
          else
            libraryCRC <<= 1;
        }
      }

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      return libraryCRC;
    }


    uint8_t SX126X::transmitReliableAutoACK(uint8_t * txbuffer, uint8_t size, uint16_t networkID, uint32_t acktimeout, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} transmitReliableAutoACK() "));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
      _streamRef->print(F(" {RELIABLE} Payload length "));
      _streamRef->println(size);
#endif

      uint8_t index, tempdata, RXPacketL;
      uint16_t payloadcrc;

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      if (size > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      setMode(MODE_STDBY_RC);
      checkBusy();
      _TXPacketL = size + 4;

      if (bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = 0;
      } else {
        payloadcrc = CRCCCITT(txbuffer, size, 0xFFFF);
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      for (index = 0; index < size; index++) {
        tempdata = txbuffer[index];
        _spi->transfer(tempdata);
      }

      _spi->transfer(lowByte(networkID));
      _spi->transfer(highByte(networkID));
      _spi->transfer(lowByte(payloadcrc));
      _spi->transfer(highByte(payloadcrc));

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      RXPacketL = waitReliableACK(networkID, payloadcrc, acktimeout);

      if (RXPacketL != 4) {
        return 0;
      }

      return _TXPacketL;
    }


    uint8_t SX126X::receiveReliableAutoACK(uint8_t * rxbuffer, uint8_t size, uint16_t networkID, uint32_t ackdelay, int8_t txpower, uint32_t rxtimeout, uint8_t wait) {
      //Maximum total packet size is 255 bytes, so allowing for the 4 bytes appended to the end of a reliable
      //packet, the maximum payload size for LORa is 251 bytes. So to avoid overwriting
      //memory, we do need to check if the passed array is big enough to take the payload received in the packet.
      //The assumed payload length will always be 4 bytes less than the received packet length.

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} receiveReliableAutoACK()"));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t payloadcrc = 0, RXcrc, RXnetworkID = 0;
      uint8_t regdataL, regdataH, index;
      uint8_t buffer[2];

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      if (size > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      setMode(MODE_STDBY_RC);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;                      //Wait for DIO1 to go high
      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
        return 0;  //packet is errored somewhere so return 0
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL < 4)  //check received packet is 4 or more bytes long
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      if ((_RXPacketL - 4) > size)  //check if calculated payload size (_RXPacketL -4) fits in array
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(0);
      _spi->transfer(0xFF);

      for (index = 0; index < (_RXPacketL - 4); index++) {
        regdataL = _spi->transfer(0);
        rxbuffer[index] = regdataL;
      }

      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);
      RXnetworkID = ((uint16_t)regdataH << 8) + regdataL;
      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);
      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      if (!bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = CRCCCITT(rxbuffer, (_RXPacketL - 4), 0xFFFF);
        RXcrc = ((uint16_t)regdataH << 8) + regdataL;

        if (payloadcrc != RXcrc) {
          bitSet(_ReliableErrors, ReliableCRCError);
        }
      }

      if (RXnetworkID != networkID) {
        bitSet(_ReliableErrors, ReliableIDError);
      }

      if (_ReliableErrors)  //if there has been a reliable error return a RX fail
      {
        return 0;
      }

      delay(ackdelay);
      _TXPacketL = sendReliableACK(RXnetworkID, payloadcrc, txpower);
      if (_TXPacketL != 4) {
        return 0;
      }

      return _RXPacketL;  //return and indicate RX OK.
    }


    uint8_t SX126X::waitReliableACK(uint16_t networkID, uint16_t payloadcrc, uint32_t acktimeout) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} waitReliableACK()"));
#endif

      uint16_t RXnetworkID, RXcrc;
      uint32_t startmS;
      uint8_t buffer[2];

      setReliableRX(0);
      startmS = millis();

      do {
        if (digitalRead(_RXDonePin))  //has a packet arrived ?
        {
          if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
            setReliableRX(0);
            continue;
          }

          readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
          _RXPacketL = buffer[0];

          RXnetworkID = readUint16SXBuffer(_RXPacketL - 4);
          RXcrc = readUint16SXBuffer(_RXPacketL - 2);

          if ((RXnetworkID == networkID) && (RXcrc == payloadcrc)) {
            bitSet(_ReliableFlags, ReliableACKReceived);
            return 4;  //return value of 4 indicates valid ack
          } else {
            setReliableRX(0);
            continue;
          }
        }

      } while (((uint32_t)(millis() - startmS) < acktimeout));

      bitSet(_ReliableErrors, ReliableACKError);
      return 0;
    }


    void SX126X::setReliableRX(uint16_t timeout) {
      //existing setRx() does not setup LoRa device as a receiver completly, just turns on receiver mode
      //this routine does all the required setup for receive mode
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} setReliableRX()"));
#endif

      setMode(MODE_STDBY_RC);         //stops receiver
      clearIrqStatus(IRQ_RADIO_ALL);  //clear current interrupt flags
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
      setRx(timeout);
    }


    uint8_t SX126X::sendReliableACK(uint16_t networkID, uint16_t payloadcrc, int8_t txpower) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} sendReliableACK()"));
#endif

      uint32_t txtimeout = 12000;  //set TX timeout to 12 seconds, longest packet is 8.7secs
      _TXPacketL = 4;              //packet is networkId (2 bytes) + payloadCRC (2 bytes)
      setMode(MODE_STDBY_RC);

      writeUint16SXBuffer(0, networkID);
      writeUint16SXBuffer(2, payloadcrc);

      checkBusy();
      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      while (!digitalRead(_TXDonePin))
        ;

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      bitSet(_ReliableFlags, ReliableACKSent);
      return 4;  //TX OK so return TXpacket length
    }


    uint8_t SX126X::transmitSXReliableAutoACK(uint8_t startaddr, uint8_t length, uint16_t networkID, uint32_t acktimeout, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} transmitSXReliableAutoACK() "));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint8_t RXPacketL;
      uint16_t payloadcrc;

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      if (startaddr + length > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      setMode(MODE_STDBY_RC);
      checkBusy();
      _TXPacketL = length + 4;

      if (bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = 0;
      } else {
        payloadcrc = CRCCCITTReliable(startaddr, startaddr + length - 1, 0xFFFF);
      }

      writeUint16SXBuffer(startaddr + _TXPacketL - 4, networkID);
      writeUint16SXBuffer(startaddr + _TXPacketL - 2, payloadcrc);
      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      if (!wait) {
        return _TXPacketL;
      }

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      RXPacketL = waitReliableACK(networkID, payloadcrc, acktimeout);

      if (RXPacketL != 4) {

        return 0;
      }

      return _TXPacketL;
    }


    uint8_t SX126X::receiveSXReliableAutoACK(uint8_t startaddr, uint16_t networkID, uint32_t ackdelay, int8_t txpower, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} receiveSXReliableAutoACK()"));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t payloadcrc = 0, RXcrc, RXnetworkID = 0;
      uint16_t temp1, temp2;
      uint8_t buffer[2];

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      setMode(MODE_STDBY_RC);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on RX done or timeout
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;                      //Wait for DIO1 to go high
      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
        return 0;  //packet is errored somewhere so return 0
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL < 4)  //check received packet is 4 or more bytes long
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      RXnetworkID = readUint16SXBuffer(startaddr + _RXPacketL - 4);

      if (RXnetworkID != networkID) {
        bitSet(_ReliableErrors, ReliableIDError);
      }

      if (!bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = CRCCCITTReliable(startaddr, (startaddr + _RXPacketL - 5), 0xFFFF);
        RXcrc = readUint16SXBuffer(startaddr + _RXPacketL - 2);

        if (payloadcrc != RXcrc) {
          bitSet(_ReliableErrors, ReliableCRCError);
        }
      }

      if (_ReliableErrors)  //if there has been a reliable error return a RX fail
      {
        return 0;
      }

      delay(ackdelay);
      temp1 = readUint16SXBuffer(startaddr);      //save bytes that would be overwritten by ack
      temp2 = readUint16SXBuffer(startaddr + 2);  //save bytes that would be overwritten by ack
      _TXPacketL = sendReliableACK(RXnetworkID, payloadcrc, txpower);
      writeUint16SXBuffer(startaddr, temp1);      //restore bytes that would be overwritten by ack
      writeUint16SXBuffer(startaddr + 2, temp2);  //restore bytes that would be overwritten by ack

      if (_TXPacketL != 4) {
        bitSet(_ReliableErrors, ReliableACKError);
        return 0;
      }

      return _RXPacketL;  //return indicating RX ack sent OK.
    }


    uint8_t SX126X::waitReliableACK(uint8_t * rxbuffer, uint8_t size, uint16_t networkID, uint16_t payloadcrc, uint32_t acktimeout) {
      //overloaded version of waitReliableACK() for use when ack contains payload data
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} waitReliableACK()"));
#endif

      uint16_t RXnetworkID, RXcrc;
      uint32_t startmS;
      uint8_t buffer[2];
      uint8_t regdata, index;

      if (size > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      setReliableRX(0);
      startmS = millis();

      do {
        if (digitalRead(_RXDonePin))  //has a packet arrived ?
        {
          regdata = readIrqStatus();

          if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
            setReliableRX(0);
            continue;
          }

          readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
          _RXPacketL = buffer[0];
          RXnetworkID = readUint16SXBuffer(_RXPacketL - 4);
          RXcrc = readUint16SXBuffer(_RXPacketL - 2);

          if ((RXnetworkID == networkID) && (RXcrc == payloadcrc)) {
            if ((_RXPacketL - 4) > size)  //check passed buffer is big enough for payload
            {
              bitSet(_ReliableErrors, ReliableACKError);
              bitSet(_ReliableErrors, ReliableSizeError);
              return 0;
            }

            bitSet(_ReliableFlags, ReliableACKReceived);
            checkBusy();

#ifdef USE_SPI_TRANSACTION
            _spi->beginTransaction(_spiSettings);
#endif

            digitalWrite(_NSS, LOW);  //start the burst read
            _spi->transfer(RADIO_READ_BUFFER);
            _spi->transfer(0);
            _spi->transfer(0xFF);

            for (index = 0; index < (_RXPacketL - 4); index++)  //read packet into rxbuffer
            {
              regdata = _spi->transfer(0);
              rxbuffer[index] = regdata;
            }
            digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
            _spi->endTransaction();
#endif

            return _RXPacketL;  //_RXPacketL should be payload length + 4
          } else {
            setReliableRX(0);
            continue;
          }
        }
      } while (((uint32_t)(millis() - startmS) < acktimeout));

      bitSet(_ReliableErrors, ReliableACKError);
      return 0;
    }


    uint8_t SX126X::sendReliableACK(uint8_t * txbuffer, uint8_t size, uint16_t networkID, uint16_t payloadcrc, int8_t txpower) {
      //overloaded version of sendReliableACK() for use when ack contains payload data
#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} sendReliableACK() "));
      _streamRef->print(F("buffer size "));
      _streamRef->println(size);
#endif

      uint32_t txtimeout = 12000;  //set TX timeout to 12 seconds, longest packet is 8.7secs
      uint8_t bufferdata, index;

      setMode(MODE_STDBY_RC);
      _TXPacketL = size + 4;
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      for (index = 0; index < size; index++) {
        bufferdata = txbuffer[index];
        _spi->transfer(bufferdata);
      }

      _spi->transfer(lowByte(networkID));
      _spi->transfer(highByte(networkID));
      _spi->transfer(lowByte(payloadcrc));
      _spi->transfer(highByte(payloadcrc));

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);                                                         //this starts the TX

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }

      bitSet(_ReliableFlags, ReliableACKSent);
      return _TXPacketL;  //TX OK so return TXpacket length
    }


    uint8_t SX126X::waitSXReliableACK(uint8_t startaddr, uint16_t networkID, uint16_t payloadcrc, uint32_t acktimeout) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} waitSXReliableACK()"));
#endif

      uint16_t RXnetworkID, RXcrc;
      uint32_t startmS;
      uint8_t buffer[2];

      setReliableRX(0);

      startmS = millis();

      do {
        if (digitalRead(_RXDonePin))  //has a packet arrived ?
        {
          if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
            setReliableRX(0);
            continue;
          }

          readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
          _RXPacketL = buffer[0];
          RXnetworkID = readUint16SXBuffer(startaddr + _RXPacketL - 4);
          RXcrc = readUint16SXBuffer(startaddr + _RXPacketL - 2);

          if ((RXnetworkID == networkID) && (RXcrc == payloadcrc)) {
            bitSet(_ReliableFlags, ReliableACKReceived);
            return _RXPacketL;  //_RXPacketL should be payload length + 4
          } else {
            setReliableRX(0);
            continue;
          }
        }
      } while (((uint32_t)(millis() - startmS) < acktimeout));

      bitSet(_ReliableErrors, ReliableACKError);
      return 0;
    }


    uint8_t SX126X::sendSXReliableACK(uint8_t startaddr, uint8_t length, uint16_t networkID, uint16_t payloadcrc, int8_t txpower) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} sendSXReliableACK() "));
#endif

      uint32_t txtimeout = 12000;  //set TX timeout to 12 seconds, longest packet is 8.7secs

      _TXPacketL = length + 4;  //packet is networkId (2 bytes) + payloadCRC (2 bytes)
      setMode(MODE_STDBY_RC);

      writeUint16SXBuffer((length + startaddr), networkID);
      writeUint16SXBuffer((length + startaddr + 2), payloadcrc);
      checkBusy();
      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);                                                         //start transmission

      while (!digitalRead(_TXDonePin))
        ;

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      bitSet(_ReliableFlags, ReliableACKSent);
      return _TXPacketL;  //TX OK so return TXpacket length
    }


    uint8_t SX126X::transmitSXReliableIRQ(uint8_t startaddr, uint8_t length, uint16_t networkID, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} transmitSXReliableIRQ() "));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t payloadcrc;

      setMode(MODE_STDBY_RC);
      checkBusy();
      _ReliableErrors = 0;
      _ReliableFlags = 0;

      if (startaddr + length > 251) {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      _TXPacketL = length + 4;

      if (bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = 0;
      } else {
        payloadcrc = CRCCCITTReliable(startaddr, startaddr + length - 1, 0xFFFF);
      }

      writeUint16SXBuffer(startaddr + _TXPacketL - 4, networkID);
      writeUint16SXBuffer(startaddr + _TXPacketL - 2, payloadcrc);
      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setTx(txtimeout);  //this starts the TX

      if (!wait) {
        return _TXPacketL;
      }

      //0x0201   = IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT
      while (!(readIrqStatus() & 0x0201))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      return _TXPacketL;
    }


    uint8_t SX126X::waitSXReliableACKIRQ(uint8_t startaddr, uint16_t networkID, uint16_t payloadcrc, uint32_t acktimeout) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} waitSXReliableACKIRQ()"));
#endif

      uint16_t RXnetworkID, RXcrc;
      uint32_t startmS;
      uint8_t buffer[2];

      setReliableRX(0);
      startmS = millis();

      do {
        //0x0202   = IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
        if (readIrqStatus() & 0x0202) {
          if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
            setReliableRX(0);
            continue;
          }

          readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
          _RXPacketL = buffer[0];
          RXnetworkID = readUint16SXBuffer(startaddr + _RXPacketL - 4);
          RXcrc = readUint16SXBuffer(startaddr + _RXPacketL - 2);

          if ((RXnetworkID == networkID) && (RXcrc == payloadcrc)) {
            bitSet(_ReliableFlags, ReliableACKReceived);
            return _RXPacketL;  //_RXPacketL should be payload length + 4
          } else {
            setReliableRX(0);
            continue;
          }
        }
      } while (((uint32_t)(millis() - startmS) < acktimeout));

      bitSet(_ReliableErrors, ReliableACKError);
      return 0;
    }


    uint8_t SX126X::receiveSXReliableIRQ(uint8_t startaddr, uint16_t networkID, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} receiveSXReliable()"));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t payloadcrc = 0, RXcrc, RXnetworkID = 0;
      uint8_t buffer[2];

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      setMode(MODE_STDBY_RC);
      checkBusy();
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      //0x0202   = IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
      while (!(readIrqStatus() & 0x0202))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
        return 0;  //no RX done and header valid only, could be CRC error
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];

      if (_RXPacketL < 4)  //check received packet is 4 or more bytes long
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      RXnetworkID = readUint16SXBuffer(startaddr + _RXPacketL - 4);

      if (RXnetworkID != networkID) {
        bitSet(_ReliableErrors, ReliableIDError);
      }

      if (!bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = CRCCCITTReliable(startaddr, (startaddr + _RXPacketL - 5), 0xFFFF);
        RXcrc = readUint16SXBuffer(startaddr + _RXPacketL - 2);

        if (payloadcrc != RXcrc) {
          bitSet(_ReliableErrors, ReliableCRCError);
        }
      }

      if (_ReliableErrors)  //if there has been a reliable error return a RX fail
      {
        return 0;
      }

      return _RXPacketL;  //return and RX OK.
    }


    uint8_t SX126X::sendSXReliableACKIRQ(uint8_t startaddr, uint8_t length, uint16_t networkID, uint16_t payloadcrc, int8_t txpower) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} sendSXReliableACK() "));
#endif

      uint32_t txtimeout = 12000;  //set TX timeout to 12 seconds, longest packet is 8.7secs

      _TXPacketL = length + 4;  //packet is networkId (2 bytes) + payloadCRC (2 bytes)
      setMode(MODE_STDBY_RC);

      writeUint16SXBuffer((length + startaddr), networkID);
      writeUint16SXBuffer((length + startaddr + 2), payloadcrc);

      checkBusy();
      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setTx(txtimeout);  //start transmission

      //0x0201   = IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT
      while (!(readIrqStatus() & 0x0201))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      }

      bitSet(_ReliableFlags, ReliableACKSent);
      return _TXPacketL;  //TX OK so return TXpacket length
    }


    uint8_t SX126X::transmitSXBufferIRQ(uint8_t startaddr, uint8_t length, uint16_t timeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUG
      _streamRef->println(F("transmitSXBuffer()"));
#endif

      setBufferBaseAddress(startaddr, 0);
      setPayloadLength(length);
      setTxParams(txpower, RAMP_TIME);
      setTx(timeout);  //this starts the TX

      if (!wait) {
        return _TXPacketL;
      }

      //0x0201   = IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT
      while (!(readIrqStatus() & 0x0201))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }
    }


    uint8_t SX126X::receiveSXBufferIRQ(uint8_t startaddr, uint16_t timeout, uint8_t wait) {
#ifdef SX126XDEBUG1
      _streamRef->println(F("receiveSXBufferIRQ()"));
#endif

      uint8_t buffer[2];

      setMode(MODE_STDBY_RC);
      setBufferBaseAddress(0, startaddr);
      setRx(timeout);

      if (!wait) {
        return 0;
      }

      //0x0202   = IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
      while (!(readIrqStatus() & 0x0202))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      if (readIrqStatus() & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT)) {
        return 0;  //no RX done and header valid only, could be CRC error
      }

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];
      return _RXPacketL;
    }


    void SX126X::setReliableConfig(uint8_t bitset) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} setReliableConfig() bit "));
      _streamRef->println(bitset);
#endif

      bitSet(_ReliableConfig, bitset);
      return;
    }


    void SX126X::clearReliableConfig(uint8_t bitset) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} clearReliableConfig() bit "));
      _streamRef->println(bitset);
#endif

      bitClear(_ReliableConfig, bitset);
      return;
    }


    //***********************************************************************************
    //Data Transfer functions - Added December 2021
    //TX and RX base addresses assumed to be 0
    //***********************************************************************************

    uint8_t SX126X::transmitDT(uint8_t * header, uint8_t headersize, uint8_t * dataarray, uint8_t datasize, uint16_t networkID, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} transmitDT() "));
#endif

      uint8_t index, bufferdata;
      uint16_t payloadcrc;

      _ReliableErrors = 0;
      _ReliableFlags = 0;

#ifdef DETECTRELIABLERRORS
      if (datasize > (251 - headersize))  //its 251 because of 4 bytes appended to packet
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }
#endif
      setMode(MODE_STDBY_RC);
      _TXPacketL = headersize + datasize + 4;

      if (bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = 0;
      } else {
        payloadcrc = CRCCCITT(dataarray, datasize, 0xFFFF);
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      //load up the header
      for (index = 0; index < headersize; index++) {
        bufferdata = header[index];
        _spi->transfer(bufferdata);
      }

      //load up the data array
      for (index = 0; index < datasize; index++) {
        bufferdata = dataarray[index];
        _spi->transfer(bufferdata);
      }

      //append the network ID and payload CRC at end
      _spi->transfer(lowByte(networkID));
      _spi->transfer(highByte(networkID));
      _spi->transfer(lowByte(payloadcrc));
      _spi->transfer(highByte(payloadcrc));

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      if (!wait) {
        return _TXPacketL;
      }

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }
    }


    uint8_t SX126X::sendACKDT(uint8_t * header, uint8_t headersize, int8_t txpower) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} sendACKDT() "));
#endif

      uint32_t txtimeout = 12000;  //set TX timeout to 12 seconds, longest packet is 8.7secs
      uint8_t bufferdata, index;
      uint16_t networkID;
      uint16_t payloadCRC;

      setMode(MODE_STDBY_RC);
      _TXPacketL = headersize + 4;
      networkID = readUint16SXBuffer(_RXPacketL - 4);
      payloadCRC = readUint16SXBuffer(_RXPacketL - 2);
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      for (index = 0; index < headersize; index++) {
        bufferdata = header[index];
        _spi->transfer(bufferdata);
      }

      _spi->transfer(lowByte(networkID));
      _spi->transfer(highByte(networkID));
      _spi->transfer(lowByte(payloadCRC));
      _spi->transfer(highByte(payloadCRC));

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      while (!digitalRead(_TXDonePin))
        ;  //Wait for DIO1 to go high

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        bitSet(_ReliableFlags, ReliableTimeout);
        return 0;
      }

      bitSet(_ReliableFlags, ReliableACKSent);
      return _TXPacketL;  //TX OK so return TXpacket length
    }


    uint8_t SX126X::receiveDT(uint8_t * header, uint8_t headersize, uint8_t * dataarray, uint8_t datasize, uint16_t networkID, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} receiveDT()"));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t index, payloadcrc = 0, RXcrc, RXnetworkID = 0;
      uint8_t regdataL, regdataH;
      uint8_t RXHeaderL;
      uint8_t RXDataL;
      uint8_t buffer[2];
      uint8_t RXstart;

      _ReliableErrors = 0;
      _ReliableFlags = 0;
      setMode(MODE_STDBY_RC);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      while (!digitalRead(_RXDonePin))
        ;  //Wait for DIO1 to go high, no timeout, RX DONE

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      //IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT = 0x0260
      if (readIrqStatus() & 0x0260) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->println(F(" {RELIABLE} Packet error"));
#endif
        return 0;  //packet is errored somewhere so return 0
      }

      RXHeaderL = getByteSXBuffer(2);
      RXDataL = getByteSXBuffer(3);

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];
      RXstart = buffer[1];

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} Header  "));
      printSXBufferHEX(0, (RXHeaderL - 1));
      _streamRef->println();
      _streamRef->print(F(" {RELIABLE} Received data payload size "));
      _streamRef->println(RXDataL);
      _streamRef->print(F(" {RELIABLE} Data payload  "));
      printSXBufferHEX(RXHeaderL, RXHeaderL + RXDataL - 1);
      _streamRef->println();
#endif

      if (RXHeaderL > headersize) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} Header size error "));
        _streamRef->println(headersize);
#endif
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      if (RXDataL > datasize) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} Data size error "));
        _streamRef->println(datasize);
#endif
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} _RXPacketL  "));
      _streamRef->println(_RXPacketL);
      _streamRef->print(F(" {RELIABLE} Header  "));
      printSXBufferHEX(0, (RXHeaderL - 1));
      _streamRef->println();
      _streamRef->print(F(" {RELIABLE} Received data payload size "));
      _streamRef->println(RXDataL);
      _streamRef->print(F(" {RELIABLE} Data payload  "));
      printSXBufferHEX(RXHeaderL, RXHeaderL + RXDataL - 1);
      _streamRef->println();
#endif

      if (_RXPacketL < 10)  //check received packet is 10 or more bytes long
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(RXstart);
      _spi->transfer(0xFF);

      for (index = 0; index < RXHeaderL; index++) {
        regdataL = _spi->transfer(0);
        header[index] = regdataL;
      }

      for (index = 0; index < RXDataL; index++) {
        regdataL = _spi->transfer(0);
        dataarray[index] = regdataL;
      }

      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);
      RXnetworkID = ((uint16_t)regdataH << 8) + regdataL;
      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      if (!bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = CRCCCITT(dataarray, RXDataL, 0xFFFF);
        RXcrc = ((uint16_t)regdataH << 8) + regdataL;

#ifdef SX126XDEBUGRELIABLE
        _streamRef->println(F(" {RELIABLE} Payload CRC check enabled"));
        _streamRef->print(F(" {RELIABLE} payloadcrc 0x"));
        _streamRef->println(payloadcrc, HEX);
        _streamRef->print(F(" {RELIABLE} RXcrc 0x"));
        _streamRef->println(RXcrc, HEX);
#endif

        if (payloadcrc != RXcrc) {
          bitSet(_ReliableErrors, ReliableCRCError);
#ifdef SX126XDEBUGRELIABLE
          _streamRef->print(F(" {RELIABLE} CRCmissmatch, received 0x"));
          _streamRef->println(RXcrc, HEX);
#endif
        }
      }

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} RXnetworkID 0x"));
      _streamRef->println(RXnetworkID, HEX);
#endif

      if (RXnetworkID != networkID) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} NetworkID missmatch received 0x"));
        _streamRef->print(RXnetworkID, HEX);
        _streamRef->print(F(" LocalID 0x"));
        _streamRef->println(networkID, HEX);
#endif
        bitSet(_ReliableErrors, ReliableIDError);
      }

      if (_ReliableErrors)  //if there has been a reliable error return a RX fail
      {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} Reliable errors"));
#endif

        return 0;
      }

      return _RXPacketL;  //return and indicate RX OK.
    }


    uint8_t SX126X::waitACKDT(uint8_t * header, uint8_t headersize, uint32_t acktimeout) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} waitACKDT()"));
#endif

      uint16_t RXnetworkID, RXcrc;
      uint32_t startmS;
      uint8_t regdata, index;
      uint16_t networkID;
      uint16_t payloadCRC;
      uint8_t buffer[2];

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      networkID = readUint16SXBuffer(_TXPacketL - 4);   //get networkID used to transmit previous packet, before next RX
      payloadCRC = readUint16SXBuffer(_TXPacketL - 2);  //get payloadCRC used to transmit previous packet, before next RX
      setReliableRX(0);
      startmS = millis();  //setReliableRX has a timeount, but here we want an overall timeout waiting for ACK

      do {
        if (digitalRead(_RXDonePin))  //has a packet arrived ?
        {
          setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

          //IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT = 0x0260
          if (readIrqStatus() & 0x0260) {
            setReliableRX(0);
            continue;
          }

          readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
          _RXPacketL = buffer[0];
          RXnetworkID = readUint16SXBuffer(_RXPacketL - 4);
          RXcrc = readUint16SXBuffer(_RXPacketL - 2);

          if ((_RXPacketL - 4) > headersize)  //check passed buffer is big enough for header
          {
            setReliableRX(0);
            continue;
          }

          if (!bitRead(_ReliableConfig, NoReliableCRC)) {
            if (payloadCRC != RXcrc) {
              bitSet(_ReliableErrors, ReliableCRCError);
              setReliableRX(0);
              continue;
            }
          }

          if ((RXnetworkID == networkID)) {
            bitSet(_ReliableFlags, ReliableACKReceived);

            checkBusy();

#ifdef USE_SPI_TRANSACTION
            _spi->beginTransaction(_spiSettings);
#endif

            digitalWrite(_NSS, LOW);  //start the burst read
            _spi->transfer(RADIO_READ_BUFFER);
            _spi->transfer(0);
            _spi->transfer(0xFF);

            for (index = 0; index < (_RXPacketL - 4); index++)  //read packet into rxbuffer
            {
              regdata = _spi->transfer(0);
              header[index] = regdata;
            }
            digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
            _spi->endTransaction();
#endif

            return _RXPacketL;  //_RXPacketL should be payload length + 4
          } else {
            setReliableRX(0);
            continue;
          }
        }
      } while (((uint32_t)(millis() - startmS) < acktimeout));

      bitSet(_ReliableErrors, ReliableACKError);
      bitSet(_ReliableErrors, ReliableTimeout);

      return 0;
    }


    uint16_t SX126X::getTXNetworkID(uint8_t length) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} getTXnetworkID() "));
#endif

      return readUint16SXBuffer(length - 4);
    }

    uint8_t SX126X::readReliableErrors() {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} readReliableErrors()"));
#endif

      return _ReliableErrors;
    }

    uint8_t SX126X::readReliableFlags() {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} readReliableFlags()"));
#endif

      return _ReliableFlags;
    }


    uint8_t SX126X::sendACKDTIRQ(uint8_t * header, uint8_t headersize, int8_t txpower) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} sendACKDT() "));
#endif

      uint32_t txtimeout = 12000;  //set TX timeout to 12 seconds, longest packet is 8.7secs
      uint8_t bufferdata, index;
      uint16_t networkID;
      uint16_t payloadCRC;

      setMode(MODE_STDBY_RC);
      _TXPacketL = headersize + 4;
      networkID = readUint16SXBuffer(_RXPacketL - 4);
      payloadCRC = readUint16SXBuffer(_RXPacketL - 2);
      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      for (index = 0; index < headersize; index++) {
        bufferdata = header[index];
        _spi->transfer(bufferdata);
      }

      _spi->transfer(lowByte(networkID));
      _spi->transfer(highByte(networkID));
      _spi->transfer(lowByte(payloadCRC));
      _spi->transfer(highByte(payloadCRC));

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      while (!(readIrqStatus() & 0x201))
        ;  //wait for IRQs going active

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        bitSet(_ReliableFlags, ReliableTimeout);
        return 0;
      }

      bitSet(_ReliableFlags, ReliableACKSent);
      return _TXPacketL;  //TX OK so return TXpacket length
    }


    uint8_t SX126X::waitACKDTIRQ(uint8_t * header, uint8_t headersize, uint32_t acktimeout) {

#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} waitACKDTIRQ()"));
#endif

      uint16_t RXnetworkID, RXcrc;
      uint32_t startmS;
      uint8_t regdata, index;
      uint16_t networkID;
      uint16_t payloadCRC;
      uint8_t buffer[2];

      _ReliableErrors = 0;
      _ReliableFlags = 0;

      networkID = readUint16SXBuffer(_TXPacketL - 4);   //get networkID used to transmit previous packet, before next RX
      payloadCRC = readUint16SXBuffer(_TXPacketL - 2);  //get payloadCRC used to transmit previous packet, before next RX

      setReliableRX(0);
      startmS = millis();  //setReliableRX has a timeount, but here we want an overall timeout waiting for ACK

      do {
        //0x02   = IRQ_RX_DONE
        if (readIrqStatus() & 0x02)  //has a packet arrived?
        {
          setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

          //IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT = 0x0260
          if (readIrqStatus() & 0x0260) {
            setReliableRX(0);
            continue;
          }
          readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
          _RXPacketL = buffer[0];
          RXnetworkID = readUint16SXBuffer(_RXPacketL - 4);
          RXcrc = readUint16SXBuffer(_RXPacketL - 2);

          if ((_RXPacketL - 4) > headersize)  //check passed buffer is big enough for header
          {
            setReliableRX(0);
            continue;
          }

          if (!bitRead(_ReliableConfig, NoReliableCRC)) {
            if (payloadCRC != RXcrc) {
              bitSet(_ReliableErrors, ReliableCRCError);
              setReliableRX(0);
              continue;
            }
          }

          if ((RXnetworkID == networkID)) {
            bitSet(_ReliableFlags, ReliableACKReceived);

            checkBusy();

#ifdef USE_SPI_TRANSACTION
            _spi->beginTransaction(_spiSettings);
#endif

            digitalWrite(_NSS, LOW);  //start the burst read
            _spi->transfer(RADIO_READ_BUFFER);
            _spi->transfer(0);
            _spi->transfer(0xFF);

            for (index = 0; index < (_RXPacketL - 4); index++)  //read packet into rxbuffer
            {
              regdata = _spi->transfer(0);
              header[index] = regdata;
            }
            digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
            _spi->endTransaction();
#endif

            return _RXPacketL;  //_RXPacketL should be payload length + 4
          } else {
            setReliableRX(0);
            continue;
          }
        }
      } while (((uint32_t)(millis() - startmS) < acktimeout));

      bitSet(_ReliableErrors, ReliableACKError);
      bitSet(_ReliableErrors, ReliableTimeout);

      return 0;
    }


    uint8_t SX126X::receiveDTIRQ(uint8_t * header, uint8_t headersize, uint8_t * dataarray, uint8_t datasize, uint16_t networkID, uint32_t rxtimeout, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println();
      _streamRef->println(F(" {RELIABLE} receiveDTIRQ()"));
      _streamRef->print(F(" {RELIABLE} _ReliableConfig "));
      _streamRef->println(_ReliableConfig, HEX);
#endif

      uint16_t index, payloadcrc = 0, RXcrc, RXnetworkID = 0;
      uint8_t regdataL, regdataH;
      uint8_t RXHeaderL;
      uint8_t RXDataL;
      uint8_t buffer[2];
      uint8_t RXstart;

      _ReliableErrors = 0;
      _ReliableFlags = 0;
      setMode(MODE_STDBY_RC);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
      setRx(rxtimeout);

      if (!wait) {
        return 0;  //not wait requested so no packet length to pass
      }

      //0x202   = IRQ_RX_TIMEOUT + IRQ_RX_DONE
      while (!(readIrqStatus() & 0x202))
        ;  //Wait for RX Done IRQ to go high

      setMode(MODE_STDBY_RC);  //ensure to stop further packet reception

      //IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT = 0x0260
      if (readIrqStatus() & 0x0260) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->println(F(" {RELIABLE} Packet error"));
#endif
        return 0;  //packet is errored somewhere so return 0
      }

      RXHeaderL = getByteSXBuffer(2);
      RXDataL = getByteSXBuffer(3);

      readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
      _RXPacketL = buffer[0];
      RXstart = buffer[1];

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} Header  "));
      printSXBufferHEX(0, (RXHeaderL - 1));
      _streamRef->println();
      _streamRef->print(F(" {RELIABLE} Received data payload size "));
      _streamRef->println(RXDataL);
      _streamRef->print(F(" {RELIABLE} Data payload  "));
      printSXBufferHEX(RXHeaderL, RXHeaderL + RXDataL - 1);
      _streamRef->println();
#endif

      if (RXHeaderL > headersize) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} Header size error "));
        _streamRef->println(headersize);
#endif
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      if (RXDataL > datasize) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} Data size error "));
        _streamRef->println(datasize);
#endif
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} _RXPacketL  "));
      _streamRef->println(_RXPacketL);
      _streamRef->print(F(" {RELIABLE} Header  "));
      printSXBufferHEX(0, (RXHeaderL - 1));
      _streamRef->println();
      _streamRef->print(F(" {RELIABLE} Received data payload size "));
      _streamRef->println(RXDataL);
      _streamRef->print(F(" {RELIABLE} Data payload  "));
      printSXBufferHEX(RXHeaderL, RXHeaderL + RXDataL - 1);
      _streamRef->println();
#endif

      if (_RXPacketL < 10)  //check received packet is 10 or more bytes long
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);  //start the burst read
      _spi->transfer(RADIO_READ_BUFFER);
      _spi->transfer(RXstart);
      _spi->transfer(0xFF);

      for (index = 0; index < RXHeaderL; index++) {
        regdataL = _spi->transfer(0);
        header[index] = regdataL;
      }

      for (index = 0; index < RXDataL; index++) {
        regdataL = _spi->transfer(0);
        dataarray[index] = regdataL;
      }

      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);
      RXnetworkID = ((uint16_t)regdataH << 8) + regdataL;
      regdataL = _spi->transfer(0);
      regdataH = _spi->transfer(0);

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      if (!bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = CRCCCITT(dataarray, RXDataL, 0xFFFF);
        RXcrc = ((uint16_t)regdataH << 8) + regdataL;

#ifdef SX126XDEBUGRELIABLE
        _streamRef->println(F(" {RELIABLE} Payload CRC check enabled"));
        _streamRef->print(F(" {RELIABLE} payloadcrc 0x"));
        _streamRef->println(payloadcrc, HEX);
        _streamRef->print(F(" {RELIABLE} RXcrc 0x"));
        _streamRef->println(RXcrc, HEX);
#endif

        if (payloadcrc != RXcrc) {
          bitSet(_ReliableErrors, ReliableCRCError);
#ifdef SX126XDEBUGRELIABLE
          _streamRef->print(F(" {RELIABLE} CRCmissmatch, received 0x"));
          _streamRef->println(RXcrc, HEX);
#endif
        }
      }

#ifdef SX126XDEBUGRELIABLE
      _streamRef->print(F(" {RELIABLE} RXnetworkID 0x"));
      _streamRef->println(RXnetworkID, HEX);
#endif

      if (RXnetworkID != networkID) {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} NetworkID missmatch received 0x"));
        _streamRef->print(RXnetworkID, HEX);
        _streamRef->print(F(" LocalID 0x"));
        _streamRef->println(networkID, HEX);
#endif
        bitSet(_ReliableErrors, ReliableIDError);
      }

      if (_ReliableErrors)  //if there has been a reliable error return a RX fail
      {
#ifdef SX126XDEBUGRELIABLE
        _streamRef->print(F(" {RELIABLE} Reliable errors"));
#endif

        return 0;
      }

      return _RXPacketL;  //return and indicate RX OK.
    }


    uint8_t SX126X::transmitDTIRQ(uint8_t * header, uint8_t headersize, uint8_t * dataarray, uint8_t datasize, uint16_t networkID, uint32_t txtimeout, int8_t txpower, uint8_t wait) {
#ifdef SX126XDEBUGRELIABLE
      _streamRef->println(F(" {RELIABLE} transmitDT() "));
#endif

      uint8_t index, bufferdata;
      uint16_t payloadcrc;

      _ReliableErrors = 0;
      _ReliableFlags = 0;

#ifdef DETECTRELIABLERRORS
      if (datasize > (251 - headersize))  //its 251 because of 4 bytes appended to packet
      {
        bitSet(_ReliableErrors, ReliableSizeError);
        return 0;
      }
#endif
      setMode(MODE_STDBY_RC);
      _TXPacketL = headersize + datasize + 4;

      if (bitRead(_ReliableConfig, NoReliableCRC)) {
        payloadcrc = 0;
      } else {
        payloadcrc = CRCCCITT(dataarray, datasize, 0xFFFF);
      }

      checkBusy();

#ifdef USE_SPI_TRANSACTION
      _spi->beginTransaction(_spiSettings);
#endif

      digitalWrite(_NSS, LOW);
      _spi->transfer(RADIO_WRITE_BUFFER);
      _spi->transfer(0);

      //load up the header
      for (index = 0; index < headersize; index++) {
        bufferdata = header[index];
        _spi->transfer(bufferdata);
      }

      //load up the data array
      for (index = 0; index < datasize; index++) {
        bufferdata = dataarray[index];
        _spi->transfer(bufferdata);
      }

      //append the network ID and payload CRC at end
      _spi->transfer(lowByte(networkID));
      _spi->transfer(highByte(networkID));
      _spi->transfer(lowByte(payloadcrc));
      _spi->transfer(highByte(payloadcrc));

      digitalWrite(_NSS, HIGH);

#ifdef USE_SPI_TRANSACTION
      _spi->endTransaction();
#endif

      setPayloadLength(_TXPacketL);
      setTxParams(txpower, RAMP_TIME);
      setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
      setTx(txtimeout);

      if (!wait) {
        return _TXPacketL;
      }

      while (!(readIrqStatus() & 0x201))
        ;  //wait for IRQs going active

      setMode(MODE_STDBY_RC);  //ensure we leave function with TX off

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }

      if (readIrqStatus() & IRQ_RX_TX_TIMEOUT)  //check for timeout
      {
        return 0;
      } else {
        return _TXPacketL;
      }
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
