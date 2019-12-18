/*
  Copyright 2019 - Stuart Robinson
  Licensed under a MIT license displayed at the bottom of this document.
  17/12/19
*/

#ifndef SX127XLT_h
#define SX127XLT_h

#include <Arduino.h>
#include <SX127XLT_Includes.h>

/**************************************************************************

  ToDO
  DONE - Investigate use of clearDeviceErrors() - not possible on SX1276
  DONE - Check rxEnable and txenable are working - not possible - comment out
  ABANDON - Check Single Reception Operating Mode and RXsymbols timeout - not practical for long RX timeouts
  DONE - Check setDioIrqParams(); for use with DIO3,4,5
  DONE - Timeouts for TX and RX
  DONE - Register flags are different to flags set via DIO, eg header valid - Add MSB to IRQflags
  DONE - For FIFO TX & RX Check writeUint8 readUint8 works with characters TX2 and Rx2
  DONE - For FIFO TX & RX Check bytes sent for writeInt16 vs writeUInt16 for positive numbers they are same
  DONE - Investigate LNA boost effect on RX, register 0x0c, RX in
  DONE - Check whether Invert Iq set in LoraWAN mode, downlinks are, uplinks not
  DONE - Add  _DonePin = pinDIO0; to begin
  ABANDON - setBufferBaseAddress(0, 0); in send and read packet routines, base address register read during routines
  DONE - Check that  setStandby(MODE_STDBY_RC); in correct position in TX and RX functions
  DONE - SX127XLT.calibrateDevice(ALLDevices) and SX127XLT.calibrateImage(Frequency); - calibrate image created
  DONE - Check this Frequency,434399968hz - roundign error, all can do is display as float.......
  DONE - Check operation of clearIrqStatus(uint16_t irqMask) 16 bit ?
  DONE - Is a default ramp time needed for setTxParams(txpower, RADIO_RAMP_40_US); - Create RADIO_RAMP_DEFAULT (40uS)
  DONE - review setting of TXpower, 17dBm max ? - working 2dBm to 20dBm
  DONE - All TX funtions to use setTxParams(txpower, RADIO_RAMP_40_US);
  DONE - Check setting of OCP current, default is 100mA - added variable settings to setTXparams
  DONE - Relavent to SX127x ? - SX127XLT.setPaConfig(0x04, HPMAXAUTO, DEVICE_SX1262); - No concept of duty cycle in SX127x
  DONE - test getFrequencyErrorRegValue() with uin8_t varibles for usb,mid,lsb - no difference
  DONE - Why are input pullups needed for DIOs in begin pinMode( _DIO0, INPUT_PULLUP); - Not needed, wrong DIO selected
  DONE - Remove setDioIrqParams from examples - setupLora
  DONE - in  setDioIrqParams() Check  writeRegister(RegIrqFlagsMask, 0x00);                           //we need to be able to read all IRQs
  DONE - Remove print registers from all bar progs 3,4
  DONE - heck if uint16_t getPreamble(); deals with MSB bits
  DONE - Replace the  REG_CRC_AND etc.
  DONE - Check two versions of SX127XLT::packetOK(uint16_t mask)
  DONE - Investigate this on RX www.LïRaTra3ker.uks,RSSI,-61dBm,SNR,4dB,Length,19,Packets,2,Errors,1,IRQreg,50, use setSTBY in receive etc
  ABANDONED - Different names for LT.setHighSensitivity(); and setLowPowerReceive();
  DONE - Change function use of *TXbuffer etc to txbuff to differentiate from SXbuffer
  DONE - Need to add a non-blocking TX and RX (optional) true enables non-blocking mode, false waits for transmission to be completed (default)
  DONE - Check if SX127XLT::setDioIrqParams(uint16_t irqMask, uint8_t dio0Mask, uint8_t dio1Mask, uint8_t dio2Mask) viable
  DONE - Change FIFO functions to Buffer, thats what the Semtech manuals call it, its not a FIFO
  ABANDONED -  Change LDRO_AUTO to 0xFF ?
  DONE - Register print does this on SX127x 0x0  00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00
  DONE - Change transmit and receive functions to not use LoRa, just in case a common routine is possible for FLRC later
  ABANDONED -  Does this receiveSXBuffer(uint32_t rxtimeout, uint8_t wait ) need setMode(MODE_STDBY_RC); regdata = readRegister(REGFIFORXBASEADDR);  writeRegister(REGFIFOADDRPTR, regdata);
  DONE - SX127XLT::setPacketParams(u  regdata = ( (readRegister(RegModemConfig2)) & REG_CRC_AND); incorrect ? - changed to (~READ_HASCRC_AND_X)
  DONE - In setmodulationparams default: bw = 0xFF;  - chnage to 0x00 for default of BW125000
  DONE - Check this writeRegister(RegModemConfig2, (regdata + (packetParam4<<2)); //write out with CRC bit 2 set appropriatly
  ABANDONED - For includes, this is OK for SX126x LORA_CR_4_5 0x01 - No idea, includes can be different
  DONE - Convert all register defines to upercase
  DONE - 4 RX on timeout prints last received packet
  DONE - This is daft; TXPacketL = LT.endWriteSXBuffer(); TXPacketL = LT.transmitSXBuffer(0, TXPacketL, 5000, TXpower, WAIT_TX);   //set a TX timeout of 5000mS
  DONE - Check setMode(MODE_STDBY_RC);  at start of all SXbuffer read write routines..
  DONE - Changes transmitSXBuffer(5000, TXpower, WAIT_TX);   to accept a start and an end
  DONE - Check SX127XLT::setPacketType(uint8_t packettype ) sets moderegister correctly
  DONE - Add link to #include <TimeLib.h> in examples
  DONE - Amend LoRa setup to cope with SX1272, plus getbandwidth etc
  DONE - 11_LoRa_HEX_Print_RX Starting - RX timeout adds to errors, corrected 11,12,20,4,9,15,18,6
  DONE - rationalise use of LT.printLoraSettings(); include device or not ?
  DONE - Create a setRXgain for manual gain control
  DONE - ADD if ( readIrqStatus() != (IRQ_RX_DONE + IRQ_HEADER_VALID) ) to readpacket
  DONE - Search for RXPacketL = LT.readRXPacketL() in programs as this can bypass error checks
  DONE - In prog easy_tracker_RX this means packets with CRC error get by if (RXPacketL == 0)
  DONE - Check use of Serial.print(RXPacketL); when packet errors occur
  DONE - Ensure that bufferless TX\RX printouts match structure TX\RX

  Add ppmoffset to frequency error check program Check this in program 12 LT.writeRegister(RegPpmCorrection,ppmoffset);
  Investigate adding internal SX1278 temperature sensor
  Check sensitivity\current for writeRegister(RegLna, 0x3B );.//at HF 150% LNA current.
  Add packet SF6 support and implicit mode support and examples

**************************************************************************/

class SX127XLT
{

  public:

    SX127XLT();

    bool begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinDIO0, int8_t pinDIO1, int8_t pinDIO2, uint8_t device);
    void resetDevice();
    void setMode(uint8_t modeconfig);         //same function as setStandby()
    void setSleep(uint8_t sleepconfig);
    bool checkDevice();
    void wake();
    void calibrateImage(uint8_t null);
    uint16_t CRCCCITT(uint8_t *buffer, uint8_t size, uint16_t startvalue);
    uint16_t CRCCCITTSX(uint8_t startadd, uint8_t endadd, uint16_t startvalue);

    void setDevice(uint8_t type);
    void printDevice();
    uint8_t getOperatingMode();
    bool isReceiveDone();                      //reads first DIO pin defined
    bool isTransmitDone();                     //reads first DIO pin defined

    void writeRegister( uint8_t address, uint8_t value );
    uint8_t readRegister( uint8_t address );
    void printRegisters(uint16_t start, uint16_t end);
    void printOperatingMode();
    void printOperatingSettings();

    void setTxParams(int8_t txPower, uint8_t rampTime);
    void setPacketParams(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5);
    void setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4);
    void setRfFrequency(uint64_t freq64, int32_t offset);
    uint32_t getFreqInt();
    int32_t getFrequencyErrorRegValue();
    int32_t getFrequencyErrorHz();

    void setTx(uint32_t timeout);
    void setRx(uint32_t timeout);
    bool readTXIRQ();
    bool readRXIRQ();


    void setLowPowerReceive();
    void setHighSensitivity();
    void setRXGain(uint8_t config);

    uint8_t getAGC();
    uint8_t getLNAgain();
    uint8_t getCRCMode();
    uint8_t getHeaderMode();
    uint8_t getLNAboostHF();
    uint8_t getLNAboostLF();
    uint8_t getOpmode();
    uint8_t getPacketMode();           //LoRa or FSK

    uint8_t readRXPacketL();
    uint8_t readPacketRSSI();
    uint8_t readPacketSNR();
    bool readPacketCRCError();
    bool readPacketHeaderValid();
    uint8_t packetOK();
    uint8_t readRXPacketType();
    uint8_t readRXDestination();
    uint8_t readRXSource();

    void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void setPacketType(uint8_t PacketType);

    void clearIrqStatus( uint16_t irqMask );
    uint16_t readIrqStatus();
    void setDioIrqParams(uint16_t irqMask, uint16_t dio0Mask, uint16_t dio1Mask, uint16_t dio2Mask );
    void printIrqStatus();


    void printASCIIPacket(uint8_t *buff, uint8_t tsize);
    void printHEXPacket(uint8_t *buff, uint8_t tsize);
    void printASCIIorHEX(uint8_t temp);
    void printHEXByte(uint8_t temp);
    void printHEXByte0x(uint8_t temp);

    bool isRXdone();
    bool isTXdone();
    bool isRXdoneIRQ();
    bool isTXdoneIRQ();
    void setTXDonePin(uint8_t pin);
    void setRXDonePin(uint8_t pin);

    //*******************************************************************************
    //Packet Read and Write Routines
    //*******************************************************************************

    uint8_t receive(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait);
    uint8_t receiveAddressed(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait);
    uint8_t readPacket(uint8_t *rxbuffer, uint8_t size);                    //needed for when receive used with NO_WAIT
    uint8_t readPacketAddressed(uint8_t *rxbuffer, uint8_t size);           //needed for when receive used with NO_WAIT

    uint8_t transmit(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txPower, uint8_t wait);
    uint8_t transmitAddressed(uint8_t *txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, uint32_t txtimeout, int8_t txpower, uint8_t wait);

    //*******************************************************************************
    //LoRa specific routines
    //*******************************************************************************

    void setupLoRa(uint32_t Frequency, int32_t Offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t modParam4);

    uint32_t getLoRaBandwidth();
    uint8_t getLoRaSF();
    uint8_t getLoRaCodingRate();
    uint8_t getOptimisation();
    uint8_t getSyncWord();
    uint8_t getInvertIQ();
    uint8_t getVersion();
    uint16_t getPreamble();

    uint32_t returnBandwidth(uint8_t BWregvalue);      //returns in hz the current set bandwidth
    uint8_t returnOptimisation(uint8_t SpreadingFactor, uint8_t Bandwidth);       //this returns the required optimisation setting
    float calcSymbolTime(float Bandwidth, uint8_t SpreadingFactor);
    void printLoraSettings();
    void setSyncWord(uint8_t syncword);


    //*******************************************************************************
    //Read Write SX12xxx Buffer commands, this is the buffer internal to the SX12xxxx
    //*******************************************************************************

    uint8_t receiveSXBuffer(uint8_t startaddr, uint32_t rxtimeout, uint8_t wait);
    uint8_t transmitSXBuffer(uint8_t startaddt, uint8_t length, uint32_t txtimeout, int8_t txpower, uint8_t wait);

    void printSXBufferHEX(uint8_t start, uint8_t end);
    void printSXBufferASCII(uint8_t start, uint8_t end);
    void fillSXBuffer(uint8_t startaddress, uint8_t size, uint8_t character);
    uint8_t getByteSXBuffer(uint8_t addr);
    void writeByteSXBuffer(uint8_t addr, uint8_t regdata);

    void startWriteSXBuffer(uint8_t ptr);
    uint8_t endWriteSXBuffer();
    void startReadSXBuffer(uint8_t ptr);
    uint8_t endReadSXBuffer();

    void writeUint8(uint8_t x);
    uint8_t readUint8();

    void writeInt8(int8_t x);
    int8_t readInt8();

    void writeChar(char x);
    char readChar();

    void writeUint16(uint16_t x);
    uint16_t readUint16();

    void writeInt16(int16_t x);
    int16_t readInt16();

    void writeUint32(uint32_t x);
    uint32_t readUint32();

    void writeInt32(int32_t x);
    int32_t readInt32();

    void writeFloat(float x);
    float readFloat();

    void writeBuffer(uint8_t *txbuffer, uint8_t size);
    uint8_t readBuffer(uint8_t *rxbuffer);

    //*******************************************************************************
    //RXTX Switch routines - Not yet tested as of 02/12/19
    //*******************************************************************************

    void rxtxInit(int8_t pinRXEN, int8_t pinTXEN);
    void rxEnable();                //not used on current SX127x modules
    void txEnable();                //not used on current SX127x modules


    //*******************************************************************************
    //Library variables
    //*******************************************************************************

  private:

    int8_t _NSS, _NRESET, _DIO0, _DIO1, _DIO2;
    uint8_t _RXPacketL;             //length of packet received
    uint8_t _RXPacketType;          //type number of received packet
    uint8_t _RXDestination;         //destination address of received packet
    uint8_t _RXSource;              //source address of received packet
    int8_t  _PacketRSSI;            //RSSI of received packet
    int8_t  _PacketSNR;             //signal to noise ratio of received packet
    int8_t  _TXPacketL;             //length of transmitted packet
    uint16_t _IRQmsb;               //for setting additional flags
    uint8_t _Device;                //saved device type
    uint8_t _TXDonePin;             //the pin that will indicate TX done
    uint8_t _RXDonePin;             //the pin that will indicate RX done
    uint8_t _UseCRC;                //when packet parameters set this flag enabled if CRC on packets in use
    int8_t _RXEN, _TXEN;            //not currently used
    uint8_t _PACKET_TYPE;           //used to save the set packet type
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
