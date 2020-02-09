#ifndef SX128XLT_h
#define SX128XLT_h

#include "Arduino.h"
#include <SX128XLT_Definitions.h>

/**************************************************************************

  ToDO

  Why is SX1280LT.setPacketType(PACKET_TYPE_LORA) required before getFreqInt works (example 1)
  Review error rate in FLRC mode
  Ranging requestor 0dBm in packet receipt mode ?
  Question the frequency error corection in the data sheet, is it really 1600/bandwithkhz ?
  Checkbusy at end of setmode ?
  Syncword for LoRa possible
  Check whether getfreqint() needs packet mode set
  Add routine to change period_base for RX,TX timeout
  Review ranging operation

**************************************************************************/

class SX128XLT  {

  public:

    SX128XLT();

    bool begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, uint8_t device);

    void rxtxInit(int8_t RXEN, int8_t TXEN);
    void rxEnable();
    void txEnable();

    void checkBusy();
    bool config();
    void readRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    uint8_t readRegister( uint16_t address );
    void writeRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    void writeRegister( uint16_t address, uint8_t value );
    void writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void readCommand( uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void resetDevice();
    bool checkDevice();
    void setupLoRa(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3);
    void setMode(uint8_t modeconfig);
    void setRegulatorMode(uint8_t mode);
    void setPacketType(uint8_t PacketType);
    void setRfFrequency( uint32_t frequency, int32_t offset );
    void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3);
    void setPacketParams(uint8_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5, uint8_t packetParam6, uint8_t packetParam7);
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
    void setHighSensitivity();
    void setLowPowerRX();
    void printLoraSettings();
    void printDevice();
    uint32_t getFreqInt();
    uint8_t getLoRaSF();
    uint32_t returnBandwidth(uint8_t data);
    uint8_t getLoRaCodingRate();
    uint8_t getInvertIQ();
    uint16_t getPreamble();
    void printOperatingSettings();
    uint8_t getLNAgain();
    void printRegisters(uint16_t Start, uint16_t End);
    void printASCIIPacket(uint8_t *buff, uint8_t tsize);
    uint8_t transmit(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait);
    void setTxParams(int8_t TXpower, uint8_t RampTime);
    void setTx(uint16_t timeout);
    //void setTx(uint8_t _periodBase, uint16_t _periodBaseCount);
    void clearIrqStatus( uint16_t irq );
    uint16_t readIrqStatus();
    void printIrqStatus();
    uint16_t CRCCCITT(uint8_t *buffer, uint8_t size, uint16_t start);
    uint8_t receive(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait);
    uint8_t readPacketRSSI();
    uint8_t readPacketSNR();
    uint8_t readRXPacketL();
    void setRx(uint16_t timeout);


  private:

    int8_t _NSS, _NRESET, _RFBUSY, _DIO1, _DIO2, _DIO3;
    int8_t _RXEN, _TXEN;
    uint8_t _RXPacketL;             //length of packet received
    uint8_t _RXPacketType;          //type number of received packet
    uint8_t _RXDestination;         //destination address of received packet
    uint8_t _RXSource;              //source address of received packet
    int8_t  _PacketRSSI;            //RSSI of received packet
    int8_t  _PacketSNR;             //signal to noise ratio of received packet
    int8_t  _TXPacketL;             //transmitted packet length
    uint8_t _RXcount;               //used to keep track of the bytes read from SX1280 buffer during readFloat() etc
    uint8_t _TXcount;               //used to keep track of the bytes written to SX1280 buffer during writeFloat() etc
    uint8_t _OperatingMode;         //current operating mode
    bool _rxtxpinmode = false;      //set to true if RX and TX pin mode is used.

    uint8_t _Device;                //saved device type
    uint8_t _TXDonePin;             //the pin that will indicate TX done
    uint8_t _RXDonePin;             //the pin that will indicate RX done
    uint8_t _PERIODBASE = PERIODBASE_01_MS; //set default PERIODBASE

    //config variables are 36 bytes, allows for device to be reset and reconfigured via config();
    uint8_t  savedRegulatorMode;
    uint8_t  savedPacketType;
    uint32_t savedFrequency, savedOffset;
    uint8_t  savedModParam1, savedModParam2, savedModParam3;
    uint8_t  savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5, savedPacketParam6, savedPacketParam7;
    uint16_t savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask;
    int8_t   savedTXPower;

};
#endif
