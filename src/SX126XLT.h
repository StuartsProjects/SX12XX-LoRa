
#ifndef SX126XLT_h
#define SX126XLT_h

#include "Arduino.h"
#include <SX126XLT_Definitions.h>

/**************************************************************************

  ToDO

  setPAconfig needs to allow for SX1268 - if (TXpower > 14){setPaConfig(0x04, 0x07, _Device)} else { setPaConfig(0x02, 0x01, _Device)}             //hpMax is quoted as 0-7, but 0 seems to be an invalid value

  Check Timeouts for TX and RX
  review setting of TXpower, 17dBm max ?
  Why is it << 8 - timeout = timeout << 8;         //timeout passed in mS, multiply by 64 to convert units of 15.625us to 1mS
  Description of how to include RxGain register in the retention memory, see Section 9.6 - manual p58
  Check that all IRQs are not masked and can be read
  Investigate if setPacketParams(savedPacketParam1, savedPacketParam2 in send routine can be avoided - TXpacketL
  Investigate use of clearDeviceErrors()
  Check recovery from busy timeout error.
  Check rxEnable and txenable are working.
  For FIFO TX & RX Check writeUint8 readUint8 works with characters
  For FIFO TX & RX Check bytes sent for writeInt16 vs writeUInt16
  For FIFO TX & RX Check bytes sent for writeInt32 vs writeUInt32
  Why get this sometimes at startup --- ERROR - Busy Timeout!, check iterations for busy check
  Review readpacketLoRa for (index = RXstart; index < RXend; index++)
  Check in addressed send  txpacketL = 3 + size;  //we have added 3 header bytes to size
  Match printlorasettings and printdevice settings with sx127x library
  Check Single Reception Operating Mode and RXsymbols timeout
  Add a library function for SetRxDutyCycle, or maybe external access to writeCommand
  Check correct setting of optimisation
  Check if SX126X has AGCauto_

**************************************************************************/

class SX126XLT  {
  public:

    SX126XLT();

    bool begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3, int8_t pinSW, uint8_t device);
    void checkBusy();
    void writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void readCommand( uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void writeRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    void writeRegister( uint16_t address, uint8_t value );
    void readRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    uint8_t readRegister( uint16_t address );
    void resetDevice();
    bool checkDevice();
    void setupLoRa(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t modParam4);
    void setMode(uint8_t StdbyConfig);
    void setRegulatorMode(uint8_t mode);

    void setPaConfig(uint8_t dutycycle, uint8_t hpMax, uint8_t device);
    void setDIO3AsTCXOCtrl(uint8_t tcxoVoltage);
    void calibrateDevice(uint8_t devices);
    void calibrateImage(uint32_t freq);
    void setDIO2AsRfSwitchCtrl();
    void setPacketType(uint8_t PacketType);
    void setRfFrequency( uint32_t frequency, int32_t offset );
    void setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4);
    uint8_t returnOptimisation(uint8_t SpreadingFactor, uint8_t Bandwidth);
    uint32_t returnBandwidth(uint8_t BWregvalue);
    float calcSymbolTime(float Bandwidth, uint8_t SpreadingFactor);
    void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void setPacketParams(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5);
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
    void setHighSensitivity();
    void setLowPowerRX();
    void setSyncWord(uint16_t syncword);
    void printLoraSettings();
    void printDevice();
    uint32_t getFreqInt();                     //this reads the SX126x registers to get the current frequency
    uint8_t getLoRaSF();

    uint32_t getLoRaBandwidth();
    uint8_t getLoRaCodingRate();
    uint8_t getOptimisation();

    uint16_t getSyncWord();
    uint8_t getInvertIQ();
    uint16_t getPreamble();
    void printOperatingSettings();
    uint8_t getHeaderMode();
    uint8_t getLNAgain();
    void printRegisters(uint16_t Start, uint16_t End);
    void printASCIIPacket(uint8_t *buff, uint8_t tsize);
    uint8_t transmit(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait);
    void setTxParams(int8_t TXpower, uint8_t RampTime);
    void setTx(uint32_t timeout);
    void clearIrqStatus( uint16_t irq );
    uint16_t readIrqStatus();
    uint16_t CRCCCITT(uint8_t *buffer, uint8_t size, uint16_t start);
    uint8_t receive(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait);
    uint8_t readPacketRSSI();
    uint8_t readPacketSNR();
    uint8_t readRXPacketL();
    void setRx(uint32_t timeout);
    void readPacketReceptionLoRa();

    void printIrqStatus();
    void rxEnable();
    void txEnable();
    bool config();

  private:

    int8_t _NSS, _NRESET, _RFBUSY, _DIO1, _DIO2, _DIO3, _SW;
    int8_t _RXEN, _TXEN;
    uint8_t _RXPacketL;             //length of packet received
    uint8_t _RXPacketType;          //type number of received packet
    uint8_t _RXDestination;         //destination address of received packet
    uint8_t _RXSource;              //source address of received packet
    int8_t  _PacketRSSI;            //RSSI of received packet
    int8_t  _PacketSNR;             //signal to noise ratio of received packet
    int8_t  _TXPacketL;
    uint8_t _RXcount;               //used to keep track of the bytes read from SX126X buffer during readFloat() etc
    uint8_t _TXcount;               //used to keep track of the bytes written to SX126X buffer during writeFloat() etc
    uint8_t _RXBufferPointer;       //pointer to first byte of packet in buffer
    uint8_t _OperatingMode;         //current operating mode
    bool _rxtxpinmode = false;      //set to true if RX and TX pin mode is used.
    uint8_t _Device;                //saved device type
    uint8_t _TXDonePin;             //the pin that will indicate TX done
    uint8_t _RXDonePin;             //the pin that will indicate RX done

    //config variables 36 bytes, allows for device to be reset and reconfigured via confg();
    uint32_t savedFrequency;
    uint32_t savedOffset;
    uint8_t  savedPacketType;
    uint8_t  savedRegulatorMode;


    uint8_t  savedModParam1, savedModParam2, savedModParam3, savedModParam4;
    uint16_t savedPacketParam1;
    uint8_t  savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5;
    uint16_t savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask;
    int8_t   savedTXPower;

};
#endif




