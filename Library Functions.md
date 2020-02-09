## Library Functions

All of the library functions are public and can be accessed from users sketches. 

## Transmitting Packets

The basic functions will be described in the order that example program **3\_LoRa\_Transmitter** uses them.

**SPI.begin()**

Standard Arduino library function. Sets up SPI. The library then internally uses;

**SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode))**

before every use of the SPI and this function after it;

**SPI.endTransaction()**

The parameters used are LTspeedMaximum, LTdataOrder and LTdataMode are defined in SX127XLT_Definitions.h as;


	LTspeedMaximum  8000000
	LTdataOrder     MSBFIRST
	LTdataMode      SPI_MODE0

The use of SPI.beginTransaction and SPI.endTransaction can be disabled by commenting out this define at the top of the SX127XLT.cpp file;

	#define USE_SPI_TRANSACTION        


**begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA\_DEVICE)**

Initialises the hardware pins used by the device. NSS, NRESET and DIO0 are required, DIO1 and DIO2 are optional and if not used define as -1. LoRA\_DEVICE tells the library which actual LoRa RF IC is being used, the choices are;

	DEVICE_SX1272
	DEVICE_SX1276
	DEVICE_SX1277
	DEVICE_SX1278
	DEVICE_SX1279

**setMode(MODE\_STDBY\_RC)**

Sets the operation mode of the LoRa device. Choices are;

	MODE_SLEEP
	MODE_STDBY
	MODE_STDBY_RC

**setPacketType(PACKET\_TYPE\_LORA)**

Set the type of packet to use, currently only LORA is supported, choices are;

	PACKET_TYPE_GFSK
	PACKET_TYPE_LORA
	PACKET_TYPE_NONE

**setRfFrequency(Frequency, Offset)**

Sets the operating frequency, in hertz. A calibration offset also in hertz can be used if there is a calibration value known for a particular module.

**calibrateImage(0)**

Carries out an internal device calibration, normally carried out after setting the initial operating frequency. 

**setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO)**

Sets the LoRa modem parameters of Spreading factor, Bandwidth, CodeRate and Optimisation. The options are; 

    //LoRa Spreading factors
    LORA_SF6
    LORA_SF7
    LORA_SF8
    LORA_SF9
    LORA_SF10
    LORA_SF11
    LORA_SF12 
    
    //LoRa Bandwidths
    LORA_BW_500  //actual 500000hz
    LORA_BW_250  //actual 250000hz
    LORA_BW_125  //actual 125000hz
    LORA_BW_062  //actual  62500hz 
    LORA_BW_041  //actual  41670hz
    LORA_BW_031  //actual  31250hz 
    LORA_BW_020  //actual  20830hz
    LORA_BW_015  //actual  15630hz
    LORA_BW_010  //actual  10420hz 
    LORA_BW_007  //actual   7810hz

    //LoRa Coding rates
    LORA_CR_4_5  
    LORA_CR_4_6  
    LORA_CR_4_7  
    LORA_CR_4_8

The SX126X and SX127X devices have an low data rate optimisation setting that needs to be set when the symbol time is greater than 16mS. You can manually turn it on or off or set it to LDRO\_AUTO and the library does the calculation for you

    //Low date rate optimisation, need to be set when symbol time > 16mS
    LDRO_OFF
    LDRO_ON
    LDRO_AUTO       //automatically calculated and set 

**setBufferBaseAddress(0x00, 0x00)**

This sets the default location for the locations in the LoRa device buffer where transmitted and received packets start. The defaults of these locations are set in the transmit and receive functions, so this function is not normally required. 
                   
**setPacketParams(PreAmblelength, LORA\_PACKET\_VARIABLE\_LENGTH, PacketLength, LORA\_CRC\_ON, LORA\_IQ\_NORMAL)**

Set the packet parameters. PreAmblelength is normally 8. There is a choise of LORA\_PACKET\_VARIABLE\_LENGTH for variable length explicit packets or LORA\_PACKET\_FIXED\_LENGTH for implicit packets. PacketLength is 1 to 255, it can be set here but is normally handled within the transmitter and receiver functions. There is the option of using a packet CRC with LORA\_CRC\_ON or not using a CRC with LORA\_CRC\_OFF. IQ can be set to LORA\_IQ\_NORMAL or LORA\_IQ\_INVERTED.

**setSyncWord(LORA\_MAC\_PRIVATE\_SYNCWORD)**

You can define the syncword here, either a 8 bit value of your own choice or the standard values of LORA\_MAC\_PRIVATE\_SYNCWORD (0x12) or LORA\_MAC\_PUBLIC\_SYNCWORD (0x34). Take care with setting your own syncwords, some values may not be compatible with other LoRa devices or can give reduced sensitivity.

**setHighSensitivity()**

Sets LoRa device for the highest sensitivity at expense of slightly higher LNA current. The alternative is setLowPowerReceive() for lower sensitivity with slightly lower current. 


**setDioIrqParams(MASK, DIO0\_MASK, DIO1\_MASK, DIO2\_MASK)**

Sets up the how the device responds to internal events. This function is written to match the style used by the SX126X and SX127X devices. MASK is applied to the IRQ settings for DIO0, DIO1 and DIO2, its normally set to IRQ\_RADIO\_ALL (0xFFFF). Whilst the SX127X only has an 8 bit IRQ register the library has extended the function to provide additional IRQ detections that can be found in the SX126X and SX127X. 

In the case of the SX127X, the function maps the internal interrupts to the DIO0, DIO1 and DIO2 pins according to this table;

	IRQ_RADIO_NONE              0x00
	IRQ_CAD_ACTIVITY_DETECTED   0x01       //active on DIO1 
	IRQ_FSHS_CHANGE_CHANNEL     0x02       //active on DIO2 
	IRQ_CAD_DONE                0x04       //active on DIO0 
	IRQ_TX_DONE                 0x08       //active on DIO0 
	IRQ_HEADER_VALID            0x10       //read from IRQ register only
	IRQ_CRC_ERROR               0x20       //read from IRQ register only
	IRQ_RX_DONE                 0x40       //active on DIO0 
	IRQ_RADIO_ALL               0xFFFF

	IRQ_TX_TIMEOUT              0x0100     //so that readIrqstatus can return additional detections 
	IRQ_RX_TIMEOUT              0x0200     //so that readIrqstatus can return additional detections  
	IRQ_NO_PACKET_CRC           0x0400     //so that readIrqstatus can return additional detections 


**setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation)**

As an alternative to setting up the LoRa device with separate functions (as above) you can use this function. The function first sets the Frequency of operation, the frequency is in hertz as a 32 bit unsigned integer. The actual programmed operating frequency is the sum of Frequency and Offset (also 32 bit integer).

SpreadingFactor, Bandwidth and CodeRate are the LoRa modem parameters and the choices are as given for the setModulationParams() described above.

When using setupLoRa() that library function calls the following functions using these defaults;

	setMode(MODE_STDBY_RC)
	setPacketType(PACKET_TYPE_LORA);
	setRfFrequency(Frequency, Offset);
	calibrateImage(0);
	setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);
	setBufferBaseAddress(0x00, 0x00);
	setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
	setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
	LORA_MAC_PUBLIC_SYNCWORD = 0x34
	setHighSensitivity();
	setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);
 

**printModemSettings()**

Prints the current modem settings to the serial monitor for diagnostic purposes. The parameters printed for lora are ; device\_type, frequency, spreading factor, bandwidth, coding\_rate, syncword, IQ\_Status, preamble\_length.

**printOperatingSettings()**

Prints the current operating settings to the serial monitor for diagnostic purposes. The settings printed are; device\_type, version\_number, packet\_mode, header\_mode, packet\_CRC\_on\_off, AGC\_auto\_on\_off, LNA\_gain, LNA\_boostHF, LNA\_boostLF.

**printRegisters(start, end)** 

Print the device registers from start address to end address. Normally prints from 0x00 to 0x4F, max register address is 0x7F. 


**printASCIIPacket(buff, length)**

Print as ASCII characters to the serial monitor the contents of the buffer name given, for the given length.

**transmit(buff, TXPacketL, timeout, TXpower, WAIT_TX)**

Transmit the contents of the buffer name given, for the given length. With a timeout in mS, with a TXpower in dBm and wait for the transmit to complete (a blocking command). To have the LoRa device start transmitting and continue as a no blocking command use NO_WAIT. 

With **transmit** and WAIT\_TX the function returns the packet length if transmit detected no errors and 0 if errors were detected. If NO_WAIT is used you will need to check when pin DIO0 goes high indicating transmission is completed.    

**CRCCCITT(buff, TXPacketL, 0xFFFF)**

Calculates the 16 bit CRC of the buffer given for the length given. Specify the initialise value for the CRC, 0xFFFF in this case. 

**readIrqStatus()** 

Reads the value of the IRQ register flags. 

## Receiving Packets

The **4\_LoRa\_Receiver** sketch is very similar, with the following differences;

**LT.receive(RXBUFFER, RXBUFFER\_SIZE, timeout, WAIT\_RX)**

Copy the received packet into the buffer address given with a maximum buffer size. If the RXBUFFER\_SIZE is smaller than the actual received packet then the packet will be truncated. If WAIT\_RX is selected then the program will wait for the time-out period (in mS) for a packet to arrive before signalling a time-out, this is a blocking command. To have the receiver wait continuously for a packet set the timeout to 0. To use the receiver in non-blocking mode set NO\_WAIT in which case you will need to check DIO0 going high to indicate a packet has arrived. 

**readPacketRSSI()** 

Read the signal strength in dBm of the received packet. 

**readPacketSNR()**

Read the signal to noise ratio in dB of the received packet. Typicall values are +10dB for strong signals and -20dB for reception at the limit.  

**printIrqStatus()**

Prints to the serial monitor the interrupt flags set, they could be;

	IRQ_RADIO_NONE 
	IRQ_CAD_ACTIVITY_DETECTED
	IRQ_FSHS_CHANGE_CHANNEL
	IRQ_CAD_DONE
	IRQ_TX_DONE
	IRQ_HEADER_VALID
	IRQ_CRC_ERROR
	IRQ_RX_DONE
	IRQ_RADIO_ALL

	IRQ_TX_TIMEOUT
	IRQ_RX_TIMEOUT 
	IRQ_NO_PACKET_CRC

The first 8 IRQs listed above are internal to the SX127X, the bottom 3 have been simulated in software to maintain compatibility with the SX126X and SX128X part of the library.

<br>
 
### Stuart Robinson
### December 2019


