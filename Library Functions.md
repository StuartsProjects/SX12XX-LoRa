## Library Functions

All of the library functions are public and can be accessed from users sketches. 

## Transmitting Packets

The basic functions will be described in the order that example program **3\_LoRa\_Transmitter** uses them.

**SPI.begin()**

Standard Arduino library function. Sets up SPI. The library then internally uses;

**SPI.beginTransaction(SPISettings(LTspeedMaximum, LTdataOrder, LTdataMode))**

before every use of the SPI and this function after it;

**SPI.endTransaction()**

The parameters used are LTspeedMaximum, LTdataOrder and LTdataMode are defined in SX127X_Includes.h as;


	#define LTspeedMaximum  8000000
	#define LTdataOrder     MSBFIRST
	#define LTdataMode      SPI_MODE0

The use of SPI.beginTransaction and SPI.endTransaction can be disabled by commenting out this define at the top of the SX127XLT.cpp file;

	#define USE_SPI_TRANSACTION        


**begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE)**

Initialises the hardware pins used by the device. NSS, NRESET and DIO0 are required, DIO1 and DIO2 are optional and if not used define as -1. LoRA\_DEVICE tells the library which actual LoRa RF IC is being used, the choices are;

	#define DEVICE_SX1272
	#define DEVICE_SX1276
	#define DEVICE_SX1277
	#define DEVICE_SX1278
	#define DEVICE_SX1279

**setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation)**

This function first sets the Frequency of operation, the frequency is in hertz as a 32 bit unsigned integer. The actual programmed operating frequency is the sum of Frequency and Offset (also 32 bit integer). If you knew (by calibration) that a particular module has a frequency off set of +5,000hz, then you can correct for it by setting the offset to -5,000hz.

SpreadingFactor, Bandwidth and CodeRate are the LoRa modem parameters and the choices are;

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

The SX126X and SX127X devices have an low data rate optimisation setting that needs to be set when the symbol time is greater than 16mS. You can manually turn it on or off or set it to LDRO_AUTO and the library does the calculation for you

    //Low date rate optimisation, need to be set when symbol time > 16mS
    LDRO_OFF
    LDRO_ON
    LDRO_AUTO       //automatically calculated and set   

**printLoraSettings()**

Prints the current LoRa settings to the serial monitor for diagnostic purposes. The parameters printed are; device\_type, frequency, spreading factor, bandwidth, coding\_rate, syncword, IQ\_Status, preamble\_length.

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

Copy the received packet into the buffer address given with a maximum buffer size. If the RXBUFFER\_SIZE is smaller than the actual received packet then the packet will be truncated. If WAI\T_RX is selected then the program will wait for the time-out period (in mS) for a packet to arrive before signalling a time-out, this is a blocking command. To have the receiver wait continuously for a packet set the timeout to 0. To use the receiver in non-blocking mode set NO\_WAIT in which case you will need to check DIO0 going high to indicate a packet has arrived. 

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


