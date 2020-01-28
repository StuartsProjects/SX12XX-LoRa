The Examples folder contains a series of test programs and applications that demonstrate how to use the library with the SX127x LoRa devices. 

A common feature of the programs is the use of a series of definitions in the Settings.h file found in the Sketch folder. These defines determine the frequency used and LoRa parameters such as the bandwidth, spreading factor, coding rate and frequency used, for example;


    //*******  Setup LoRa Parameters Here ! ***************

    //LoRa Modem Parameters
    const uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
    const uint32_t Offset = 0;                      //offset frequency for calibration purposes

    const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
    const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
    const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
    const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

    const int8_t TXpower = 10;                      //LoRa transmit power in dBm

    const uint16_t packet_delay = 1000;             //mS delay between packets`


The full range of possible settings is within the 'SX127XLT_Definitions.h' file. A summary of the relevant LoRa modem settings is below.  


<br> 

### LoRa Modem Settings

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

    //Low date rate optimisation, need to be set when symbol time > 16mS
    LDRO_OFF
    LDRO_ON
    LDRO_AUTO       //automatically calculated and set   


<br>
<br>


## Packet Addressing

LoRa is a two way technology, each device is a transceiver. Most often on a particular frequency there will be one transmitter and one receiver. However, this may not always be the case and there could be several nodes in use on the same frequency. 

In order to keep the software simple and allow for the receipt of signals from multiple receivers or directed commands to a particular node, a basic addressing scheme can be used and is implemented by some example programs, see '17_Sensor_Transmitter' for an example. There are library routines to send and receive packets in addressed and non-addressed format so you choose which to send. When using addressed mode regardless of the data content of the actual payload each packet sent has 3 control bytes at the beginning of the packet. In the case of the sensor example mentioned above, the use of the addressing allows the receiver to know from which sensor transmitter the packet came. 

In general the control bytes have been restricted to ASCII printable characters so that they can be shown directly on a terminal monitor. The 3 bytes are;

**Packet type**. This either describes the content of the packet, which could be a GPS location payload or is a command to do something and there is no payload. Details of the packet types defined are in the library file 'ProgramLT_Definitions.h'

**Packet Destination**. The node number that the packet is destined for.

**Packet Source**. The node number that the packet was sent from.

The destination and source packet bytes mean that node ‘2’ (could be your base station receiver) can send a command that only station ‘3’ will respond to. This command could be a reset command, a request to turn on and off certain transmission types etc. Node ‘3’ can be set-up so that it only accepts commands from a particular node.

In addressed mode the 3 control bytes are automatically stripped from each received packet.

An example of the 3 control bytes from a tracker would be;

T*2

Which means there is a test packet (T) its been sent as a broadcast (*) and its from node 2.

For the example programs set the serial monitor baud rate to 9600 baud. As summary list of the example programs is below, they are numbered to make them easier to refer too.  

Enjoy. 
<br>
<br>
### Stuart Robinson
### December 2019


