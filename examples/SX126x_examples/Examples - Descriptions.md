SX126X Library Example Programs

#### 1\_LED\_Blink &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

This program blinks an LED connected the pin number defined by LED1.The pin 13 LED, fitted to some Arduinos is blinked as well. The blinks should be close to one per second. Messages are sent to the Serial Monitor also.

#### 2\_Register\_Test &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)
This program is stand alone, it is not necessary to install the SX12XX-LoRa library to use it.

The program checks that a SX126X LoRa device can be accessed by doing a test register write and read. If there is no device found a message is printed on the serial monitor. The contents of the registers from 0x800 to 0x9FF are printed, there is a copy of a typical printout below. Note that the read back changed frequency may be slightly different to the programmed frequency, there is a rounding error due to the use of floats to calculate the frequency.

The Arduino pin numbers that the NSS and NRESET pins on the LoRa device are connected to must be specified in the hardware definitions section below. The LoRa device type in use, SX1261, SX1262, or SX1268 must be specified also.

Typical printout;

	2_Register_Test Starting
	Reset device
	LoRa Device found
	Reset device
	Registers at reset
	Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	0x800  00 00 00 00 01 07 20 1E 00 10 19 04 0F FF 0F FF 
	0x810  10 00 10 00 10 00 10 00 00 00 00 00 00 00 00 00 
	0x820  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x830  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x840  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x850  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x860  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x870  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x880  03 00 00 5F 10 08 00 00 08 05 00 39 30 00 00 0C 
	0x890  00 00 00 00 00 0F 0A 07 10 00 26 01 01 53 06 07 
	0x8A0  10 00 AA 20 5A 04 F0 00 56 56 54 43 94 20 40 00 
	0x8B0  00 83 11 00 01 04 0A 4C 14 0A 2F 01 6B FF FF 00 
	0x8C0  00 A0 20 00 00 00 AC 00 1C 00 00 AB 05 30 00 00 
	0x8D0  0C 14 14 40 06 00 00 10 C8 00 00 00 00 00 31 39 
	0x8E0  90 39 0C 04 40 20 1C 18 03 00 05 04 03 02 01 01 
	0x8F0  00 00 00 00 30 00 00 00 00 00 00 00 00 00 00 00 
	0x900  30 00 00 00 00 00 00 00 00 00 00 00 24 04 47 04 
	0x910  14 12 12 04 00 03 0A 00 15 35 09 00 02 1F 5F 08 
	0x920  01 04 05 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x930  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x940  00 07 00 03 02 00 10 0E 0D 0C 03 04 03 70 0C 00 
	0x950  00 00 00 04 00 00 00 00 00 00 00 00 00 00 00 00 
	0x960  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x970  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x980  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x990  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9A0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9B0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9C0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9D0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9F0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 


	Frequency at reset 915000000
	Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	0x880  03 00 00 5F 10 08 00 00 08 05 00 39 30 00 00 0C 
	Change Frequency 434100000
	Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	0x880  03 00 00 5F 10 08 00 00 08 05 00 1B 21 99 A0 0C 
	Changed Frequency 434100000


#### 3\_LoRa\_Transmitter &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)
This is a simple LoRa test transmitter. A packet containing ASCII text is sent according to the frequency and LoRa settings specified in the 'Settings.h' file. The pins to access the lora device need to be defined in the 'Settings.h' file also.

The details of the packet sent and any errors are shown on the Serial Monitor, together with the transmit power used, the packet length and the CRC of the packet. The matching receive program, '4_LoRa_Receive'   can be used to check the packets are being sent correctly, the frequency and LoRa settings (in Settings.h) must be the same for the Transmit and Receive program. Sample Serial Monitor output;

	10dBm Packet> {packet contents*}  BytesSent,19  CRC,3882  TransmitTime,54mS  PacketsSent,1

Serial monitor baud rate is set at 9600


#### 4\_LoRa\_Receiver &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

The program listens for incoming packets using the LoRa settings in the 'Settings.h' file. The pins to access the lora device need to be defined in the 'Settings.h' file also. There is a printout of the valid packets received, the packet is assumed to be in ASCII printable text, if its not ASCII text characters from 0x20 to 0x7F, expect weird things to happen on the Serial Monitor. The LED will flash for each packet received and the buzzer will sound, if fitted.

Sample serial monitor output;

	1109s  {packet contents}  CRC,3882,RSSI,-69dBm,SNR,10dB,Length,19,Packets,1026,Errors,0,IRQreg,50

If there is a packet error it might look like this, which is showing a CRC error,

	1189s PacketError,RSSI,-111dBm,SNR,-12dB,Length,0,Packets,1126,Errors,1,IRQreg,70,IRQ_HEADER_VALID,IRQ_CRC_ERROR,IRQ_RX_DONE

<br>



#### 14\_LoRa\_Structure\_TX &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

This program demonstrates the transmitting of a structure as a LoRa packet. The contents of the structure are the same as in the **8\_LoRa\_LowMemory\_TX** program. The packet sent is typical of what might be sent from a GPS tracker. The structure type is defined as trackerPacket and an instance called location1 is created. The structure which includes a character array (text) is filled with values and transmitted.

The matching receiving program '15_LoRa_RX_Structure' can be used to  receive and display the packet, though the program '9_LoRa_LowMemory_RX' should receive it as well, since the contents are the same.

Note that the structure definition and variable order (including the buffer size) used in the transmitter need to match those used in the receiver. 

The contents of the packet transmitted should be;
  
"LoRaTracker1" (buffer)      - trackerID  
1+             (uint32\_t)   - packet count     
51.23456       (float)       - latitude   
-3.12345       (float)       - longitude  
199            (uint16\_t)   - altitude  
8              (uint8\_t)    - number of satellites   
3999           (uint16\_t)   - battery voltage 
-9             (int8_t)      - temperature

#### 15\_LoRa\_Structure\_RX &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)


This program demonstrates the receiving of a structure as a LoRa packet. The packet sent is typical of what might be sent from a GPS tracker.

The structure type is defined as trackerPacket and an instance called location1 is created. The structure includes a character array (text).

The matching receiving program is **15\_LoRa\_RX\_Structure** can be used to receive and display the packet, though the program **9\_LoRa\_LowMemory\_RX** should receive it as well, since the packet contents are the same.

Not that the structure definition and variable order (including the buffer size) used in the transmitter need to match those used in the receiver. Good luck.

The contents of the packet received, and printed to serial monitor, should be;
  
"LoRaTracker1" (buffer)      - trackerID  
1+             (uint32\_t)   - packet count     
51.23456       (float)       - latitude   
-3.12345       (float)       - longitude  
199            (uint16\_t)   - altitude  
8              (uint8\_t)    - number of satellites   
3999           (uint16\_t)   - battery voltage 
-9             (int8_t)      - temperature
