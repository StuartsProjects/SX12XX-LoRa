SX128X Library Example Programs

#### 1\_LED\_Blink &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

This program blinks an LED connected the pin number defined by LED1.The pin 13 LED, fitted to some Arduinos is blinked as well. The blinks should be close to one per second. Messages are sent to the Serial Monitor also.

#### 2\_Register\_Test &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

	2_Register_Test Starting
	Reset device
	LoRa Device found
	Reset device
	Registers at reset
	Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	0x900  80 0C 7B 02 20 FA C0 00 00 80 00 00 00 00 00 FF 
	0x910  FF FF 00 00 00 19 00 00 00 19 87 65 43 21 7F FF 
	0x920  FF FF FF 0C 70 37 0A 50 D0 80 00 C0 5F D2 8F 0A 
	0x930  00 C0 00 00 00 24 00 21 28 B0 30 09 1A 59 70 08 
	0x940  58 0B 32 0A 14 24 6A 96 00 18 00 00 00 00 00 00 
	0x950  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x960  00 00 00 00 00 00 00 00 00 00 FF FF FF FF FF FF 
	0x970  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 04 
	0x980  00 0B 18 70 00 00 00 4C 00 F0 64 00 00 00 00 00 
	0x990  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9A0  00 08 EC B8 9D 8A E6 66 06 00 00 00 00 00 00 00 
	0x9B0  00 08 EC B8 9D 8A E6 66 06 00 00 00 00 00 00 00 
	0x9C0  00 16 00 3F E8 01 FF FF FF FF 5E 4D 25 10 55 55 
	0x9D0  55 55 55 55 55 55 55 55 55 55 55 55 55 00 00 00 
	0x9E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
	0x9F0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 


	 Frequency at reset 2495996672hz
	Change Frequency to 2445000000hz
          Frequency now 2444999936hz

	Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	0x900  80 0C 7B 02 20 FA BC 13 C1 80 00 00 00 00 00 61 


Serial monitor baud rate is set at 9600.

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


#### 52\_FLRC\_Transmitter &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

This is a test transmitter for the Fast Long Range Communication (FLRC) mode introduced in the SX128X devices. A packet containing ASCII text is sent according to the frequency and FLRC settings specified in the 'Settings.h' file. The pins to access the SX128X device need to be defined
in the 'Settings.h' file also.

The details of the packet sent and any errors are shown on the Serial Monitor, together with the transmit
power used, the packet length and the CRC of the packet. The matching receive program, '53_FLRC_Receiver'
can be used to check the packets are being sent correctly, the frequency and FLRC settings (in Settings.h)
must be the same for the Transmit and Receive program. Sample Serial Monitor output;

10dBm Packet> {packet contents*}  BytesSent,19  CRC,3882  TransmitTime,54mS  PacketsSent,1

Serial monitor baud rate is set at 9600

#### 53\_FLRC\_Receiver &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

This is a test receiver for the Fast Long Range Communication (FLRC) mode introduced in the SX128X devices. The program listens for incoming packets using the FLRC settings in the 'Settings.h' file. The pins to access the SX128X device need to be defined in the 'Settings.h' file also.

There is a printout of the valid packets received, the packet is assumed to be in ASCII printable text,
if its not ASCII text characters from 0x20 to 0x7F, expect weird things to happen on the Serial Monitor.
The LED will flash for each packet received and the buzzer will sound, if fitted.

Sample serial monitor output;

1109s  {packet contents}  CRC,3882,RSSI,-69dBm,Length,19,Packets,1026,Errors,0,IRQreg,50

If there is a packet error it might look like this, which is showing a CRC error,

1189s -  PacketError,RSSI,-111dBm,Length,0,Packets,1126,Errors,1,IRQreg,70,IRQ_HEADER_VALID,IRQ_CRC_ERROR,IRQ_RX_DONE

Serial monitor baud rate is set at 9600.

