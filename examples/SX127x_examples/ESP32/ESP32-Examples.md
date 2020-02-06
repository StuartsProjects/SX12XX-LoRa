## ESP32 - SX127X Library Example Programs

Having originally tested the SX127X part of the library on a ATMega328P based micro controller I decided to check that the library was compatible with another popular processor that can be programmed in the Arduino IDE, the ESP32. 

I had developed a PCB that could be used for a small ESP32 based GPS tracker node, initially for use on The Things Network. The board has a GPS, I2C SSD1306 OLED and RFM98 lora device. There are options for a micro SD card, DS18B20 temperature sensor and a I2C FRAM. With it all assembled, the node consumes around 31uA in deep sleep with all devices connected, and the GPS running in backup mode for hot fixing. 


![Picture 1](/pictures/ESP32_Micro_Node.jpg)


When building a board where the deep sleep current is important, you need to build it in stages, checking the deep sleep current after adding each component. Debugging a high deep sleep current on a fully assembled board can be very difficult. So when building this board I followed the same basic procedures that I do with other boards, start with the power supply and then add parts in stages, using testing programs at each stage to check sleep current. If problems develop with a significant increase in sleep current, it should then be clear which part introduced the problem. 

Many of the ESP32 example programs use added features of the board to reduce sleep current. There is a MOSFET switch to turn off the resistor divider used to measure battery voltage. There is a high side MOSFET switch to remove power from devices such as the LoRa device and micro SD card. Then there is an additional high side MOSFET switch to turn off the power to the GPS. With all these options active the power used in deep sleep, with the GPS in backup mode, is in the order of 31uA.

The example programs include options to disable the MOSFET switching mentioned above, in general if the pin controlling the particular MOSFET is defined as -1, instead of an actual pin number, that MOSFET option is disabled. The segment of code below is an example, if the switch on\off of the LoRa device in not required the option can be disabled by defining BATVREADON as -1. In addition if your hardware is different and does not have the MOSFETS then the sections of code such as below could be removed;

    if (BATVREADON >= 0)
    {
    pinMode(BATVREADON, OUTPUT);
    }


I do not intend to present a fully annotated set of build instructions, but there are brief notes on the build sequence, which test programs were used and the deep sleep current changes as the build progressed. 

1. First add the power supply and regulator components, the battery connector (2 way 0.1" screw terminal), FS1, power switch, Q2, the HCT7833 regulator, C1, C2, C5. Check output voltage is 3.3V, load current should be circa 4uA.
2. Fit the PROG connector (6 way 0.1" pin header), C7, D1, R5, R11, BOOT switch, RESET switch and LED L1. Load the program **1\_LED\_Blink\_ESP32** and check the LED blinks at once per second. To load a program you will need to hold down the BOOT switch and press the RESET switch when the program upload has finished. 
3. Load the program **47\_DeepSleep\_Timed\_Wakeup\_ESP32**, the deep sleep current at this point was around 21uA.
4. To be continued ...............
  

For the majority of the program examples you will need to define the pins used, the frequency and the LoRa settings in the Settings.h file. The default provided settings may not be optimised for long distance. See the 'What is LoRa' document for information on how LoRa settings affect range. 

Some of the examples use sleep mode on the processor and LoRa device to save power. Typical sleep currents may be mentioned in the description of the program. In most all cases a 'bare bones' Arduino has been used to measure the sleep currents and you may not get even close to the quoted figures using standard Arduinos such as Pro Minis or similar. Optimising particular Arduino boards for low power sleep is outside of the scope of this library and examples.  


#### 1\_LED\_Blink &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

This program blinks an LED connected the pin number defined by LED1.The pin 13 LED, fitted to some Arduinos is blinked as well. The blinks should be close to one per second. Messages are sent to the Serial Monitor also.

#### 2\_Register\_Test\_SX1272 &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)
This program is stand alone, it is not necessary to install the SX12XX-LoRa library to use it. 

The program checks that a SX1272 LoRa device can be accessed by doing a test register write and read. If there is no device found a message is printed on the serial monitor. The contents of the registers from 0x00 to 0x7F are printed, there is a copy of a typical printout below. Note that the read back changed frequency may be different to the programmed frequency, there is a rounding error due to the use of floats to calculate the frequency. 

	2_Register_Test_SX1272 Starting
	SX1272 Selected
	LoRa Device found

	Frequency at reset 915000000
	Registers at reset
	Reg0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	0x00  00 01 1A 0B 00 52 E4 C0 00 0F 19 2B 20 08 02 0A 
	0x10  FF 63 15 0B 28 0C 12 47 32 3E 00 00 00 00 00 40 
	0x20  00 00 00 00 05 00 03 93 55 55 55 55 55 55 55 55 
	0x30  90 40 40 00 00 0F 00 00 00 F5 20 82 01 02 80 40 
	0x40  00 00 22 13 0E 5B DB 24 0E 7F 3A 2E 00 03 00 00 
	0x50  00 00 04 23 00 BD 00 09 09 05 84 0B D0 0B D0 32 
	0x60  2B 14 00 00 10 00 00 00 0F E0 00 0C 01 14 25 07 
	0x70  00 5C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 


	Changed Frequency 434099968
	Reg0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	0x00  00 01 1A 0B 00 52 6C 86 66 0F 19 2B 20 08 02 0A 
	0x10  FF 63 15 0B 28 0C 12 47 32 3E 00 00 00 00 00 40 
	0x20  00 00 00 00 05 00 03 93 55 55 55 55 55 55 55 55 
	0x30  90 40 40 00 00 0F 00 00 00 F5 20 82 01 02 80 40 
	0x40  00 00 22 13 0E 5B DB 24 0E 7F 3A 2E 00 03 00 00 
	0x50  00 00 04 23 00 BD 00 09 09 05 84 0B D0 0B D0 32 
	0x60  2B 14 00 00 10 00 00 00 0F E0 00 0C 01 14 25 07 
	0x70  00 5C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 


#### 3\_LoRa\_Transmitter &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)
This is a simple LoRa test transmitter. A packet containing ASCII text is sent according to the frequency and LoRa settings specified in the 'Settings.h' file. The pins to access  the SX127X need to be defined in the 'Settings.h' file also.

The details of the packet sent and any errors are shown on the Serial Monitor, together with the transmit power used, the packet length and the CRC of the packet. The matching receive program, '4\_LoRa\_Receiver' can be used to check the packets are being sent correctly, the frequency and LoRa settings (in Settings.h) must be the same for the Transmit and Receive program. 

Sample Serial Monitor output;

2dBm Packet> {packet contents*}  BytesSent,19  CRC,3882  TransmitTime,54mS  PacketsSent,1

#### 4\_LoRa\_Receiver &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Basics folder)

The program listens for incoming packets using the LoRa settings in the 'Settings.h' file. The pins to access the SX127X need to be defined in the 'Settings.h' file also. There is a printout of the valid packets received, the packet is assumed to be in ASCII printable text, if its not ASCII text characters from 0x20 to 0x7F, expect weird things to happen on the Serial Monitor. The LED will flash for each packet received and the buzzer will sound, if fitted.

Sample serial monitor output;

	1109s  {packet contents}  CRC,3882,RSSI,-69dBm,SNR,10dB,Length,19,Packets,1026,Errors,0,IRQreg,50

If there is a packet error it might look like this, which is showing a CRC error,

	1189s PacketError,RSSI,-111dBm,SNR,-12dB,Length,0,Packets,1126,Errors,1,IRQreg,70,IRQ_HEADER_VALID,IRQ_CRC_ERROR,IRQ_RX_DONE

<br>

#### 5\_LoRa\_TX\_Sleep\_Timed\_Wakeup\_Atmel &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Sleep folder) 

This program tests the sleep mode and register retention of the SX127X in sleep mode, it assumes an Atmel ATMega328P processor is in use. The LoRa settings to use are specified in the 'Settings.h' file.

A packet is sent, containing the text 'Before Device Sleep' and the LoRa device and Atmel processor are put to sleep. The processor watchdog timer should wakeup the processor in 15 seconds (approx) and register values should be retained.  The device then attempts to transmit another packet 'After Device Sleep' without re-loading all the LoRa settings. The receiver should see 'After Device Sleep' for the first packet and 'After Device Sleep' for the second.

Tested on a 'bare bones' ATmega328P board, the current in sleep mode was 6.5uA.


#### 17\_Sensor\_Transmitter &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Sensor folder)

The program transmits a LoRa packet without using a processor buffer, the LoRa devices internal buffer is filled directly with variables. 
  
The sensor used is a BME280. The pressure, humidity, and temperature are read and transmitted. There is also a 16bit value of battery mV (simulated) and and a 8 bit status value at the packet end.

Although the LoRa packet transmitted and received has its own internal CRC error checking, you could still receive packets of the same length from another source. If this valid packet were to be used to recover the sensor values, you could be reading rubbish. To reduce the risk of this, when the packet is transmitted the CRC value of the actual sensor data is calculated and sent out with the packet. This CRC value is read by the receiver and used to check that the received CRC matches the supposed
sensor data in the packet. As an additional check there is some addressing information at the beginning of the packet which is also checked for validity. Thus we can be relatively confident when reading the
received packet that its genuine and from this transmitter. The packet is built and sent in the sendSensorPacket() function, there is a 'highlighted section' where the actual sensor data is added to the packet.

Between readings the LoRa device, BME280 sensor, and Atmel micro controller are put to sleep in units of 8 seconds using the Atmel processor internal watchdog.

The pin definitions, LoRa frequency and LoRa modem settings are in the Settings.h file. The Atmel watchdog timer is a viable option for a very low current sensor node. A bare bones ATmega328P with regulator and LoRa device has a sleep current of 6.6uA, add the LoRa devices and BME280 sensor module and the average sleep current only rises to 6.8uA.

One of these transmitter programs is running on a long term test with a 175mAh battery, to see how long the battery actually lasts.


#### 18\_Sensor\_Receiver &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Sensor folder)

The program receives a LoRa packet without using a processor buffer, the LoRa devices internal buffer is read direct for the received sensor data. 
  
The sensor used in the matching '17\_Sensor\_Transmitter' program is a BME280 and the pressure, humidity, and temperature are being and received. There is also a 16bit value of battery mV and and a 8 bit status value at the end of the packet.

When the program starts, the LoRa device is set-up to set the DIO0 pin high when a packet is received, the Atmel processor is then put to sleep and will wake up when a packet is received. When a packet is received, its printed and assuming the packet is validated, the sensor results are printed to the serial monitor and screen. Between readings the sensor transmitter is put to sleep in units of 8 seconds using the Atmel
processor internal watchdog.

For the sensor data to be accepted as valid the flowing need to match;

The 16bit CRC on the received sensor data must match the CRC value transmitted with the packet. 
The packet must start with a byte that matches the packet type sent, 'Sensor1'
The RXdestination byte in the packet must match this node ID of this receiver node, defined by 'This_Node'

In total that's 16 + 8 + 8  = 32bits of checking, so a 1:4294967296 chance (approx) that an invalid packet is acted on and erroneous values displayed.

The pin definitions, LoRa frequency and LoRa modem settings are in the Settings.h file.

With a standard Arduino Pro Mini and SSD1306 display the current consumption was 20.25mA with the display and 16.6mA without the display. 


#### 23\_Simple\_GPS\_Tracker\_Transmitter &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Tracker folder) 

This program is an example of a basic GPS tracker. The program reads the GPS, waits for an updated fix and transmits location and altitude, number of satellites in view, the HDOP value, the fix time of the GPS and the battery voltage. This transmitter can be also be used to investigate GPS performance. At start-up there should be a couple of seconds of  recognisable text from the GPS printed to the serial monitor. If you see garbage or funny characters its likely the GPS baud rate is wrong. If the transmitter is turned on from cold, the receiver will pick up the cold fix time, which is an indication of GPS performance. The GPS will be powered on for around 4 seconds before the timing of the fix starts. Outside with a good view of the sky most GPSs should produce a fix in around 45 seconds. The number of satellites and HDOP are good indications to how well a GPS is working. 

The program writes direct to the LoRa devices internal buffer, no memory buffer is used.
  
The LoRa settings are configured in the Settings.h file.

The program has the option of using a pin to control the power to the GPS, if the GPS module being used has this feature. To use the option change the define in the Settings.h file; '#define GPSPOWER -1' from -1 to the pin number being used. Also set the GPSONSTATE and GPSOFFSTATE to the appropriate logic levels.


#### 25\_GPS\_Tracker\_Receiver\_with\_Display\_and\_GPS  &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Tracker folder) 

This program is an example of a basic portable GPS tracker receiver. The program receives the location packets from the remote tracker and displays them on an OLED display. The program also reads a local GPS and when that has a fix, will display the distance and direction to the remote tracker.
  
The program writes direct to the LoRa devices internal buffer, no memory buffer is used.

The LoRa settings are configured in the Settings.h file.

The received information is printed to screen in this order top to bottom;

Latitude, Longitude, Altitude, HDOP, GPS Fixtime, Tracker battery mV, Number of received packets, Distance and direction to tracker, if local GPS fix. In addition if there is a recent tracker transmitter GPS fix a 'T' is shown on line 0 right of screen and if there is a recent local (receiver) GPS fix a 'R' is displayed line 1 right of screen.
  
The received information is printed to the Serial Monitor as CSV data in this order;
  
Packet Address information, Latitude, Longitude, Altitude, Satellites in use, HDOP, TX status byte, GPS Fixtime, Tracker battery mV, Number of received packets, Distance and direction to tracker, if local GPS fix. 

The program has the option of using a pin to control the power to the GPS, if the GPS module being used has this feature. To use the option change the define in Settings.h; '#define GPSPOWER -1' from -1 to the pin number being used. Also set the GPSONSTATE and GPSOFFSTATE to the appropriate logic levels.

The program by default uses software serial to read the GPS, you can use hardware serial by commenting out this line in the Settings.h file;

 #define USE\_SOFTSERIAL\_GPS

And then defining the hardware serial port you are using, which defaults to Serial1. 

#### 26\_GPS\_Echo  &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Tracker folder) 

This is a simple program to test a GPS. It reads characters from the GPS using software serial and sends them (echoes) to the IDE serial monitor. If your ever having problems with a GPS (or just think you are) use this program first.

If you get no data displayed on the serial monitor, the most likely cause is that you have the receive data pin into the Arduino (RX) pin connected incorrectly.

If the data displayed on the serial terminal appears to be random text with odd symbols its very likely you have the GPS serial baud rate set incorrectly.

Note that not all pins on all Arduinos will work with software serial, see here;

https://www.arduino.cc/en/Reference/softwareSerial

For a more detailed tutorial on GPS problems see here;

https://github.com/LoRaTracker/GPSTutorial

Serial monitor baud rate is set at 115200.


#### 29\_GPS\_Checker\_With\_Display  &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Tracker folder) 

This program is a GPS checker with a display output. It uses an SSD1306 or SH1106 128x64 I2C OLED display. At start-up the program starts checking the data coming from the GPS for a valid fix. It reads the GPS for 5 seconds and if there is no fix, prints a message on the serial monitor and updates the seconds without a fix on the display. During this time the data coming from the GPS is copied to the serial monitor also.

When the program detects that the GPS has a fix, it prints the Latitude, Longitude, Altitude, Number of satellites in use, the HDOP value, time and date to the serial monitor. If the I2C OLED display is attached that is updated as well. Display is assumed to be on I2C address 0x3C.


#### 30\_I2C\_Scanner &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Diagnostics and Test folder)

The program scans the I2C bus and displays the addresses of any devices found. Useful first check when using I2C devices.


#### 31\_SSD1306\_OLED\_Checker &emsp; &emsp; &emsp;  &emsp; &emsp; &emsp; (Diagnostics and Test folder)

This program is a simple test program for the SSD1306 and SH1106 OLEDs. The program prints a short message on each line, pauses, clears the screen, and starts again. 

OLED address is defined as 0x3C.


### To be continued


February 2020

