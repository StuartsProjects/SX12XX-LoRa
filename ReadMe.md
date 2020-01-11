# SX12XX Library

This library currently supports the SX1272, SX1276, SX1277, SX1278 and SX1279 Semtech LoRa devices. These Semtech devices are used to manufacture a range of LoRa modules sold by companies such as Hope, Dorji, NiceRF and others.
At a later date the library will be extended to support the SX1261,SX1262, SX1268, SX1280 and SX1281 devices whilst using the same sketch function style used here. 

The objective of the library was to allow the same program sketches to be used across the range of SX126x, SX127x and SX128x modules. A sketch written for the SX1278 (for example) should then run with very minor changes on the SX1262 or SX1280. However, whilst the SX126x and SX128x modules use the same style of device programming, the SX127x programming is completely different. The function style used for the SX126x and SX128x devices has been copied to create a matching style for the SX127x. 
### Warning
**The Semtech devices that this library supports are all 3.3V logic level devices so do not use directly with 5V logic level Arduinos, some form of logic level conversion is needed.** There are no specific logic level converters I could recommend. The programs have only been tested on 3.3V 8Mhz ATMega328P and ATMega1284P processors.

### Library installation

To install the library select the 'Clone or download' button on the main Github page, then select 'Download Zip'. In the Arduino IDE select 'Sketch' then 'Include Library'. Next select 'Add .ZIP library' and browse to and select the ZIP file you downloaded, it's called 'SX12xx-master.zip'.


### Library Functions

For details on how to use the library functions see the document; 'Library Functions'


### Testing
For testing the library and the example programs I used a board of my own design, it uses a 3.3V/8Mhz Arduino Pro Mini which is soldered with a minimum amount of other components onto a board to which you can plug in a LoRa device as a Mikrobus style module. The board is small enough to be used for a GPS tracker application using the connections for a GPS and display as shown in the picture. The Pro Mini used includes a supply reverse protection diode and a fuse, so the board does not need these components.
<br>
  
All example programs were checked against version 1.8.10 of the Arduino IDE, and the latest copies of any external libraries, as of 16/12/19. The operating system was Windows 10. 

### Program examples

The Examples folder contains a number of practical working applications. There is an example for a very low sleep current sensor transmitter and matching receiver. There are examples for remote control of outputs and servos. There is a GPS tracker transmitter and receiver application. These applications utilise LoRa for the communications so even at low powers they can operate over several kilometres. 

There are additional program examples for testing testing devices, antennas and long distance links.

There are demonstrations on how to send data as a plain character array, as a structure and by writing variables direct to the LoRa device.  

For most of the example programs the pin connection definitions are in the Settings.h file that is in the sketches folder. These pins need to be defined and connected, NSS, NRESET and DIO0. In addition the normal SPI pins, SCK, MISO and MOSI have to be connected also (these are pins 13, 12 and 11 respectively on an Arduino Pro Mini). The LoRa devices DIO1 and DIO2 pins are not used directly by the library. Such unused  pins should be defined as -1 in the Settings.h file. 
<br>

The Settings.h file also contains the settings for the LoRa device such as frequency, spreading factor, bandwidth and coding rate.  The example programs use a frequency of 434.000Mhz, you will need to check if that frequency is permitted in your part of the World. The radio frequency spectrum is not a free for all, what frequencies, transmitter powers and duty cycles you are permitted to use varies by region and country. By default CRC checking is added to transmitted packets and used to check for errors on receipt.


The example programs contain a mix of test and functional examples. There are programs for transmitting and receiving packets, as a simple buffer, using structures and a method that directly addresses the SX127x internal buffer. 

There are example applications of sending and receiving data from sensors (BME280) and remote control of LEDs or other outputs. There is a functional GPS tracker transmitter and receiver example. With an example program written and tested on this SX127x library the example should work with some minor changes with the SX126x and SX128x devices when support for those devices has been added to the library. Many of the example programs have already been tested and are working on SX126x, conversion typically takes less than a minute.  

The first program to test your layout and connections would be the Example program in the Basics folder **2\_Register_Test**, this just does a simple register print of the LoRa device. If this program does not work, then the rest of the example programs wont either.

This library is in its first revision, there are still some issues to attend to and changes to be made, see the section 'Changes Required to Library' at the bottom of this document. 

### Compatibility

Full tested on 3.3V 8Mhz ATMega328P and ATMega1284P only.
 
It was not the intention to specifically support non-Atmel platforms with the library, but the following programs have been tested and work on an ESP32 WROOM board;

**3\_LoRa\_Transmitter** and **4\_LoRa\_Receiver**

Using the ESP32 Dev Module board in the Arduino IDE. The pins used were;  NSS 5, NRESET 27, DIO0 35, SCK 18, MISO 19, MOSI 23. Some of the example programs use Atmel specific libraries for functions such as sleep mode and interrupts, these would not be expected to work on the ESP32. 


### Support
The examples do work, so if for you they do not, assume there is a problem with how you have wired the modules or that your modules are faulty or that your Arduino set-up is faulty or unsupported. Please do not ask for basic level support on how to connect the SX127x devices to a particular Arduino or other device, I just don't have the time to do help with this. 

If you find a bug, or other error in the SX12xx library or examples, please let me know.


### Future Changes and Enhancements to Library

Add ppmoffset to frequency error check program Check this in program 12 LT.writeRegister(RegPpmCorrection,ppmoffset)

Investigate adding internal SX1278 temperature sensor

Check sensitivity\current for writeRegister(RegLna, 0x3B );.//at HF 150% LNA current.

Add packet SF6 support and implicit mode support and examples

<br>


### Stuart Robinson

### December 2019

