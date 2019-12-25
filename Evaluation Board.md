# Evaluation Board

To test the example programs in the SX12XX library I developed a small Pro Mini based board that could be used for general purpose LoRa based sensor and tracking applications. The ‘Easy Mikrobus Pro Mini’ board uses a minimum number of parts and can be easily built in its basic form, using just a few standard wired components and pin headers. 
The board takes a single Mikrobus compatible plug in board in addition to plug in boards that  LoRaTracker produce which extend the pin out to 10 pins either side, this is a great help when using LoRa modules. The parts needed for a basic build ‘Easy Mikrobus Pro Mini’ are shown below;


![Picture 1](Pictures/Easy_Mikrobus_Parts.jpg)

<br>

The base boards can be used for most of the current LoRa modules just by plugging the appropriate on in, shown below (left to right) are plug in modules for SX1276 (Dorji), SX1272 (Dorji), RFM98 (Hope), SX1262 (NiceRF), SX12768 (Dorji) and SX1280 (NiceRF). 

<br>

![Picture 1](Pictures/LoRa_Mikrobus_Modules.jpg)

<br>

When assembled the boards looks like this;



![Picture 1](Pictures/Easy_Mikrobus_Assembled.jpg)

<br>

There are 10 way 0.1” header sockets on the long edge and 2 x 8 way connectors on the bottom edge. The Pro Mini used is an 3.3V 8Mhz commonly advertised at low cost on sites such as eBay. These Pro Minis have on the RAW pin power input a reverse protection diode and a fuse. 

One of the example program is an application that reads a BME280 sensor and transmits the values to a remote receiver which shows the sensor values on an OLED display. The two boards used the test the sensor application are shown below, with the LoRa modules plugged in;

<br>

![Picture 1](Pictures/Easy_Mikrobus_Sensor.jpg)

<br>
 
On of the other example programs is a GPS tracker, this reads a GPS and uses LoRa to transmit the location of the tracker to a remote receiver, the receiver shown (program to be released soon) has its own GPS and is able to calculate the distance to and direction of the remote transmitter see below;

<br>

![Picture 1](Pictures/Easy_Mikrobus_GPS_Tracker.jpg)

<br>


The GPS tracker transmitter and receiver are quite small and suitable for portable operations such a trackers for ‘lost’ radio controlled models and similar. The advantage of using the Mikrobus modules is that its easy to change the board between the range of LoRa modules and makes testing easier too. 

Its true that you could make smaller boards for trackers and similar, but this inevitably involves using surface mount component which results in far more complex assembly. The Easy Mikrobus Pro Mini board needs only minimal soldering skills to assemble, its very low cost too.
 
The  Easy Mikrobus Pro Mini board can be fitted with additional surface mount components that can;

Turn off the power to connected devices

Provide non-Volatile FRAM storage

Turn off the resistor divider that measures battery voltage to save power

Add a DS18B20 temperature sensor.

However,  do appreciate that the example programs should be usable with other Arduinos, you don't need to use the boards shown above, also remember that the LoRa modules are 3.3V logic devices, so do not connect directly to 5V Arduinos, some form of logic level conversion is needed. 



<br>
### Stuart Robinson

### December 2019

