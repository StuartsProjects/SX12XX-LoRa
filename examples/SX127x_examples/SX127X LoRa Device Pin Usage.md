##SX127X LoRa Device Pin Usage

The LoRa devices this library supports all use a Serial Peripheral Interface (SPI)  for connection. 

In addition to the 3 basic SPI pins, SCK, MOSI and MISO, additional pins are needed for the Arduino controller to be able to transmit and receive data from the SPI device. Some of these pins on the LoRa device can be left not connected. 

**NSS** – Select pin for the LoRa device, needed.

**NRESET** – Reset pin for LoRa device. Has an internal pull-up and can often not be used, just leave it not connected and the LoRa device should clear itself on power up.

**DIO0** – Used by the library to sense RX and TX done, can be programmed for other functions, see device datasheet. Can be not connected if using the library IRQ transmit and receive functions.

**DIO1** – Not currently used by the library so can be left not connected. Can be programmed for some device functions, see device datasheet. Used by some libraries for TTN\LoRaWAN applications.

**DIO2** – Can be connected and used for direct modulation of FSK tones, see library example 80\_Direct\_DIO2\_Tone. Can be programmed for some device functions, see device datasheet. Used by some libraries for TTN\LoRaWAN applications.

**DIO3** – Not currently used by the library so can be left not connected.  Can be programmed for some device functions, see device datasheet

**DIO4** – Not currently used by the library so can be left not connected.  Can be programmed for some device functions, see device datasheet

**DIO5** – Not currently used by the library so can be left not connected.  Can be programmed for some device functions, see device datasheet.

**RX or RXEN** – RX enable. Not currently supported by the SX127X library. Some other LoRa devices, typically SX126x and SX1278X devices, have pins which must be set (normally low) when in receive mode.

**TX or TXEN** – TX enable. Not currently supported by the SX127X library. Some LoRa devices, typically SX126x and SX1278X devices typically have pins which must be set (normally low) when in transmit mode.

Check the datasheet of the LoRa module for the pins you need to use. Check the pinout diagram of the Arduino you are using for the pin numbers of the SCK, MOSI and MISO SPI pins. The SX127x library will use the default SPI pins for the Arduino but needs to know the numbers of the other pins. The library is informed of the pins using the begin() method, below is the command for all supported pins;

**begin(NSS, NRESET, DIO0, DIO1, DIO2, DEVICE);**

Note that DEVICE specifies the type of LoRa device in use and can be one of the following;

For modules that use the PA\_BOOST pin for RF output, legacy library support;

DEVICE\_SX1272
          
DEVICE\_SX1276

DEVICE\_SX1277

DEVICE\_SX1278

DEVICE\_SX1279

For modules that use the RF\_BOOST pin for RF output;

DEVICE\_SX1272\_PABOOST

DEVICE\_SX1276\_PABOOST

DEVICE\_SX1277\_PABOOST

DEVICE\_SX1278\_PABOOST

DEVICE\_SX1279\_PABOOST 

For modules that use the RFO\ LF\_ANT or HF\_ANT pin for RF;

DEVICE\_SX1276\_RFO
 
DEVICE\_SX1277\_RFO

DEVICE\_SX1278\_RFO

DEVICE\_SX1279\_RFO

<br>

You will need to check the datasheet for the LoRa device you have as to the exact type you have, but most common are the types; DEVICE\_SX1272, DEVICE\_SX1276, DEVICE\_SX1277, DEVICE\_SX1278, DEVICE\_SX1279.

As mentioned earlier DIO1 and DIO2 are often not used so the pin allocation and begin() can be;

**begin(NSS, NRESET, DIO0, DEVICE);**

There are also library functions (IRQ) that don't need to use the DIO0 to tell when a packet is sent or received so you can use this begin() and save a pin;

**begin(NSS, NRESET, DEVICE);**

And if you can manage without NRESET you can use this;

**begin(NSS, DEVICE);**

Which means you can operate the LoRa device with the library with just the 3 SPI pins and one other for NSS.

<br>

###March 2024