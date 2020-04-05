## ESP32 - SX126X Library Example Programs

Having originally tested the SX126X part of the library on a ATMega328P based micro controller I decided to check that the library was compatible with another popular processor that can be programmed in the Arduino IDE, the ESP32. 

I had designed a bare bones ESP32 base to which I can plug in via a Mikrobus shield a series of LoRa modules, including the SX1262. The base has options for a micro SD card, DS18B20 temperature sensor and a I2C FRAM. With it all assembled, the node consumes around 31uA in deep sleep with all devices connected. 

![Picture 1](/pictures/ESP32_Shield.jpg)

<br>

The examples in this folder were tested with that shield using the pin connections given in the examples;


	SCK 18                                  //SCK on SPI3
	MISO 19                                 //MISO on SPI3 
	MOSI 23                                 //MOSI on SPI3 
	NSS 5                                   //select pin on LoRa device
	NRESET 27                               //reset pin on LoRa device
	RFBUSY 25                               //busy line
	LED1 2                                  //on board LED, high for on
	DIO1 35                                 //DIO1 pin on LoRa device, used for RX and TX done
	SW -1                                   //SW pin on Dorji devices is used to turn RF switch on\off, set to -1 if not used    

The above pins work on a bare bones ESP32 module, but may not work on some assembled ESP32 modules who may use some of the pins for other purpose.

<br>

### Stuart Robinson

### April 2020

