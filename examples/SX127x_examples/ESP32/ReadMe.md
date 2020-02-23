## ESP32 - SX127X Library Example Programs

Having originally tested the SX127X part of the library on a ATMega328P based micro controller I decided to check that the library was compatible with another popular processor that can be programmed in the Arduino IDE, the ESP32. 

I had developed a PCB that could be used for a small ESP32 based GPS tracker node, initially for use on The Things Network. The board has a GPS, I2C SSD1306 OLED and RFM98 lora device. There are options for a micro SD card, DS18B20 temperature sensor and a I2C FRAM. With it all assembled, the node consumes around 31uA in deep sleep with all devices connected, and the GPS running in backup mode for hot fixing. 


![Picture 1](/pictures/ESP32_Micro_Node.jpg)


**3\_LoRa\_Transmitter** and **4\_LoRa\_Receiver**

These example programs work using the ESP32 Dev Module board type in the Arduino IDE. The pins used were;  NSS 5, NRESET 27, DIO0 35, SCK 18, MISO 19, MOSI 23. Some of the example programs use Atmel specific libraries for functions such as sleep mode and interrupts, these would not be expected to work on the ESP32.

When building a board where the deep sleep current is important, you need to build it in stages, checking the deep sleep current after adding each component. Debugging a high deep sleep current on a fully assembled board can be very difficult. So when building this board I followed the same basic procedures that I do with other boards, start with the power supply and then add parts in stages, using testing programs at each stage to check sleep current. If problems develop with a significant increase in sleep current, it should then be clear which part introduced the problem. 

Many of the ESP32 example programs use added features of the board to reduce sleep current. There is a MOSFET switch to turn off the resistor divider used to measure battery voltage. There is a high side MOSFET switch to remove power from devices such as the LoRa device and micro SD card. Then there is an additional high side MOSFET switch to turn off the power to the GPS. With all these options active the power used in deep sleep, with the GPS in backup mode, is in the order of 31uA.

The example programs include options to disable the MOSFET switching mentioned above, in general if the pin controlling the particular MOSFET is defined as -1, instead of an actual pin number, that MOSFET option is disabled. The segment of code below is an example, if the switch on\off of the LoRa device in not required the option can be disabled by defining BATVREADON as -1. In addition if your hardware is different and does not have the MOSFETS then the sections of code such as below could be removed;

    if (BATVREADON >= 0)
    {
    pinMode(BATVREADON, OUTPUT);
    }


For the majority of the program examples you will need to define the pins used, the frequency and the LoRa settings in the Settings.h file. The default provided settings may not be optimised for long distance. See the 'What is LoRa' document for information on how LoRa settings affect range. 
Some of the examples use sleep mode on the processor and LoRa device to save power. Typical sleep currents may be mentioned in the description of the program. In most all cases a 'bare bones' Arduino has been used to measure the sleep currents and you may not get even close to the quoted figures using standard Arduinos such as Pro Minis or similar. Optimising particular Arduino boards for low power sleep is outside of the scope of this library and examples.  


**Stuart Robinson**

**February 2020**

