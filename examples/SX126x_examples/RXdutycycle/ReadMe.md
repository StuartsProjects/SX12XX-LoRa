## Power Reduction Using RXdutycycle

Whilst working on the LoRa remote camera project a node management issue arose. The ESP32S3 has a running current in the 80mA region but its straight forward to save power by having the ESP32S3 go into deep sleep between pictures. The ESP32S3 can then be woken up using the RTC timer, take a picture, save it to SD, send the image using LoRa and finally go back into power saving deep sleep. When sending the image its also possible for the camera to pick up changes to its settings from the receiver or request a specific image for download. In these circumstances, when the ESP32S3 and LoRa device go into deep sleep, the power consumed by the camera board is in the range of 1mA to 1.5mA. 

Such a low power does make battery or battery plus solar operation of the remote camera possible. However if the remote camera deep sleep period is long you may have a long wait (the deep sleep period) if you turn up at a location where you want to receive images from the camera that is high up in a tree or on top of a building. 

Normally when the camera board is put into deep sleep the LoRa device is also put into deep sleep and so it cannot receive packets. What would be useful is to be able to use the image receiver to send a LoRa wake packet that the camera board does receive and this then causes the camera board to wakeup out of deep sleep, then you can remotely access the camera and images. 

Semtech did introduce an RXdutycycle mode first on the SX128X and then on the SX126X. When the LoRa device receiver is put into RXdutycycle mode the receiver part automatically switches between powered receive mode and a very low power sleep mode with the receiver parts off. If the powered receive period is much shorter than the sleep period then the average receive current is much lower. 

## RXdutycycle Settings
  
The required rxPeriod (when the LoRa device is powered and actively listening) and the sleepPeriod (when the LoRa receiver is off) depend on the symbol time which in turn depends on the LoRa spreading factor and bandwidth used. 

The LoRa calculator at [https://www.semtech.com/design-support/lora-calculator](https://www.semtech.com/design-support/lora-calculator "https://www.semtech.com/design-support/lora-calculator") can be used to give you the symbol time for the LoRa settings you are using. 

For an example I decided that the LoRa device would operate at SF7 with a Bandwidth of 125khz. The symbol time is therefore 1.024mS. The rxPeriod, often called the wake period or time, should be a minimum of 9 symbols, so for this example is 9216uS.

The amount of time the LoRa receiver is put to sleep depends on the application, longer sleep times will reduce the average current consumption but the long range settings will slow down how quickly the remote transmitter can wake the receiver due to the much longer preamble used.

To implement a 100:1 reduction in the LoRa devices receive current, the sleep time needs to be 100 times the wake time so needs to be set to 9216uS. The transmitter must send a packet that has a preamble that is longer than the LoRa devices wake and sleep periods combined. The combined preamble time then needs to be 9216uS + 921600uS = 930816uS. At 1024us per symbol that means the preamble needs to be 909 symbols. A couple of extra symbols will compensate for some of the short delays as the LoRa device switches modes. 

Preamble time = waketime + sleeptime so;

Preamble time = 9216uS + 921600uS = 930816uS.

At 1024uS per symbol that means the preamble needs to be;

930816uS/1024us = 909 symbols.

So in this example the LoRa receiver now has the receiver current reduced by a factor of 100, so from circa 7mA to an average of 70uA and the wakeup of the camera from deep sleep will occur about 1 second after the wake packet starts to send.  

The examples of the program just described above are to be found in the library /examples/SX126x_examples/RXdutycycle folder. 

[https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/examples/SX126x_examples/RXdutycycle](https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/examples/SX126x_examples/RXdutycycle "https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/examples/SX126x_examples/RXdutycycle") 

The programs are;

    1_LoRa_Receiver_RXdutycycle_SF7BW125
    2_LoRa_Transmitter_RXdutycycle_SF7BW125

Those examples use light sleep for the ESP32S3 which has the advantage that the the LoRa device setups and SPI bus remain configured during sleeps so its easy to access the LoRa device (and received packet) when the board is woken out of light sleep by reception of a packet.

To send the packet that should wakeup the Receiver, press the Boot button on the Transmitters ESP32S3 Dev board.  

## RXdutycycle Wakeup from ESP32S3 deep sleep.

This second example is based on the requirements of the LoRa remote camera which uses a default long range mode for the initial communications between camera and receiver. Between pictures the ESP32S3 is put into deep sleep to save power. However this means that when using RXdutycycle listen then on the wakeup following a received packet the SPI bus etc is not configured. Therefore in order to read the packet that caused the deep sleep wakeup care needs to be taken when setting up the SPI bus and LoRa device to avoid deleting the packet buffer in the LoRa device. 

The LoRa settings used for the camera board are spreading factor 12 with a bandwidth of 125Khz. The symbol time is therefore 32.768mS. These are long range settings so that there is a good chance the camera board is in range.  

The rxPeriod, often called the wake period or time, should be a minimum of 9 symbols, so for this example is 294912uS.

The planned power saving of receive mode was 15:1, so the average LoRa receive current would drop from circa 7mA to about 0.5mA. To implement a 15:1 reduction in the LoRa device receive current, the sleep time needs to be 15 times the wake time so is set at 4423680uS. The transmitter must send a packet that has a preamble that is longer than the LoRa devices wake and sleep periods combined. The preamble time therefore needs to be;

Preamble time =  waketime + sleeptime so;
  
Preamble time =  294912uS + 4423680uS = 4718592uS. 

At 32768uS per symbol that means the preamble needs to be;

4718592uS/32768us = 144 symbols. 

A couple of extra symbols will compensate for some of the short delays as the LoRa device switches modes. 

When the LoRa receiver detects a packet the LoRa device DIO1 signal wakes up the ESP32S3 from deep sleep and then code initialises the SPI bus and LoRa device in such a way that the packet received is not lost. The packet is read by directly addressing the buffer in the LoRa device and if the 'node number' in the packet sent by the transmitter is the same as that defined in the receiver program the ESP32S3 RGB LED goes green. If there is no node match the RGB LED goes red. The board then goes back into deep sleep with the LoRa receiver again in RX duty cycle mode. 

The receive code for the packets uses reliable packets which have a 16 bit NetworkID and payload CRC automatically appended to the end of the packet. These are checked in the receiver code for a match so the chance of a false packet detect is remote.      

To make the transmitter send the packet to wake the receiver, press the Boot button on the transmitters ESP32S3 Dev board.  

The board current in deep sleep was 1.24mA with the occasional 0.3 second current spikes to 7.5mA. The Average current would then be 1.24 + (7.5/15) = 1.74mA.

The receiver program also shows how a LoRa node can have its LoRa parameters, frequency, spreading factor etc changed by a menu that you can access with a serial terminal program such as TeraTerm. You can also use the serial menu to send the wake packet. 

The programs are;

    3_LoRa_Receiver_RXdutycycle_SF12BW125
    4_LoRa_Transmitter_RXdutycycle_SF12BW125

## Note:
The programs described here require the use of an updated version of the SX126XLT library files, the updated files are now included in the Github repository and are named;

    SX126X.h
    SX126X.cpp
    SX126X_Definitions.h


### Stuart Robinson 
### September 2026


