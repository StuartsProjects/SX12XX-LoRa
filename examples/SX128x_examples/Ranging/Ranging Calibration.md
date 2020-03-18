## Ranging Calibration


The ranging function needs a calibration value which changes according to the spreading factor used, SF5 to SF10 is supported, and the hardware in use. The calibration value will allso vary between each individual module and type of module. Best to label each module and record the calibration value for each module.  

There is a fixed time that it takes the ranging packet exchange to start, be transmitted, received by the remote device the reply sent and the reply processed. This fixed time is static and should be the same no matter how far apart the initiation and receiver are. 

This fixed time needs to be subtracted from the total round trip time. The remainder is then proportional to the distance in a linear way.  

There is a calibration method described in a Semtech document;

[https://www.semtech.com/uploads/documents/introduction_to_ranging_sx1280.pdf](https://www.semtech.com/uploads/documents/introduction_to_ranging_sx1280.pdf "Introduction_to_ranging_sx1280.pdf")

My own approach to the ranging was simplified, and so far I have only used the ranging at Spreading factor 10 and bandwidth 406khz. The procedure I followed would need to be used if other LoRa parameters are used. 

For the ranging programs in the \Examples folder the settings.h file has this definition for the ranging calibration value;

**\#define CalibrationSF10BW400 10120   //calibration value for ranging, SF10, BW400**

The CalibrationSF10BW400 value of 10120 is then used by the ranging calculation. This value was determined by running setting up one Arduino node with the 'SX1280LT_Ranging_Receiver' program and then loading the program 'SX1280LT_Ranging_Calibration_Checker' onto another Arduino node.

The Calibration checker output can be observed in the Arduino IDE serial monitor. Place the two nodes about 2M or so apart and observer the output of the Calibration checker. It has a start Calibration value and an end calibration value;

CalibrationStart = 9700
CalibrationEnd = 10700;

The program carries out a ranging operation and reports the result as the distance on the serial monitor. At some point, as the calibration value changes, the reported distance will be close to 2m and you can then use that value as the calibration value for your setup. See the example output from the calibration program below;

    ResetDevice,0dBm,Valid,Irq,200,RAW,11,Calibration,9990,Distance,3.4m,Time,170mS,Valid,50,Errors,0
    ResetDevice,0dBm,Valid,Irq,200,RAW,19,Calibration,10000,Distance,5.0m,Time,168mS,Valid,51,Errors,0
    ResetDevice,0dBm,Valid,Irq,200,RAW,7,Calibration,10010,Distance,1.4m,Time,168mS,Valid,52,Errors,0
    ResetDevice,0dBm,Valid,Irq,200,RAW,800003,Calibration,10020,Distance,0.0m,Time,168mS,Valid,53,Errors,0
    ResetDevice,0dBm,Valid,Irq,200,RAW,800006,Calibration,10030,Distance,0.0m,Time,170mS,Valid,54,Errors,0
    ResetDevice,0dBm,Valid,Irq,200,RAW,80000C,Calibration,10040,Distance,0.0m,Time,170mS,Valid,55,Errors,0

Here a calibration value of around 10020 is about right. 

The ranging programs transmitter program (SX1280LT_Ranging_Requester) initiates and calculates the distance and takes the measured value and adjusts it with this value;

**const float distance_adjustment = 0.8967;        //adjustment to calculated distance** 

The value, 0.8967 in this case was determined by using the ranging programs to measure the distance over a known path, in my case a path that was measured, via Google maps, to be 4.42km.




### Stuart Robinson
### July 2019