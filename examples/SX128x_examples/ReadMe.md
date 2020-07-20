# SX128X Library
<cr>


This part of the SX12XX library supports the SX1280 and SX1281 2.4Ghz modules.

The objective of the SX12XX library is to allow the same program sketches to be used across the range of UHF lora modules SX126x and SX127x (UHF) as well as the 2.4Ghz SX128x modules. 

###Considerations for pin usage

There is a range of SX128X modules available and they have slightly different pins usage. 

The library only supports the SPI based LoRa modules and these all require that the SPI bus pins, SCK, MOSI and MISO are connected. All modules also need a NSS (chip select pin) and NRESET (reset) pin. All devices need the RFBUSY pin to be used also. 

The basic SX1280 modules from NiceRF and Ebyte were used to test the examples in this library. There may be newer devices out there that have additional features, such as TCXOs, that may require different set-ups or pin usage. 

Of the LoRa devices DIO pins the SX128X library in standard form only uses DIO1. Some SX128x modules have RX and TX enable pins that need to be appropriately activated when receiving or transmitting.

Thus a begin function that initialised all possible permutations of pins would look like this;

begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX\_EN, TX\_EN, LORA\_DEVICE);

Clearly the above begin statement is somewhat cumbersome and could potentially be shortened for the NiceRF SX128X devices to;

begin(NSS, NRESET, RFBUSY, DIO1, LORA\_DEVICE);

And shortened for the Ebyte devices to;

begin(NSS, NRESET, RFBUSY, DIO1, RX\_EN, TX\_EN, LORA\_DEVICE);

Which is a lot more manageable.

You can use the shorter formats of the begin commands can be used in your own programs if you have the appropriate module, NiceRF or Ebyte. 

The full format of begin, see above, is still valid and is used in most of the example programs. If you have written your own programs using the earlier library these programs do not need changing. You could not have used the newer constructs of the begin command (to support the newer devices) in your programs since the newer constructs did not exist in the older version library. 

Accepted its all a bit confusing, but regrettably module manufacturers have different ideas about design. 