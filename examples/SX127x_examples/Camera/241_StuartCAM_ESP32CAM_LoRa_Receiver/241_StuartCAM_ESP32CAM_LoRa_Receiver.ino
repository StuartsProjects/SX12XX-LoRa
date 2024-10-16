/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 10/10/23

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation -  This program is for an ESP32CAM board that has an SPI LoRa module set up on the
  following pins; NSS 12, NRESET 15, SCK 4, MISO 13, MOSI 2, 3.3V VCC and GND. All other pins on the
  SX127X are not connected.

  The received pictures are saved to the ESP32CAMs SD card.

  Note that the white LED on pin 4 or the transistor controlling it need to be removed so that the LoRa
  device can properly use pin 4.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>
#include "FS.h"                            //SD Card ESP32
#include "SD_MMC.h"                        //SD Card ESP32
SPIClass sdSPI(HSPI);
#include "soc/soc.h"                       //disable brownout problems
#include "soc/rtc_cntl_reg.h"              //disable brownout problems
#include "driver/rtc_io.h"
#include <SX127XLT.h>                      //SX12XX-LoRa library
#include <ProgramLT_Definitions.h>         //part of SX12XX-LoRa library
#include "Settings.h"                      //LoRa settings etc.

#define ENABLEMONITOR                      //enable this define to monitor data transfer information, needed for ARtransferIRQ.h
#define ENABLEARRAYCRC                     //enable this define to check and print CRC of sent array                   
#define PRINTSEGMENTNUM                    //enable this define to print segment numbers during data transfer
//#define DISABLEPAYLOADCRC                //enable this define if you want to disable payload CRC checking
//#define DEBUG                            //enable more detail of transfer progress


SX127XLT LoRa;                             //create an SX127XLT library instance called LoRa
#include <ARtransferIRQ.h>

uint8_t *PSRAMptr;                         //create a global pointer to the array to send, so all functions have access
uint32_t available_PSRAM;
uint32_t allocated_PSRAM;
bool SDOK;
bool savedtoSDOK;


void loop()
{
  uint32_t arraylength;
  SDOK = false;

  setupLoRaDevice();
  Serial.println(F("LoRa file transfer receiver ready"));

  arraylength = ARreceiveArray(PSRAMptr, allocated_PSRAM, ReceiveTimeoutmS);

  SPI.end();

  digitalWrite(NSS, HIGH);
  digitalWrite(NRESET, HIGH);

  if (arraylength)
  {
    Serial.print(F("Received picture length "));
    Serial.println(arraylength);

    if (initMicroSDCard())
    {
      SDOK = true;
      Serial.println("SD Card OK");
      Serial.print(ARDTfilenamebuff);
      Serial.println(F(" Save picture to SD card"));

      fs::FS &fs = SD_MMC;                                //save picture to microSD card
      File file = fs.open(ARDTfilenamebuff, FILE_WRITE);
      if (!file)
      {
        Serial.println("*********************************************");
        Serial.println("ERROR Failed to open SD file in writing mode");
        Serial.println("*********************************************");
        savedtoSDOK = false;
      }
      else
      {
        file.write(PSRAMptr, arraylength);                // pointer to array and length
        Serial.print(ARDTfilenamebuff);
        Serial.println(" Saved to SD card");
        savedtoSDOK = true;
      }
      file.close();
      SD_MMC.end();
    }
    else
    {
      Serial.println("No SD available");
    }
  }
  else
  {
    Serial.println(F("Error receiving picture"));
    if (ARDTArrayTimeout)
    {
      Serial.println(F("Timeout receiving picture"));
    }
  }
  Serial.println();
}


bool setupLoRaDevice()
{
  SPI.begin(SCK, MISO, MOSI, NSS);

  if (LoRa.begin(NSS, NRESET, LORA_DEVICE))
  {
    Serial.println(F("LoRa device found"));
  }
  else
  {
    Serial.println(F("LoRa Device error"));
    return false;
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

#ifdef DISABLEPAYLOADCRC
  LoRa.setReliableConfig(NoReliableCRC);
#endif

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    Serial.println(F("Payload CRC disabled"));
  }
  else
  {
    Serial.println(F("Payload CRC enabled"));
  }
  return true;
}


bool initMicroSDCard()
{
  if (!SD_MMC.begin("/sdcard", true))               //use this line for 1 bit mode
  {
    Serial.println("*****************************");
    Serial.println("ERROR - SD Card Mount Failed");
    Serial.println("*****************************");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card found");
    return false;
  }
  return true;
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(REDLED, LOW);
    delay(delaymS);
    digitalWrite(REDLED, HIGH);
    delay(delaymS);
  }
}


void setup()
{
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);       //disable brownout detector
  //pinMode(REDLED, OUTPUT);                         //setup pin as output for indicator LED
  //led_Flash(2, 125);                               //two quick LED flashes to indicate program start
  //ARsetDTLED(REDLED);                              //setup LED pin for data transfer indicator

  digitalWrite(NSS, HIGH);
  pinMode(NSS, OUTPUT);                            //disable LoRa device for now

  Serial.begin(115200);
  Serial.println();

  if (psramInit())
  {
    Serial.println("PSRAM is initialised");
    available_PSRAM = ESP.getFreePsram();
    Serial.print("PSRAM available: ");
    Serial.print(available_PSRAM);
    Serial.println(" bytes");
  }
  else
  {
    Serial.println("PSRAM not available");
    Serial.println("Program halted");
    while (1);
  }

  allocated_PSRAM = available_PSRAM / 2;                 //dont use all PSRAM for array

  Serial.print("Allocate ");
  Serial.print(allocated_PSRAM);
  Serial.println(" bytes of PSRAM as receive array");

  uint8_t *byte_array = (uint8_t *) ps_malloc((allocated_PSRAM) * sizeof(uint8_t));
  PSRAMptr = byte_array;                                 //save the pointer to byte_array as global pointer
}
