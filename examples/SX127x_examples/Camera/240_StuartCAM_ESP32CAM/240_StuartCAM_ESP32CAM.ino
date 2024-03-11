/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 09/10/23

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  Github repository at https://github.com/RuiSantosdotme/ESP32-CAM-Arduino-IDE

  IMPORTANT!!!
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/


/*******************************************************************************************************
  Program Operation - This is a program using the ESP32CAM to take pictures and transmit those pictures via
  LoRa radio to another remote Arduino.

  This program is for an ESP32CAM board that has an SPI LoRa module set up on the following pins; NSS 12,
  NRESET 15, SCK 4, MISO 13, MOSI 2, 3.3V VCC and GND. All other pins on the SX127X are not connected.

  Note that the white LED on pin 4 or the transistor controlling it need to be removed so that the LoRa
  device can properly use pin 4.

  The program wakes up, takes a picture and starts the transfer of the picture (from its memory array in
  PSRAM) with LoRa, more details of the file transfer process will be found here;

  https://stuartsprojects.github.io/2021/09/20/Large-Data-Transfers-with-LoRa-Part3.html

  Note that if the camera fails then the program will attempt to send, and wait for the acknowledge, for a
  DTinfo packet reporting the fail.

  The frame size can be set in configInitCamera() to;

  FRAMESIZE_UXGA (1600 x 1200)
  FRAMESIZE_QVGA (320 x 240)
  FRAMESIZE_CIF (352 x 288)
  FRAMESIZE_VGA (640 x 480)
  FRAMESIZE_SVGA (800 x 600)
  FRAMESIZE_XGA (1024 x 768)
  FRAMESIZE_SXGA (1280 x 1024)

  The image quality (config.jpeg_quality) is a number between 0 and 63. Lower numbers give a higher quality
  image. Low numbers for config.jpeg_quality, in particular at high resolutions, may cause the ESP32-CAM to
  crash or not take images correctly.

  Serial monitor baud rate is set at 115200
*******************************************************************************************************/

#include <Arduino.h>
#include "FS.h"                                  //SD Card ESP32
#include "SD_MMC.h"                              //SD Card ESP32
#include "soc/soc.h"                             //disable brownout problems
#include "soc/rtc_cntl_reg.h"                    //disable brownout problems
#include "driver/rtc_io.h"

#include <SPI.h>
#include <SX127XLT.h>                            //get library here > https://github.com/StuartsProjects/SX12XX-LoRa  
#include <ProgramLT_Definitions.h>
#include "Settings.h"                            //LoRa and program settings 
SX127XLT LoRa;                                   //create a library class instance called LoRa, needed for ARtransferIRQ.h

#define ENABLEMONITOR                            //enable define to see progress messages in ARtransferIRQ.h
#define PRINTSEGMENTNUM                          //enable to print segment numbers as transfer progresses  
#define ENABLEARRAYCRC                           //enable this define to use and show CRCs
//#define DISABLEPAYLOADCRC                      //enable this define if you want to not use packet payload CRC checking
//#define DEBUG                                  //enable this define to show data transfer debug info

RTC_DATA_ATTR int16_t bootCount = 0;             //variables to save in RTC ram
RTC_DATA_ATTR uint16_t sleepcount = 0;
RTC_DATA_ATTR uint16_t pictureNumber = 0;        //number of picture taken, set to 0 on reset

#include "esp_camera.h"
camera_config_t config;                          //stores the camera configuration parameters
#include <ARtransferIRQ.h>                       //library of array transfer functions

bool SDOK;


void loop()
{
  SDOK = false;
  ARDTflags = 0;

  if (initMicroSDCard())                          //need to setup SD card before camera
  {
    Monitorport.println(F("SD Card OK"));
    SDOK = true;
  }
  else
  {
    Monitorport.println(F("****************************"));
    Monitorport.println(F("ERROR - SD Card Mount Failed"));
    Monitorport.println(F("****************************"));
    bitSet(ARDTflags, ARNoFileSave);
  }

  if (!configInitCamera())
  {
    bitSet(ARDTflags, ARNoCamera);                  //set flag bit for no camera working
    Monitorport.println(F("Camera config failed"));
    Monitorport.println(F("Sending DTInfo packet"));
    setupLoRaDevice();
    ARsendDTInfo();
    startSleep();
  }
  else
  {
    if (takePhotoSend(PicturesToTake, PictureDelaymS))
    {
      //picture taken OK
      startSleep();
    }
    else
    {
      //picture take failed
      Monitorport.println("********************************");
      Monitorport.println("ERROR - Take picture send failed");
      Monitorport.println("********************************");
      Monitorport.println();
      Monitorport.println("Sending DTInfo packet");
      Monitorport.flush();
      bitSet(ARDTflags, ARNoCamera);             //set flag bit for no camera working
      setupLoRaDevice();
      ARsendDTInfo();
      startSleep();
    }
  }
  startSleep();
}

void startSleep()
{
  LoRa.setSleep(CONFIGURATION_RETENTION);
  rtc_gpio_hold_en(GPIO_NUM_4);
  rtc_gpio_hold_en(GPIO_NUM_12);                  //hold LoRa device off in sleep
  esp_sleep_enable_timer_wakeup(SleepTimesecs * uS_TO_S_FACTOR);
  Monitorport.print(F("Start Sleep "));
  Monitorport.print(SleepTimesecs);
  Monitorport.println(F("s"));
  Monitorport.flush();
  sleepcount++;
  esp_deep_sleep_start();
  Monitorport.println("This should never be printed !!!");
}



bool initMicroSDCard()
{
  if (!SD_MMC.begin("/sdcard", true))               //use this line for 1 bit mode, pin 2 only, 4,12,13 not used
  {
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE)
  {
    Monitorport.println(F("Unknown SD card type"));
    return false;
  }
  return true;
}


void redFlash(uint16_t flashes, uint16_t ondelaymS, uint16_t offdelaymS)
{
  uint16_t index;

  pinMode(REDLED, OUTPUT);                    //setup pin as output

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(REDLED, LOW);
    delay(ondelaymS);
    digitalWrite(REDLED, HIGH);
    delay(offdelaymS);
  }
  pinMode(REDLED, INPUT);                     //setup pin as input
}

bool setupLoRaDevice()
{
  SPI.begin(SCK, MISO, MOSI, NSS);

  if (LoRa.begin(NSS, NRESET, LORA_DEVICE))
  {
    Monitorport.println(F("LoRa device found"));
  }
  else
  {
    Monitorport.println(F("LoRa Device error"));
    return false;
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  Monitorport.println();
  return true;
}


void setup()
{
  redFlash(4, 125, 125);

  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector
  rtc_gpio_hold_dis(GPIO_NUM_4);
  rtc_gpio_hold_dis(GPIO_NUM_12);             //LoRa NSS back to normal control after sleep

  pinMode(2, INPUT_PULLUP);
  digitalWrite(NSS, HIGH);
  pinMode(NSS, OUTPUT);

  Monitorport.begin(115200);
  Monitorport.println();

  if (bootCount == 0)                         //run this only the first time after programming or power up
  {
    bootCount = bootCount + 1;
  }

  Monitorport.println(F("Awake !"));
  Monitorport.print(F("Bootcount "));
  Monitorport.println(bootCount);
  Monitorport.print(F("Sleepcount "));
  Monitorport.println(sleepcount);

#ifdef DISABLEPAYLOADCRC
  LoRa.setReliableConfig(NoReliableCRC);
#endif

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    Monitorport.println(F("Payload CRC disabled"));
  }
  else
  {
    Monitorport.println(F("Payload CRC enabled"));
  }
}


//***********************************************************************************************
// Start camera Code
//***********************************************************************************************

bool configInitCamera()
{
  Monitorport.println(F("Initialising the camera module "));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;      //YUV422,GRAYSCALE,RGB565,JPEG

  //Select lower framesize if the camera doesn't support PSRAM
  if (psramFound())
  {
    Monitorport.println(F("PSRAM found"));
    config.frame_size = FRAMESIZE_UXGA;      //FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 2;                 //0-63 lower number means higher quality
    config.fb_count = 2;

    // approximate file sizes
    // @ jpeg_quality = 10, SVGA image size = 25K+, UXGA image size = 55K+,
    // @ jpeg_quality = 2, SVGA image size = 60K+, UXGA image size = 160K+,
    // image sizes of circa 200K+ can cause ESP32CAM camera code to crash

  }
  else
  {
    Monitorport.println(F("No PSRAM"));
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);   //Initialize the Camera
  if (err != ESP_OK)
  {
    Monitorport.printf("Camera init failed with error 0x%x", err);
    Monitorport.println();
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 1);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 450);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 0);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  return true;
}


uint16_t  takePhotoSend(uint8_t num, uint32_t gapmS)
{
  uint8_t index = 1;
  char filenamearray[32];
  bool sentOK = false;
  String path;
  camera_fb_t  * fb = esp_camera_fb_get();

  for (index = 1; index <= num; index++)                     //take a number of pictures, send last
  {
    pictureNumber++;
    path = "/pic" + String(pictureNumber) + ".jpg";
    Monitorport.print("Next picture file name ");
    Monitorport.println(path.c_str());

    if (!fb)
    {
      Monitorport.println(F("*****************************"));
      Monitorport.println(F("ERROR - Camera capture failed"));
      Monitorport.println(F("*****************************"));
      delay(1000);
      pictureNumber--;                                       //restore picture number
      bitSet(ARDTflags, ARNoFileSave);
    }

    Monitorport.println(F("Camera capture success"));

#ifdef DEBUG
    Monitorport.print(F("First 8 bytes "));
    printarrayHEX(fb->buf, 0, 8);
    Monitorport.println();
    Monitorport.print(F("Last 8 bytes "));
    printarrayHEX(fb->buf, (fb->len - 8), 8);
    Monitorport.println();
#endif

    if (SDOK)
    {
      Monitorport.println(F("Save picture to SD card"));
      fs::FS &fs = SD_MMC;                            //save picture to microSD card
      File file = fs.open(path.c_str(), FILE_WRITE);
      if (!file)
      {
        Monitorport.println(F("*********************************************"));
        Monitorport.println(F("ERROR Failed to open SD file in writing mode"));
        Monitorport.println(F("*********************************************"));
        bitSet(ARDTflags, ARNoFileSave);
      }
      else
      {
        file.write(fb->buf, fb->len); // payload (image), payload length
        //Monitorport.printf("Saved file to path: %s\r\n ", path.c_str());
        Monitorport.print(F("Saved file "));
        Monitorport.print(path.c_str());
        Monitorport.print(F(" "));
        Monitorport.print(fb->len);
        Monitorport.println(F(" bytes"));
      }
      file.close();
    }
    else
    {
      Monitorport.println(F("***************"));
      Monitorport.println(F("No SD available"));
      Monitorport.println(F("***************"));
    }
  }

  SD_MMC.end();

  if (setupLoRaDevice())
  {
    Monitorport.print(F("Send with LoRa "));
    Monitorport.println(path.c_str());
    uint8_t tempsize = path.length();
    path.toCharArray(filenamearray, tempsize + 1);             //copy file name to the local filenamearray
    filenamearray[tempsize + 1] = 0;                           //ensure there is a null at end of filename in filenamearray
    sentOK = ARsendArray(fb->buf, fb->len, filenamearray, tempsize + 1); //pass array pointer and length across to LoRa send function
  }
  else
  {
    Monitorport.println(F("LoRa device not available"));
  }

  esp_camera_fb_return(fb);                                    //return the frame buffer back to the driver for reuse

  delay(gapmS);

  if (sentOK)
  {
    Monitorport.print(filenamearray);
    Monitorport.println(F(" Sent OK"));
    return pictureNumber;
  }
  else
  {
    Monitorport.print(filenamearray);
    Monitorport.println(F(" Send picture failed"));
  }
  return 0;
}


void printarrayHEX(uint8_t *buff, uint32_t startaddr, uint32_t len)
{
  uint32_t index;
  uint8_t buffdata;

  for (index = startaddr; index < (startaddr + len); index++)
  {
    buffdata = buff[index];
    if (buffdata < 16)
    {
      Monitorport.print(F("0"));
    }
    Monitorport.print(buffdata, HEX);
    Monitorport.print(F(" "));
  }
}
