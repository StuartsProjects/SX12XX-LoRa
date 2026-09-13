/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 10/09/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - The program demonstrates the use of the LoRa RX duty cycle receive whereby the LoRa
  receiver automatically switches between receive and sleep modes. With the receiver not powered and 
  listening for long periods there are significant power savings and the normal active receive current of
  7mA or so can be reduced to an average of 200uA. This low average current makes it feasible to keep the
  LoRa receiver active, wating for a wake packet, which can be used to bring the ESP32S3 out of sleep mode. 

  The program was tested using an ESP32S3 Camera Dev Board.

  RXdutycycle Settings
  ********************
  The required rxPeriod (when the LoRa device is activly listening) and the sleepPeriod (when the LoRa 
  receive is off) depend on the symbol time which in turn depends on the spreading factor and bandwidth used. 

  The LoRa calculator at https://www.semtech.com/design-support/lora-calculator can be used to give you the 
  symbol time for the settings you are using. This example uses spreading factor 12 with a bandwidth of 
  125Khz. The symbol time is therefore 32.768mS. 

  The rxPeriod, often called the wake period or time, should be a minimum of 9 symbols, so for this example is 
  294912uS. 

  The amount of time the LoRa receiver sleeps depends on the application, longer sleep times will reduce the
  average current consumption but for long range settings will then slow down how quickly the remote transmitter
  can wake the receiver.

  To implement a 15:1 reduction in the LoRa device receive current, the sleep time needs to be 15 times the  
  wake time so needs to be set to 4423680uS.

  The transmitter, see program 4_LoRa_Transmitter_RXdutycycle_SF12BW125, must send a packet that has a preamble
  that is longer than the LoRa devices wake and sleep periods combined. The combined preamble time then needs
  to be 294912uS + 4423680uS = 4718592uS. At 32768us per symbol that means the preamble needs to be 144 symbols.
  A couple of extra symbols will comepensate for some of the short delays as the LoRa device switches modes. 
    
  When the LoRa receiver detects a packet the LoRa device DIO1 signal wakes up the ESP32S3 from deep sleep
  and then code initialises the SPI bus and LoRa device in such a way that the packet received is not lost.
  The packet is read and if the 'node number' in the packet is the same as that defined by this program the
  ESP32S3 RGB LED goes green. If there is no node match the RGB LED goes red. The board then goes back into
  deep sleep with the LoRa receiver again in RX duty cycle mode. 

  The receive code for the packets uses reliable packets which have a 16bit NetworkID and payload CRC
  automatically appended to the end of the packet. These are checked in the receiver code for a match so the
  chance of a false packet detect is remote.      

  The board current in deep sleep was 1.24mA with the ocaisional 0.3 second current spikes to 7.5mA. The 
  Average current would then be 1.24 + (7.5/15) = 1.74mA.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/


#include <SPI.h>     //the LoRa` device is SPI based so load the SPI library
#include <SX126X.h>  //include the appropriate library
SX126X LoRa;         //create a library class instance called LoRa

//hardware pins for ESP32S3 Development board
#define LORA_NSS 14                //select pin on LoRa device
#define LORA_RESET 21              //reset pin on LoRa device
#define LORA_BUSY 47               //SX126X busy pin
#define LORA_DIO1 1                //DIO1 pin on LoRa device, used for RX and TX done
#define LED1 RGB_BUILTIN           //GPIO 97
#define WAKEUP_GPIO GPIO_NUM_1     //LoRa pin for ext interrupt wakeup
#define LORA_DEVICE DEVICE_SX1262  //we need to define the device we are using

#define LORA_MOSI 2   //LoRa MOSI pin
#define LORA_SCK 42   //LoRa SCK pin
#define LORA_MISO 41  //LoRa MISO pin

#define RGB_LED 48  //pin number for RGB (neopixel)
#define BUZZER 20   //buzzer for test purposes, normally used for SCL in camera mode

#include "driver/rtc_io.h"

//Program settings
const uint32_t Frequency = 434000000;       //Frequency of transmissions in hertz
const uint32_t Offset = 0;                  //Offset frequency for calibration purposes
const uint8_t SpreadingFactor = LORA_SF12;  //LoRa spreading factor
const uint8_t Bandwidth = LORA_BW_125;      //LoRa bandwidth
const uint8_t CodeRate = LORA_CR_4_5;       //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;     //Low data rate optimisation setting, normally set to auto
const int8_t TXpower = 10;                  //LoRa transmit power in dBm
const uint32_t TimeToSleep = 86400;         //sleep time in seconds, 24 hours
const uint8_t ThisNodeNumber = 1;           //this node number
const uint16_t NetworkID = 0x3210;          //network ID, needs to match network ID used by sender of wakeup packet
uint16_t LedTimemS = 2500;                  //time the RGB LED is on indication node match status
uint8_t Brightness = 64;                    //RGB LED brightness

//RX duty cycle settings for SF12, BW 125Khz
//each preamble symbol is 32768uS
const uint32_t rxPeriod = 294912;      //9 times symbol time, number of microseconds for RX listen for SF12, BW 125Khz
const uint32_t sleepPeriod = 4423680;  //number of microseconds for RX sleep\wake ratio of 15:1 for SF12, BW 125Khz
const uint16_t PreambleSymbols = 146;  //length of packet preamble in symbols, minimun needs to be rxPeriod + sleepPeriod

#define ATWake 0xF8                //packet type for Wake
#define uS_TO_S_FACTOR 1000000ULL  //conversion factor for micro seconds to seconds

uint8_t RXPacketL;  //stores length of packet received
int8_t PacketRSSI;  //stores RSSI of received packet
int8_t PacketSNR;   //stores signal to noise ratio (SNR) of received packet
uint8_t RXPacketType;
uint8_t RXNodeNumber;
bool LORAWAKEUP = false;
bool TIMERWAKEUP = false;
bool NODEMATCH = false;


void loop() {
  receiver_sleep();
}


void receiver_sleep() {
  Serial.println(F("Set LoRa receiver to listen during sleep"));

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);  //configure frequency and LoRa settings
  LoRa.setPacketParams(PreambleSymbols, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
  LoRa.receiveSXBufferRxDutyCycle(0, NO_WAIT, rxPeriod, sleepPeriod);

  config_ESP32S3_sleep();
  LEDoff();

  LORAWAKEUP = false;
  TIMERWAKEUP = false;
  NODEMATCH = false;

  Serial.print(F("Start deep sleep "));
  Serial.print(TimeToSleep);
  Serial.println(F(" Secs"));
  Serial.println();
  Serial.flush();
  esp_sleep_enable_timer_wakeup(TimeToSleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();

  Serial.println(F("Awake !!!!!!!!!! should not see this"));
}


void config_ESP32S3_sleep() {
  //configuration needed for LoRa pins so it stays in receive mode during sleep
  gpio_hold_en(GPIO_NUM_14);  //hold state of LORA_NSS
  gpio_hold_en(GPIO_NUM_21);  //hold state of LORA_RESET
  gpio_hold_en(GPIO_NUM_1);   //hold state of LORA_DIO1
  rtc_gpio_pullup_dis(WAKEUP_GPIO);
  rtc_gpio_pulldown_en(WAKEUP_GPIO);
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);
}


void config_ESP32S3_awake() {
  //configuration needed for LoRa pins to continue working after sleep
  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH);
  gpio_hold_dis(GPIO_NUM_14);  //hold state of LORA_NSS
  gpio_hold_dis(GPIO_NUM_21);  //hold state of LORA_RESET
  gpio_hold_dis(GPIO_NUM_1);   //hold state of LORA_DIO1
  rtc_gpio_pullup_dis(GPIO_NUM_14);
  rtc_gpio_pullup_dis(GPIO_NUM_21);
  rtc_gpio_pullup_dis(GPIO_NUM_1);
  rtc_gpio_pulldown_dis(GPIO_NUM_14);
  rtc_gpio_pulldown_dis(GPIO_NUM_21);
  rtc_gpio_pulldown_dis(GPIO_NUM_1);
}


void led_Flash(uint16_t flashes, uint16_t delaymS) {
  uint16_t index;

  for (index = 1; index <= flashes; index++) {
    digitalWrite(LED1, HIGH);  //Turn on the LED
    delay(delaymS);
    digitalWrite(LED1, LOW);  //Turn off the LED
    delay(delaymS);
  }
}


void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      LORAWAKEUP = true;
      break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      TIMERWAKEUP = true;
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}


void read_wakeup_packet() {
  //there has been a RTC_IO wakeup from deep sleep, presumably caused by DIO1 on the LoRa module going high indicating a packet has been received
  uint16_t IRQStatus;

  //need to setup SPI and LoRa device
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_NSS, -1, LORA_BUSY, LORA_DIO1, -1, -1, LORA_DEVICE);  //normal LoRa.begin() would reset LoRa device;

  RXPacketL = LoRa.readRXPacketL();    //read the received packet length
  PacketRSSI = LoRa.readPacketRSSI();  //read the received packets RSSI value
  PacketSNR = LoRa.readPacketSNR();    //read the received packets SNR value
  IRQStatus = LoRa.readIrqStatus();    //read the LoRa device IRQ status register

  Serial.print(F("Packet length "));
  Serial.println(RXPacketL);

  Serial.print(F("Packet RSSI "));
  Serial.print(PacketRSSI);
  Serial.println(F("dBm"));

  Serial.print(F("Packet SNR "));
  Serial.print(PacketSNR);
  Serial.println(F("dB"));

  Serial.print(F("IRQStatus 0x"));
  Serial.println(IRQStatus, HEX);

  //check IRQ status for packet errors

  if (IRQStatus & IRQ_CRC_ERROR) {
    Serial.println(F("Packet CRC error"));
    Serial.println();
    Serial.println();
    redLEDon();
    delay(5000);
    return;
  }

  if (IRQStatus & IRQ_HEADER_ERROR) {
    Serial.println(F("Packet Header error"));
    Serial.println();
    Serial.println();
    redLEDon();
    delay(5000);
    return;
  }

  //now read packet buffer on LoRa device directly
  RXPacketType = LoRa.getByteSXBuffer(0);
  Serial.print(F("Packet type 0x"));
  Serial.println(RXPacketType, HEX);

  RXNodeNumber = LoRa.getByteSXBuffer(1);
  Serial.print(F("Node requested "));
  Serial.println(RXNodeNumber);

  Serial.print(F("This Node "));
  Serial.println(ThisNodeNumber);

  Serial.print(F("TXActions 0x"));
  Serial.println(LoRa.getByteSXBuffer(2), HEX);

  //the transmitted packet is a 'reliable' type so check that the CRC of received payload matches CRC appended to packet
  uint16_t RXPayloadCRC = LoRa.CRCCCITTReliable(0, RXPacketL - 5, 0xFFFF);
  Serial.print(F("CRC of received packet payload 0x"));
  Serial.println(RXPayloadCRC, HEX);

  uint16_t TXPayloadCRC = LoRa.getRXPayloadCRC(RXPacketL);  //retrieve the payload CRC from end of received packet
  Serial.print(F("Transmitted payload CRC 0x"));
  Serial.println(TXPayloadCRC, HEX);

  //check the NetworkID appended to packet matches NetworkID in settings.
  uint16_t RXNetworkID = LoRa.getRXNetworkID(RXPacketL);
  Serial.print(F("Received NetworkID 0x"));
  Serial.println(RXNetworkID, HEX);  //retrieve the network ID from end of received packet

  uint16_t TXNetworkID = NetworkID;
  Serial.print(F("RX NetworkID 0x"));
  Serial.println(TXNetworkID, HEX);  //retrieve the network ID from end of received packet

  if ((RXPacketType == ATWake) && (ThisNodeNumber == RXNodeNumber) && (NetworkID == RXNetworkID) && (RXPayloadCRC == TXPayloadCRC)) {
    NODEMATCH = true;
    Serial.println(F("NODEMATCH"));
    greenLEDon();
    delay(LedTimemS);
  } else {
    NODEMATCH = false;
    Serial.println(F("No NODEMATCH on wake"));
    redLEDon();
    delay(LedTimemS);
    Serial.flush();
  }
}


void greenLEDon() {
  rgbLedWrite(RGB_LED, 0, Brightness, 0);  //green for node match on wakeup
}


void redLEDon() {
  rgbLedWrite(RGB_LED, Brightness, 0, 0);  //red for LoRa packet errors
}


void whiteLEDon() {
  rgbLedWrite(RGB_LED, Brightness, Brightness, Brightness);  //white for initial program running
}


void LEDoff() {
  rgbLedWrite(RGB_LED, 0, 0, 0);  //RGB led off
}


void setup() {

  config_ESP32S3_awake();

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(100);

  whiteLEDon();

  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println(__DATE__);
  Serial.println(__FILE__);
  Serial.println(F("Program starting"));

  print_wakeup_reason();

  if (LORAWAKEUP) {
    Serial.println();
    Serial.println(F("******************"));
    Serial.println(F("LoRa packet wakeup"));
    Serial.println(F("******************"));
    Serial.println();
    digitalWrite(BUZZER, HIGH);  //two buzzer beeps for LoRa packet wakeup
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(1000);
    read_wakeup_packet();
  }

  if (TIMERWAKEUP) {
    Serial.println();
    Serial.println(F("************"));
    Serial.println(F("Timer wakeup"));
    Serial.println(F("************"));
    Serial.println();
    delay(2000);
  }

  Serial.println();
  Serial.println(F("Initialise LoRa device"));
  SPI.end();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  if (LoRa.begin(LORA_NSS, LORA_RESET, LORA_BUSY, LORA_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa Device found"));
  } else {
    Serial.println(F("No device responding"));
    while (1)
      ;
  }

  Serial.println(F("Receiver ready"));
  Serial.println();
}
