/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 10/09/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - The program is a demonstration of the power saving that can be achieved by using the 
  LoRa devices RXdutycycle mode. The program is for an ESP32S3 Dev Board.
  
  The program was first tested by putting the LoRa device into continuous RX listen mode. Then the program
  was run with the ESP32S3 in light sleep and the LoRa device in continuous RX listen mode. Finally with
  ESP32S3 in light sleep mode LoRa device was put in RX duty cycle mode. 
  The current consumption in each mode was;

  Current with ESP232S3 and LoRa device in continuous RX listen mode 72.3mA.
  Current with ESP232S3 in light sleep and LoRa device in continuous RX listen mode 9.7mA.
  Current with ESP232S3 in light sleep and LoRa device in RXdutycycle mode 3.2mA.
  
  For the RXdutycycle mode the transmitter program 2_LoRa_Transmitter_RXdutycycle_SF7BW125 has an extended
  preamble of 909 symbols to ensure that the preamble is longer than the sleepPeriod + rxPeriod used in
  this receiver. 
  
  Sample serial monitor output;
    
  310s Packet received > LoRa00004,RSSI,-10dBm,SNR,13dB,Length,10,Packets,25,Errors,0,IRQreg,16
  
  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"

#include <SPI.h>     //the LoRa` device is SPI based so load the SPI library
#include <SX126X.h>  //include the library which supports RXdutycycle

SX126X LoRa;  //create a library class instance called LoRa

//hardware pins for ESP32S3 Development board
#define LORA_NSS 14                //select pin on LoRa device
#define LORA_RESET 21              //reset pin on LoRa device
#define LORA_BUSY 47               //SX126X busy pin
#define LORA_DIO1 1                //DIO1 pin on LoRa device, used for RX and TX done
#define LED1 RGB_BUILTIN           //GPIO 97
#define WAKEUP_GPIO GPIO_NUM_1     //LoRa pin for external interrupt wakeup
#define LORA_DEVICE DEVICE_SX1262  //we need to define the device we are using

#define LORA_MOSI 2   //LoRa MOSI pin
#define LORA_SCK 42   //LoRa SCK pin
#define LORA_MISO 41  //LoRa MISO pin

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;      //Frequency of transmissions in hertz
const uint32_t Offset = 0;                 //Offset frequency for calibration purposes
const uint8_t SpreadingFactor = LORA_SF7;  //LoRa spreading factor
const uint8_t Bandwidth = LORA_BW_125;     //LoRa bandwidth
const uint8_t CodeRate = LORA_CR_4_5;      //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;    //Low data rate optimisation setting, normally set to auto
const int8_t TXpower = 10;                 //LoRa transmit power in dBm
const uint8_t RXBUFFER_SIZE = 32;          //RX buffer size
uint8_t RXBUFFER[RXBUFFER_SIZE];           //create the buffer that received packets are copied into

uint32_t RXpacketCount;  //keep count of packets received
uint32_t errors;         //keep count of packets with errors
uint8_t RXPacketL;       //stores length of packet received
int8_t PacketRSSI;       //stores RSSI of received packet
int8_t PacketSNR;        //stores signal to noise ratio (SNR) of received packet
uint16_t IRQStatus;      //stores IRQ status of LoRa device

//RX duty cycle settings for SF7, BW 125Khz.
//each preamble symbol is 1024uS
const uint32_t rxPeriod = 9216;        //9 times symbol time, number of microseconds for RX listen for SF7, BW 125Khz
const uint32_t sleepPeriod = 930816;   //number of microseconds for RX sleep\wake ratio of 100:1 for SF7, BW 125Khz
const uint16_t PreambleSymbols = 909;  //length of packet preamble in symbols, minimun is (rxPeriod + sleepPeriod)/symboltime


void loop() {

  LoRa.receiveRxDutyCycle(RXBUFFER, RXBUFFER_SIZE, NO_WAIT, rxPeriod, sleepPeriod);

  config_ESP32S3_sleep();

  Serial.println(F("Going to light sleep"));
  Serial.flush();

  esp_light_sleep_start();

  config_ESP32S3_awake();

  Serial.println(F("Awake !!!!"));

  while (!digitalRead(LORA_DIO1))  //wait for LoRa device to indicate packet received
    ;

  digitalWrite(LED1, HIGH);            //Turn on the LED
  IRQStatus = LoRa.readIrqStatus();    //read the LoRa device IRQ status register
  RXPacketL = LoRa.readRXPacketL();    //read the received packet length
  PacketRSSI = LoRa.readPacketRSSI();  //read the received packets RSSI value
  PacketSNR = LoRa.readPacketSNR();    //read the received packets SNR value

  Serial.print(F("Packet received > "));

  if (IRQStatus == (IRQ_PREAMBLE_DETECTED + IRQ_HEADER_VALID + IRQ_RX_DONE)) {
    packet_is_OK();
  } else {
    packet_is_Error();
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);                 //configure frequency and LoRa settings
  LoRa.setPacketParams(PreambleSymbols, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);  //ensure preamble length is set
  delay(500);                                                                                            //leave LED on a bit longer, easier to see
  digitalWrite(LED1, LOW);                                                                               //Turn off the LED

  Serial.println();
}


void config_ESP32S3_sleep() {
  //configuration needed for LoRa pins so it stays in receive mode during sleep
  gpio_hold_en(GPIO_NUM_14);  //hold state of LORA_NSS
  gpio_hold_en(GPIO_NUM_21);  //hold state of LORA_RESET
  gpio_hold_en(GPIO_NUM_1);   //hold state of LORA_DIO1
  rtc_gpio_pullup_dis(WAKEUP_GPIO);
  rtc_gpio_pulldown_en(WAKEUP_GPIO);
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);  //set LoRa DIO1 to be wakeup pin
}


void config_ESP32S3_awake() {
  //configuration needed for LoRa pins to continue working after sleep
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

void packet_is_OK() {
  RXpacketCount++;

  read_LoRaBUFFER(RXBUFFER, RXPacketL);   //read the contents of LoRa device buffer
  printASCIIPacket(RXBUFFER, RXPacketL);  //print the packet as ASCII characters

  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
}


void printASCIIPacket(uint8_t *buffer, uint8_t size) {
  uint8_t index;

  for (index = 0; index < size; index++) {
    Serial.write(buffer[index]);
  }
}


void read_LoRaBUFFER(uint8_t *rxbuffer, uint8_t size) {
  uint8_t index, buffdata;

  LoRa.startReadSXBuffer(0);  //start buffer read at location 0
  for (index = 0; index <= size - 1; index++) {
    buffdata = LoRa.readUint8();
    rxbuffer[index] = buffdata;
  }
  LoRa.endReadSXBuffer();
}


void packet_is_Error() {

  if (IRQStatus & IRQ_RX_TIMEOUT)  //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
  } else {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LoRa.readRXPacketL());  //get the real packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LoRa.printIrqStatus();  //print the names of the IRQ registers set
  }
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


void setup() {
  pinMode(LED1, OUTPUT);
  led_Flash(5, 250);  //flash LED to sugnal wakeup

  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.println();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  if (LoRa.begin(LORA_NSS, LORA_RESET, LORA_BUSY, LORA_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa device OK"));
  } else {
    Serial.println(F("No LoRa device responding"));
    while (1) {
      led_Flash(1, 50);  //flash LED to sugnal wakeup
    };
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);  //configure frequency and LoRa settings
  LoRa.setPacketParams(PreambleSymbols, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);

  Serial.println(F("Receiver ready"));
  Serial.println();
}
