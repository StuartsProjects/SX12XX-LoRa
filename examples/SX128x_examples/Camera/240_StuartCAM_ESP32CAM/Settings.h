/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 17/11/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


#define NSS 13                                  //NSS on LoRa device
#define RFBUSY 4                                //pin used for busy pin on LoRa device
#define SCK 14                                  //pin used for SCK
#define MISO 15                                 //pin used for MISO 
#define MOSI 2                                  //pin used for MOSI
#define REDLED 33                               //ESp32CAM LED, set logic level low for on

#define LORA_DEVICE DEVICE_SX1280               //this is the device we are using

const int8_t PCA9536Addr = 0x41;                //I2C address of PCA9536
const uint8_t SDApin = 1;                       //yes, these are the program upload pins  
const uint8_t SCLpin = 3;                       //yes, hese are the program upload pins

const uint32_t Frequency = 2445000000;          //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes
const int8_t  TXpower = 10;                     //LoRa transmit power

//*******  Setup LoRa modem parameters here ! ***************
const uint8_t Bandwidth = LORA_BW_1600;                   //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF5;                 //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;                     //LoRa coding rate

//*******  Setup FLRC modem parameters here ! ***************
const uint8_t BandwidthBitRate = FLRC_BR_1_300_BW_1_2;     //FLRC bandwidth and bit rate, 1.3Mbs
//const uint8_t BandwidthBitRate = FLRC_BR_0_260_BW_0_3;   //FLRC 260kbps
const uint8_t CodingRate = FLRC_CR_1_0;                    //FLRC coding rate
const uint8_t BT = RADIO_MOD_SHAPING_BT_1_0;               //FLRC BT
const uint32_t Syncword = 0x01234567;                      //FLRC uses syncword
const uint32_t TXtimeoutmS = 5000;                         //mS to wait for TX to complete
const uint32_t RXtimeoutmS = 60000;                        //mS to wait for receiving a packet
const uint32_t ACKdelaymS = 0;                             //ms delay after packet actioned and ack sent

const uint32_t ACKsegtimeoutmS = 75;                       //mS to wait for receiving an ACK before re-trying transmit segment
const uint32_t ACKopentimeoutmS = 500;                     //mS to wait for receiving an ACK before re-trying transmit file open
const uint32_t ACKclosetimeoutmS = 500;                    //mS to wait for receiving an ACK before re-trying transmit file close
const uint32_t DuplicatedelaymS = 25;                      //ms delay if there has been an duplicate segment or command receipt
const uint32_t NoAckCountLimit = 250;                      //if no NoAckCount exceeds this value - restart transfer

const uint32_t packetdelaymS = 0;                                                                                                                                                                                                                                                                                                                //mS delay between transmitted packets

#ifdef USELORA
const uint8_t DTSegmentSize = 245;              //number of bytes in each segment or payload for LoRa
#endif

#ifdef USEFLRC
const uint8_t DTSegmentSize = 117;              //number of bytes in each segment or payload for FLRC
#endif

const uint8_t DTfilenamesize = 32;              //size of DTfilename buffer
const uint8_t DTSendAttempts = 10;              //number of attempts sending a packet before a restart

const uint16_t NetworkID = 0x3210;              //a unique identifier to go out with packet


const uint16_t SleepTimesecs = 60;              //sleep time in seconds after each TX loop
const uint32_t uS_TO_S_FACTOR = 1000000;        //Conversion factor for micro seconds to seconds
const uint8_t PicturesToTake = 2;               //number of pictures to take at each wakeup, only last is sent
const uint32_t PictureDelaymS = 1000;           //delay in mS between pictures


// Pin definition for CAMERA_MODEL_AI_THINKER
// Change pin definition if you're using another ESP32 with camera module
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
