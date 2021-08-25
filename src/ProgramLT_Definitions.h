/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 17/12/19

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*
******************************************************************************************************
  Definitions for packet types
******************************************************************************************************
*/

const char Sensor1 = '!';               //Sensor packet1
const char HABPacket = '$';             //HAB style CSV ASCII packet
const char Broadcast = '*';             //Broadcast destination address
const char RControl1 = 'D';             //Remote Control packet
const char TestMode1 = '1';             //used to switch to Testmode1 settings
const char TestPacket = 'T';            //Test packet
const char TXError = 't';               //Transmitter error
const char PowerUp = 'P';               //sent on tracker start
const char LocationPacket = 'L';        //LT library tracker location packet in binary format
const char LocationBinaryPacket = 's';  //short location packet in binary format
const char NoFix = 'F';                 //GPS no fix
const char NoGPS = 'G';                 //No GPS found, or GPS error.
const char ACK = 'A';                   //Acknowledge
const char NACK = 'N';                  //Not Acknowledge, error
const char AFC = 'a';                   //Packet sent for AFC purposes

#define Reliable 0x80                   //this packet type indicates reliable data packet
#define ReliableACK 0x81                //this packet type indicates reliable data packet acknowledge
#define ReliableNACK 0x82               //this packet type indicates reliable data packet not acknowledge (error)
#define ReliableREQACK 0x83             //this packet type indicates a request to send an ACK

//file transfer packetype definitions
//these are base types, an ACK would be +1, a NACK would be +2.

#define DTSegmentWrite 0xA0                //packet type for segment write
#define DTSegmentWriteACK 0xA1             //packet type for segment write ACK
#define DTSegmentWriteNACK 0xA2            //packet type for segment write NACK


#define DTFileOpen 0xA4                    //packet type for file open, filename
#define DTFileOpenACK 0xA5                 //packet type for file open, filename ACK
#define DTFileOpenNACK 0xA6                //packet type for file open, filename NACK


#define DTFileClose 0xA8                    //packet type for file close, filename
#define DTFileCloseACK 0xA9                 //packet type for file close, filename ACK
#define DTFileCloseNACK 0xAA                //packet type for file close, filename NACK

const uint8_t FTposition = 0xAC;        //FT set write position in file packet
const uint8_t FTdelete  = 0xB0;         //FT delete file packet
const uint8_t FTrestart  = 0xB4;        //FT restart current file transfer packet ??

//GPS Tracker Status byte settings
const uint8_t GPSFix = 0;               //flag bit set when GPS has a current fix
const uint8_t GPSConfigError = 1;       //flag bit set to indicate cannot configure GPS or wrong configuration
const uint8_t CameraError = 2;          //flag bit indicating a camera device error
const uint8_t GPSError = 3;             //flag bit set to indicate GPS error, response timeout for instance
const uint8_t LORAError = 4;            //flag bit indication a lora device error
const uint8_t SDError = 5;              //flag bit indication a SD card device error
const uint8_t TrackerLost = 6;          //flag bit indication that tracker in lost mode
const uint8_t NoGPSTestMode = 7;        //flag bit number to indicate tracker in no GPS test mode

/*********************************************************************
  START Stored Program data
**********************************************************************/
const uint16_t addr_StartMemory = 0x00;          //the start of memory
const uint16_t addr_StartProgramData = 0x100;    //the start of program data in memory
const uint16_t addr_ResetCount = 0x100;          //unsigned long int 4 bytes
const uint16_t addr_SequenceNum = 0x104;         //unsigned long int 4 bytes
const uint16_t addr_TXErrors = 0x108;            //uint16_t 2 bytes
const uint16_t addr_EndMemory = 0x3FF;


/*********************************************************************
  START GPS CoordinateData
**********************************************************************/
//for storing last received GPS co-ordinates from local and remote tracker GPS
const uint16_t addr_StartCoordinateData = 0x300;
const uint16_t addr_RemoteLat = 0x300;           //float 4 bytes
const uint16_t addr_RemoteLon = 0x304;           //float 4 bytes
const uint16_t addr_RemoteAlt = 0x308;           //uint16_t 2 bytes
const uint16_t addr_RemoteHour = 0x30C;          //byte 1 byte;  Note times for last tracker co-ordinates come from local GPS time
const uint16_t addr_RemoteMin = 0x310;           //byte 1 byte
const uint16_t addr_RemoteSec = 0x311;           //byte 1 byte
const uint16_t addr_RemoteDay = 0x312;           //byte 1 byte
const uint16_t addr_RemoteMonth = 0x313;         //byte 1 byte
const uint16_t addr_RemoteYear = 0x314;          //byte 1 byte
const uint16_t addr_LocalLat = 0x318;            //float 4 bytes
const uint16_t addr_LocalLon = 0x31C;            //float 4 bytes
const uint16_t addr_LocalAlt = 0x320;            //uint16_t 2 bytes
const uint16_t addr_LocalHour = 0x322;           //byte 1 byte
const uint16_t addr_LocalMin = 0x323;            //byte 1 byte
const uint16_t addr_LocalSec = 0x324;            //byte 1 byte
const uint16_t addr_LocalDay = 0x325;            //byte 1 byte
const uint16_t addr_LocalMonth = 0x326;          //byte 1 byte
const uint16_t addr_LocalYear = 0x327;           //byte 1 byte
const uint16_t addr_EndCoordinateData = 0x327;

const uint16_t addr_RemotelocationCRC = 0x340;   //the 16 bit CRC of the last tracker location data is saved here
const uint16_t addr_LocallocationCRC = 0x342;    //the 16 bit CRC of the last local location data is saved here

const uint16_t addr_TestLocation_page3 = 0x3FF;  //used as a location for read\write tests

/*********************************************************************
  END GPS CoordinateData
**********************************************************************/


/*
******************************************************************************************************
  Bit numbers for current_config byte settings  end definitions for packet types
******************************************************************************************************
*/

//Bit numbers for current_config byte settings in transmitter (addr_Default_config1)
const uint8_t SearchEnable = 0;           //bit num to set in config byte to enable search mode packet
const uint8_t TXEnable = 1;               //bit num to set in config byte to enable transmissions
const uint8_t FSKRTTYEnable = 2;          //bit num to set in config byte to enable FSK RTTY
const uint8_t DozeEnable = 4;             //bit num to set in config byte to put tracker in Doze mode
const uint8_t GPSHotFix = 7;              //bit when set enables GPS Hot Fix mode.


/*
******************************************************************************************************
  Values for reliable transmit\receive errors
******************************************************************************************************
*/

//Reliable transmit, receive and ack return values. Bits 31 to 8 of the uint32_t function return value
//are available to return errors, these need to match the definitions in ProgramLT_Definitions.h

#define RELIABLE_PACKET_ERROR 0x0100     //receive packet CRC errors
#define RELIABLE_NOT_RELIABLE 0x0200     //not a reliable packet type, 0x80, r 0x81.
#define RELIABLE_TIMEOUT_ERROR 0x0400    //receive or transmit timeout 
#define RELIABLE_DATACRC 0x0800          //CRC error for packet or data array

#define RELIABLE_ACK_TIMEOUT 0x1000      //no ack in timeout period
#define RELIABLE_NETWORKID 0x2000        //network ID missmatch
#define RELIABLE_HEADERBUFFER_LENGTH 0x4000
#define RELIABLE_DATABUFFER_LENGTH 0x8000
#define RELIABLE_NOT_ACK 0x10000
#define RELIABLE_ATTEMPTS 0x20000
#define RELIABLE_ACK_GOOD 0x40000

//#define RELIABLE_TIMEOUT_ERROR 0x0100    //receive packet timeout
//#define RELIABLE_PACKET_ERROR 0x0200     //receive packet CRC errors
//#define RELIABLE_NOT_RELIABLE 0x0400     //not a realible packet type, 0x80 or 0x81.
//#define RELIABLE_NETWORKID 0x0800        //network ID missmatch
//#define RELIABLE_HEADERBUFFER_LENGTH 0x1000
//#define RELIABLE_DATABUFFER_LENGTH 0x2000
//#define RELIABLE_DATACRC 0x4000          //CRC error for packet or data array
//#define RELIABLE_NOT_ACK 0x8000
//#define RELIABLE_ACK_TIMEOUT 0x10000
//#define RELIABLE_ATTEMPTS 0x20000
//#define RELIABLE_ACK_GOOD 0x80000000

#define CRC_ON 1
#define CRC_OFF 0

#define ACK_ON 1
#define ACK_OFF 0
