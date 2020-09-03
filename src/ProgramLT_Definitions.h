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
const char Reliable = 'R';              //this is a realiable type packet, assumed an ACK is required

const uint8_t FT = 0xF0;                //this packet type indicates File Transfer packet, with subtype
const uint8_t FTstart = 0x01;           //FTsubtype file transfer start information, filename etc
const uint8_t FTsegment = 0x02;         //FTsubtype packet contains a numbered segment
const uint8_t FTACK = 0x03;             //FTsubtype ACK response for file transfers
const uint8_t FTNACK = 0x04;            //FTsubtype NACK response for file transfers
const uint8_t FTclose  = 0x05;          //FTsubtype request from TX to close file, transfer finished
const uint8_t FTrestart  = 0x06;        //FTsubtype request from RX to restart current file transfer

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
const uint16_t addr_TXErrors = 0x108;            //unsigned int 2 bytes

const uint16_t addr_EndMemory = 0x3FF;


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
const uint16_t packettypeErr = 0x0001;
const uint16_t destErr = 0x0002;
const uint16_t sourceErr = 0x0004;
const uint16_t timeoutErr = 0x0008;
const uint16_t IDErr = 0x0010;
const uint16_t crcErr = 0x020;
const uint16_t seqErr = 0x040;