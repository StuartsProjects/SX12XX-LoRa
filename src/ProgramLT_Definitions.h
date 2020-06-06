/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 17/12/19

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
const char AFC = 'a';                   //Packet sent for AFC purposes

//GPS Tracker Status byte settings
const byte GPSFix = 0;                  //flag bit set when GPS has a current fix
const byte GPSConfigError = 1;          //flag bit set to indicate cannot configure GPS or wrong configuration 
const byte GPSError = 3;                //flag bit set to indicate GPS error, response timeout for instance
const byte LORAError = 4;               //flag bit indication a lora device error
const byte SDError = 5;                 //flag bit indication a SD card device error
const byte TrackerLost = 6;             //flag bit indication that tracker in lost mode
const byte NoGPSTestMode = 7;           //flag bit number to indicate tracker in no GPS test mode

/*********************************************************************
  START Stored Program data
**********************************************************************/
const unsigned int addr_StartMemory = 0x00;                   //the start of memory
const unsigned int addr_StartProgramData = 0x100;             //the start of program data in memory
const unsigned int addr_ResetCount = 0x100;                   //unsigned long int 4 bytes
const unsigned int addr_SequenceNum = 0x104;                  //unsigned long int 4 bytes
const unsigned int addr_TXErrors = 0x108;                     //unsigned int 2 bytes

const unsigned int addr_EndMemory = 0x3FF;


/*
******************************************************************************************************
  Bit numbers for current_config byte settings  end definitions for packet types
******************************************************************************************************
*/

//Bit numbers for current_config byte settings in transmitter (addr_Default_config1)
const byte SearchEnable = 0;           //bit num to set in config byte to enable search mode packet
const byte TXEnable = 1;               //bit num to set in config byte to enable transmissions
const byte FSKRTTYEnable = 2;          //bit num to set in config byte to enable FSK RTTY
const byte DozeEnable = 4;             //bit num to set in config byte to put tracker in Doze mode
const byte GPSHotFix = 7;              //bit when set enables GPS Hot Fix mode.


