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
const char Broadcast = '*';             //Broadcast address
const char RControl1 = 'D';             //Remote Control packet
const char TestMode1 = '1';             //used to switch to Testmode1 settings
const char TestPacket = 'T';            //Test packet
const char PowerUp = 'P';               //sent on tracker start
const char LocationBinaryPacket = 's';  //Short location packet in binary format
const char NoFix = 'F';                 //GPS no fix

//GPS Tracker Status byte settings
const byte GPSFix = 0;                  //flag bit number to indicate GPS has current fix
const byte UBLOXDynamicModel6Set = 1;   //flag bit set when UBLOX dynamic mode 6 has been set (and Checked)
const byte GLONASSisoutput = 2;         //flag bit number to indicate GLONASS found
const byte GPSError = 3;                //flag bit to indicate GPS error or some sort.
const byte TrackerLost = 6;             //flag bit indication that tracker in lost mode
const byte NoGPSTestMode = 7;           //flag bit number to indicate tracker in in GPS test mode