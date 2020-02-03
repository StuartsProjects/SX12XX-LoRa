/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 03/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a simple program to test a GPS. It reads characters from the GPS using
  using the ESp32 hardware serial port and sends them (echoes) to the IDE serial monitor. If your ever
  having problems with a GPS (or just think you are) use this program first.

  The program allows for a logic pin that may be being used to turn the GPS on\off (GPSPOWER). If this
  feature is not used, set GPSPOWER to -1. 

  If you get no data displayed on the serial monitor, the most likely cause is that you have the receive
  data pin into the Arduino (RX) pin connected incorrectly.

  If the data displayed on the serial terminal appears to be random text with odd symbols its very
  likely you have the GPS serial baud rate set incorrectly.

  For a more detailed tutorial on GPS problems see here;

  https://github.com/LoRaTracker/GPSTutorial

  Serial monitor baud rate is set at 115200.

*******************************************************************************************************/


#define GPSBAUD 9600          //this is the serial baud rate that will be used for the GPS, a common default

#define GPSTX 16              //pin number for TX output from Arduino - RX into GPS
#define GPSRX 17              //pin number for RX input into Arduino - TX from GPS
#define GPSPOWER 26           //pin controls power to external GPS, LOW for on. Set to -1 if not used.

#define GPSserial Serial2     //define GPSserial as ESP32 Serial2 


void loop()
{
  while (GPSserial.available())
  {
    Serial.write(GPSserial.read());
  }
}


void setup()
{
  
  if (GPSPOWER >= 0)
  {
  pinMode(GPSPOWER, OUTPUT);                                 //for GPS power control
  digitalWrite(GPSPOWER, LOW);
  }

  
  GPSserial.begin(9600, SERIAL_8N1, GPSTX, GPSRX);           //format is baud, mode, UART RX data, UART TX data 
                                                                 
  Serial.begin(115200);
  Serial.println("26_GPS_Echo_ESP32 Starting");
}
