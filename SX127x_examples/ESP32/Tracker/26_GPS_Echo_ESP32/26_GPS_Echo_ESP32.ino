/*******************************************************************************************************
  lora Programs for Arduino - Copyright of the author Stuart Robinson - 14/12/19

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a simple program to test a GPS. It reads characters from the GPS using
  software serial and sends them (echoes) to the IDE serial monitor. If your ever having problems with
  a GPS (or just think you are) use this program first.

  If you get no data displayed on the serial monitor, the most likely cause is that you have the receive
  data pin into the Arduino (RX) pin connected incorrectly.

  If the data displayed on the serial terminal appears to be random text with odd symbols its very
  likely you have the GPS serial baud rate set incorrectly.

  Note that not all pins on a all Arduinos will work with software serial, see here;

  https://www.arduino.cc/en/Reference/softwareSerial

  For a more detailed tutorial on GPS problems see here;

  https://github.com/LoRaTracker/GPSTutorial

  Serial monitor baud rate is set at 115200.

*******************************************************************************************************/


#define GPSBAUD 9600          //this is the serial baud rate that will be used for the GPS, a common default
#define MONITORBAUD 115200    //this is the serial baud rate that will be used for the serial monitor 

#define GPSTX 16              //pin number for TX output from Arduino - RX into GPS
#define GPSRX 17              //pin number for RX input into Arduino - TX from GPS
#define GPSPOWER 26           //pin controls power to external GPS, LOW for on

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
  pinMode(GPSPOWER, OUTPUT);                                 //for GPS power control
  digitalWrite(GPSPOWER, LOW);
  
  GPSserial.begin(9600, SERIAL_8N1, GPSTX, GPSRX);           //format is baud, mode, UART RX data, UART TX data 
                                                                 
  Serial.begin(MONITORBAUD);
  Serial.println("26_GPS_Echo Starting");
}
