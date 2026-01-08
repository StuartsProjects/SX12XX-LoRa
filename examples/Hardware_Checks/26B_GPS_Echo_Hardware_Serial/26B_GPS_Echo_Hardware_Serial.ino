/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 14/12/19

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a simple program to test a GPS. Its for an Arduino that has additional
  hardware serial ports such as the ESP32s. It reads characters from a GPS connected to the Serial2
  hardware serial port and then sends them (echoes) to the Arduino IDE serial monitor. If your ever
  having problems with a GPS (or just think you are) use this program first.

  If you get no data displayed on the serial monitor, the most likely cause is that you have the receive
  data pin into the Arduino (RX) pin connected incorrectly.

  At program start you should see '26B_GPS_Echo_Hardware_Serial Starting' in the serial monitor, if you
  dont the serial monitor baud rate is probably incorrectly set. If you then see data displayed on the
  serial terminal which appears to be random text with odd symbols its very likely you have the GPS
  serial baud rate set incorrectly.

  Serial monitor baud rate is set at 115200.

*******************************************************************************************************/

#define RXD2 16
#define TXD2 17

void loop()
{
  while (Serial2.available())
  {
    Serial.write(Serial2.read());
  }
}


void setup()
{
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200);
  Serial.println();
  Serial.println("26B_GPS_Echo_Hardware_Serial Starting");
}
