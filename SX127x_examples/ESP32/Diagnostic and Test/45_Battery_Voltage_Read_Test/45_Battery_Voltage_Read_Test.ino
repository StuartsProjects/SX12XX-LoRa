//Battery_Voltage_Read_Test.ino
#define programversion "V1.0"


/*
*****************************************************************************************************
  LoRaTracker Test Programs

  Copyright of the author Stuart Robinson - 11/10/2019

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.
******************************************************************************************************
*/

/*
******************************************************************************************************
  Program operation

  This test program has been written to check that hardware for reading the battery voltage has been
  assembled correctly such that it is funtional. The ESP32 voltage reference comes from the 3.3V supply
  the value defined as 'ADMultiplier' in settings.h is used to adjust the value read from the 91K\11K 
  resistor divider and convert into mV. 

******************************************************************************************************
*/



#include "ESP32_LoRa_Micro_Node.h"
#define DIODEMV 98                   //mV voltage drop accross diode @ low idle current




void loop()
{
  Serial.println(F("LED Flash"));
  led_Flash(4, 125);
  print_SupplyVoltage();
  Serial.println();
  delay(1500);
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void print_SupplyVoltage()
{
  //get and display supply volts on terminal or monitor
  Serial.print(F("Supply Volts "));
  Serial.print(read_SupplyVoltage());
  Serial.println(F("mV"));
}


unsigned int read_SupplyVoltage()
{
  //relies on 3V3 reference, reading is 0-4095, resistor divider is 91K & 11K resistor divider
  //returns supply in mV @ 10mV per AD bit read
  uint16_t temp;
  uint16_t volts = 0;
  byte index;

  temp = analogRead(SupplyAD);

  for (index = 0; index <= 4; index++)                      //sample AD 3 times
  {
    temp = analogRead(SupplyAD);
    volts = volts + temp;
    Serial.print(F("AD value "));
    Serial.println(temp);
  }
  volts = ((volts / 5) * ADMultiplier);
  return volts;



}


void setup()
{
  Serial.begin(9600);                        //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println("12_Battery_Voltage_Read_Test Starting");

  pinMode(LED1, OUTPUT);                     //for PCB LED
  pinMode (BATREAD, OUTPUT);
  digitalWrite(BATREAD, HIGH);               //turn MOSFET connection resitor divider in circuit
}
