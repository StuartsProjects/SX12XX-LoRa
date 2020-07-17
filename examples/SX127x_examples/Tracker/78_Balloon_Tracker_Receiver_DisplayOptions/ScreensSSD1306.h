/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 28/05/20
  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

// screenHABREC_1() shows the received location data and packet reception on display
// screenHABREC_2() show tracker transmitter powerup data on display
// screenHABREC_3() show receive mode on display
// screenHABREC_4() put RX and TX GPS fix status on display
// screenHABREC_5() put distance and direction on display
// screenHABREC_6() Indicate Tracker has no GPS fix
// screenHABREC_7() put packet counts on display

void screenHABREC_SETUP();         //do any display setup, default colours fonts etc in here
void screenHABREC_1();
void screenHABREC_2();
void screenHABREC_3();
void screenHABREC_4();
void screenHABREC_5();
void screenHABREC_6();
void screenHABREC_7();


void screenHABREC_SETUP()
{
  //do any display setup, default colours fonts etc in here
  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
}


void screenHABREC_1()
{
  //shows the received location data and packet reception on display
  uint8_t index;

  disp.clearLine(0);
  disp.setCursor(0, 0);

  if (PacketType == HABPacket)
  {
    for (index = 0; index < FlightIDlen; index++)
    {
      disp.write(FlightID[index]);
    }
    disp.write(',');
    disp.print(TXSequence);
  }

  if (PacketType == LocationBinaryPacket)
  {
    disp.print(Source);
  }

  disp.clearLine(1);
  disp.setCursor(0, 1);
  disp.print(F("Lat "));
  disp.print(TXLat, 5);
  disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(F("Lon "));
  disp.print(TXLon, 5);
  disp.clearLine(3);
  disp.setCursor(0, 3);
  disp.print(F("Alt "));
  disp.print(TXAlt);
  disp.print(F("m"));

  disp.clearLine(4);
  disp.setCursor(0, 4);
  disp.print(F("RSSI "));
  disp.print(PacketRSSI);
  disp.print(F("dBm"));
  disp.clearLine(5);
  disp.setCursor(0, 5);
  disp.print(F("SNR  "));

  if (PacketSNR > 0)
  {
    disp.print(F("+"));
  }

  if (PacketSNR == 0)
  {
    disp.print(F(" "));
  }

  if (PacketSNR < 0)
  {
    disp.print(F("-"));
  }

  disp.print(PacketSNR);
  disp.print(F("dB"));

  disp.clearLine(6);
  disp.setCursor(0, 6);
  disp.print(F("Packets "));
  disp.print(RXpacketCount);
}


void screenHABREC_2()
{
  //show tracker transmitter powerup data on display
  float tempfloat;
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("TXPowerup"));
  disp.setCursor(0, 1);
  disp.print(F("Bat,"));
  tempfloat = ((float) TXVolts / 1000);
  disp.print(tempfloat, 2);
  disp.print(F("v"));
}


void screenHABREC_3()
{
  //show receive mode on display
  disp.setCursor(14, 0);

  if (modeNumber == TrackerMode)
  {
    disp.print(F("TR"));
    return;
  }

  if (modeNumber == SearchMode)
  {
    disp.print(F("SE"));
    return;
  }

  disp.print(modeNumber);
}


void screenHABREC_4()
{
  //put RX and TX GPS fix status on display

  disp.setCursor(14, 1);

  if (RXGPSfix)
  {
    disp.print(F("RG"));
  }
  else
  {
    disp.setCursor(14, 1);
    disp.print(F("R?"));
  }

  disp.setCursor(14, 2);

  if (readTXStatus(GPSFix))
  {
    disp.print(F("TG"));
  }
  else
  {
    disp.print(F("T?"));
  }

}


void screenHABREC_5()
{
  //put distance and direction on display

  disp.clearLine(7);
  disp.setCursor(0, 7);
  disp.print(F("D&D "));
  disp.print(TXdistance, 0);
  disp.print(F("m "));
  disp.print(TXdirection);
  disp.print(F("d"));
}


void screenHABREC_6()
{
  //Indicate Tracker has no GPS fix
  //disp.clearLine(7);
  //disp.setCursor(0, 7);
}


void screenHABREC_7()
{
  //put packet counts on display
  disp.clearLine(6);
  disp.setCursor(0, 6);
  disp.print(F("Packets "));
  disp.print(RXpacketCount);
  disp.print(F(" Err"));
}

