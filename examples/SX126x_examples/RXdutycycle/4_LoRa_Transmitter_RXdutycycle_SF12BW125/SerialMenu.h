/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 09/09/26

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*
  Issues:
*/


//#define DEBUG


//general functions
void clear_serial();
uint32_t get_number32(char* numprompt, uint8_t numpromptsize, uint8_t digits, int32_t minval, int32_t maxval);
void send_Packet();
void packet_is_OK();
void packet_is_Error();
void update_LoRa();

//menus
void menu_LoRa(uint32_t timeout);
void menu_FREQ(uint32_t timeout);
void menu_FREQOFF(uint32_t timeout);
void menu_SF(uint32_t timeout);
void menu_BW(uint32_t timeout);
void menu_CR(uint32_t timeout);
void menu_TXPOW(uint32_t timeout);
void menu_PreambleSize(uint32_t timeout);
void menu_NodeNumber(uint32_t timeout);


//***********************************************************
// Menus
//***********************************************************

void menu_LoRa(uint32_t timeout) {

  uint8_t readchar;
  bool printmenu = false;
  clear_serial();

  do {
    printmenu = false;
    Serial.println();
    Serial.println(F("Transmitter LoRa Settings"));
    Serial.println(F("----------------------"));
    Serial.print(F("1 Frequency "));
    Serial.print(Frequency);
    Serial.println(F("Hz"));
    Serial.print(F("2 Offset "));
    Serial.print(Offset);
    Serial.println(F("Hz"));
    Serial.print(F("3 SpreadingFactor "));
    Serial.println(SpreadingFactor);
    Serial.print(F("4 Bandwidth "));
    Serial.print(LoRa.returnBandwidth(Bandwidth));
    Serial.println(F("Hz"));
    Serial.print(F("5 CodeRate "));
    Serial.println(CodeRate + 4);
    Serial.print(F("6 TXpower "));
    Serial.print(TXpower);
    Serial.println(F("dBm"));
    Serial.print(F("7 Preamble Length "));
    Serial.print(PreambleSymbols);
    Serial.println();
    Serial.print(F("8 Wake Node number "));
    Serial.print(ListenNodeNumber);
    Serial.println();
    Serial.println();
    Serial.println(F("W Send Wake Packet"));
    Serial.println();

    do {

      if (!digitalRead(SWITCH1)) {
        Serial.println(F(" "));
        digitalWrite(LED1, HIGH);  //Turn on the LED
        send_Packet();
        digitalWrite(LED1, LOW);  //Turn off the LED
        clear_serial();
        printmenu = true;
      }


      if (Serial.available() > 0) {
        readchar = Serial.read();
        clear_serial();
        Serial.write(readchar);

        if (readchar == '1') {
          menu_FREQ(timeout);
          update_LoRa();
          printmenu = true;  //run menu print again
        }

        if (readchar == '2') {
          menu_FREQOFF(timeout);
          update_LoRa();
          printmenu = true;
        }

        if (readchar == '3') {
          menu_SF(timeout);
          update_LoRa();
          printmenu = true;
        }

        if (readchar == '4') {
          menu_BW(timeout);
          update_LoRa();
          printmenu = true;
        }

        if (readchar == '5') {
          menu_CR(timeout);
          update_LoRa();
          printmenu = true;
        }

        if (readchar == '6') {
          menu_TXPOW(timeout);
          update_LoRa();
          printmenu = true;
        }

        if (readchar == '7') {
          menu_PreambleSize(timeout);
          update_LoRa();
          printmenu = true;
        }

        if (readchar == '8') {
          menu_NodeNumber(timeout);
          printmenu = true;
        }

        if ((readchar == 'w') || (readchar == 'W')) {
          Serial.println(F(" "));
          digitalWrite(LED1, HIGH);  //Turn on the LED
          send_Packet();
          digitalWrite(LED1, LOW);  //Turn off the LED
          clear_serial();
          printmenu = true;
        }
        continue;
      }

      if (printmenu) {
        break;
      }

    } while (1);

  } while (1);
}


void clear_serial() {
  uint8_t nochar;
  delay(10);  //allow time for a few characters to arrive, '1' can be followed by CR & LF.

  while (Serial.available() > 0) {
    nochar = Serial.read();
  };
}


uint32_t get_number32(char* numprompt, uint8_t numpromptsize, uint8_t digits, int32_t minval, int32_t maxval) {
  uint8_t readchar;
  uint8_t index = 0;
  uint8_t index2 = 0;
  int32_t num;
  //uint32_t timeoutmS;
  char numberbuff[11];
  clear_serial();
  memset(numberbuff, 0, sizeof(numberbuff));
  numbervalid = false;

  do {
    if (Serial.available() > 0) {
      readchar = Serial.read();

      if ((readchar == 0x0a) || (readchar == 0x0d)) {
        clear_serial();
        if (index == 0) {
          return 0;
        }

        num = atoi(numberbuff);

        if ((num < minval) || (num > maxval)) {
          Serial.print(F("  "));
          Serial.print(num);
          Serial.println(F(" Number outside limits"));
          return 0;
        } else {
          numbervalid = true;
          return num;
        }
      }

      if ((index == 0) && ((readchar == '-') || (readchar == '+'))) {
        Serial.write(readchar);
        numberbuff[0] = readchar;
        index++;
      } else {
        if ((readchar == 'x') || (readchar == 'X') || (readchar == 0x1B))  //0x1B is ESC key
        {
          return false;
        }

        if ((readchar < '0') || (readchar > '9')) {
          Serial.write(readchar);
          Serial.println(F(" ERROR - not number"));
          index = 0;
          memset(numberbuff, 0, sizeof(numberbuff));  //fill array with nulls
          clear_serial();
          for (index2 = 0; index2 < numpromptsize; index2++) {
            Serial.write(numprompt[index2]);
          }
        } else {
          numberbuff[index] = readchar;
          index++;

          if (index > digits) {
            Serial.write(readchar);
            Serial.println(F(" ERROR - to long"));
            Serial.println();
            index = 0;
            memset(numberbuff, 0, sizeof(numberbuff));  //fill array with nulls
            clear_serial();
          } else {
            Serial.write(readchar);
          }
        }
      }
    }
  } while (1);

  return 0;
}


void send_Packet() {
  PacketCount++;
  uint32_t startms;

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);                 //configure frequency and LoRa settings
  LoRa.setPacketParams(PreambleSymbols, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);  //set preamble

  LoRa.startWriteSXBuffer(0);         //start the write at location 0
  LoRa.writeUint8(ATWake);            //wake packet ID
  LoRa.writeUint8(ListenNodeNumber);  //Node number to wake
  LoRa.writeUint8(0);
  TXPayloadL = LoRa.endWriteSXBuffer();
  TXPacketL = TXPayloadL + 4;

  Serial.print(TXpower);  //print the transmit power defined
  Serial.print(F("dBm > Sending "));

  startms = micros();

  if (LoRa.transmitSXReliable(0, TXPayloadL, NetworkID, 60000, TXpower, WAIT_TX)) {
    Packetus = micros() - startms;
    TXPacketCount++;
    Serial.print(F(" Sent"));
    packet_is_OK();
  } else {
    packet_is_Error();  //transmit packet returned 0, there was an error
  }

  Serial.println();
}


void update_LoRa() {
  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);  //configure frequency and LoRa settings
  LoRa.setPacketParams(PreambleSymbols, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
}


void menu_FREQ(uint32_t timeout) {
  int32_t num;
  char numprompt[] = "Enter Frequency in Hz > ";
  Serial.println();
  Serial.print(numprompt);
  num = get_number32(numprompt, sizeof(numprompt), 9, 400000000, 950000000);

  if (numbervalid) {
    Serial.println();
    Serial.print(F("Entered Frequency "));
    Serial.print(num);
    Serial.println(F("Hz"));
    Frequency = num;
  }
  return;
}


void menu_FREQOFF(uint32_t timeout) {
  int32_t num;

  char numprompt[] = "Enter Offset Frequency in Hz > ";
  Serial.println();
  Serial.print(numprompt);
  num = get_number32(numprompt, sizeof(numprompt), 6, -250000, 250000);

  if (numbervalid) {
    Serial.println();
    Serial.print(F("Entered Frequency Offset "));
    Serial.print(num);
    Serial.println(F("Hz"));
    Offset = num;
  }
}


void menu_SF(uint32_t timeout) {
  int32_t num;

  char numprompt[] = "Enter LoRa Spreading factor, 5 to 12 > ";
  Serial.println();
  Serial.print(numprompt);
  num = get_number32(numprompt, sizeof(numprompt), 3, 5, 12);

  if (numbervalid) {
    Serial.println();
    Serial.print(F("Entered Spreading Factor "));
    Serial.println(num);

    switch (num) {
      case 5:
        SpreadingFactor = LORA_SF5;
        break;
      case 6:
        SpreadingFactor = LORA_SF6;
        break;
      case 7:
        SpreadingFactor = LORA_SF7;
        break;
      case 8:
        SpreadingFactor = LORA_SF8;
        break;
      case 9:
        SpreadingFactor = LORA_SF9;
        break;
      case 10:
        SpreadingFactor = LORA_SF10;
        break;
      case 11:
        SpreadingFactor = LORA_SF11;
        break;
      case 12:
        SpreadingFactor = LORA_SF12;
        break;
    }
  }
  return;
}


void menu_BW(uint32_t timeout) {
  uint32_t timeoutmS;
  uint8_t readchar;
  bool printmenu = false;

  timeoutmS = millis() + timeout;
  clear_serial();

  do {
    Serial.println();
    Serial.println(F("LoRa Bandwidth"));
    Serial.println(F("--------------"));
    Serial.println(F("1 7810Hz"));
    Serial.println(F("2 10420Hz"));
    Serial.println(F("3 15630Hz"));
    Serial.println(F("4 20830Hz"));
    Serial.println(F("5 31250Hz"));
    Serial.println(F("6 41670Hz"));
    Serial.println(F("7 62500Hz"));
    Serial.println(F("8 125000Hz"));
    Serial.println(F("9 250000Hz"));
    Serial.println(F("A 500000Hz"));
    Serial.println();
    Serial.println(F("X Exit menu"));
    Serial.println();
    Serial.print(F("Select Bandwidth > "));

    do {
      if (Serial.available() > 0) {
        readchar = Serial.read();
        timeoutmS = millis() + timeout;  //restart timeout
        clear_serial();                  //get rid of any spurious serial characters

        if ((readchar == 'x') || (readchar == 'X') || (readchar == 0x1B))  //0x1B is ESC key
        {
          Serial.println(F("Exit menu"));
          clear_serial();
          return;
        }

        if (readchar == '1') {
          Serial.println(F("7810Hz selected"));
          Bandwidth = LORA_BW_007;
          printmenu = true;  //run menu print again
          break;
        }

        if (readchar == '2') {
          Serial.println(F("10420Hz selected"));
          Bandwidth = LORA_BW_010;
          printmenu = true;
          break;
        }

        if (readchar == '3') {
          Serial.println(F("15630Hz selected"));
          Bandwidth = LORA_BW_015;
          printmenu = true;
          break;
        }

        if (readchar == '4') {
          Serial.println(F("20830Hz selected"));
          Bandwidth = LORA_BW_020;
          printmenu = true;
          break;
        }

        if (readchar == '5') {
          Serial.println(F("31250Hz selected"));
          Bandwidth = LORA_BW_031;
          printmenu = true;
          break;
        }

        if (readchar == '6') {
          Serial.println(F("41670Hz selected"));
          Bandwidth = LORA_BW_041;
          printmenu = true;
          break;
        }

        if (readchar == '7') {
          Serial.println(F("62500Hz selected"));
          Bandwidth = LORA_BW_062;
          printmenu = true;
          break;
        }

        if (readchar == '8') {
          Serial.println(F("125000Hz selected"));
          Bandwidth = LORA_BW_125;
          printmenu = true;
          break;
        }

        if (readchar == '9') {
          Serial.println(F("250000Hz selected"));
          Bandwidth = LORA_BW_250;
          printmenu = true;
          break;
        }

        if ((readchar == 'a') || (readchar == 'A')) {
          Serial.println(F("500000Hz selected"));
          Bandwidth = LORA_BW_500;
          printmenu = true;
          break;
        }
      }
    } while (millis() < timeoutmS);

    if (millis() >= timeoutmS) {
      return;
    }

    if (!printmenu) {
      break;
    }
  } while (1);
  return;
}


void menu_CR(uint32_t timeout) {
  uint32_t timeoutmS;
  uint8_t readchar;
  bool printmenu = false;

  timeoutmS = millis() + timeout;
  clear_serial();

  do {
    Serial.println();
    Serial.println(F("LoRa Coding rate"));
    Serial.println(F("----------------"));
    Serial.println(F("5 4 : 5"));
    Serial.println(F("6 4 : 6"));
    Serial.println(F("7 4 : 7"));
    Serial.println(F("8 4 : 8"));
    Serial.println();
    Serial.println(F("X Exit menu"));
    Serial.println();
    Serial.print(F("Select Coding rate > "));

    do {
      if (Serial.available() > 0) {
        readchar = Serial.read();
        //timeoutmS = millis() + timeout;  //restart timeout
        clear_serial();  //get rid of any spurious serial characters

        if ((readchar == 'x') || (readchar == 'X') || (readchar == 0x1B))  //0x1B is ESC key
        {
          Serial.println(F("Exit menu"));
          clear_serial();
          return;
        }

        if (readchar == '5') {
          Serial.println(F("4 : 5 selected"));
          CodeRate = LORA_CR_4_5;
          printmenu = true;  //run menu print again
          break;
        }

        if (readchar == '6') {
          Serial.println(F("4 : 6 selected"));
          CodeRate = LORA_CR_4_6;
          printmenu = true;
          break;
        }

        if (readchar == '7') {
          Serial.println(F("4 : 7 selected"));
          CodeRate = LORA_CR_4_7;
          printmenu = true;
          break;
        }

        if (readchar == '8') {
          Serial.println(F("4 : 8 selected"));
          CodeRate = LORA_CR_4_8;
          printmenu = true;
          break;
        }
      }
    } while (1);

    if (millis() >= timeoutmS) {
      return;
    }

    if (!printmenu) {
      break;
    }
  } while (1);
  return;
}


void menu_TXPOW(uint32_t timeout) {
  int32_t num;

  char numprompt[] = "Enter LoRa TX Power dBm > ";
  Serial.println();
  Serial.print(numprompt);
  num = get_number32(numprompt, sizeof(numprompt), 3, -9, 22);

  if (numbervalid) {
    Serial.println();
    Serial.print(F("Entered TXpower "));
    Serial.print(num);
    Serial.println(F("dBm"));
    TXpower = num;
  }
  return;
}


void menu_PreambleSize(uint32_t timeout) {
  uint16_t num;

  char numprompt[] = " Enter number of preamble symbols > ";
  Serial.print(numprompt);
  num = get_number32(numprompt, sizeof(numprompt), 5, 0, 65535);

  if (numbervalid) {
    Serial.println();
    Serial.print(F("Entered number of preamble symbols "));
    Serial.println(num);
    PreambleSymbols = num;
  }
  return;
}


void menu_NodeNumber(uint32_t timeout) {
  int32_t num;
  Serial.println();
  Serial.println();
  Serial.println(F("Node number to listen for 0 for all or 1 to 255"));
  char numprompt[] = "Node number > ";
  Serial.println();
  Serial.print(numprompt);
  num = get_number32(numprompt, sizeof(numprompt), 3, 0, 255);

  if (numbervalid) {
    Serial.println();
    Serial.print(F("Entered Node number "));
    Serial.println(num);
    ListenNodeNumber = num;
  }

  return;
}



void packet_is_OK() {
  //if here packet has been sent OK
  Serial.print(F("  Bytes,"));
  Serial.print(TXPacketL);  //print transmitted packet length
  Serial.print(F("  TX,"));
  Serial.print(TXPacketCount);  //print total of packets sent OK
  Serial.print(F("  Preamble,"));
  Serial.print(PreambleSymbols);  //print total of packets sent OK
  Serial.print(F("  Time,"));
  Serial.print(Packetus);
  Serial.println(F("uS"));
}


void packet_is_Error() {
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LoRa.readIrqStatus();  //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);  //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);  //print IRQ status
  LoRa.printIrqStatus();         //prints the text of which IRQs set
}
