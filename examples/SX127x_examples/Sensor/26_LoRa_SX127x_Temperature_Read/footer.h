pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
led_Flash(2, 125);                           //two quick LED flashes to indicate program start

Serial.begin(9600);
Serial.println();
Serial.print(__TIME__);
Serial.print(F(" "));
Serial.println(__DATE__);
Serial.println(F(Program_Version));
Serial.println();

Serial.println(F("1_LT_LED_Blink Starting"));
