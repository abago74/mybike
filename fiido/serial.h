// SERIAL AT METHODS ***********************************************************************************

void serialAtCommandListener() { // Espera un comando AT con un buffer de hasta serialBufferSize (32)
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  while (Serial.available() > 0 && newSerialData == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= serialBufferSize) {
        ndx = serialBufferSize - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      String receivedCharsStr = receivedChars;
      if (receivedCharsStr.startsWith("at") || receivedCharsStr.startsWith("AT")) {
        newSerialData = true;
      }
    }
  }
}


/*
  AT Commands
  at+save
  at+init
  at+eelist
  at+pwdup=[0-5]
  at+pwddw=[0-5]
  at+mpuon
  at+mpuoff
  at+anglemaxpwr=[0-30]
  at+level
*/
void ATCommandsManager() {

  if (newSerialData == true) {
    blinkLed(STATUS_LED_OUT, 1, 500);
    String command = getAtData(receivedChars, '=' , 0);
    command.toLowerCase();
    int value = getAtData(receivedChars, '=' , 1).toInt();

    oled1306.clearDisplay();
    oled1306.setCursor(0, 0);
    oled1306.print(F("> "));
    String tmpCommand = command;
    tmpCommand.toUpperCase();
    oled1306.print(tmpCommand);
    if (value > 0) {
      oled1306.print(F("?"));
      oled1306.print(value);
    }
    oled1306.setCursor(0, 10);

    if (command.indexOf(F("at+save")) > -1) {
      updateEepromData();

    } else if (command.indexOf(F("at+init")) > -1) {
      initDefaultEepromData();

    } else if (command.indexOf(F("at+eelist")) > -1) {
      //printEeprom();
      showEepromDataScreen();
      
    } else if (command.indexOf(F("at+pwrup")) > -1) { // modo de incremento de potencia progresiva.
      eStorage.powerMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print(F("> powerBrakeMode: "));
      oled1306.print(eStorage.powerMode);

    } else if (command.indexOf(F("at+pwrdw")) > -1) { // modo de decremento de potencia progresiva.
      eStorage.powerBrakeMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print(F("> powerBrakeMode: "));
      oled1306.print(eStorage.powerBrakeMode);

    } else if (command.indexOf(F("at+mpuon")) > -1) {
      eStorage.mpuenabled = true;

    } else if (command.indexOf(F("at+mpuoff")) > -1) {
      eStorage.mpuenabled = false;
      ax = 0; ay = 0; az = 1;

    } else if (command.indexOf(F("at+shutdown")) > -1) {
      asm volatile ("  jmp 0");

    } else if (eStorage.mpuenabled) {

      if (command.indexOf(F("at+anglemaxpwr")) > -1) { // ángulo para la máxima potencia;
        eStorage.maxPowerAngle = value < DEFAULT_MAX_POWER_ANGLE ? value : DEFAULT_MAX_POWER_ANGLE;
        oled1306.print(F("> maxPowerAngle: "));
        oled1306.print(eStorage.maxPowerAngle);

      } else if (command.indexOf("at+level") > -1) { // Calibrar posición de placa.
        float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
        float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
        eStorage.levelXAngle = (int) accel_ang_x;
        eStorage.levelYAngle = (int) accel_ang_y;
        oled1306.print(F("> [X|Y]: "));
        oled1306.print(eStorage.levelXAngle);
        oled1306.print(F(" | "));
        oled1306.print(eStorage.levelYAngle);
        oled1306.display();
      }
    }

    newSerialData = false;
    blinkLed(STATUS_LED_OUT, 15, 30);
  }
}

String getAtData(String data, char separator, int index) { // Split string and get index data.
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (byte i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}