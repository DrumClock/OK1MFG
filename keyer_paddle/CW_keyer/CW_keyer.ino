/*
zapojení pro aktivni HIGH

buzzeru - arduino

GND -> GND
I/O -> GND
VCC -> PIN 12

*/


#include <TM1637Display.h>
#include <Encoder.h>
#include <EEPROM.h>

// -------------------------
// Konfigurace
// -------------------------

#define BUZZER_ACTIVE_LOW false   // nastav na false, pokud je buzzer aktivní na HIGH

// Piny
#define DIT_PIN 9
#define DAH_PIN 8

#define BUZZER_PIN 12

#define ENC_CLK 2
#define ENC_DT 3
#define ENC_SW 4

#define DISP_CLK 6
#define DISP_DIO 5

// -------------------------
// Inicializace
// -------------------------

TM1637Display display(DISP_CLK, DISP_DIO);
Encoder encoder(ENC_CLK, ENC_DT);

// EEPROM
const int EEPROM_WPM_ADDR = 0;
const int EEPROM_DIR_ADDR = 1;

int wpm = 40;
int lastSavedWPM = 40;
const int minWPM = 5;
const int maxWPM = 60;
unsigned long ditLength = 60;

bool paddleReversed = false;
String currentMorse = "";
unsigned long lastKeyTime = 0;
unsigned long charGap = 250;

long lastEncValue = 0;

// Tlačítko enkodéru
bool buttonPressed = false;
unsigned long buttonDownTime = 0;

// Buzzer stav
bool tonePlaying = false;
unsigned long toneDuration = 0;
unsigned long lastToneTime = 0;

// -------------------------
// Pomocné funkce
// -------------------------

void buzzerOn() {
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? LOW : HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? HIGH : LOW);
}

void updateDitLength() {
  ditLength = 1200 / wpm;
  charGap = ditLength*3;

}

void updateEncoder() {
  long encVal = encoder.read() / 4;
  if (encVal != lastEncValue) {
    int newWPM = constrain(wpm + (encVal - lastEncValue), minWPM, maxWPM);
    if (newWPM != wpm) {
      wpm = newWPM;
      updateDitLength();
    }
    lastEncValue = encVal;
  }
  display.showNumberDec(wpm, false);
}

char decodeMorse(String morse) {
  if (morse == ".-") return 'A';
  if (morse == "-...") return 'B';
  if (morse == "-.-.") return 'C';
  if (morse == "-..") return 'D';
  if (morse == ".") return 'E';
  if (morse == "..-.") return 'F';
  if (morse == "--.") return 'G';
  if (morse == "....") return 'H';
  if (morse == "..") return 'I';
  if (morse == ".---") return 'J';
  if (morse == "-.-") return 'K';
  if (morse == ".-..") return 'L';
  if (morse == "--") return 'M';
  if (morse == "-.") return 'N';
  if (morse == "---") return 'O';
  if (morse == ".--.") return 'P';
  if (morse == "--.-") return 'Q';
  if (morse == ".-.") return 'R';
  if (morse == "...") return 'S';
  if (morse == "-") return 'T';
  if (morse == "..-") return 'U';
  if (morse == "...-") return 'V';
  if (morse == ".--") return 'W';
  if (morse == "-..-") return 'X';
  if (morse == "-.--") return 'Y';
  if (morse == "--..") return 'Z';
  if (morse == "-----") return '0';
  if (morse == ".----") return '1';
  if (morse == "..---") return '2';
  if (morse == "...--") return '3';
  if (morse == "....-") return '4';
  if (morse == ".....") return '5';
  if (morse == "-....") return '6';
  if (morse == "--...") return '7';
  if (morse == "---..") return '8';
  if (morse == "----.") return '9';
  if (morse == "..--..") return '?';
  return '-';
}

void togglePaddleDirection() {
  paddleReversed = !paddleReversed;
  EEPROM.write(EEPROM_DIR_ADDR, paddleReversed ? 1 : 0);

  if (paddleReversed) {
    tonePlaying = true;
    buzzerOn();
    delay(ditLength * 3);
    buzzerOff();
    delay(ditLength);
    buzzerOn();
    delay(ditLength);
    buzzerOff();
  } else {
    tonePlaying = true;
    buzzerOn();
    delay(ditLength);
    buzzerOff();
    delay(ditLength);
    buzzerOn();
    delay(ditLength * 3);
    buzzerOff();
  }
}

void handleEncoderButton() {
  if (digitalRead(ENC_SW) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonDownTime = millis();
    } else {
      if (millis() - buttonDownTime > 1500) {
        togglePaddleDirection();
        while (digitalRead(ENC_SW) == LOW)
          ;  // čeká na puštění
        buttonPressed = false;
      }
    }
  } else if (buttonPressed) {
    EEPROM.write(EEPROM_WPM_ADDR, wpm);
    lastSavedWPM = wpm;
    buttonPressed = false;
  }
}

// -------------------------
// Setup 
// -------------------------

void setup() {
  pinMode(DIT_PIN, INPUT_PULLUP);
  pinMode(DAH_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ENC_SW, INPUT_PULLUP);

  buzzerOff();  // buzzer vypnutý na startu

  display.setBrightness(5);
  Serial.begin(9600);

  // EEPROM načtení
  int storedWPM = EEPROM.read(EEPROM_WPM_ADDR);
  if (storedWPM >= minWPM && storedWPM <= maxWPM) {
    wpm = storedWPM;
  }

  byte storedDir = EEPROM.read(EEPROM_DIR_ADDR);
  paddleReversed = (storedDir == 1);

  lastSavedWPM = wpm;
  encoder.write(0);
  updateDitLength();
}

// -------------------------
// Loop
// -------------------------

void loop() {
  unsigned long now = millis();

  updateEncoder();
  handleEncoderButton();

  bool ditPressedRaw = digitalRead(paddleReversed ? DAH_PIN : DIT_PIN) == LOW;
  bool dahPressedRaw = digitalRead(paddleReversed ? DIT_PIN : DAH_PIN) == LOW;

  if (!tonePlaying) {
    if (ditPressedRaw) {
      buzzerOn();
      toneDuration = ditLength;
      tonePlaying = true;
      lastToneTime = now;
      currentMorse += ".";
      lastKeyTime = now;
    } else if (dahPressedRaw) {
      buzzerOn();
      toneDuration = ditLength * 3;
      tonePlaying = true;
      lastToneTime = now;
      currentMorse += "-";
      lastKeyTime = now;
    }
  } else {
    if (now - lastToneTime >= toneDuration) {
      buzzerOff();
      tonePlaying = false;
      lastToneTime = now;
    }
  }

  if (!ditPressedRaw && !dahPressedRaw && currentMorse.length() > 0 && (now - lastKeyTime > charGap)) {
    char decoded = decodeMorse(currentMorse);
    Serial.print(currentMorse);
    Serial.print(" → ");
    Serial.println(decoded);
    currentMorse = "";
  }

  delay(10); // malý delay pro stabilitu
}
