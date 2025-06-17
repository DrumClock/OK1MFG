#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <EEPROM.h>

// -------------------------
// Konfigurace
// -------------------------

#define BUZZER_ACTIVE_LOW true  // nastav na false, pokud je buzzer aktivní na HIGH

#define DIT_PIN 9
#define DAH_PIN 8

#define BUZZER_PIN 12

#define CLEAR_BUTTON_PIN 6
#define PLAY_BUTTON_PIN 7

#define ENC_CLK 2
#define ENC_DT 3
#define ENC_SW 4

// LCD I2C adresa a velikost
LiquidCrystal_I2C lcd(0x27, 20, 4);
// SDA (data): pin A4
// SCL (clock): pin A5

// -------------------------

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
bool buttonPressed = false;
unsigned long buttonDownTime = 0;
bool tonePlaying = false;
unsigned long toneDuration = 0;
unsigned long lastToneTime = 0;

// Pro rolování znaků
String receivedText = "";

// Globální proměnné pro kontrolu přehrávání
volatile bool stopPlayback = false;
bool isPlaying = false;

// Nové proměnné pro non-blocking přehrávání
String playbackText = "";
int playbackIndex = 0;
String currentPlaybackMorse = "";
int currentPlaybackMorseIndex = 0;
unsigned long playbackTimer = 0;
int playedCharCount = 0;  // Nová proměnná pro počítání znaků
enum PlaybackState { IDLE, PLAYING_ELEMENT, ELEMENT_GAP, CHAR_GAP, WORD_GAP, LONG_GAP };
PlaybackState playbackState = IDLE;

// -------------------------
// Funkce
// -------------------------

void buzzerOn() {
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? LOW : HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? HIGH : LOW);
}

void updateDitLength() {
  ditLength = 1200 / wpm;
  charGap = ditLength * 3;
}

// ------------------------------------------------------------

void updateEncoder() {
  long encVal = encoder.read() / 4;
  if (encVal != lastEncValue) {
    int newWPM = constrain(wpm - (encVal - lastEncValue), minWPM, maxWPM);
    if (newWPM != wpm) {
      wpm = newWPM;
      updateDitLength();
    }
    lastEncValue = encVal;
  }

  // 1. řádek: WPM a MODE pevně zarovnané
  lcd.setCursor(0, 0);  // sloupec 0, řádek 0
  lcd.print("WPM: ");
  lcd.print(wpm);
  lcd.print("        ");  // pro smazání zbytku

  lcd.setCursor(12, 0);  // pevná pozice MODE
  lcd.print("MODE: ");
  lcd.print(paddleReversed ? "-." : ".-");
}

// ------------------------------------------------------------

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
  if (morse == "--..-.") return '/';
  if (morse == "-...-") return '=';
  if (morse == "--..--") return ',';
  if (morse == ".-.-.") return '.';
  return '*';  // neznámý znak
}

// ------------------------------------------------------------

String charToMorse(char c) {
  c = toupper(c);
  
  if (c == 'A') return ".-";
  else if (c == 'B') return "-...";
  else if (c == 'C') return "-.-.";
  else if (c == 'D') return "-..";
  else if (c == 'E') return ".";
  else if (c == 'F') return "..-.";
  else if (c == 'G') return "--.";
  else if (c == 'H') return "....";
  else if (c == 'I') return "..";
  else if (c == 'J') return ".---";
  else if (c == 'K') return "-.-";
  else if (c == 'L') return ".-..";
  else if (c == 'M') return "--";
  else if (c == 'N') return "-.";
  else if (c == 'O') return "---";
  else if (c == 'P') return ".--.";
  else if (c == 'Q') return "--.-";
  else if (c == 'R') return ".-.";
  else if (c == 'S') return "...";
  else if (c == 'T') return "-";
  else if (c == 'U') return "..-";
  else if (c == 'V') return "...-";
  else if (c == 'W') return ".--";
  else if (c == 'X') return "-..-";
  else if (c == 'Y') return "-.--";
  else if (c == 'Z') return "--..";
  else if (c == '0') return "-----";
  else if (c == '1') return ".----";
  else if (c == '2') return "..---";
  else if (c == '3') return "...--";
  else if (c == '4') return "....-";
  else if (c == '5') return ".....";
  else if (c == '6') return "-....";
  else if (c == '7') return "--...";
  else if (c == '8') return "---..";
  else if (c == '9') return "----.";
  else if (c == '?') return "..--..";
  else if (c == '/') return "--..-.";
  else if (c == '=') return "-...-";
  else if (c == ',') return "--..--";
  else if (c == '.') return ".-.-.";
  else if (c == '*') return "*";  // speciální znak pro dlouhý tón
  else if (c == ' ') return " ";  // mezera
  else return "";  // neznámý znak
}

// ------------------------------------------------------------

void togglePaddleDirection() {
  paddleReversed = !paddleReversed;
  EEPROM.write(EEPROM_DIR_ADDR, paddleReversed ? 1 : 0);

  buzzerOn();
  delay(paddleReversed ? ditLength * 3 : ditLength);
  buzzerOff();
  delay(ditLength);
  buzzerOn();
  delay(paddleReversed ? ditLength : ditLength * 3);
  buzzerOff();
}

// ------------------------------------------------------------

void handleEncoderButton() {
  if (digitalRead(ENC_SW) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonDownTime = millis();
    } else if (millis() - buttonDownTime > 1500) {
      togglePaddleDirection();
      while (digitalRead(ENC_SW) == LOW)
        ;
      buttonPressed = false;
    }
  } else if (buttonPressed) {
    EEPROM.write(EEPROM_WPM_ADDR, wpm);
    lastSavedWPM = wpm;
    buttonPressed = false;
  }
}

// ------------------------------------------------------------

void updateLCDText() {
  // Vymaž řádky 1–3
  for (int row = 1; row <= 3; row++) {
    lcd.setCursor(0, row);
    lcd.print("                    ");
  }

  // rozdělení na 3 řádky po 20 znacích
  int maxLines = 3;       // řádky 2, 3, 4
  int charsPerLine = 20;  // znaků na řádek
  int maxChars = maxLines * charsPerLine;

  String displayText = receivedText;
  if (displayText.length() > maxChars) {
    // oříznout na posledních 60 znaků
    displayText = displayText.substring(displayText.length() - maxChars);
  }

  // doplnit do násobku 20 mezerami, aby každý řádek měl 20 znaků
  while (displayText.length() % charsPerLine != 0) {
    displayText += " ";
  }

  int totalLines = displayText.length() / charsPerLine;

  // vypisovat od řádku 2 (index 1), pak 3 (2), pak 4 (3)
  for (int i = 0; i < totalLines; i++) {
    int lcdRow = 1 + i;     // LCD řádky: 1 = řádek 2, 2 = řádek 3, 3 = řádek 4
    if (lcdRow > 3) break;  // nepřekročit řádek 4
    int startIdx = i * charsPerLine;
    String lineText = displayText.substring(startIdx, startIdx + charsPerLine);
    lcd.setCursor(0, lcdRow);
    lcd.print(lineText);
  }
}

// ------------------------------------------------------------

void startPlayback() {
  String toPlay = receivedText;
  if (toPlay.length() > 60)
    toPlay = toPlay.substring(toPlay.length() - 60);
    
  playbackText = toPlay;
  playbackIndex = 0;
  currentPlaybackMorse = "";
  currentPlaybackMorseIndex = 0;
  playbackState = IDLE;
  playedCharCount = 0;  // Reset počítadla znaků
  stopPlayback = false;
  isPlaying = true;
  playbackTimer = millis();
}

void stopPlaybackNow() {
  stopPlayback = true;
  isPlaying = false;
  playbackState = IDLE;
  buzzerOff();
}

// ------------------------------------------------------------

void handlePlayback() {
  if (!isPlaying || stopPlayback) {
    if (isPlaying) {
      buzzerOff();
      isPlaying = false;
      playbackState = IDLE;
    }
    return;
  }
  
  unsigned long now = millis();
  
  switch (playbackState) {
    case IDLE:
      if (playbackIndex >= playbackText.length()) {
        // Konec přehrávání
        isPlaying = false;
        buzzerOff();
        return;
      }
      
      // Získej morse kód pro aktuální znak
      currentPlaybackMorse = charToMorse(playbackText[playbackIndex]);
      currentPlaybackMorseIndex = 0;
      
      if (currentPlaybackMorse == " ") {
        // Mezera mezi slovy
        playbackState = WORD_GAP;
        playbackTimer = now;
      } else if (currentPlaybackMorse == "*") {
        // Speciální dlouhý tón (7 ditLength)
        buzzerOn();
        playbackTimer = now;
        playbackState = PLAYING_ELEMENT;
        currentPlaybackMorseIndex = 0;  // Označíme jako první (a jediný) element
      } else if (currentPlaybackMorse == "") {
        // Neznámý znak, přeskočit
        playbackIndex++;
        // Zůstat v IDLE pro další znak
      } else {
        // Začni přehrávat první element znaku
        char element = currentPlaybackMorse[currentPlaybackMorseIndex];
        buzzerOn();
        playbackTimer = now;
        playbackState = PLAYING_ELEMENT;
      }
      break;
      
    case PLAYING_ELEMENT:
      {
        unsigned long elementDuration;
        
        if (currentPlaybackMorse == "*") {
          // Speciální dlouhý tón pro znak *
          elementDuration = ditLength * 10;
        } else {
          // Normální elementy
          char element = currentPlaybackMorse[currentPlaybackMorseIndex];
          elementDuration = (element == '.') ? ditLength : (ditLength * 3);
        }
        
        if (now - playbackTimer >= elementDuration) {
          buzzerOff();
          playbackTimer = now;
          currentPlaybackMorseIndex++;
          
          if (currentPlaybackMorse == "*" || currentPlaybackMorseIndex >= currentPlaybackMorse.length()) {
            // Dokončen celý znak (včetně speciálního znaku *)
            playedCharCount++;  // Zvýšit počítadlo dokončených znaků
            
            // Kontrola, zda je čas na dlouhou mezeru (po každém 5. znaku)
            if (playedCharCount % 5 == 0) {
              playbackState = LONG_GAP;
            } else {
              playbackState = CHAR_GAP;
            }
          } else {
            // Mezera mezi elementy
            playbackState = ELEMENT_GAP;
          }
        }
      }
      break;
      
    case ELEMENT_GAP:
      if (now - playbackTimer >= ditLength) {
        // Začni hrát další element
        char element = currentPlaybackMorse[currentPlaybackMorseIndex];
        buzzerOn();
        playbackTimer = now;
        playbackState = PLAYING_ELEMENT;
      }
      break;
      
    case CHAR_GAP:
      if (now - playbackTimer >= (ditLength * 2)) { // celková mezera je 3*ditLength, ale už máme 1 z element_gap
        playbackIndex++;
        playbackState = IDLE;
      }
      break;
      
    case WORD_GAP:
      if (now - playbackTimer >= (ditLength * 10)) { // mezera mezi slovy je 10*ditLength
        playbackIndex++;
        playbackState = IDLE;
      }
      break;
      
    case LONG_GAP:
      // Dlouhá mezera po každém 5. znaku (10 * ditLength)
      if (now - playbackTimer >= (ditLength * 10)) {
        playbackIndex++;
        playbackState = IDLE;
      }
      break;
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
  pinMode(CLEAR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_BUTTON_PIN, INPUT_PULLUP);

  buzzerOff();

  lcd.init();
  lcd.backlight();

  updateEncoder();  // Zobraz první řádek

  int storedWPM = EEPROM.read(EEPROM_WPM_ADDR);
  if (storedWPM >= minWPM && storedWPM <= maxWPM) {
    wpm = storedWPM;
  }

  byte storedDir = EEPROM.read(EEPROM_DIR_ADDR);
  paddleReversed = (storedDir == 1);

  lastSavedWPM = wpm;
  encoder.write(0);
  updateDitLength();

  // info "boot" screen
  lcd.clear();

  lcd.setCursor(3, 0);  // 4 sloupec, 1 řádek
  lcd.print("CW Paddle Key");

  lcd.setCursor(2, 1);  // 5 sloupec, 2 řádek
  lcd.print("Morse - training");

  lcd.setCursor(0, 2);  // 1 sloupec, 3 řádek
  lcd.print("");

  lcd.setCursor(4, 3);  // 5 sloupec, 4 řádek
  lcd.print("2025-OK1MFG");

  delay(3000);  // 3s

  lcd.clear();
}


// -------------------------
// loop
// -------------------------

void loop() {
  unsigned long now = millis();

  updateEncoder();
  handleEncoderButton();

  // Zpracování přehrávání (non-blocking)
  handlePlayback();

  bool ditPressedRaw = digitalRead(paddleReversed ? DAH_PIN : DIT_PIN) == LOW;
  bool dahPressedRaw = digitalRead(paddleReversed ? DIT_PIN : DAH_PIN) == LOW;

  // Paddle input pouze pokud se nepřehrává
  if (!isPlaying) {
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
    } else if (now - lastToneTime >= toneDuration) {
      buzzerOff();
      tonePlaying = false;
      lastToneTime = now;
    }

    if (!ditPressedRaw && !dahPressedRaw && currentMorse.length() > 0 && (now - lastKeyTime > charGap)) {
      char decoded = decodeMorse(currentMorse);
      receivedText += decoded;
      if (receivedText.length() > 60) {
        receivedText = receivedText.substring(receivedText.length() - 60);  // max 3 řádky po 20 znacích
      }
      updateLCDText();
      currentMorse = "";
    }
  }

  unsigned long loopStart = millis();
  while (millis() - loopStart < 1)
    ;  // rychlý "non-blocking" delay

  // Kontrola tlačítka pro vymazání
  if (digitalRead(CLEAR_BUTTON_PIN) == LOW) {
    delay(50);  // debounce
    if (digitalRead(CLEAR_BUTTON_PIN) == LOW) {
      // Vymazání bufferu a LCD řádků 2-4
      currentMorse = "";
      receivedText = "";
      for (int row = 1; row <= 3; row++) {  // LCD řádky 2-4 (indexováno od 0)
        lcd.setCursor(0, row);
        lcd.print("                    ");  // 20 mezer na vymazání celého řádku
      }
      // Počkej, dokud tlačítko nebude uvolněné, aby se nevymazalo vícekrát
      while (digitalRead(CLEAR_BUTTON_PIN) == LOW) {
        delay(10);
      }
    }
  }

  // Kontrola tlačítka PLAY/STOP
  static bool lastPlayButtonState = HIGH;
  bool currentPlayButtonState = digitalRead(PLAY_BUTTON_PIN);
  
  // Detekce sestupné hrany (stisknutí tlačítka)
  if (lastPlayButtonState == HIGH && currentPlayButtonState == LOW) {
    delay(50);  // debounce
    
    // Ověř stav tlačítka po debounce
    if (digitalRead(PLAY_BUTTON_PIN) == LOW) {
      if (isPlaying) {
        // Pokud přehrává, zastav
        stopPlaybackNow();
                
      } else {
        // Pokud nepřehrává, spusť přehrávání
        if (receivedText.length() > 0) {
          startPlayback();
          
        }
      }
    }
  }
  
  lastPlayButtonState = currentPlayButtonState;
}