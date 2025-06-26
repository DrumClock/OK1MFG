/*
Tempo „PARIS"
Při vysílání Morseovy abecedy v akustické podobě se používají následující pravidla:

základní časovou jednotkou je délka tečky
čárka má stejnou dobu trvání jako tři tečky
zvuková pauza uvnitř značky má stejnou dobu trvání jako jedna tečka
zvuková pauza mezi značkami má stejnou dobu trvání jako jedna čárka
zvuková pauza mezi slovy má stejnou dobu trvání jako sedm teček

Tyto poměry umožňují sluchem zcela spolehlivě rozlišit tečku od čárky i druh akustické pauzy.
Za další, poměr celkové doby trvání vysílání tečky nebo čárky včetně pauzy uvnitř značky je 2:1,
což usnadňuje udržet konstantní rychlost vysílání. 
  
Pro určení rychlosti vysílání se bere jako reference pětipísmenné slovo „PARIS" (celkem 10 teček, 4 čárky, 4 mezery).
Celková doba jeho odvysílání včetně mezery za slovem je tedy 50 základních jednotek (délek teček). 
Pokud tedy například hovoříme o tempu vysílání 12 slov za minutu (WPM, words per minute), 
odpovídá to právě rychlosti průměrně jednoho písmene (znaku) za sekundu (12*5 = 60 znaků za minutu), 
délka tečky je 60/(12*50) = 0,1 sekundy.

*/

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// -------------------------
// Konfigurace
// -------------------------

#define BUZZER_ACTIVE_LOW true  // nastav na false, pokud je buzzer aktivní na HIGH

#define DIT_PIN 2  // pin podporuje "interrupt"
#define DAH_PIN 3  // pin podporuje "interrupt"

#define ENC_CLK 4  // pin podporuje PCINT2 (pin change interrupt 2)
#define ENC_DT 5   // pin podporuje PCINT2 (pin change interrupt 2)
//#define ENC_SW 6   // pin nefunguje, nahrazen pinem 10 !

#define CLEAR_BUTTON_PIN 7
#define PLAY_BUTTON_PIN 8

#define BUZZER_PIN 9

#define ENC_SW 10  // náhrada za pin 6 !

// LCD I2C adresa a velikost
LiquidCrystal_I2C lcd(0x27, 20, 4);
// SDA (data): pin A4
// SCL (clock): pin A5

// EEPROM
const int EEPROM_WPM_ADDR = 0;
const int EEPROM_DIR_ADDR = 1;

// -------------------------
// Morseova tabulka
// -------------------------

// Struktura pro jednu položku morseovky
struct MorseCode {
  char character;
  const char* morse;
};

// Pole s definicemi morseovky  
const MorseCode morseTable[] = {
  { 'A', ".-" }, { 'B', "-..." }, { 'C', "-.-." }, { 'D', "-.." },
  { 'E', "." }, { 'F', "..-." }, { 'G', "--." }, { 'H', "...." }, 
  { 'I', ".." }, { 'J', ".---" }, { 'K', "-.-" }, { 'L', ".-.." }, 
  { 'M', "--" }, { 'N', "-." }, { 'O', "---" }, { 'P', ".--." }, 
  { 'Q', "--.-" }, { 'R', ".-." }, { 'S', "..." }, { 'T', "-" }, 
  { 'U', "..-" }, { 'V', "...-" }, { 'W', ".--" }, { 'X', "-..-" }, 
  { 'Y', "-.--" }, { 'Z', "--.." }, { '0', "-----" }, { '1', ".----" }, 
  { '2', "..---" }, { '3', "...--" }, { '4', "....-" }, { '5', "....." }, 
  { '6', "-...." }, { '7', "--..." }, { '8', "---.." }, { '9', "----." }, 
  { '?', "..--.." }, { '/', "-..-." }, { '=', "-...-" }, { ',', "--..--" }, 
  { '.', ".-.-.-" }, { '*', "*" }, { ' ', " " }
};

const int morseTableSize = sizeof(morseTable) / sizeof(morseTable[0]);

// -------------------------
// Globální proměnné
// -------------------------

int wpm = 20;
int lastSavedWPM = 20;
const int minWPM = 5;
const int maxWPM = 60;
unsigned long ditLength = 60;

bool paddleReversed = false;
String currentMorse = "";
unsigned long lastKeyTime = 0;
bool elementPause = false;
unsigned long elementPauseStart = 0;
bool endDecode = true;


volatile bool pinClkState;
volatile bool pinDtState;
bool buttonPressed = false;
unsigned long buttonDownTime = 0;


bool tonePlaying = false;
unsigned long toneDuration = 0;
unsigned long lastToneTime = 0;

unsigned long lastDisplayUpdate = 0;
int lastDisplayedWPM = 0;
bool lastDisplayedPaddleState = false;

String receivedText = "";
String lastReceivedText = "";
String trainigText = "ABCDEFGHIJKLMNOPQRSTUVWXYZ 1234567890 ?/=,.";

// proměnné pro kontrolu přehrávání
volatile bool stopPlayback = false;
bool isPlaying = false;

// proměnné pro non-blocking přehrávání
String playbackText = "";
int playbackIndex = 0;
String currentPlaybackMorse = "";
int currentPlaybackMorseIndex = 0;
unsigned long playbackTimer = 0;
int playedCharCount = 0;  // Nová proměnná pro počítání znaků
enum PlaybackState { IDLE,
                     PLAYING_ELEMENT,
                     ELEMENT_GAP,
                     CHAR_GAP,
                     WORD_GAP,
                     LONG_GAP };
PlaybackState playbackState = IDLE;

// --- Časovače pro kontrolu tlačítek ---
unsigned long lastButtonCheckTime = 0;
unsigned long endDecodeTime = 0;  // Časovač pro endDecode delay

// ISR flagy
volatile bool ditHeld = false;
volatile bool dahHeld = false;


// -------------------------
// Funkce
// -------------------------

// ISR pro paddle 

void onDitPressed() {
  ditHeld = true;
}

void onDahPressed() {
  dahHeld = true;
}

// Nová funkce pro nastavení interruptů podle aktuálního paddle mode
void setupPaddleInterrupts() {
  // Nejprve odpojíme všechny interrupty
  detachInterrupt(digitalPinToInterrupt(DIT_PIN));
  detachInterrupt(digitalPinToInterrupt(DAH_PIN));
  
  // Pak je připojíme podle aktuálního stavu paddleReversed
  if (paddleReversed) {
    attachInterrupt(digitalPinToInterrupt(DIT_PIN), onDahPressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(DAH_PIN), onDitPressed, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(DIT_PIN), onDitPressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(DAH_PIN), onDahPressed, FALLING);
  }
}

// ----------------------------------------------------------

void buzzerOn() {
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? LOW : HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? HIGH : LOW);
}

void updateDitLength() {
  ditLength = 1200 / wpm;  
}

// ------------------------------------------------------------

String charToMorse(char c) {
  c = toupper(c);

  for (int i = 0; i < morseTableSize; i++) {
    if (morseTable[i].character == c) {
      return morseTable[i].morse;
    }
  }
  return "";  // neznámý znak
}


char decodeMorse(String morse) {
  for (int i = 0; i < morseTableSize; i++) {
    if (morse == morseTable[i].morse) {
      return morseTable[i].character;
    }
  }
  return '*';  // neznámý znak
}

// ------------------------------------------------------------

void updateEncoderDisplay() {
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

void togglePaddleDirection() {
  paddleReversed = !paddleReversed;
  EEPROM.write(EEPROM_DIR_ADDR, paddleReversed ? 1 : 0);
  
  // DŮLEŽITÉ: Přenastavit interrupty po změně paddle mode
  setupPaddleInterrupts();
  
  // Vyčistit současný morse kód, aby se předešlo nekonzistenci
  currentMorse = "";
  
  // přehraje .- nebo -. 
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

      // změna MODE a uložení do EEPROM
    } else if (millis() - buttonDownTime > 1500) {
      togglePaddleDirection();
      while (digitalRead(ENC_SW) == LOW)
        ;
      buttonPressed = false;
    }

    // změna WPM a uložení do EEPROM
  } else if (buttonPressed) {  // změna WPM
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
        // Speciální dlouhá pauza (10 ditLength)
        buzzerOff();
        playbackTimer = now;
        playbackState = PLAYING_ELEMENT;
        currentPlaybackMorseIndex = 0;  // Označíme jako první (a jediný) element
      } else if (currentPlaybackMorse == "") {
        // Neznámý znak, přeskočit
        playbackIndex++;
        // Zůstat v IDLE pro další znak
      } else {
        // Začni přehrávat první element znaku
        buzzerOn();
        playbackTimer = now;
        playbackState = PLAYING_ELEMENT;
      }
      break;

    case PLAYING_ELEMENT:
      {
        unsigned long elementDuration;

        if (currentPlaybackMorse == "*") {
          // Speciální dlouhá pauza pro znak *
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
        buzzerOn();
        playbackTimer = now;
        playbackState = PLAYING_ELEMENT;
      }
      break;

    case CHAR_GAP:
      if (now - playbackTimer >= (ditLength * 3)) {  // Opraveno: mezera mezi znaky je 3*ditLength
        playbackIndex++;
        playbackState = IDLE;
      }
      break;

    case WORD_GAP:
      if (now - playbackTimer >= (ditLength * 7)) {  // Opraveno: mezera mezi slovy je 7*ditLength
        playbackIndex++;
        playbackState = IDLE;
      }
      break;

    case LONG_GAP:
      // Dlouhá mezera po každém 5. znaku (7 * ditLength)
      if (now - playbackTimer >= (ditLength * 7)) {  // Opraveno: bylo 10, teď 7
        playbackIndex++;
        playbackState = IDLE;
      }
      break;
  }
}

// ------------------------------------------------------------

void playCustomSequence() {
  int originalWPM = wpm;

  // Můžete zde zadat libovolnou zprávu
  receivedText = "QRV";
  wpm = 20;  // rychlost
  updateDitLength();
  startPlayback();
}

//-----------------------------------------------------------------------

ISR(PCINT2_vect) {
  bool newPinClkState = PIND & (1 << PIND4);
  bool newPinDtState = PIND & (1 << PIND5);

  if (newPinClkState != pinClkState) {
    if (newPinClkState) {  // náběžná hrana
      if (newPinDtState != newPinClkState) {
        if (wpm < maxWPM) wpm++;
      } else {
        if (wpm > minWPM) wpm--;
      }
      updateDitLength();
    }
  }

  pinClkState = newPinClkState;
  pinDtState = newPinDtState;
}


// -------------------------
// Setup
// -------------------------

void setup() {

  // Serial.begin(9600); // do setup()

  pinMode(DIT_PIN, INPUT_PULLUP);
  pinMode(DAH_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);

  pinMode(CLEAR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_BUTTON_PIN, INPUT_PULLUP);

  buzzerOff();

  lcd.init();
  lcd.backlight();

  // Nastavení interruptů pro encoder
  pinClkState = digitalRead(ENC_CLK);
  pinDtState = digitalRead(ENC_DT);
  // Povolit PCI pro PCINT2 (piny 0-7 portu D)
  PCICR |= (1 << PCIE2);
  // Povolit PCI jen pro piny ENC_CLK a ENC_DT (PCINT20 a PCINT21)
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);
  sei();  // Globální povolení přerušení

  // Načtení hodnot z EEPROM
  int storedWPM = EEPROM.read(EEPROM_WPM_ADDR);
  if (storedWPM >= minWPM && storedWPM <= maxWPM) {
    wpm = storedWPM;
  }

  byte storedDir = EEPROM.read(EEPROM_DIR_ADDR);
  paddleReversed = (storedDir == 1);

  // Nastavení interruptů pro dit a dah podle načteného stavu
  setupPaddleInterrupts();

  lastSavedWPM = wpm;
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

  // Spusť přehrávání sekvence
  playCustomSequence();

  // Čekej až se přehrávání dokončí
  while (isPlaying) {
    handlePlayback();   // Zpracovává přehrávání
    delay(1);           // Malá pauza pro stability
    receivedText = "";  // smaže zprávu
  }

  // vrátí původní WPM
  wpm = lastSavedWPM;
  updateDitLength();
  lcd.clear();

  updateEncoderDisplay(); // Zobraz první řádek

  // nastaví úvodní text
  receivedText = trainigText;
  updateLCDText();
}


// -------------------------
// loop
// -------------------------

void loop() {
  unsigned long now = millis();
  

  if (!isPlaying) {
    // Pokud není tón a není pauza, můžeme hrát nový element
    if (!tonePlaying && !elementPause) {
      if (ditHeld) {
        buzzerOn();
        toneDuration = ditLength;
        tonePlaying = true;
        lastToneTime = now;
        currentMorse += ".";
        lastKeyTime = now;
      } else if (dahHeld) {
        buzzerOn();
        toneDuration = ditLength * 3;
        tonePlaying = true;
        lastToneTime = now;
        currentMorse += "-";
        lastKeyTime = now;
      }
    }

    // Konec tónu → začít pauzu
    else if (tonePlaying && (now - lastToneTime >= toneDuration)) {
      buzzerOff();
      tonePlaying = false;
      elementPause = true;
      elementPauseStart = now;
    }

    // Konec pauzy → připraven na nový element
    else if (elementPause && (now - elementPauseStart >= ditLength)) {
      elementPause = false;
      lastKeyTime = now;
      endDecode = false;
      endDecodeTime = 0;  // Reset časovače
    }

    // Detekce konce znaku  →  mezera mezi znaky je minimálně ditLength * 1
    if (!tonePlaying && !elementPause && currentMorse.length() > 0 && (now - lastKeyTime > ditLength * 0.8 ) ) {   
      char decoded = decodeMorse(currentMorse);
      receivedText += decoded;
      if (receivedText.length() > 60) {
        receivedText = receivedText.substring(receivedText.length() - 60);
      }
      //updateLCDText();
      currentMorse = "";
      endDecode = true;
      endDecodeTime = now;  // Uložit čas kdy byl nastaven endDecode
    }


  }

  // Detekce uvolnění tlačítek (v hlavní smyčce)
  ditHeld = digitalRead(paddleReversed ? DAH_PIN : DIT_PIN) == LOW;
  dahHeld = digitalRead(paddleReversed ? DIT_PIN : DAH_PIN) == LOW;

 // ------------------------------------------------------------------


  // Podmínka s 1 sekundovým zpožděním po endDecode
  if (endDecode && (now - endDecodeTime >= 1000)) {

   // obnoví LCD tex jen pokud se zmnění
   if (receivedText != lastReceivedText) {
      updateLCDText();
      lastReceivedText = receivedText;
      }


    handleEncoderButton();
    handlePlayback();  // Zpracování přehrávání (non-blocking)

    // -------------------------------------------------------------

    // Update Encoder display
    if (wpm != lastDisplayedWPM || paddleReversed != lastDisplayedPaddleState || now - lastDisplayUpdate > 200) {  // max každých 200ms
      updateEncoderDisplay();
      lastDisplayUpdate = now;
      lastDisplayedWPM = wpm;
      lastDisplayedPaddleState = paddleReversed;
    }
    
    // -------------------------------------------------------------

    // Kontrola tlačítek
    if (now - lastButtonCheckTime >= 200) {  // max každých 200 ms
      lastButtonCheckTime = now;

      // === Kontrola tlačítka CLEAR / RESET ===
      if (digitalRead(CLEAR_BUTTON_PIN) == LOW) {
        delay(50);  // debounce
        if (digitalRead(CLEAR_BUTTON_PIN) == LOW) {
          unsigned long pressStartTime = millis();
          bool longPressDetected = false;

          // Čekáme, dokud je tlačítko stisknuto a měříme čas
          while (digitalRead(CLEAR_BUTTON_PIN) == LOW) {
            delay(10);

            // dlouhé stisknutí > 1,5 s
            if (millis() - pressStartTime > 1500 && !longPressDetected) {
              longPressDetected = true;

              // Zobrazit zprávu o restartu
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("RESTART...");
              delay(1000);

              // Reset MCU
              asm volatile("jmp 0");  // Pro Arduino Uno/Nano
            }
          }

          // Pokud nebylo dlouhé stisknutí, provedeme normální vymazání
          if (!longPressDetected) {
            currentMorse = "";
            receivedText = "";
            for (int row = 1; row <= 3; row++) {
              lcd.setCursor(0, row);
              lcd.print("                    ");
            }
          }
        }
      }

      // === Kontrola tlačítka PLAY/STOP ===
      static bool lastPlayButtonState = HIGH;
      bool currentPlayButtonState = digitalRead(PLAY_BUTTON_PIN);

      if (lastPlayButtonState == HIGH && currentPlayButtonState == LOW) {
        delay(50);  // debounce
        if (digitalRead(PLAY_BUTTON_PIN) == LOW) {
          if (isPlaying) {
            stopPlaybackNow();
          } else {
            if (receivedText.length() > 0) {   
              //updateLCDText();           
              startPlayback();
            }
          }
        }
      }

      lastPlayButtonState = currentPlayButtonState;
    }
  }
}