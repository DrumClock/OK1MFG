/*
 ######################################################################
 ##################  VERZE - 19.4.2026  ###############################
 ######################################################################

 - Ovládání motoru rotátoru pomocí H-můstku
 - Snímání azimutu pomocí AS5600 (analog 0-5V → 0-360°)
 - Zobrazení azimutu / úhlu na displeji TM1637
 - Zobrazení azimutu pomocí LED na kruhové mapě
 - AUTOROTACE pomocí nastavení encoderem
 - Ovládání pomocí HAMlib protokolu - LOG Tučňák
 - Funkce "CQ Contest" - Postupně otáčí kyvadlově anténou v daném rozmezí úhlů
 - Kalibrace a nastavení pomocí tlačítek M1/C, M2/S, M3/F
 - AutoTest rotátoru při zapnutí pomocí tlačítek (M1-M3)
 - Nastavení a kalibrace se ukládá do FRAM (Adafruit MB85RC256V, I2C)
 - Softwarově nastavitelné END-stopy 
 - "Watchdog" ochrana při poruše čidla AS5600 


 ######################################################################
 #########################  nastavení #################################
 ######################################################################

  Krátký stisk je signalizovaný přehráním "." dlouhý poak ".."
 
 ------------------------ Při startu  ---------------------------------

 Na displeji se objeví "SEt" a začne odpočet 10,9,8... 
 
 Kalibrace severu ( uloží do FRAM ):
   enkodér PIN_SW  krátký stisk   na displeji → "nort", "SEt" 
    
 Funkce AutoTest:
  krátký stisk :
     M1 = mode 1 (sekvence úhlů)   na displeji → "tESt" → "rot"  → "1" 
     M2 = mode 2 (náhodný 60 min)  na displeji → "tESt" → "rot" → "60"
     M3 = mode 3 (náhodný 120 min) na displeji → "tESt" → "rot" → "120"
  
   Ukončení: CW / CCW → "End" → normální provoz

   ---------------      Během provozu ----------------------------------------

 Kalibrace AS5600 - tlačítka M1/C, M2/S, M3/F:
   Dlouhý stisk = vstup do kalibrace
     M1/C - nastavení GearRatio   encoderem (X.XXX) → krátký stisk M1/C = ulož
     M2/S - nastavení endStopCW   fyzickým otočením antény → krátký stisk M2/S = ulož + turns=0
     M3/F - nastavení endStopCCW  fyzickým otočením antény → krátký stisk M3/F = ulož + turns=0 + restart MCU
 
   Dlouhý stisk libovolného M = zrušit krok kalibrace

 ---------------------------------------------------------

  Krátký stisk 
     M1/C - Změna směru KY-040 Rotary Encoderu → na displeji zobrazí "Enc" + "rEV"
     M2/S - Změna směru snímače AS5600  → na displeji zobrazí " Sen" + "rEV"
     M3/F - reset MCU

 -------------------------------------------------------- 

 Funkce Contest:
   Dlouhý stisk encoderu 1,5s = nastavení + start od aktuální pozice
     "An 1" → enkodér → krátký stisk  = první úhel
     "An 2" → enkodér → krátký stisk  = druhý úhel
     "StEP" → enkodér → krátký stisk  = krok ve stupních
     "Int " → enkodér → krátký stisk  = interval v minutách
 
   Krátký stisk encoderu → restart od aktuální pozice
  
   Zrušení: PTT vstup, CW / CCW / encoder → "End" → normální provoz


######################################################################
##########################   ostatní  ################################
######################################################################

FRAM rozložení (Adafruit MB85RC256V, I2C, addr 0x50):
   [0]    = rezerva 
   [1]    = northOffset AS5600   (int, 2 bajty)   → addr 1
   [3]    = turns       AS5600   (int, 2 bajty)   → addr 3
   [5]    = endStopCW   AS5600   (int, 2 bajty)   → addr 5
   [7]    = endStopCCW  AS5600   (int, 2 bajty)   → addr 7
   [9]    = směr encoderu KY-040 (1 bajt)         → addr 9
   [10]   = směr snímače AS5600  (1 bajt)         → addr 10
   [11]   = gearRatio            (float, 4 bajty) → addr 11
   [15]   = sensorAccum          (float, 4 bajty) → addr 15 

  Výhoda oproti EEPROM: velmi vysoká odolnost proti zápisu.
  turns se uloží pouze při skutečné změně (přechod přes sever).
  sensorAccum se uloží jen při pohybu ≥ minMov (static lastSaved v saveSensorAccum).

  -------------------------------------------------------------------
 
 END-stopy (softwarové, absolutní pozice):
   Používají absolutní pozici = azimut + (turns × 360).
   turns sleduje přechody přes sever (CW = turns++, CCW = turns--).
   endStopCW  = max dovolená abs. pozice pro CW  (např. +400°)
   endStopCCW = min dovolená abs. pozice pro CCW (např. -40°)

  -------------------------------------------------------------------
 
 Watchdog zaseknutého snímače:
   Pokud motor běží a azimut se do 3s nemění → zastav (ochrana při poruše čidla).
  
   -------------------------------------------------------------------

*/


// ########################################################
// ############### inicializace knihoven ##################
// ########################################################

#include <Adafruit_NeoPixel.h>
#include <TM1637Display.h>
#include <Encoder.h>
#include <Adafruit_FRAM_I2C.h>
#include <math.h>

// ########################################################
// ############### zapojení  - Pinout    ##################
// ########################################################

/*
D0 (RX)
D1 (TX)
  jsou trvale připojené na USB převodník (CH340/FTDI)
  nepoužívat na nic jiného kvůli "Hamlibu"
*/

// KY-040 Rotary Encoder
#define PIN_CLK 2   // pin podporuje "interrupt"
#define PIN_DT  3   // pin podporuje "interrupt"
#define PIN_SW  4   // encoder SW

// Displej TM1637
#define DIO 5
#define CLK 6

// Tlačítka rotace
#define CCW_BUTTON 7
#define CW_BUTTON  8

// Buzzer - chyba snímače / SW endstop (aktivní LOW)
#define PIN_BUZZER 9

// Piny pro BTS7960 - motor rotátoru
#define CCW_RUN_PIN 10   // pin R_PWM
#define ENABLE_PWM  11   // pin EN_R + EN_L
#define CW_RUN_PIN  12   // pin L_PWM

// Neopixel Ring LED "mapa"
#define PIN_LED 13

// Analogový vstup - azimut antény
#define ANALOG_PIN A0

// Tlačítka pro kalibraci a předvolbu rotátoru
#define BUTTON_CAL_PIN  A1   // M1
#define BUTTON_SET_PIN  A2   // M2
#define BUTTON_FULL_PIN A3   // M3

/* I2C  - FRAM modul
   SDA  A4
   SCL  A5
*/

// PTT - přerušení Contest modu (jen analogový vstup)
#define PTT_PIN       A6
#define A6_THRESHOLD 200   // úroveň LOW

/*
#define xxx_PIN        A7   // jen analogový vstup
#define A7_THRESHOLD  200

if ( analogRead(xxx_PIN) < A7_THRESHOLD) {
   ....
   }
	
*/

// ########################################################
// ############### definice proměnných   ##################
// ########################################################

// --- Softwarové END-stopy (absolutní pozice) ---
int endStopCW  =  400;   // max abs. pozice CW  (°)
int endStopCCW =  -40;   // min abs. pozice CCW (°)

// --- FRAM adresy ---
const int FRAM_NORTH_OFFSET   =  1;   // 2 bajty: north offset AS5600
const int FRAM_TURNS          =  3;   // 2 bajty: počet otáček turns
const int FRAM_ENDSTOP_CW     =  5;   // 2 bajty: endStopCW
const int FRAM_ENDSTOP_CCW    =  7;   // 2 bajty: endStopCCW
const int FRAM_ENC_REVERSED   =  9;   // 1 bajt:  encoder směr 0=normální, 1=otočený
const int FRAM_SENSOR_REVERSED = 10;  // 1 bajt:  AS5600 směr  0=normální, 1=otočený
const int FRAM_GEAR_RATIO      = 11;  // 4 bajty: gear ratio (float)
const int FRAM_SENSOR_ACCUM    = 15;  // 4 bajty: sensorAccum (float)

// --- Kalibrační režim AS5600 ---
bool calibrationModeAS = false;
const int CALIB_CANCEL = -32768;

// --- Sledování přechodů přes sever ---
int turns  = 0;   // počet celých otáček přes sever
int lastAz = 0;   // předchozí azimut pro updateTurns()

// --- Akumulovaný úhel snímače (pro gear ratio > 1 nebo < 1) ---
float sensorAccum      = 0.0;  // absolutní úhel snímače (bez limitu 0-360)
float lastRawSensorDeg = -1.0; // předchozí raw stupeň snímače (0-359) pro detekci přechodu

// --- KY-040 Rotary Encoder ---
bool ENCODER_REVERSED = false; // Směr enkodéru: true=otočený
int lastPos = -1;
int ledPos  = 0;
int16_t AutoRotate = -1;                  // -1 = žádný cíl
const uint8_t HYSTERESIS_END_ANGLE = 5;   // hystereze zastavení AutoRotace (°)
unsigned long lastOutputChange = 0;
unsigned long lastChangeTime   = 0;
const unsigned long InactiveTime = 4000;  // ms

// --- Displej TM1637 ---
unsigned long lastUpdateTime   = 0;
const unsigned long updateInterval  = 50;    // ms
unsigned long changeUpdateTime = 0;
const unsigned long changeInterval  = 600;   // ms
int displayMode       = 0;    // 0 = úhel, 1 = Auto/Cont/tESt
int displayBrightness = 0x06;

// --- Neopixel ring ---
const uint8_t NumPixels      = 48;
const uint8_t RingBrightness = 2;
const float   DegPerLED      = 360.0 / NumPixels;   // = 7.5 — musí být float!

// --- Motor BTS7960 ---
const int   maxSpeed     = 255;
const uint16_t rampTimeUpMs   = 500;   // ms
const uint16_t rampTimeDownMs = 500;   // ms
const int   stepDelay    = 10;    // ms
bool isRunning    = false;
bool direction    = true;    // true = CW, false = CCW
int  currentSpeed = 0;

// --- Watchdog zaseknutého snímače ---
const unsigned long WatchdogTimeout = 2500;   // ms
unsigned long watchdogStart  = 0;
int16_t       watchdogLastAz = -999;
bool          watchdogActive = false;
const uint8_t   minMov = 6;        // minimální reálný pohyb ve ° pro reset watchdogu

// --- Analogový vstup ---
bool SENSOR_REVERSED  = false; // Směr AS5600:   true=otočený (mapuje 359→0 místo 0→359)
const int   ADC_MaxValue   = 1023;
int16_t lastAngle      = -1;
int   lastSensorValue = 0;
const uint8_t HysteresisAngle = 3;

// --- North offset ---
int northOffset = 0;

// --- Gear Ratio ---
float gearRatio = 1.0;   // převodový poměr snímač→výstup (např. 2.5 = snímač 2.5× rychlejší)

// --- tlacitka M1/M2/M3 ---
const unsigned int SHORT_PRESS_DURATION = 500;   // ms
const unsigned int LONG_PRESS_DURATION = 1500;   // ms
unsigned long pressStartSetup = 0;   // čas stisku detekovaného v setup() odpočtu

// --- Hamlib / Tučňák ---
int Hamlib_Azimuth   = 0;
int Hamlib_Elevation = 0;
int lastElevation    = 0;

// --- AutoTest ---
bool          testRunning            = false;
unsigned long testEndTime            = 0;
unsigned long testStartTime          = 0;
unsigned long testPauseUntil         = 0;
int           testStepCount          = 0;
bool          testWaitingForRotation = false;
int           testDurationSetup      = 0;   // 0=žádný, 1=sekvence, 2=60min, 3=120min

const int testSequence[] = { 1, 10, 90, 180, 270, 360, 315, 225, 135, 45 };
const int testSeqLen     = sizeof(testSequence) / sizeof(testSequence[0]);
int       testSeqIndex   = 1;
int       testSeqCycle   = 1;

// --- Contest (Scan) ---
bool          scanRunning            = false;
bool          scanWaitingForRotation = false;
unsigned long scanPauseUntil         = 0;
int           scanAngle1             = 135;
int           scanAngle2             = 235;
int           scanStep               = 2 * (int)round(360.0 / 48);
int           scanInterval           = 1;    // minuty
int           scanCurrentTarget      = 0;
bool          scanDirectionUp        = true;

int  scanAnimPos = 0;
bool scanAnimDir = true;
unsigned long scanAnimTime = 0;
const int SCAN_ANIM_STEP_MS = 50;
bool scanAnimReset = false;

// ########################################################
// ############### inicializace zařízení ##################
// ########################################################

Adafruit_NeoPixel strip = Adafruit_NeoPixel(48, PIN_LED, NEO_GRB + NEO_KHZ800);
TM1637Display display(CLK, DIO);
Encoder enc(PIN_CLK, PIN_DT);
Adafruit_FRAM_I2C fram;


// ########################################################
// ##### funkce které musí být před setup a loop ##########
// ########################################################


/* ------ tabulka zpráv na displeji ---------------
 volání:   displayStatus("klic", 800)
*/

struct SegMsg {
  const char*  key;
  uint8_t      segs[4];
};

// Přidej sem nové zprávy podle potřeby. Klíč je malými písmeny,   
const SegMsg segMessages[] = {
  { "set",   { 0x6d, 0x79, 0x78, 0x00 } },  // "SEt "  (set/save)
  { "end",   { 0x00, 0x79, 0x54, 0x5E } },  // " End"
  { "stop",  { 0x6d, 0x78, 0x5c, 0x73 } },  // "StoP"
  { "err",   { 0x79, 0x50, 0x50, 0x00 } },  // "Err "
  { "sen",   { 0x00, 0x6d, 0x79, 0x54 } },  // " SEn"
  { "test",  { 0x78, 0x79, 0x6D, 0x78 } },  // "tESt"
  { "rot",   { 0x00, 0x50, 0x5C, 0x78 } },  // " rot"
  { "enc",   { 0x00, 0x79, 0x54, 0x58 } },  // " Enc"
  { "rev",   { 0x50, 0x79, 0x3e, 0x00 } },  // "rEV "
  { "ptt",   { 0x73, 0x78, 0x78, 0x00 } },  // "Ptt "
  { "cal",   { 0x39, 0x77, 0x38, 0x00 } },  // "CAL " (kalibrace)
  { "abor",  { 0x77, 0x7c, 0x5c, 0x50 } },  // "Abor"  
  { "nort",  { 0x54, 0x5c, 0x50, 0x78 } },  // "nort"
  { "ecw",   { 0x79, 0x00, 0x39, 0x1c } },  // "E Cu" (end CW)
  { "eccw",  { 0x79, 0x39, 0x39, 0x1c } },  // "ECCu" (end CCW)  
  { "an1",   { 0x77, 0x54, 0x00, 0x06 } },  // "An 1"
  { "an2",   { 0x77, 0x54, 0x00, 0x5B } },  // "An 2"
  { "step",  { 0x6D, 0x78, 0x79, 0x73 } },  // "StEP"
  { "int",   { 0x06, 0x54, 0x78, 0x00 } },  // "Int "  
  { "gear",  { 0x7d, 0x79, 0x77, 0x50 } },  // "GEAr" (gear ratio)
  { "---",   { 0x40, 0x40, 0x40, 0x40 } },  // "----" (Neznámý klíč)
};

// Zobrazí zprávu podle klíče a počká delayMs ms, pak smaže displej.
void displayStatus(const char* key, uint16_t delayMs = 800) {
  const int count = sizeof(segMessages) / sizeof(segMessages[0]);
  for (int i = 0; i < count; i++) {
    if (strcmp(segMessages[i].key, key) == 0) {
      display.setSegments(segMessages[i].segs, 4, 0);
      delay(delayMs);
      display.clear();
      return;
    }
  }
  // fallback: neznámý klíč → rekurzivně zobraz "----"
  displayStatus("---", delayMs);
}



/* ----------- scrollovací zprávy na displeji ----------------
Volání:
msgScroll("as");         // výchozí rychlost 250 ms/znak
msgScroll("as", 150);    // rychlejší
*/

struct ScrollMsg {
  const char* key;
  const uint8_t* data;
  uint8_t len;
};

// Data scrollovacích zpráv 
const uint8_t scrollAS[] = {
  0x00, 0x00, 0x00, 0x00,  // mezera (nájezd)
  0x77, 0x54, 0x77, 0x38,  // AnAL
  0x5c, 0x3d, 0x00, 0x6d,  // oG S
  0x79, 0x54, 0x6d, 0x5c,  // EnSo
  0x50, 0x00, 0x77, 0x6d,  // r AS
  0x40, 0x6d, 0x7d, 0x3f,  // -560
  0x3f, 0x00, 0x00, 0x00,  // 0
  0x00, 0x00, 0x00, 0x00   // mezera (odjezd)
};

// sem přidávej další scrollovací zprávy...
// const uint8_t scrollBoot[] = { ... };

const ScrollMsg scrollMessages[] = {
   { "as",   scrollAS,   sizeof(scrollAS)   }, 
// { "boot", scrollBoot, sizeof(scrollBoot) },
};


void msgScroll(const char* key, uint16_t stepMs = 250) {
  const int count = sizeof(scrollMessages) / sizeof(scrollMessages[0]);
  for (int i = 0; i < count; i++) {
    if (strcmp(scrollMessages[i].key, key) == 0) {
      const uint8_t* d   = scrollMessages[i].data;
      uint8_t        len = scrollMessages[i].len;
      uint8_t window[4];
      for (int pos = 0; pos < len - 3; pos++) {
        for (int j = 0; j < 4; j++) window[j] = d[pos + j];
        display.setSegments(window, 4, 0);
        delay(stepMs);
      }
      return;
    }
  }
}



// -------------- readButtons  ----------------------
//
// Vrátí aktuální stav M1/M2/M3 jako bitmask (aktivní LOW):
//   bit 0 = M1 (BUTTON_CAL_PIN)
//   bit 1 = M2 (BUTTON_SET_PIN)
//   bit 2 = M3 (BUTTON_FULL_PIN)
//
// Kombinace:
//   0b000 = 0  žádné
//   0b001 = 1  M1
//   0b010 = 2  M2
//   0b011 = 3  M1+M2
//   0b100 = 4  M3
//   0b101 = 5  M1+M3
//   0b110 = 6  M2+M3
//   0b111 = 7  M1+M2+M3

// Pojmenované konstanty pro čitelnost v kódu
const uint8_t BTN_NONE     = 0b000;
const uint8_t BTN_M1       = 0b001;
const uint8_t BTN_M2       = 0b010;
const uint8_t BTN_M1_M2    = 0b011;
const uint8_t BTN_M3       = 0b100;
const uint8_t BTN_M1_M3    = 0b101;
const uint8_t BTN_M2_M3    = 0b110;
const uint8_t BTN_M1_M2_M3 = 0b111;

uint8_t readButtons() {
  return (!digitalRead(BUTTON_CAL_PIN)  ? BTN_M1 : 0) |
         (!digitalRead(BUTTON_SET_PIN)  ? BTN_M2 : 0) |
         (!digitalRead(BUTTON_FULL_PIN) ? BTN_M3 : 0);
}


// --- Sledování přechodů přes sever ---
// CW přechod (359→0): diff < -180 → turns++
// CCW přechod (0→359): diff > +180 → turns--
void updateTurns(int az) {
  int diff = az - lastAz;
  if (diff < -180) turns++;
  if (diff >  180) turns--;
  lastAz = az;
  saveTurns();
}

// --- Absolutní pozice: azimut + celé otáčky ---
int getAbsolutePosition(int az) {
  return az + (turns * 360);
}

// --- Čtení azimutu ze AS5600 (analog 0-5V → akumulovaný úhel → gear ratio → north offset) ---
//
// sensorAccum sleduje absolutní úhel snímače včetně více otáček (např. 0-720° pro gear ratio 2.0).
// Výstupní úhel = (sensorAccum / gearRatio) normalizovaný na 0-359°.
//
float readSensorAngle() {
  int rawADC = analogRead(ANALOG_PIN);
  lastSensorValue = rawADC;

  // Raw stupně snímače 0-359 (s případnou reverzí)
  float rawDeg = SENSOR_REVERSED
    ? (float)map(rawADC, 0, 1023, 359, 0)
    : (float)map(rawADC, 0, 1023, 0, 359);

  // Akumulace: detekce přechodu přes 0/360 snímače
  if (lastRawSensorDeg >= 0.0) {
    float diff = rawDeg - lastRawSensorDeg;
    if (diff < -180.0) diff += 360.0;   // CW přechod přes 360→0
    if (diff >  180.0) diff -= 360.0;   // CCW přechod přes 0→360
    sensorAccum += diff;
    saveSensorAccum();
  }
  lastRawSensorDeg = rawDeg;

  // Výstupní úhel: northOffset v raw stupních snímače, pak gear ratio
  float outputDeg = sensorAccum - (float)northOffset;
  if (gearRatio > 0.01f) outputDeg = outputDeg / gearRatio;

  // Normalizace na 0-359°
  outputDeg = fmod(outputDeg, 360.0);
  if (outputDeg <    0) outputDeg += 360.0;
  if (outputDeg >= 360) outputDeg -= 360.0;
  return outputDeg;
}

// --- END-stopy: kontrola pohybu (jen AS5600, absolutní pozice) ---
bool movementAllowed(bool movingCW, int absPos) {
  if ( movingCW && absPos >= endStopCW)  return false;
  if (!movingCW && absPos <= endStopCCW) return false;
  return true;
  }


// --- Beeper - pípnutí (aktivní LOW) ---
bool endstopBeepDone = false;

void beep(int count, int onMs = 150, int offMs = 150) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_BUZZER, LOW);  delay(onMs);
    digitalWrite(PIN_BUZZER, HIGH); delay(offMs);
  }
}

// --- Morse Beeper ----
// např.:  beepMorse("..--.."); // zahraje ?

const int DIT = 50;
const int DAH = 150;
const int GAP = 50;        // mezera mezi prvky znaku
const int CHAR_GAP = 150;  // mezera mezi znaky
const int WORD_GAP = 350;  // mezera mezi slovy

// přehraje Morse kód ze stringu (např. ".-.-")
void beepMorse(const char* code) {
  for (int i = 0; code[i] != '\0'; i++) {
    if (code[i] == '.') {
      beep(1, DIT, GAP);
    } else if (code[i] == '-') {
      beep(1, DAH, GAP);
    }
  }
  delay(CHAR_GAP);
}

/*
    // A-Z
    {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},  {'E', "."},
    {'F', "..-."}, {'G', "--."},  {'H', "...."}, {'I', ".."},   {'J', ".---"},
    {'K', "-.-"},  {'L', ".-.."}, {'M', "--"},   {'N', "-."},   {'O', "---"},
    {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."},  {'S', "..."},  {'T', "-"},
    {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"}, {'Y', "-.--"},
    {'Z', "--.."},

    // 0-9
    {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"},
    {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."},

    // Speciální znaky
    {'?', "..--.."}, {'.', ".-.-.-"}, {',', "--..--"}, {'!', "-.-.--"},
    {'/', "-..-."},  {'(', "-.--."},  {')', "-.--.-"} 
*/


// --- END-stopy: varování na displeji + beeper ---
// beeper pouze jednou při prvním volání, dokud není "endstopBeepDone" resetován.


void showEndstopWarning() {
  if (!endstopBeepDone) {    
    beepMorse("-.-");   // beeper zahraje "K" jako konec
    endstopBeepDone = true;
  }
  uint8_t segEnd[]  = { 0x00, 0x79, 0x54, 0x5E };  // "End"
  uint8_t segStop[] = { 0x6d, 0x78, 0x5c, 0x73 };  // "StoP"
  display.setSegments(segEnd,  4, 0); delay(300);
  display.setSegments(segStop, 4, 0); delay(300);
}

// ########################################################
// ############### FRAM read / write helpers ##############
// ########################################################
//
// Náhrada za EEPROM.get() / EEPROM.put() pro libovolný typ.
// Adafruit_FRAM_I2C pracuje po bajtech — čteme/zapisujeme po bajtu.

template<typename T>
void framWrite(uint16_t addr, const T& val) {
  const uint8_t* p = (const uint8_t*)&val;

  //Serial.print(F(" FRAM Write ["));Serial.print(addr);Serial.print(F("] "));Serial.println(val);
  
  for (uint8_t i = 0; i < sizeof(T); i++) {
    fram.write(addr + i, p[i]);   
  }
  
}

template<typename T>
void framRead(uint16_t addr, T& val) {
  uint8_t* p = (uint8_t*)&val;
  for (uint8_t i = 0; i < sizeof(T); i++) {
    p[i] = fram.read(addr + i);
  }
}

// --- North offset: načtení z FRAM ---
void loadNorthOffset() {
  framRead(FRAM_NORTH_OFFSET, northOffset);
  if (northOffset < 0 || northOffset > 359) northOffset = 0;
}

// --- North offset: uložení do FRAM ---
void saveNorthOffset(int offset) {
  northOffset = offset;
  framWrite(FRAM_NORTH_OFFSET, northOffset);
}

// --- Gear Ratio: načtení/uložení ---
void loadGearRatio() {
  float val;
  framRead(FRAM_GEAR_RATIO, val);
  if (val >= 0.1 && val <= 9.999) {
    gearRatio = val;
  } else {
    gearRatio = 1.0;
    framWrite(FRAM_GEAR_RATIO, gearRatio);  // uložit default
  }
}

void saveGearRatio(float val) {
  gearRatio = val;
  framWrite(FRAM_GEAR_RATIO, gearRatio);
}

// --- SensorAccum: načtení/uložení ---
void loadSensorAccum() {
  float val;
  framRead(FRAM_SENSOR_ACCUM, val);
  if (!isnan(val) && val > -36000.0 && val < 36000.0) sensorAccum = val;
  else sensorAccum = 0.0;
}

void saveSensorAccum() {
  static float lastSaved = -99999.0;
  if (fabs(sensorAccum - lastSaved) >= minMov) {
    lastSaved = sensorAccum;
    framWrite(FRAM_SENSOR_ACCUM, sensorAccum);
  }
}

// Vynucený zápis (při kalibraci) - resetuje interní práh
void saveSensorAccumForce() {
  framWrite(FRAM_SENSOR_ACCUM, sensorAccum);
  // reset static threshold v saveSensorAccum přes pomocný trik:
  // příště normální saveSensorAccum se chová jako by se nic neuložilo → safe
}

// --- Turns: uložení/načtení ---
void saveTurns() {
  static int lastSaved = -999;
  if (turns != lastSaved) {
    lastSaved = turns;
    framWrite(FRAM_TURNS, turns);
  }
}

void loadTurns() {
  framRead(FRAM_TURNS, turns);
  if (turns < -10 || turns > 10) turns = 0;
}

// --- END-stopy AS5600: uložení/načtení ---
void saveEndStops() {
  framWrite(FRAM_ENDSTOP_CW,  endStopCW);
  framWrite(FRAM_ENDSTOP_CCW, endStopCCW);
  //Serial.print(F("Ulozeny endStopCW=")); Serial.print(endStopCW);
  //Serial.print(F("  endStopCCW="));      Serial.println(endStopCCW);
}

void loadEndStops() {
  int cw, ccw;
  framRead(FRAM_ENDSTOP_CW,  cw);
  framRead(FRAM_ENDSTOP_CCW, ccw);
  // sanity check: rozsah -360..720
  if (cw  > -360 && cw  <= 720) endStopCW  = cw;
  if (ccw > -360 && ccw <= 720) endStopCCW = ccw;
  //Serial.print(F("Nacteny endStopCW=")); Serial.print(endStopCW);
  //Serial.print(F("  endStopCCW="));      Serial.println(endStopCCW);
}

// --- Kalibrace Gear Ratio encoderem ---
//
// Displej zobrazuje X.XXX (4 cifry, desetinná tečka za první cifrou).
// Aktivní cifra bliká každých 400 ms.
// Encoder otočení → mění cifru 0-9
// Encoder stisk   → posun na další cifru (kruhově 0→1→2→3→0)
// M1 krátký stisk → uloží do FRAM a vrátí se
// M1 dlouhý stisk → zruší (vrátí beze změny)
//
void calibrateGearRatio() {
  // Rozložení aktuálního gearRatio do 4 číslic: d[0].d[1]d[2]d[3]
  int d[4];
  int tmp = (int)round(gearRatio * 1000.0);
  tmp = constrain(tmp, 100, 9999);   // 0.100 .. 9.999
  d[0] = tmp / 1000;
  d[1] = (tmp / 100) % 10;
  d[2] = (tmp /  10) % 10;
  d[3] =  tmp        % 10;

  int  activeDigit  = 0;
  bool blinkState   = true;
  unsigned long lastBlink = millis();
  const unsigned long BLINK_MS = 400;
  const unsigned long LONG_MS  = 1500;

  long lastEncPos = enc.read() / 4;

  // --- Pomocná lambda: zobraz 4 cifry, aktivní cifra bliká (prázdný segment) ---
  auto showDigits = [&](bool showActive) {
    const uint8_t SEG_DIGITS[] = {
      0x3F, 0x06, 0x5B, 0x4F, 0x66,   // 0 1 2 3 4
      0x6D, 0x7D, 0x07, 0x7F, 0x6F    // 5 6 7 8 9
    };
    uint8_t segs[4];
    for (int i = 0; i < 4; i++) {
      segs[i] = (i == activeDigit && !showActive) ? 0x00 : SEG_DIGITS[d[i]];
    }
    segs[0] |= 0x80;   // desetinná tečka za prvním digitem zleva
    display.setSegments(segs, 4, 0);
    // Některé verze TM1637 ignorují 0x80 v setSegments → zapsat digit 0 znovu zvlášť
    uint8_t dot = segs[0];
    display.setSegments(&dot, 1, 0);
  };

  showDigits(true);

  while (true) {

    // --- Blikání aktivní cifry ---
    if (millis() - lastBlink >= BLINK_MS) {
      lastBlink  = millis();
      blinkState = !blinkState;
      showDigits(blinkState);
    }

    // --- Encoder otočení: mění aktivní cifru 0-9 ---
    long encPos = enc.read() / 4;
    if (encPos != lastEncPos) {
      int delta = ENCODER_REVERSED ? -(encPos - lastEncPos) : (encPos - lastEncPos);
      d[activeDigit] = (d[activeDigit] - delta % 10 + 10) % 10;
      // Digit 0 nesmí být 0 (gear ratio >= 1.000 ... vlastně může být 0 = 0.xxx)
      // Necháme volné 0-9 pro flexibilitu, sanity check proběhne při ukládání
      lastEncPos = encPos;
      blinkState = true; lastBlink = millis();
      showDigits(true);
    }

    // --- Encoder stisk: posun cifry ---
    if (digitalRead(PIN_SW) == LOW) {
      unsigned long t = millis();
      while (digitalRead(PIN_SW) == LOW) {
        if (millis() - t > LONG_MS) {
          while (digitalRead(PIN_SW) == LOW);
          // dlouhý stisk encoderu = jen ignoruj, neopouštěj (rezerva)
          break;
        }
      }
      if (millis() - t < LONG_MS) {
        activeDigit = (activeDigit + 1) % 4;
        blinkState = true; lastBlink = millis();
        showDigits(true);
      }
    }

    // --- M1 tlačítko ---
    if (digitalRead(BUTTON_CAL_PIN) == LOW) {
      unsigned long t = millis();
      while (digitalRead(BUTTON_CAL_PIN) == LOW) {
        if (millis() - t > LONG_MS) {
          while (digitalRead(BUTTON_CAL_PIN) == LOW);
          // Dlouhý stisk M1 = zrušit
          displayStatus("abor", 800);
          return;
        }
      }
      // Krátký stisk M1 = uložit
      float newVal = d[0] + d[1] * 0.1f + d[2] * 0.01f + d[3] * 0.001f;
      if (newVal < 0.1f) newVal = 0.1f;   // sanity: nenulové
      saveGearRatio(newVal);
      displayStatus("set", 1000);
      //Serial.print(F("Gear ratio ulozeno: ")); Serial.println(gearRatio, 3);
      return;
    }
  }
}

// --- Kalibrace severu  ---
void calibrateNorth() {
  //Serial.println(F("=== Kalibrace severu (AS5600) ==="));
  //Serial.println(F("Drz enkoder... pust = ulozit sever"));

  displayStatus("nort", 800);
  
  while (digitalRead(PIN_SW) == LOW) {
    display.showNumberDec(map(analogRead(ANALOG_PIN), 0, 1023, 0, 359), false);
    delay(50);
  }

  saveNorthOffset(map(analogRead(ANALOG_PIN), 0, 1023, 0, 359));
  turns       = 0;
  sensorAccum = (float)northOffset;   // výchozí pozice = offset (= 0° výstupu)
  lastRawSensorDeg = SENSOR_REVERSED
    ? (float)map(analogRead(ANALOG_PIN), 0, 1023, 359, 0)
    : (float)map(analogRead(ANALOG_PIN), 0, 1023, 0, 359);
  lastAz = (int)readSensorAngle();
  saveTurns();
  saveSensorAccumForce();
  
  displayStatus("set", 1000);
  
  //Serial.print(F("North offset (deg): ")); Serial.println(northOffset);
  //Serial.println(F("Turns reset na 0."));
}

// --- Kalibrace AS5600: nastavení hodnoty fyzickým otočením antény ---
//
// label[4]    = 4bajtový segment kód popisku (např. "nort", "E Cu", "E CCU")
//               zobrazí se 1000 ms, pak se střídá s live azimutem každých 600 ms.
// confirmBtn  = tlačítko pro uložení: 1=M1/C, 2=M2/S, 3=M3/F
//
// Vrací aktuální azimut (0-359°, s northOffset) v okamžiku uložení.
// Volající si sám rozhodne co s hodnotou udělá (uloží přímo / +360 / -360).
// Dlouhý stisk libovolného M tlačítka = zrušit (vrátí CALIB_CANCEL).
//
int calibAS_setPhysical(const uint8_t label[4], uint8_t confirmBtn) {
  bool          showLabel = true;
  unsigned long lastFlip  = millis();
  const unsigned long FLIP_MS = 600;
  const unsigned long LONG_MS = 1500;

  display.setSegments(label, 4, 0);
  delay(1000);

  while (true) {
    // --- Živý azimut (s northOffset, stejně jako readSensorAngle()) ---
    int rawDeg = map(analogRead(ANALOG_PIN), 0, 1023, 0, 359);
    int az     = rawDeg - northOffset;
    if (az <    0) az += 360;
    if (az >= 360) az -= 360;

    // --- Neopixel: živá pozice antény (oranžová tečka) ---
    strip.clear();
    int ledIdx = (int)round((float)az / DegPerLED) % NumPixels;
    strip.setPixelColor(ledIdx, strip.Color(255, 80, 0));
    strip.show();

    // --- CW / CCW tlačítka: roztočit motor ---
    if      (digitalRead(CW_BUTTON)  == LOW) { CW_Motor();  }
    else if (digitalRead(CCW_BUTTON) == LOW) { CCW_Motor(); }
    else    { if (isRunning) stopMotor(); }

    // --- Tlačítka M1 / M2 / M3 ---
    bool bCal  = !digitalRead(BUTTON_CAL_PIN);
    bool bSet  = !digitalRead(BUTTON_SET_PIN);
    bool bFull = !digitalRead(BUTTON_FULL_PIN);

    if (bCal || bSet || bFull) {
      uint8_t which = bCal ? 1 : (bSet ? 2 : 3);
      unsigned long t = millis();
      while (digitalRead(BUTTON_CAL_PIN) == LOW || digitalRead(BUTTON_SET_PIN) == LOW || digitalRead(BUTTON_FULL_PIN) == LOW) {
        if (millis() - t > LONG_MS) {
          while (digitalRead(BUTTON_CAL_PIN) == LOW || digitalRead(BUTTON_SET_PIN) == LOW || digitalRead(BUTTON_FULL_PIN) == LOW);
          if (isRunning) stopMotor();
          strip.clear(); strip.show();
          return CALIB_CANCEL;
        }
      }
      if (which == confirmBtn) {
        if (isRunning) stopMotor();
        // Čerstvé čtení po zastavení motoru
        rawDeg = map(analogRead(ANALOG_PIN), 0, 1023, 0, 359);
        az     = rawDeg - northOffset;
        if (az <    0) az += 360;
        if (az >= 360) az -= 360;
        strip.clear(); strip.show();
        return az;
      }
    }

    // --- Střídání displeje popisek / azimut ---
    if (millis() - lastFlip >= FLIP_MS) {
      lastFlip  = millis();
      showLabel = !showLabel;
      if (showLabel) display.setSegments(label, 4, 0);
      else           display.showNumberDec(az, false);
    }

    delay(20);
  }
}

// --- Test LED + displej při startu ---
void test_LED_DISPLAY(int interval = 1) {
  for (int ledIndex = 0; ledIndex < NumPixels; ledIndex++) {
    int sec = round(NumPixels / 4.0);
    strip.setPixelColor(ledIndex, (ledIndex % sec == 0) ? strip.Color(255, 0, 0) : strip.Color(0, 255, 0));
    strip.show();
    display.showNumberDec((int)map(ledIndex, 0, NumPixels - 1, 0, 360), false);
    delay(interval);
  }
  delay(500);
  strip.clear(); strip.show(); display.clear();
}


// ########################################################
// #################### setup  ############################
// ########################################################

void setup() {
  Serial.begin(9600);

  strip.begin();
  strip.setBrightness(RingBrightness);
  strip.show();

  display.setBrightness(displayBrightness);
  display.clear();

  pinMode(PIN_CLK,         INPUT_PULLUP);
  pinMode(PIN_DT,          INPUT_PULLUP);
  pinMode(PIN_SW,          INPUT_PULLUP);
  pinMode(CW_BUTTON,       INPUT_PULLUP);
  pinMode(CCW_BUTTON,      INPUT_PULLUP);
  pinMode(PIN_BUZZER,      OUTPUT);
  
  pinMode(ENABLE_PWM,      OUTPUT);
  pinMode(CW_RUN_PIN,      OUTPUT);
  pinMode(CCW_RUN_PIN,     OUTPUT);
  pinMode(BUTTON_CAL_PIN,  INPUT_PULLUP);
  pinMode(BUTTON_SET_PIN,  INPUT_PULLUP);
  pinMode(BUTTON_FULL_PIN, INPUT_PULLUP);
  
  digitalWrite(PIN_BUZZER, HIGH);    // HIGH = mlčí (aktivní LOW)
  digitalWrite(ENABLE_PWM,    LOW);
  digitalWrite(CW_RUN_PIN,    LOW);
  digitalWrite(CCW_RUN_PIN,   LOW);

  // Inicializace FRAM
  if (!fram.begin()) {
    Serial.println(F("!!! FRAM nenalezena na I2C !!!"));
    beep(3, 300, 200);   // trojité pípnutí = chyba HW
  } else {
    Serial.println(F("FRAM OK"));
  }

  // Načtení směru enkodéru a snímače
  ENCODER_REVERSED = fram.read(FRAM_ENC_REVERSED)    == 1;
  SENSOR_REVERSED  = fram.read(FRAM_SENSOR_REVERSED) == 1;

  // Načtení kalibračních hodnot AS5600
  loadNorthOffset();
  loadGearRatio();
  loadTurns();
  loadSensorAccum();
  loadEndStops();

  // Inicializace lastRawSensorDeg bez akumulace (jen čtení raw)
  {
    int rawADC = analogRead(ANALOG_PIN);
    lastRawSensorDeg = SENSOR_REVERSED
      ? (float)map(rawADC, 0, 1023, 359, 0)
      : (float)map(rawADC, 0, 1023, 0, 359);
  }
  lastAz = (int)readSensorAngle();

  Serial.println(F("=== SENSOR: AS5600 analog ==="));
  Serial.print(F("North offset: ")); Serial.println(northOffset);
  Serial.print(F("Gear ratio:   ")); Serial.println(gearRatio, 3);
  Serial.print(F("SensorAccum:  ")); Serial.println(sensorAccum, 1);
  Serial.print(F("Turns: "));        Serial.println(turns);
  
  lastPos        = 0;
  lastChangeTime = millis();
 
  test_LED_DISPLAY();  // test displeje a neopixel LED

  Serial.println(F("=== Rotator start ==="));
  Serial.print(F("Abs. pozice: ")); Serial.println(getAbsolutePosition((int)readSensorAngle()));
  Serial.print(F("endStopCW: "));  Serial.println(endStopCW);
  Serial.print(F("endStopCCW: ")); Serial.println(endStopCCW);

  
  //msgScroll("as"); // scrolovací text: Analog Senzor AS-5600
  displayStatus("set", 800);

  // --- Odpočet 10s: čeká jen na fyzický stisk, nic nevyhodnocuje ---
  // Jakmile je detekován stisk (libovolné tlačítko nebo SW), skočí na do_check.
  // Po 10s bez stisku přeskočí checkButtons a jde rovnou na setup_end.
  for (int countdown = 10; countdown >= 1; countdown--) {
    display.showNumberDec(countdown, false);
    unsigned long tickStart = millis();
    while (millis() - tickStart < 1000UL) {
      if (digitalRead(PIN_SW)          == LOW ||
          digitalRead(BUTTON_CAL_PIN)  == LOW ||
          digitalRead(BUTTON_SET_PIN)  == LOW ||
          digitalRead(BUTTON_FULL_PIN) == LOW) {
        pressStartSetup = millis();   // zaznamenej čas stisku
        goto do_check;
      }
      delay(10);
    }
  }
  display.clear();
  goto setup_end;

  do_check:
    display.clear();
    checkButtons("setup");   // tlačítko stále drženo → rozliší krátký/dlouhý + aktivuje funkce
  setup_end:

  beepMorse(".-.");   // beeper zahraje "R" jako ready

  
}

 
// ########################################################
// #################### loop   ############################
// ########################################################

void loop() {

  // --- AutoTest ---
  if (testDurationSetup > 0) AutoTest();

  // --- Contest ---
  scanRun();
  if (scanRunning && analogRead(PTT_PIN) < A6_THRESHOLD) {
    displayStatus("ptt", 1500);
    scanStop();
  }

  // --- Hamlib / Tučňák ---
  Hamlib_Tucnak();

  // --- Tlačítka M1/M2/M3: volba snímače + kalibrace ---
  checkButtons("loop");

  // --- Čtení azimutu ---
  float angle = readSensorAngle();
  if (lastAngle == -1 || abs(angle - lastAngle) >= HysteresisAngle) {
    lastAngle = angle;
  }

  // --- Absolutní pozice + sledování otáček ---
  updateTurns((int)angle);
  int absPos = getAbsolutePosition(lastAngle);



  // --- Watchdog zaseknutého snímače ---
  watchdog(angle);


  // --- Zobrazení: Neopixel + displej ---
  int ledIndex = (int)round((float)lastAngle / DegPerLED) % NumPixels;

  if (millis() - lastChangeTime > InactiveTime && AutoRotate == -1) {
    ledPos = ledIndex;
  }

  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis();
    strip.clear();

    // Animace Cont
    if (scanRunning) {
      int ledA1 = (scanAngle1 * NumPixels / 360) % NumPixels;
      int ledA2 = (scanAngle2 * NumPixels / 360) % NumPixels;
      strip.setPixelColor(ledA1, strip.Color(0, 0, 255));
      strip.setPixelColor(ledA2, strip.Color(0, 0, 255));
      if (AutoRotate == -1) {
        if (scanAnimReset) {
          scanAnimPos   = constrain((int)round((float)lastAngle / DegPerLED) % NumPixels, ledA1 + 1, ledA2 - 1);
          scanAnimDir   = true;
          scanAnimReset = false;
        }
        if (millis() - scanAnimTime >= SCAN_ANIM_STEP_MS) {
          scanAnimTime = millis();
          if (scanAnimDir) { scanAnimPos++; if (scanAnimPos >= ledA2 - 1) scanAnimDir = false; }
          else             { scanAnimPos--; if (scanAnimPos <= ledA1 + 1) scanAnimDir = true;  }
          scanAnimPos = constrain(scanAnimPos, ledA1 + 1, ledA2 - 1);
        }
        strip.setPixelColor(scanAnimPos, strip.Color(0, 0, 255));
      }
    }


    // Barva aktuální pozice
    // turns > 0 (CW přes sever):  červené od LED 0 do ledIndex
    // turns < 0 (CCW přes sever): červené od LED 0 dozadu do ledIndex
    // turns == 0: normální zobrazení
    int sec = round(NumPixels / 4.0);

    if (turns > 0) {
      for (int i = 0; i <= ledIndex; i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
      }
    } else if (turns < 0) {
      for (int i = 0; i >= -((NumPixels - ledIndex) % NumPixels); i--) {
        int idx = (NumPixels + i) % NumPixels;
        strip.setPixelColor(idx, strip.Color(255, 0, 0));
      }
    }


    // Aktuální pozice - červená na kardinálních bodech (0,90,180,270°), jinak zelená
    if (ledIndex % sec == 0) {
      strip.setPixelColor(ledIndex, strip.Color(255, 0, 0));
    } else {
      strip.setPixelColor(ledIndex, strip.Color(0, 255, 0));
    }

    // Přepínání displeje
    if ((AutoRotate != -1 || scanRunning) && millis() - changeUpdateTime >= changeInterval) {
      changeUpdateTime = millis();
      displayMode = !displayMode;
    }

    if (displayMode) {
      if (testDurationSetup > 0 && testRunning) {
        uint8_t sT[] = { 0x78, 0x79, 0x6D, 0x78 }; display.setSegments(sT, 4, 0);  // "tESt" (live, bez delay)
      } else if (scanRunning) {
        Scan_display();
      } else {
        Auto_display();
      }
    } else if (AutoRotate != -1) {
      Angle_display(AutoRotate);
    } else if (scanRunning) {
      Angle_display(scanCurrentTarget);
    } else {
      Angle_display(lastAngle);
    }
  }

  // --- Enkodér: nastavení AutoRotace ---
  long pos = enc.read() / 4;
  if (pos != lastPos) {
    long ch = pos - lastPos;
    ledPos = ENCODER_REVERSED ? ledPos + ch : ledPos - ch;   // ledPos = ledPos - ch;
    if (ledPos < 0)          ledPos += NumPixels;
    if (ledPos >= NumPixels) ledPos -= NumPixels;
    lastPos        = pos;
    lastChangeTime = millis();
  }

  // Modrá LED enkodéru
  if (millis() - lastChangeTime > InactiveTime && AutoRotate == -1) {
    if (strip.getPixelColor(ledPos) == strip.Color(0, 0, 255))
      strip.setPixelColor(ledPos, strip.Color(0, 0, 0));
  } else {
    strip.setPixelColor(ledPos, strip.Color(0, 0, 255));
  }

  // Stisk enkodéru
  if (digitalRead(PIN_SW) == LOW) {
    unsigned long t0 = millis();
    while (digitalRead(PIN_SW) == LOW) {
      if (millis() - t0 > 1500) {
        // Dlouhý stisk → nastavení + spuštění scanu
        while (digitalRead(PIN_SW) == LOW);
        if (!testRunning) {
          stop_AutoRotate();
          scanRunning = false;
          scanStart();
        }
        return;
      }
    }
    // Krátký stisk
    if (!testRunning) {
      if (scanRunning) {
        scanStop();                          // scan běží → zastav
      } else if (AutoRotate != -1) {
        stop_AutoRotate();                   // AutoRotate běží → zastav
      } else if (millis() - lastChangeTime < InactiveTime) {
        AutoRotate = (int16_t)((float)ledPos / NumPixels * 360.0);  // encoder aktivní → spustí AutoRotate
        Auto_display(); delay(500);
      } else {
        if (scanAngle1 != scanAngle2)
          scanRestart();                     // encoder neaktivní → spustí scan
      }
    }
  }

// --- AutoRotace ---
  if (AutoRotate != -1) {
    // AS5600: výběr nejkratší dostupné cesty k cíli
    // tgt1 = přímá, tgt2 = CW přes sever, tgt3 = CCW přes sever
    int tgt1 = AutoRotate;
    int tgt2 = AutoRotate + 360;
    int tgt3 = AutoRotate - 360;

    auto inRange = [&](int t) { return t >= endStopCCW && t <= endStopCW; };

    int tgt  = tgt1;
    int best = inRange(tgt1) ? abs(tgt1 - absPos) : 30000;
    if (inRange(tgt2) && abs(tgt2 - absPos) < best) { tgt = tgt2; best = abs(tgt2 - absPos); }
    if (inRange(tgt3) && abs(tgt3 - absPos) < best) { tgt = tgt3; }

    if (!inRange(tgt)) {
      showEndstopWarning(); stop_AutoRotate();
    } else {
      int diff = tgt - absPos;
      if (abs(diff) > HYSTERESIS_END_ANGLE) {
        if (diff > 0) {
          if (movementAllowed(true,  absPos)) { CW_Motor();  }
          else { showEndstopWarning(); stop_AutoRotate(); }
        } else {
          if (movementAllowed(false, absPos)) { CCW_Motor(); }
          else { showEndstopWarning(); stop_AutoRotate(); }
        }
      } else { stop_AutoRotate(); }
    }
  }


  // --- Manuální rotace ---
  bool cwPressed  = digitalRead(CW_BUTTON)  == LOW;
  bool ccwPressed = digitalRead(CCW_BUTTON) == LOW;
  unsigned long now = millis();

  if (AutoRotate != -1 && (cwPressed || ccwPressed)) {
    lastOutputChange = now;
    stop_AutoRotate();
    if (testRunning) {
      testRunning = false; testDurationSetup = 0;
      displayStatus("test", 1500);
      displayStatus("end",  1500);
    }
  }

  if (cwPressed && ccwPressed) {
    stopMotor();
  } else if (cwPressed && !ccwPressed) {
    if (!isRunning || !direction) {
      if (now - lastOutputChange > 1000) {
        if (movementAllowed(true, absPos)) {
          stopMotor(); delay(100); lastOutputChange = now; CW_Motor();
        } else { stopMotor(); showEndstopWarning(); }
      }
    }
  } else if (ccwPressed && !cwPressed) {
    if (!isRunning || direction) {
      if (now - lastOutputChange > 1000) {
        if (movementAllowed(false, absPos)) {
          stopMotor(); delay(100); lastOutputChange = now; CCW_Motor();
        } else { stopMotor(); showEndstopWarning(); }
      }
    }
  } else if (AutoRotate == -1) {
    stopMotor();
  }

  // Průběžná kontrola END-stopy při manuálním chodu
  if (isRunning && AutoRotate == -1) {
    if ( direction && !movementAllowed(true,  absPos)) { stopMotor(); showEndstopWarning(); }
    if (!direction && !movementAllowed(false, absPos)) { stopMotor(); showEndstopWarning(); }
  }

  strip.show();
}

// ########################################################
// ################ ostatní funkce void  ##################
// ########################################################

// --- Watchdog zaseknutého snímače ---
// Volá se z loop() s aktuálním surovým azimutem ze senzoru.
// Pokud motor běží a azimut se po dobu WatchdogTimeout nezmění
// o více než minMov stupňů → zastav motor a zobraz chybu.
void watchdog(float angle) {
  if (!isRunning) {
    watchdogActive = false;
    return;
  }

  if (!watchdogActive) {
    watchdogActive = true;
    watchdogStart  = millis();
    watchdogLastAz = angle;
    return;
  }

  int diff = (int)angle - watchdogLastAz;
  if (diff >  180) diff -= 360;
  if (diff < -180) diff += 360;

  if (abs(diff) >= minMov) {
    // senzor se hýbe → reset timeru
    watchdogStart  = millis();
    watchdogLastAz = angle;
    return;
  }

  if (millis() - watchdogStart >= WatchdogTimeout) {
    stopMotor();
    stop_AutoRotate();
    if (scanRunning) {scanStop(); }
    testDurationSetup = 0;
    watchdogActive = false;

    Serial.println(F("!!! WATCHDOG: azimut se nemeni, motor zastaven !!!"));

    beepMorse("..--.."); // beeper zahraje "?" jako chyba
    
    displayStatus("stop", 1500);
    displayStatus("sen",   500);
    displayStatus("err",  1000);
    displayStatus("sen",   500);
    displayStatus("err",  1000);
    displayStatus("sen",   500);
    displayStatus("err",  2000);
  }
}



// --- Displej ---
void Angle_display(int displayAngle) {
  if (displayAngle > 360) {
    int over = displayAngle - 360;
    uint8_t d[] = { display.encodeDigit(1), 0x00, display.encodeDigit(over / 10), display.encodeDigit(over % 10) };
    display.setSegments(d, 4, 0);
  } else {
    display.showNumberDec((int)displayAngle, false);
  }
}

void Auto_display() {
  uint8_t d[] = { 0x77, 0x1C, 0x78, 0x5C };  // "Auto"
  display.setSegments(d, 4, 0);
}

void Scan_display() {
  uint8_t d[] = { 0x39, 0x5c, 0x54, 0x78 };  // "Cont"
  display.setSegments(d, 4, 0);
}

// ########################################################
// ##################### Motor  ###########################
// ########################################################

void clear_blue_LED() {
  lastChangeTime = InactiveTime + 1000;
}

void stop_AutoRotate() {
  stopMotor();
  AutoRotate = -1;
  clear_blue_LED();
  displayMode = 0;
  endstopBeepDone = false;
}

void setMotorSpeed(int speed) {
  analogWrite(ENABLE_PWM, constrain(speed, 0, 255));
}

void accelerateMotor(int targetSpeed, bool isUp) {
  uint16_t rampMs = isUp ? rampTimeUpMs : rampTimeDownMs;
  int   stepSize = max(1, (int)(maxSpeed / (rampMs / stepDelay)));
  if (currentSpeed < targetSpeed) {
    for (int s = currentSpeed; s <= targetSpeed; s += stepSize) { setMotorSpeed(min(s, targetSpeed)); delay(stepDelay); }
  } else {
    for (int s = currentSpeed; s >= targetSpeed; s -= stepSize) { setMotorSpeed(max(s, targetSpeed)); delay(stepDelay); }
  }
  currentSpeed = targetSpeed;
}

void CW_Motor() {
  if (isRunning && !direction) stopMotor();
  endstopBeepDone = false;
  direction = true;
  digitalWrite(CCW_RUN_PIN, LOW);
  digitalWrite(CW_RUN_PIN,  HIGH);
  delay(10);
  accelerateMotor(maxSpeed, true);
  isRunning = true;
}

void CCW_Motor() {
  if (isRunning && direction) stopMotor();
  endstopBeepDone = false;
  direction = false;
  digitalWrite(CW_RUN_PIN,  LOW);
  digitalWrite(CCW_RUN_PIN, HIGH);
  delay(10);
  accelerateMotor(maxSpeed, true);
  isRunning = true;
}

void stopMotor() {
  accelerateMotor(0, false);
  endstopBeepDone = false;
  isRunning = false;
  digitalWrite(CCW_RUN_PIN, LOW);
  digitalWrite(CW_RUN_PIN,  LOW);
  analogWrite(ENABLE_PWM, 0);
  delay(50);
}


// ########################################################
// ############### checkButtons  ##########################
// ########################################################
//
// mode = "setup"  → volá se jednou ze setup()
//                   čte okamžitý stav tlačítek (bez čekání na uvolnění)
//                   
// mode = "loop"   → volá se z loop() každý průchod
//                   sleduje krátký / dlouhý stisk
//
// Stav M1/M2/M3 čte přes readButtons() jako bitmask:
// BTN_M1=0b001  BTN_M2=0b010  BTN_M3=0b100  a jejich kombinace
//
void checkButtons(const char* mode) {

  // ======================================================
  // SETUP mód — okamžité čtení tlačítek (bez čekání)
  // ======================================================
  //
  // Voláno opakovaně ze smyčky FOR v setup() (odpočet 10s).
  // Rozlišuje krátký / dlouhý stisk, signalizuje buzzerem.
  //
  //   SW encoder krátký → calibrateNorth()
  //   SW encoder dlouhý → rezerva
  //
  //   M1/M2/M3 krátký  → testDurationSetup = 1/2/3
  //   M1/M2/M3 dlouhý  → rezerva
  //
  if (strcmp(mode, "setup") == 0) {

    // --- SW encoder ---
    // pressStartSetup = čas stisku změřený už v FOR smyčce → správná délka stisku
    if (digitalRead(PIN_SW) == LOW) {
      bool longBeepSW = false;
      while (digitalRead(PIN_SW) == LOW) {
        if (!longBeepSW && millis() - pressStartSetup >= LONG_PRESS_DURATION) {
          beepMorse("..");
          longBeepSW = true;
        }
      }
      unsigned long dur = millis() - pressStartSetup;
      if (dur >= LONG_PRESS_DURATION) {
        // dlouhý stisk SW → rezerva
      } else {
        beepMorse(".");        // cokoliv kratší než LONG_PRESS_DURATION = krátký stisk
        calibrateNorth();
      }
    }

    // --- Tlačítka M1 / M2 / M3 ---
    uint8_t btn = readButtons();
    if (btn != BTN_NONE) {
      uint8_t which   = (btn & BTN_M1) ? 1 : (btn & BTN_M2) ? 2 : 3;
      bool longBeepM  = false;
      while (readButtons() != BTN_NONE) {
        if (!longBeepM && millis() - pressStartSetup >= LONG_PRESS_DURATION) {
          beepMorse("..");
          longBeepM = true;
        }
      }
      unsigned long dur = millis() - pressStartSetup;
      if (dur >= LONG_PRESS_DURATION) {
        // dlouhý stisk Mx → rezerva
      } else {
        beepMorse(".");        // cokoliv kratší než LONG_PRESS_DURATION = krátký stisk
        if      (which == 1) testDurationSetup = 1;
        else if (which == 2) testDurationSetup = 2;
        else if (which == 3) testDurationSetup = 3;
      }
    }

    if (testDurationSetup > 0) {
      displayStatus("test", 1500);
      displayStatus("rot",  1500);
      display.showNumberDec(
        (testDurationSetup == 1) ? testSequence[0] :
        (testDurationSetup == 2) ? 60 : 120, false);
      delay(1500);
    }

    return;
  }

  // ======================================================
  // LOOP mód — sleduje stisky za normálního provozu
  // ======================================================
  //
  // Krátký stisk →  ozve se "."
  //   M1 = revers Encoder + uložení
  //   M2 = revers AS5600 + uložení
  //   M3 = reset MCU
  //
  // Dlouhý stisk →  ozve se ".."
  //   M1 = nastavení Gear Ratio   encoderem (X.XXX)
  //   M2 = nastavení endStopCW    fyzickým otočením antény → krátký stisk M2 = ulož + turns=0
  //   M3 = nastavení endStopCCW   fyzickým otočením antény → krátký stisk M3 = ulož + turns=0 + restart
  //   Dlouhý stisk libovolného M = zrušit krok kalibrace

  static unsigned long pressStart    = 0;
  static bool          buttonPressed = false;
  static uint8_t       pressedButton = 0;   // 1/2/3 = první stisknuté tlačítko
  static bool          shortBeepDone = false;
  static bool          longBeepDone  = false;

  uint8_t btn = readButtons();
  bool anyPressed = (btn != BTN_NONE);

  // --- Zaznamenání začátku stisku ---
  if (anyPressed && !buttonPressed) {
    buttonPressed = true;
    shortBeepDone = false;
    longBeepDone  = false;
    pressStart    = millis();
    if      (btn & BTN_M1) pressedButton = 1;
    else if (btn & BTN_M2) pressedButton = 2;
    else if (btn & BTN_M3) pressedButton = 3;
  }

  // --- Pípnutí při dosažení krátkého stisku (ještě během držení) ---
  if (anyPressed && buttonPressed && !shortBeepDone) {
    if (millis() - pressStart >= SHORT_PRESS_DURATION) {
      beepMorse(".");
      shortBeepDone = true;
    }
  }

  // --- Pípnutí při dosažení dlouhého stisku (ještě během držení) ---
  if (anyPressed && buttonPressed && !longBeepDone) {
    if (millis() - pressStart >= LONG_PRESS_DURATION) {
      beepMorse("..");
      longBeepDone = true;
    }
  }

  // --- Zpracování po uvolnění ---
  if (!anyPressed && buttonPressed) {
    buttonPressed = false;
    unsigned long dur = millis() - pressStart;

    // --- Příliš krátký stisk (< SHORT_PRESS_DURATION) = ignoruj (ochrana před zákmitem) ---
    if (dur < SHORT_PRESS_DURATION) {
      pressedButton = 0;
      return;
    }

    // --- Dlouhý stisk: kalibrace AS5600 ---
    if (dur >= LONG_PRESS_DURATION) {
        if (pressedButton == 1) {
          // M1 dlouhý stisk = nastavení Gear Ratio
          displayStatus("cal",  800);
          displayStatus("gear", 800);
          calibrateGearRatio();
          calibrationModeAS = false;

        } else if (pressedButton == 2) {
          calibrationModeAS = true;
          displayStatus("cal", 1000);
          uint8_t sECW[] = { 0x79, 0x00, 0x39, 0x1c };  // "E Cu" (end CW)
          Serial.print(F("  [endStopCW] turns=")); Serial.println(turns);
          int result = calibAS_setPhysical(sECW, 2);
          if (result != CALIB_CANCEL) {
            endStopCW = result + 360;
            turns       = 0;
            sensorAccum = (float)northOffset;   // reset akumulace na nulovou pozici
            lastRawSensorDeg = SENSOR_REVERSED
              ? (float)map(analogRead(ANALOG_PIN), 0, 1023, 359, 0)
              : (float)map(analogRead(ANALOG_PIN), 0, 1023, 0, 359);
            lastAz = (int)readSensorAngle();
            saveTurns();
            saveSensorAccumForce();
            saveEndStops();
            displayStatus("set", 1000);
            Serial.print(F("AS5600 endStopCW=")); Serial.println(endStopCW);
            Serial.println(F("Turns reset na 0."));
          } else {
            displayStatus("abor", 800);
          }
          calibrationModeAS = false;

        } else if (pressedButton == 3) {
          calibrationModeAS = true;
          displayStatus("cal", 1000);
          uint8_t sECC[] = { 0x79, 0x39, 0x39, 0x1c };  // "ECCv" (end CCW)
          Serial.print(F("  [endStopCCW] turns=")); Serial.println(turns);
          int result = calibAS_setPhysical(sECC, 3);
          if (result != CALIB_CANCEL) {
            endStopCCW = result - 360;
            turns       = -1;
            sensorAccum = (float)northOffset - 360.0;  // reset akumulace na -1 otáčku
            lastRawSensorDeg = SENSOR_REVERSED
              ? (float)map(analogRead(ANALOG_PIN), 0, 1023, 359, 0)
              : (float)map(analogRead(ANALOG_PIN), 0, 1023, 0, 359);
            lastAz = (int)readSensorAngle();
            saveTurns();
            saveSensorAccumForce();
            saveEndStops();
            Serial.print(F("AS5600 endStopCCW=")); Serial.println(endStopCCW);
            Serial.println(F("Turns reset na -1."));
            displayStatus("end", 1000);
            asm volatile("  jmp 0");
          } else {
            displayStatus("abor", 800);
          }
          calibrationModeAS = false;
        }

    // --- Krátký stisk 
    } else {

    // M1 = otočení směru enkodéru
    if (pressedButton == 1) {
      ENCODER_REVERSED = !ENCODER_REVERSED;
      fram.write(FRAM_ENC_REVERSED, ENCODER_REVERSED ? 1 : 0);
      displayStatus("enc", 1000);
      displayStatus("rev", 1000);
      //Serial.print(F("Encoder reversed: ")); Serial.println(ENCODER_REVERSED);
    }

    // M2 = otočení směru snímače AS5600
    if (pressedButton == 2) {
      SENSOR_REVERSED = !SENSOR_REVERSED;
      fram.write(FRAM_SENSOR_REVERSED, SENSOR_REVERSED ? 1 : 0);
      displayStatus("sen", 1000);
      displayStatus("rev",  1000);
      //Serial.print(F("Sensor reversed: ")); Serial.println(SENSOR_REVERSED);
    }
    
    // M3 = reset MCU
    if (pressedButton == 3) {
       asm volatile("  jmp 0");
      }
    }
    pressedButton = 0;
  }
}

// --- Hamlib / Tučňák ---
void Hamlib_Tucnak() {
  static String lineBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      lineBuffer.trim();
      if (lineBuffer.length() >= 2) {
        if (lineBuffer.startsWith("C2")) {
          send_LastPosition();
        } else if (lineBuffer.startsWith("W")) {
          String params = lineBuffer.substring(1);
          params.trim();
          int sp = params.indexOf(' ');
          if (sp > 0) {
            int az = params.substring(0, sp).toInt();
            int el = params.substring(sp + 1).toInt();
            if (az < 0)    az += 360;
            if (az >= 360) az -= 360;
            if (az < 0)    az = 0;
            Hamlib_Azimuth   = az;
            Hamlib_Elevation = el;
            if (scanRunning) scanStop();
            int absTarget = getAbsolutePosition(az);
            if (absTarget < endStopCCW || absTarget > endStopCW) {
              //Serial.print(F("Hamlib: mimo END-stopy abs=")); Serial.println(absTarget);
              showEndstopWarning();
            } else {
              AutoRotate = az; lastElevation = el;
              ledPos = (int)round((float)az / DegPerLED) % NumPixels;
              display.showNumberDec(az, false);
            }
          }
        }
      }
      lineBuffer = "";
    } else {
      lineBuffer += c;
    }
  }
}

void send_LastPosition() {
  char azBuf[6], elBuf[6], buf[12];
  auto fmt = [](int val, char *out) {
    snprintf(out, 6, "%c0%03d", (val < 0) ? '-' : '+', abs(val));
  };
  fmt(lastAngle, azBuf);
  fmt(lastElevation,  elBuf);
  snprintf(buf, sizeof(buf), "%s%s\r", azBuf, elBuf);
  Serial.print(buf);
}

// --- Contest (Scan) ---
int scanSetValue(int val, int minVal, int maxVal, int stepVal) {
  long lastEncPos = enc.read() / 4;
  int  cur        = constrain(val, minVal, maxVal);
  display.showNumberDec(cur, false);
  while (true) {
    if (digitalRead(PIN_SW) == LOW) {
      unsigned long t = millis();
      while (digitalRead(PIN_SW) == LOW) {
        if (millis() - t > 1500) { while (digitalRead(PIN_SW) == LOW); return -1; }
      }
      return cur;
    }
    long encPos = enc.read() / 4;
    if (encPos != lastEncPos) {
      long delta = encPos - lastEncPos;
      if (ENCODER_REVERSED) delta = -delta;
      cur -= delta * stepVal;
      cur  = constrain(cur, minVal, maxVal);
      lastEncPos = encPos;
      display.showNumberDec(cur, false);
      if (maxVal == 360) {
        int lp = (cur * NumPixels / 360) % NumPixels;
        strip.clear(); strip.setPixelColor(lp, strip.Color(0, 0, 255)); strip.show();
      }
    }
  }
}

void scanStart() {
  displayStatus("an1", 1000);
  int val = scanSetValue(scanAngle1, 0, 360, (int)round(DegPerLED));
  if (val < 0) return; scanAngle1 = val;

  displayStatus("an2", 1000);
  val = scanSetValue(scanAngle2, 0, 360, (int)round(DegPerLED));
  if (val < 0) return; scanAngle2 = val;

  displayStatus("step", 1000);
  val = scanSetValue(scanStep, (int)round(DegPerLED), 90, (int)round(DegPerLED));
  if (val < 0) return; scanStep = val;

  displayStatus("int", 1000);
  val = scanSetValue(scanInterval, 1, 10, 1);
  if (val < 0) return; scanInterval = val;

  int startPos = constrain((int)round((float)lastAngle / DegPerLED) * (int)round(DegPerLED), scanAngle1, scanAngle2);
  scanCurrentTarget = startPos; scanDirectionUp = (startPos <= scanAngle2);
  scanRunning = true; scanWaitingForRotation = false; scanPauseUntil = 0;
  Scan_display(); delay(1500);

  if (lastAngle < scanAngle1 || lastAngle > scanAngle2)
    scanCurrentTarget = (abs(lastAngle - scanAngle1) < abs(lastAngle - scanAngle2)) ? scanAngle1 : scanAngle2;

  AutoRotate = scanCurrentTarget;
  ledPos     = (scanCurrentTarget * NumPixels / 360) % NumPixels;
  scanWaitingForRotation = true;
}

void scanRestart() {
  int startPos = constrain((int)round((float)lastAngle / DegPerLED) * (int)round(DegPerLED), scanAngle1, scanAngle2);
  scanCurrentTarget = startPos; scanDirectionUp = (startPos <= scanAngle2);
  scanRunning = true; scanWaitingForRotation = false; scanPauseUntil = 0;
  scanAnimPos = (startPos * NumPixels / 360) % NumPixels;
  Scan_display(); delay(1000);
  if (lastAngle < scanAngle1 || lastAngle > scanAngle2)
    scanCurrentTarget = (abs(lastAngle - scanAngle1) < abs(lastAngle - scanAngle2)) ? scanAngle1 : scanAngle2;
  AutoRotate = scanCurrentTarget;
  ledPos     = (scanCurrentTarget * NumPixels / 360) % NumPixels;
  scanWaitingForRotation = true;
}

void scanStop() {
  scanRunning = false; stop_AutoRotate();
  displayStatus("stop", 1500);
}

void scanRun() {
  if (!scanRunning) return;
  if (digitalRead(CW_BUTTON) == LOW || digitalRead(CCW_BUTTON) == LOW || digitalRead(PIN_SW) == LOW) {
    scanStop();
    while (digitalRead(CW_BUTTON) == LOW || digitalRead(CCW_BUTTON) == LOW || digitalRead(PIN_SW) == LOW);
    return;
  }
  if (scanWaitingForRotation) {
    if (AutoRotate != -1) return;
    scanWaitingForRotation = false; scanAnimReset = true;
    scanPauseUntil = millis() + (unsigned long)scanInterval * 60000UL;
    Scan_display(); return;
  }
  if (millis() < scanPauseUntil) return;

  if (scanDirectionUp) {
    scanCurrentTarget += scanStep;
    if (scanCurrentTarget >= scanAngle2) { scanCurrentTarget = scanAngle2; scanDirectionUp = false; }
  } else {
    scanCurrentTarget -= scanStep;
    if (scanCurrentTarget <= scanAngle1) { scanCurrentTarget = scanAngle1; scanDirectionUp = true; }
  }
  strip.setPixelColor(scanAnimPos, strip.Color(0, 0, 0)); strip.show();
  AutoRotate = scanCurrentTarget;
  ledPos     = (scanCurrentTarget * NumPixels / 360) % NumPixels;
  scanWaitingForRotation = true;
}

// --- AutoTest ---
void AutoTest() {
  if (!testRunning) {
    testRunning = true; testStartTime = millis(); testPauseUntil = 0;
    testStepCount = 0; testWaitingForRotation = false; testSeqIndex = 1; testSeqCycle = 1;
    randomSeed(analogRead(A6));
    Serial.println(F("=============================")); Serial.println(F("  START - TEST VYDRZE MOTORU"));
    if (testDurationSetup == 1) {
      Serial.print(F("  Mode: sekvence | cyklu: ")); Serial.println(testSequence[0]);
    } else {
      int dur = (testDurationSetup == 2) ? 60 : 120;
      testEndTime = testStartTime + (unsigned long)dur * 60000UL;
      Serial.print(F("  Mode: nahodny | doba: ")); Serial.print(dur); Serial.println(F(" min"));
    }
    Serial.println(F("============================="));
  }

  if (testWaitingForRotation) {
    if (AutoRotate != -1) return;
    testWaitingForRotation = false;
    int pauseSec = (testDurationSetup == 1) ? 5 : random(5, 61);
    testPauseUntil = millis() + (unsigned long)pauseSec * 1000UL;
    unsigned long el  = (millis() - testStartTime) / 1000UL;
    unsigned long rem = (testEndTime > millis()) ? (testEndTime - millis()) / 1000UL : 0;
    Serial.print(F("  >> Dosazeno ")); Serial.print(lastAngle);
    Serial.print(F("° | pauza: ")); Serial.print(pauseSec);
    Serial.print(F("s | cas: ")); Serial.print(el / 60); Serial.print(F("m ")); Serial.print(el % 60); Serial.print(F("s"));
    Serial.print(F(" | zbývá: ")); Serial.print(rem / 60); Serial.print(F("m ")); Serial.print(rem % 60); Serial.println(F("s"));
    return;
  }

  if (millis() < testPauseUntil) return;

  if (testDurationSetup == 1) {
    if (testSeqIndex >= testSeqLen) {
      if (testSeqCycle >= testSequence[0]) {
        testRunning = false; testDurationSetup = 0; stop_AutoRotate();
        Serial.print(F("  KONEC TESTU | Cyklu: ")); Serial.print(testSequence[0]);
        Serial.print(F(" | Celkem kroku: ")); Serial.println(testStepCount);
        displayStatus("test", 1500);
        displayStatus("end",  1500);
        return;
      }
      testSeqCycle++; testSeqIndex = 1;
    }
    int tgt = testSequence[testSeqIndex++]; testStepCount++;
    AutoRotate = tgt; ledPos = (tgt * NumPixels / 360) % NumPixels;
    testWaitingForRotation = true;
    Serial.print(F("Krok ")); Serial.print(testStepCount); Serial.print(F(" | Cil: ")); Serial.print(tgt);
    Serial.print(F("° | cyklus: ")); Serial.print(testSeqCycle); Serial.print(F("/")); Serial.println(testSequence[0]);
    return;
  }

  if (millis() >= testEndTime) {
    if (testRunning) {
      testRunning = false; testDurationSetup = 0; stop_AutoRotate();
      Serial.print(F("  KONEC TESTU | Celkem kroku: ")); Serial.println(testStepCount);
      displayStatus("test", 1500);
      displayStatus("end",  1500);
    }
    return;
  }

  int tgt = random(11, 351); testStepCount++;
  AutoRotate = tgt; ledPos = (tgt * NumPixels / 360) % NumPixels;
  testWaitingForRotation = true;
  Serial.print(F("Krok ")); Serial.print(testStepCount); Serial.print(F(" | Cil: ")); Serial.print(tgt); Serial.println(F("°"));
}


// --- Debug ---
void debug_monitor() {
  Serial.println(F(" >>>"));
}
