/*
 ###############  VERZE - 11.4.2026  #####################

 - Ovládání motoru rotátoru pomocí H-můstku
 - Snímání azimutu pomocí potenciometru (napěťový dělič) nebo AS5600 (analog 0-5V)
 - Zobrazení azimutu / úhlu na displeji TM1637
 - Zobrazení azimutu pomocí LED na kruhové mapě
 - AUTOROTACE pomocí nastavení encoderem
 - Ovládání pomocí HAMlib protokolu - LOG Tučňák
 - Funkce "CQ Contest" - Postupně otáčí kyvadlově anténou v daném rozmezí úhlů
  
 - Kalibrace a nastavení pomocí tlačítek M1/C, M2/S, M3/F
 - AutoTest rotátoru při zapnutí pomocí tlačítek (M1-M3) 
 - Nastavení a kalibrace se ukládá do EEPROM
 
 -------------------------------------------------------
 Volba snímače - tlačítky při normálním provozu (ukládá se do EEPROM):
   M1 (krátký stisk) = pot1  - Potenciometr, profil 0
   M2 (krátký stisk) = pot2  - Potenciometr, profil 1
   M3 (krátký stisk) = AS5600 analog (0-5V → 0-360°)

  
 Změna směru KY-040 Rotary Encoderu (jen při startu):
   Držet tlačítka M1+M2 než se na displeji zobrazí "Enc" + "rEV"  

 ------------------------------------------------------

 Kalibrace severu (jen při startu, jen pro AS5600):
   Držet enkodér PIN_SW při resetu → uvolnit = uloží degree offset do EEPROM.
   Pro potenciometr se kalibrace severu NEPROVÁDÍ - má mechanické dorazy.

 END-stopy (softwarové, jen pro AS5600):
   Používají absolutní pozici = azimut + (turns × 360).
   turns sleduje přechody přes sever (CW = turns++, CCW = turns--).
   endStopCW  = max dovolená abs. pozice pro CW  (např. +400°)
   endStopCCW = min dovolená abs. pozice pro CCW (např. -40°)
   Pro potenciometr END-stopy nejsou potřeba - motor zastaví mechanicky.

 Watchdog zaseknutého snímače:
   Pokud motor běží a azimut se do 3s nemění → zastav (ochrana při poruše čidla).

 -------------------------------------------------------
 EEPROM rozložení:
   [0]    = currentProfile (1 bajt): 0=Pot1/POT, 1=Pot2/POT, 2=AS5600
   [1..72]= profily POT    (2 profily × 24 bajtů + 1 rezerva)
   [73]   = northOffset    AS5600 (int, 2 bajty)
   [75]   = turns          AS5600 (int, 2 bajty) 
   [77]   = endStopCW      AS5600 (int, 2 bajty)
   [79]   = endStopCCW     AS5600 (int, 2 bajty)
   [81]   = směr encoderu  KY-040 (int, 1 bajt)
 -------------------------------------------------------

 Kalibrace AS5600 - tlačítka M1/C, M2/S, M3/F:
   Dlouhý stisk M1/C, M2/S nebo M3/F (při AS5600) = vstup/zrušení kalibrace AS5600
     M1/C - nastavení northOffset  fyzickým otočením antény (CW/CCW) → krátký stisk M1/C = ulož
     M2/S - nastavení endStopCW    fyzickým otočením antény (CW/CCW) → krátký stisk M2/S = ulož
     M3/F - nastavení endStopCCW   fyzickým otočením antény (CW/CCW) → krátký stisk M3/F = ulož + restart MCU

 Kalibrace potenciometru (jen pot1/pot2) - tlačítka M1/C, M2/S, M3/F:
   Dlouhý stisk = vstup/výstup z kalibrace (výstup = restart MCU)
     M1/C - přepíná kalibrační úhel (0-360°)
     M2/S - uloží napětí pro aktuální úhel → "SEtc" → další úhel
     M3/F - uloží napětí pro MAX úhel → "FuLL" → restart MCU
   Po 5 min nečinnosti se kalibrace ukončí automaticky (bez restartu)

 --------------------------------------------------------
 Funkce AutoTest (jen při startu):
   Stisknout a držet při zapnutí dokud se neobjeví "tESt"
     M1 = mode 1 (sekvence úhlů)
     M2 = mode 2 (náhodný 60 min)
     M3 = mode 3 (náhodný 120 min)
   Ukončení: CW / CCW → "End" → normální provoz

 -------------------------------------------------------
 Funkce Contest:
   Dlouhý stisk encoderu 1,5s = nastavení + start od aktuální pozice
     "An 1" → enkodér → krátký stisk  = první úhel
     "An 2" → enkodér → krátký stisk  = druhý úhel
     "StEP" → enkodér → krátký stisk  = krok ve stupních
     "Int " → enkodér → krátký stisk  = interval v minutách
   Krátký stisk encoderu → restart od aktuální pozice
   Zrušení: PTT vstup, CW / CCW / encoder → "End" → normální provoz
*/


// ############### inicializace knihoven ##################

#include <Adafruit_NeoPixel.h>
#include <TM1637Display.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <math.h>


// ############### zapojení  - Pinout    ##################

/*
D0 (RX)
D1 (TX)
  jsou trvale připojené na USB převodník (CH340/FTDI)
  nepoužívat na nic jiného kvůli "Hamlibu"
*/

// KY-040 Rotary Encoder
#define PIN_CLK 2   // pin podporuje "interrupt"
#define PIN_DT  3   // pin podporuje "interrupt"
#define PIN_SW  4

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


// ############### definice proměnných   ##################

// --- Typ snímače (načítá se z EEPROM podle profilu, mění se tlačítky M1/M2/M3) ---
// false = Potenciometr (kalibrace + interpolace)
// true  = AS5600 analog (0-5V → 0-360°, přímé mapování)
bool SENSOR_AS5600 = false;   // přepíše setup() podle EEPROM[0]

// --- Softwarové END-stopy (jen AS5600, absolutní pozice) ---
// Hodnoty jsou měnitelné za běhu kalibrací (M2/M3 v AS5600 cal. režimu)
int endStopCW  =  400;   // max abs. pozice CW  (°)
int endStopCCW =  -40;   // min abs. pozice CCW (°)

// --- EEPROM adresy ---
const int EEPROM_PROFILE      =  0;   // 1 bajt: profil (0=pot1, 1=pot2, 2=AS5600)
const int EEPROM_NORTH_OFFSET = 73;   // 2 bajty: north offset AS5600
const int EEPROM_TURNS        = 75;   // 2 bajty: počet otáček turns
const int EEPROM_ENDSTOP_CW   = 77;   // 2 bajty: endStopCW  AS5600
const int EEPROM_ENDSTOP_CCW  = 79;   // 2 bajty: endStopCCW AS5600
const int EEPROM_ENC_REVERSED = 81;   // 1 bajt: encoder - směr 0=normální, 1=otočený

// --- Kalibrační režim AS5600 ---
bool calibrationModeAS = false;   // true = probíhá kalibrace AS5600
const int CALIB_CANCEL = -32768; //  hodnota pro calibAS_setPhysical(): dlouhý stisk = zrušit

// --- Sledování přechodů přes sever (jen AS5600) ---
int turns  = 0;   // počet celých otáček přes sever
int lastAz = 0;   // předchozí azimut pro updateTurns()

// --- KY-040 Rotary Encoder ---
bool ENCODER_REVERSED = false; // Směr enkodéru: true (CW)/ false (CCW)
int lastPos = -1;
int ledPos  = 0;
float AutoRotate = -1.0;                  // -1 = žádný cíl
const float HYSTERESIS_END_ANGLE = 5.0;   // hystereze zastavení AutoRotace (°)
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
int   NumPixels      = 48;
int   RingBrightness = 2;
float DegPerLED      = 360.0 / 48;

// --- Motor BTS7960 ---
const int   maxSpeed     = 255;
const float rampTimeUp   = 0.5;   // s
const float rampTimeDown = 0.5;   // s
const int   stepDelay    = 10;    // ms
bool isRunning    = false;
bool direction    = true;    // true = CW, false = CCW
int  currentSpeed = 0;

// --- Watchdog zaseknutého snímače ---
const unsigned long WatchdogTimeout = 2500;   // ms
unsigned long watchdogStart  = 0;
float         watchdogLastAz = -999.0;
bool          watchdogActive = false;
const float   minMov = 6.0;        // minimální reálný pohyb ve ° pro reset watchdogu

// --- Analogový vstup ---
const int   ADC_MaxValue   = 1023;
const float ADC_RefVoltage = 5.0;
float lastAngle      = -1;
int   lastSensorValue = 0;
const float HysteresisAngle = 3.0;

// --- Kalibrace potenciometru ---
const int NumCalibrationPoints = 5;
float voltagePoints[NumCalibrationPoints];
float anglePoints[NumCalibrationPoints] = { 0, 90, 180, 270, 360 };
float voltageAtMax = 0.00;
float MaxAngle     = 365.00;
const unsigned int  LONG_PRESS_DURATION = 1500;    // ms
const unsigned long IDLE_TIMEOUT        = 300000;  // 5 min
unsigned long lastActivityTime = 0;
bool calibrationMode = false;
int  angleIndex      = 0;

// --- Profily: 0=pot1/POT, 1=pot2/POT, 2=AS5600 ---
int currentProfile    = 0;
const int MaxProfiles = 3;

// --- North offset (jen AS5600) ---
int northOffset = 0;

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


// ############### inicializace zařízení ##################

Adafruit_NeoPixel strip = Adafruit_NeoPixel(48, PIN_LED, NEO_GRB + NEO_KHZ800);
TM1637Display display(CLK, DIO);
Encoder enc(PIN_CLK, PIN_DT);


// ############### funkce které musí být před setup a loop ###############

// --- Interpolace / Extrapolace dráhy potenciometru ---
float voltageToAngleInterpolated(float voltage) {
  for (int i = 0; i < NumCalibrationPoints - 1; i++) {
    float v1 = voltagePoints[i];
    float v2 = voltagePoints[i + 1];
    float a1 = anglePoints[i];
    float a2 = anglePoints[i + 1];
    if ((v1 <= voltage && voltage <= v2) || (v1 >= voltage && voltage >= v2)) {
      return a1 + (voltage - v1) / (v2 - v1) * (a2 - a1);
    }
  }
  float v1    = voltagePoints[NumCalibrationPoints - 2];
  float v2    = voltagePoints[NumCalibrationPoints - 1];
  float a1    = anglePoints[NumCalibrationPoints - 2];
  float a2    = anglePoints[NumCalibrationPoints - 1];
  float slope = (a2 - a1) / (v2 - v1);
  return a2 + slope * (voltage - v2);
}

// --- EEPROM: bázová adresa profilu potenciometru ---
int getProfileBaseAddress(int profile) {
  return 1 + profile * (NumCalibrationPoints * sizeof(float) + sizeof(float));
}

// --- Sledování přechodů přes sever (jen AS5600) ---
// CW přechod (359→0): diff < -180 → turns++
// CCW přechod (0→359): diff > +180 → turns--
void updateTurns(int az) {
  int diff = az - lastAz;
  if (diff < -180) turns++;
  if (diff >  180) turns--;
  lastAz = az;
  saveTurns();  // *** Po přidání FRAM: framWriteInt(EEPROM_TURNS, turns); ***
}

// --- Absolutní pozice: azimut + celé otáčky ---
int getAbsolutePosition(int az) {
  return az + (turns * 360);
}

// --- Čtení azimutu (jednotný vstup pro oba snímače) ---
float readSensorAngle() {
  int rawADC = analogRead(ANALOG_PIN);
  lastSensorValue = rawADC;

  if (SENSOR_AS5600) {
    // AS5600: přímé mapování 0-1023 → 0-359° + north offset
    float angle = (float)map(rawADC, 0, 1023, 0, 359);
    angle = angle - (float)northOffset;
    if (angle <    0) angle += 360.0;
    if (angle >= 360) angle -= 360.0;
    return angle;
  } else {
    // Potenciometr: interpolace napětí → úhel
    float inputVoltage = (rawADC / (float)ADC_MaxValue) * ADC_RefVoltage;
    return constrain(voltageToAngleInterpolated(inputVoltage), 0, MaxAngle);
  }
}

// --- END-stopy: kontrola pohybu (jen AS5600, absolutní pozice) ---
bool movementAllowed(bool movingCW, int absPos) {
  if ( movingCW && absPos >= endStopCW)  return false;
  if (!movingCW && absPos <= endStopCCW) return false;
  return true;
  }


// --- Buzzer: N × pípnutí (aktivní LOW) ---
bool endstopBeepDone = false;

void beep(int count, int onMs = 150, int offMs = 150) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_BUZZER, LOW);  delay(onMs);
    digitalWrite(PIN_BUZZER, HIGH); delay(offMs);
  }
}


// --- END-stopy: varování na displeji + beeper ---
// beeper pouze jednou při prvním volání, dokud není "endstopBeepDone" resetován.

void showEndstopWarning() {
  if (!endstopBeepDone) {    
    beepMorse("K");   // beeper zahraje "-.-" jako konec
    endstopBeepDone = true;
  }
  uint8_t segEnd[]  = { 0x00, 0x79, 0x54, 0x5E };  // "End"
  uint8_t segStop[] = { 0x6d, 0x78, 0x5c, 0x73 };  // "StoP"
  display.setSegments(segEnd,  4, 0); delay(300);
  display.setSegments(segStop, 4, 0); delay(300);
}

// --- North offset: načtení z EEPROM ---
void loadNorthOffset() {
  EEPROM.get(EEPROM_NORTH_OFFSET, northOffset);
  if (northOffset < 0 || northOffset > 359) northOffset = 0;
}

// --- North offset: uložení do EEPROM ---
void saveNorthOffset(int offset) {
  northOffset = offset;
  EEPROM.put(EEPROM_NORTH_OFFSET, northOffset);
}

// --- Turns: uložení/načtení ---
void saveTurns() {
  EEPROM.put(EEPROM_TURNS, turns);
  // *** Po přidání FRAM: framWriteInt(EEPROM_TURNS, turns); ***
}

void loadTurns() {
  EEPROM.get(EEPROM_TURNS, turns);
  if (turns < -10 || turns > 10) turns = 0;
}

// --- END-stopy AS5600: uložení/načtení ---
void saveEndStops() {
  EEPROM.put(EEPROM_ENDSTOP_CW,  endStopCW);
  EEPROM.put(EEPROM_ENDSTOP_CCW, endStopCCW);
  Serial.print(F("Ulozeny endStopCW=")); Serial.print(endStopCW);
  Serial.print(F("  endStopCCW="));      Serial.println(endStopCCW);
}

void loadEndStops() {
  int cw, ccw;
  EEPROM.get(EEPROM_ENDSTOP_CW,  cw);
  EEPROM.get(EEPROM_ENDSTOP_CCW, ccw);
  // sanity check: rozsah -360..720
  if (cw  > -360 && cw  <= 720) endStopCW  = cw;
  if (ccw > -360 && ccw <= 720) endStopCCW = ccw;
  Serial.print(F("Nacteny endStopCW=")); Serial.print(endStopCW);
  Serial.print(F("  endStopCCW="));      Serial.println(endStopCCW);
}

// --- Kalibrace severu při startu (jen AS5600) ---
void calibrateNorth() {
  Serial.println(F("=== Kalibrace severu (AS5600) ==="));
  Serial.println(F("Drz enkoder... pust = ulozit sever"));

  uint8_t segOff[]  = { 0x5c, 0x71, 0x71, 0x00 };  // "oFF "
  uint8_t segSet[]  = { 0x6d, 0x79, 0x78, 0x00 };  // "SEt "
  uint8_t segSave[] = { 0x6d, 0x77, 0x3e, 0x79 };  // "SAVE"

  display.setSegments(segOff, 4, 0); delay(800); display.clear();
  display.setSegments(segSet, 4, 0); delay(800); display.clear();

  while (digitalRead(PIN_SW) == LOW) {
    display.showNumberDec(map(analogRead(ANALOG_PIN), 0, 1023, 0, 359), false);
    delay(50);
  }

  saveNorthOffset(map(analogRead(ANALOG_PIN), 0, 1023, 0, 359));
  turns  = 0;
  lastAz = (int)readSensorAngle();
  saveTurns();

  display.setSegments(segSave, 4, 0); delay(1000); display.clear();
  Serial.print(F("North offset (deg): ")); Serial.println(northOffset);
  Serial.println(F("Turns reset na 0."));
}

// --- Kalibrace AS5600: nastavení hodnoty fyzickým otočením antény ---
//
// label[4]    = 4bajtový segment kód popisku (např. "nor ", "EndC", "Endc")
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
    int ledIdx = round((float)az / DegPerLED) % NumPixels;
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




// ############### setup  ############################

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

    
  test_LED_DISPLAY();

  // Načtení směru enkodéru
  ENCODER_REVERSED = EEPROM.read(EEPROM_ENC_REVERSED) == 1;

  // Přepnutí směru enkodéru: držet M1+M2 při startu
  if (digitalRead(BUTTON_CAL_PIN) == LOW && digitalRead(BUTTON_SET_PIN) == LOW) {
    ENCODER_REVERSED = !ENCODER_REVERSED;
    EEPROM.update(EEPROM_ENC_REVERSED, ENCODER_REVERSED ? 1 : 0);

   // zobraz potvrzení
    uint8_t segEnc[] = { 0x00, 0x79, 0x54, 0x58 };  // " Enc "
    uint8_t segRev[] = { 0x50, 0x79, 0x3e, 0x00 };  // "rEV "
    display.setSegments(segEnc, 4, 0); delay(1000);
    display.setSegments(segRev, 4, 0); delay(1000);  }


  // Načtení profilu z EEPROM → určí typ snímače
  loadCurrentProfile();   // currentProfile = 0 / 1 / 2

  if (currentProfile == 2) {
    // --- AS5600 ---
     uint8_t segAS[] = { 0x40, 0x77, 0x6d, 0x40 };  // "-AS-"
     display.setSegments(segAS, 4, 0); delay(1500);
    SENSOR_AS5600 = true;
    MaxAngle      = 360.0;
    loadNorthOffset();
    loadTurns();
    loadEndStops();
    lastAz = (int)readSensorAngle();
    Serial.println(F("=== SENSOR: AS5600 analog ==="));
    Serial.print(F("North offset: ")); Serial.println(northOffset);
    Serial.print(F("Turns: "));        Serial.println(turns);
    // Kalibrace severu - jen při startu, jen AS5600
    if (digitalRead(PIN_SW) == LOW) {
      calibrateNorth();
    }
  } else {
    // --- Potenciometr ---
    SENSOR_AS5600 = false;
    loadProfileFromEEPROM(currentProfile);
    Serial.println(F("=== SENSOR: Potenciometr ==="));
    // Kalibrace severu se pro POT NEPROVÁDÍ
  }

  lastPos        = 0;
  lastChangeTime = millis();

  // Detekce tlačítek pro AutoTest (stisknout při startu)
  if      (digitalRead(BUTTON_CAL_PIN)  == LOW) testDurationSetup = 1;
  else if (digitalRead(BUTTON_SET_PIN)  == LOW) testDurationSetup = 2;
  else if (digitalRead(BUTTON_FULL_PIN) == LOW) testDurationSetup = 3;

  if (testDurationSetup > 0) {
    uint8_t sT[] = { 0x78, 0x79, 0x6D, 0x78 }; display.setSegments(sT, 4, 0); delay(1500);
    uint8_t sR[] = { 0x00, 0x50, 0x5C, 0x78 }; display.setSegments(sR, 4, 0); delay(1500);
    display.showNumberDec((testDurationSetup == 1) ? testSequence[0] : (testDurationSetup == 2) ? 60 : 120, false);
    delay(1500);
  }

  Serial.println(F("=== Rotator start ==="));
  if (SENSOR_AS5600) {
    Serial.print(F("Abs. pozice: ")); Serial.println(getAbsolutePosition((int)readSensorAngle()));
    Serial.print(F("endStopCW: "));  Serial.println(endStopCW);
    Serial.print(F("endStopCCW: ")); Serial.println(endStopCCW);
  }

   beepMorse("R");   // beeper zahraje ".-." test a ready

}

 

// ############### loop   ############################

void loop() {

  // --- AutoTest ---
  if (testDurationSetup > 0) AutoTest();

  // --- Contest ---
  scanRun();
  if (scanRunning && analogRead(PTT_PIN) < A6_THRESHOLD) {
    uint8_t sP[] = { 0x73, 0x78, 0x78, 0x00 };  // "Ptt"
    display.setSegments(sP, 4, 0); delay(1500);
    scanStop();
  }

  // --- Hamlib / Tučňák ---
  Hamlib_Tucnak();

  // --- Tlačítka M1/M2/M3: volba snímače + kalibrace ---
  checkButtons();

  // --- Čtení azimutu ---
  float angle = readSensorAngle();
  if (lastAngle == -1 || abs(angle - lastAngle) >= HysteresisAngle) {
    lastAngle = angle;
  }

  // --- Absolutní pozice + sledování otáček (jen AS5600) ---
  int absPos = (int)lastAngle;
  if (SENSOR_AS5600) {
    updateTurns((int)angle);
    absPos = getAbsolutePosition((int)lastAngle);
  }



  // --- Watchdog zaseknutého snímače ---
  watchdog(angle);


  // --- Zobrazení: Neopixel + displej ---
  int ledIndex = round(lastAngle / DegPerLED) % NumPixels;

  if (millis() - lastChangeTime > InactiveTime && AutoRotate == -1.0) {
    ledPos = ledIndex;
  }

  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis();
    strip.clear();

    // Animace Cont
    if (scanRunning) {
      int ledA1 = round((float)scanAngle1 / 360.0 * NumPixels) % NumPixels;
      int ledA2 = round((float)scanAngle2 / 360.0 * NumPixels) % NumPixels;
      strip.setPixelColor(ledA1, strip.Color(0, 0, 255));
      strip.setPixelColor(ledA2, strip.Color(0, 0, 255));
      if (AutoRotate == -1.0) {
        if (scanAnimReset) {
          scanAnimPos   = constrain(round((float)lastAngle / DegPerLED) % NumPixels, ledA1 + 1, ledA2 - 1);
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
    // turns > 0 (CW přes sever):  červené od LED 0 do ledIndex (0,1,2,3...)
    // turns < 0 (CCW přes sever): červené od LED 0 dozadu do ledIndex (0,47,46...)
    // turns == 0: normální zobrazení - zelená nebo červená na kardinálních bodech
    int sec = round(NumPixels / 4.0);

    if (SENSOR_AS5600 && turns > 0) {
      // CW přes sever - červené od 0 do ledIndex (0,1,2,3...)
      for (int i = 0; i <= ledIndex; i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
      }
    } else if (SENSOR_AS5600 && turns < 0) {
       // CCW přes sever - červené od 0 dozadu do ledIndex (0,47,46...)
       for (int i = 0; i >= -((NumPixels - ledIndex) % NumPixels); i--) {
        int idx = (NumPixels + i) % NumPixels;
       strip.setPixelColor(idx, strip.Color(255, 0, 0));
      }
    } else if (!SENSOR_AS5600 && round(lastAngle) > 360) {
      // Potenciometr: starý způsob pro úhly nad 360
      for (int i = 0; i < ledIndex; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
    }


    // Aktuální pozice - červená na kardinálních bodech (0,90,180,270°), jinak zelená
    if (ledIndex % sec == 0) {
      strip.setPixelColor(ledIndex, strip.Color(255, 0, 0));
    } else {
      strip.setPixelColor(ledIndex, strip.Color(0, 255, 0));
    }

    // Přepínání displeje
    if ((AutoRotate != -1.0 || scanRunning) && millis() - changeUpdateTime >= changeInterval) {
      changeUpdateTime = millis();
      displayMode = !displayMode;
    }

    if (displayMode) {
      if (testDurationSetup > 0 && testRunning) {
        uint8_t sT[] = { 0x78, 0x79, 0x6D, 0x78 }; display.setSegments(sT, 4, 0);  // "tESt"
      } else if (scanRunning) {
        Scan_display();
      } else {
        Auto_display();
      }
    } else if (AutoRotate != -1.0) {
      Angle_display(AutoRotate);
    } else if (scanRunning) {
      Angle_display(scanCurrentTarget);
    } else if (!calibrationMode) {
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
  if (millis() - lastChangeTime > InactiveTime && AutoRotate == -1.0) {
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
        if (AutoRotate == -1.0 && !scanRunning && !testRunning) {
          while (digitalRead(PIN_SW) == LOW);
          scanStart();
        }
        return;
      }
    }
    if (!scanRunning) {
      if (millis() - lastChangeTime < InactiveTime) {
        AutoRotate = (float)ledPos / NumPixels * 360.0;
        Auto_display(); delay(500);
      } else {
        if (!testRunning && AutoRotate == -1.0 && scanAngle1 != scanAngle2)
          scanRestart();
      }
    }
  }
// --- AutoRotace ---
  if (AutoRotate != -1.0) {
    if (SENSOR_AS5600) {
      // AS5600: výběr nejkratší dostupné cesty k cíli
      //
      // Tři možné absolutní cílové pozice:
      //   tgt1 = AutoRotate          přímá cesta (bez přechodu přes sever)
      //   tgt2 = AutoRotate + 360    cesta CW přes sever  (o otáčku výš)
      //   tgt3 = AutoRotate - 360    cesta CCW přes sever (o otáčku níž)
      //
      // Ze kandidátů v rozsahu [endStopCCW, endStopCW] se vybere nejbližší k absPos.

      float tgt1 = AutoRotate;
      float tgt2 = AutoRotate + 360.0;
      float tgt3 = AutoRotate - 360.0;

      auto inRange = [&](float t) { return t >= (float)endStopCCW && t <= (float)endStopCW; };

      float tgt  = tgt1;
      float best = inRange(tgt1) ? abs(tgt1 - absPos) : 1e9;

      if (inRange(tgt2) && abs(tgt2 - absPos) < best) {
        tgt  = tgt2;
        best = abs(tgt2 - absPos);
      }
      if (inRange(tgt3) && abs(tgt3 - absPos) < best) {
        tgt  = tgt3;
      }

      if (!inRange(tgt)) {
        showEndstopWarning(); stop_AutoRotate();
      } else {
        float diff = tgt - absPos;
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
    } else {
      // Potenciometr: bez softwarových END-stop
      float tgt1 = AutoRotate;
      float tgt2 = AutoRotate + 360.0;
      float cur  = angle;
      float tgt  = (tgt2 <= MaxAngle && abs(tgt2 - cur) < abs(tgt1 - cur)) ? tgt2 : tgt1;

      if (tgt < 0 || tgt > MaxAngle) {
        stop_AutoRotate();
      } else {
        float diff = tgt - cur;
        if (abs(diff) > HYSTERESIS_END_ANGLE) {
          if (diff > 0) { CW_Motor();  }
          else          { CCW_Motor(); }
        } else { stop_AutoRotate(); }
      }
    }
  }


  // --- Manuální rotace ---
  bool cwPressed  = digitalRead(CW_BUTTON)  == LOW;
  bool ccwPressed = digitalRead(CCW_BUTTON) == LOW;
  unsigned long now = millis();

  if (AutoRotate != -1.0 && (cwPressed || ccwPressed)) {
    lastOutputChange = now;
    stop_AutoRotate();
    if (testRunning) {
      testRunning = false; testDurationSetup = 0;
      uint8_t sT[] = { 0x78, 0x79, 0x6D, 0x78 }; display.setSegments(sT, 4, 0); delay(1500);
      uint8_t sE[] = { 0x00, 0x79, 0x54, 0x5E }; display.setSegments(sE, 4, 0); delay(1500);
    }
  }

  if (cwPressed && ccwPressed) {
    stopMotor();
  } else if (cwPressed && !ccwPressed) {
    if (!isRunning || !direction) {
      if (now - lastOutputChange > 1000) {
        if (!SENSOR_AS5600 || movementAllowed(true, absPos)) {
          stopMotor(); delay(100); lastOutputChange = now; CW_Motor();
        } else { stopMotor(); showEndstopWarning(); }
      }
    }
  } else if (ccwPressed && !cwPressed) {
    if (!isRunning || direction) {
      if (now - lastOutputChange > 1000) {
        if (!SENSOR_AS5600 || movementAllowed(false, absPos)) {
          stopMotor(); delay(100); lastOutputChange = now; CCW_Motor();
        } else { stopMotor(); showEndstopWarning(); }
      }
    }
  } else if (AutoRotate == -1.0) {
    stopMotor();
  }

  // Průběžná kontrola END-stopy při manuálním chodu (jen AS5600)
  if (isRunning && AutoRotate == -1.0 && SENSOR_AS5600) {
    if ( direction && !movementAllowed(true,  absPos)) { stopMotor(); showEndstopWarning(); }
    if (!direction && !movementAllowed(false, absPos)) { stopMotor(); showEndstopWarning(); }
  }

  strip.show();
}


// ############### ostatní funkce ####################

// --- Watchdog zaseknutého snímače ---
// Volá se z loop() s aktuálním surovým azimutem ze senzoru.
// Pokud motor běží a azimut se po dobu WatchdogTimeout nezmění
// o více než minMov stupňů → zastav motor a zobraz chybu.
void watchdog(float angle) {
  if (!isRunning || !SENSOR_AS5600) {
    watchdogActive = false;
    return;
  }

  if (!watchdogActive) {
    watchdogActive = true;
    watchdogStart  = millis();
    watchdogLastAz = angle;
    return;
  }

  float diff = angle - watchdogLastAz;
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
    scanStop();
    testDurationSetup = 0;
    watchdogActive = false;

    Serial.println(F("!!! WATCHDOG: azimut se nemeni, motor zastaven !!!"));

    beepMorse("?"); // beeper zahraje "?" jako chyba

    uint8_t segErr[] = { 0x79, 0x50, 0x50, 0x00 };  // "Err"
    uint8_t segSEn[] = { 0x00, 0x6d, 0x79, 0x54 };  // "SEn"
    display.setSegments(segSEn, 4, 0); delay(500);
    display.setSegments(segErr, 4, 0); delay(1000);
    display.setSegments(segSEn, 4, 0); delay(500);
    display.setSegments(segErr, 4, 0); delay(1000);
    display.setSegments(segSEn, 4, 0); delay(500);
    display.setSegments(segErr, 4, 0); delay(2000);
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

void Calibrate_display() {
  int angle = (int)anglePoints[angleIndex];
  uint8_t d[4];
  d[0] = 0x58;
  d[1] = (angle >= 100) ? display.encodeDigit((angle / 100) % 10) : 0x00;
  d[2] = (angle >= 10)  ? display.encodeDigit((angle / 10)  % 10) : 0x00;
  d[3] = display.encodeDigit(angle % 10);
  display.setSegments(d, 4, 0);
}

// --- Motor ---
void clear_blue_LED() {
  lastChangeTime = InactiveTime + 1000;
}

void stop_AutoRotate() {
  stopMotor();
  AutoRotate = -1.0;
  clear_blue_LED();
  displayMode = 0;
  endstopBeepDone = false;   // reset příznaku pro příští endstop varování
}

void setMotorSpeed(int speed) {
  analogWrite(ENABLE_PWM, constrain(speed, 0, 255));
}

void accelerateMotor(int targetSpeed, bool isUp) {
  float rampTime = isUp ? rampTimeUp : rampTimeDown;
  int   stepSize = max(1, (int)(maxSpeed / (rampTime * 1000.0 / stepDelay)));
  if (currentSpeed < targetSpeed) {
    for (int s = currentSpeed; s <= targetSpeed; s += stepSize) { setMotorSpeed(min(s, targetSpeed)); delay(stepDelay); }
  } else {
    for (int s = currentSpeed; s >= targetSpeed; s -= stepSize) { setMotorSpeed(max(s, targetSpeed)); delay(stepDelay); }
  }
  currentSpeed = targetSpeed;
}

void CW_Motor() {
  if (isRunning && !direction) stopMotor();
  endstopBeepDone = false;   // reset příznaku při novém rozjezdu
  direction = true;
  digitalWrite(CCW_RUN_PIN, LOW);
  digitalWrite(CW_RUN_PIN,  HIGH);
  delay(10);
  accelerateMotor(maxSpeed, true);
  isRunning = true;
}

void CCW_Motor() {
  if (isRunning && direction) stopMotor();
  endstopBeepDone = false;   // reset příznaku při novém rozjezdu
  direction = false;
  digitalWrite(CW_RUN_PIN,  LOW);
  digitalWrite(CCW_RUN_PIN, HIGH);
  delay(10);
  accelerateMotor(maxSpeed, true);
  isRunning = true;
}

void stopMotor() {
  accelerateMotor(0, false);
  endstopBeepDone = false;   // reset příznaku pro příští endstop varování
  isRunning = false;  
  digitalWrite(CCW_RUN_PIN, LOW);
  digitalWrite(CW_RUN_PIN,  LOW);
  analogWrite(ENABLE_PWM, 0);
  delay(50);
}

// --- EEPROM: profily potenciometru ---
void loadProfileFromEEPROM(int profile) {
  if (profile < 0 || profile >= MaxProfiles) return;
  int base = getProfileBaseAddress(profile);
  for (int i = 0; i < NumCalibrationPoints; i++) {
    EEPROM.get(base + i * sizeof(float), voltagePoints[i]);
  }
  EEPROM.get(base + NumCalibrationPoints * sizeof(float), voltageAtMax);
  MaxAngle = floor(voltageToAngleInterpolated(voltageAtMax));

  Serial.print(F("Nacten profil: "));  Serial.println(profile);
  Serial.print(F("Rotator c. "));      Serial.println(currentProfile + 1);
  for (int i = 0; i < NumCalibrationPoints; i++) {
    Serial.print(F("Uhel ")); Serial.print(anglePoints[i]);
    Serial.print(F(": "));   Serial.print(voltagePoints[i]); Serial.println(F(" V"));
  }
  Serial.print(F("MAX ")); Serial.print(MaxAngle);
  Serial.print(F(" = ")); Serial.print(voltageAtMax, 3); Serial.println(F(" V"));

  uint8_t d[4] = {  0x73, 0x5c, 0x78, display.encodeDigit(profile + 1) };  // "Pot1/2"
  display.setSegments(d, 4, 0); delay(1500);
  saveCurrentProfile();
}

void saveCurrentProfile() {
  EEPROM.update(EEPROM_PROFILE, currentProfile);
  Serial.print(F("Ulozen profil: ")); Serial.println(currentProfile);
}

void loadCurrentProfile() {
  int s = EEPROM.read(EEPROM_PROFILE);
  currentProfile = (s >= 0 && s < MaxProfiles) ? s : 0;
}

void saveAngleVoltage(int index) {
  if (index < 0 || index >= NumCalibrationPoints) return;
  float v = analogRead(ANALOG_PIN) * (ADC_RefVoltage / ADC_MaxValue);
  voltagePoints[index] = v;
  EEPROM.put(getProfileBaseAddress(currentProfile) + index * sizeof(float), v);
  Serial.print(F("Ulozeno voltagePoints[")); Serial.print(index);
  Serial.print(F("] = ")); Serial.println(v);
}

void saveMaxVoltage() {
  float v = analogRead(ANALOG_PIN) * (ADC_RefVoltage / ADC_MaxValue);
  voltageAtMax = v;
  EEPROM.put(getProfileBaseAddress(currentProfile) + NumCalibrationPoints * sizeof(float), v);
  Serial.print(F("Ulozeno voltageAtMax = ")); Serial.println(v);
}

// --- Tlačítka M1 / M2 / M3 ---
//
// Krátký stisk mimo kalibraci:
//   M1 = pot1 (POT, profil 0)   → "pot1"
//   M2 = pot2 (POT, profil 1)   → "pot2"
//   M3 = AS5600 (profil 2)      → "AS  "
//
// Dlouhý stisk = vstup do kalibrace:
//   POT (pot1/pot2):
//     Libovolné M tlačítko (dlouhý) = vstup/výstup z kalibrace POT
//     V kalibračním režimu: M1=přepni úhel, M2=ulož napětí, M3=ulož MAX+restart
//   AS5600:
//     M1 (dlouhý) = nastavení northOffset (otočit CW/CCW, krátký M1 = ulož)
//     M2 (dlouhý) = nastavení endStopCW   (otočit CW/CCW, krátký M2 = ulož)
//     M3 (dlouhý) = nastavení endStopCCW  (otočit CW/CCW, krátký M3 = ulož + restart)
//     Dlouhý stisk libovolného M = zrušit aktuální krok kalibrace
//
void checkButtons() {
  static unsigned long pressStart   = 0;
  static bool          buttonPressed = false;
  static uint8_t       pressedButton = 0;

  bool bCal  = !digitalRead(BUTTON_CAL_PIN);
  bool bSet  = !digitalRead(BUTTON_SET_PIN);
  bool bFull = !digitalRead(BUTTON_FULL_PIN);

  if ((bCal || bSet || bFull) && !buttonPressed) {
    buttonPressed = true;
    pressStart    = millis();
    if      (bCal  && !bSet  && !bFull) pressedButton = 1;
    else if (bSet  && !bCal  && !bFull) pressedButton = 2;
    else if (bFull && !bCal  && !bSet)  pressedButton = 3;
    else                                pressedButton = 0;
  }

  if (!bCal && !bSet && !bFull && buttonPressed) {
    buttonPressed = false;
    unsigned long dur = millis() - pressStart;
    uint8_t d[4];

    uint8_t segCAS[] = { 0x58, 0x00, 0x77, 0x6d };  // "c AS"
	uint8_t segAbt[] = { 0x77, 0x7c, 0x5c, 0x50 };  // "Abor" (abort)
 
    // --- Dlouhý stisk: kalibrace ---
    if (dur >= LONG_PRESS_DURATION) {
      if (SENSOR_AS5600) {
        // AS5600: každé tlačítko vstupuje do svého kalibračního kroku
        //   M1/C (dlouhý) = nastavení northOffset
        //   M2/S (dlouhý) = nastavení endStopCW
        //   M3/F (dlouhý) = nastavení endStopCCW + restart

        if (pressedButton == 1) {
          // --- M1: northOffset ---          calibrationModeAS = true;
          
          display.setSegments(segCAS, 4, 0); delay(1000);

          uint8_t segNor[] = { 0x54, 0x5c, 0x50, 0x78 };  // "nort "
          // Pro northOffset: otočit anténu na sever, krátký M1 = ulož
          // calibAS_setPhysical vrací azimut (0-359 s aktuálním northOffset),
          // ale northOffset = raw hodnota senzoru při az=0
          // Proto northOffset přepočítáme: newOffset = (starý northOffset + vrácený az) % 360
          int result = calibAS_setPhysical(segNor, 1);
          if (result != CALIB_CANCEL) {
            int newOffset = (northOffset + result) % 360;
            saveNorthOffset(newOffset);
            turns  = 0;
            lastAz = (int)readSensorAngle();
            saveTurns();
            uint8_t sSave[] = { 0x6d, 0x77, 0x3e, 0x79 };  // "SAVE"
            display.setSegments(sSave, 4, 0); delay(1000);
            Serial.print(F("AS5600 northOffset=")); Serial.println(northOffset);
          } else {
            uint8_t segAbt[] = { 0x77, 0x7c, 0x5c, 0x50 };  // "Abor" (abort)
            display.setSegments(segAbt, 4, 0); delay(800);
          }
          calibrationModeAS = false;

        } else if (pressedButton == 2) {
          // --- M2: endStopCW ---
          calibrationModeAS = true;
          display.setSegments(segCAS, 4, 0); delay(1000);

          uint8_t sECW[] = { 0x79, 0x00, 0x39, 0x1c };  // "E Cu" (end CW)
          // vrací azimut (0-359°) v poloze max CW → endStopCW = az + 360
          Serial.print(F("  [endStopCW] turns pred kalibraci=")); Serial.println(turns);
          int result = calibAS_setPhysical(sECW, 2);
          if (result != CALIB_CANCEL) {
            endStopCW = result + 360;   // anténa je za severem CW → vždy +360
            saveEndStops();
            uint8_t sSave[] = { 0x6d, 0x77, 0x3e, 0x79 };  // "SAVE"
            display.setSegments(sSave, 4, 0); delay(1000);
            Serial.print(F("AS5600 endStopCW=")); Serial.println(endStopCW);
          } else {
             display.setSegments(segAbt, 4, 0); delay(800);
          }
          calibrationModeAS = false;

        } else if (pressedButton == 3) {
          // --- M3: endStopCCW + restart ---
          calibrationModeAS = true;
          display.setSegments(segCAS, 4, 0); delay(1000);

          uint8_t sECC[] = { 0x79, 0x39, 0x39, 0x1c };  // "ECCv" (end CCW)
          // vrací azimut (0-359°) v poloze max CCW → endStopCCW = az - 360
          Serial.print(F("  [endStopCCW] turns pred kalibraci=")); Serial.println(turns);
          int result = calibAS_setPhysical(sECC, 3);
          if (result != CALIB_CANCEL) {
            endStopCCW = result - 360;   // anténa je za severem CCW → vždy -360
            saveEndStops();
            Serial.print(F("AS5600 endStopCCW=")); Serial.println(endStopCCW);
            uint8_t sEnd[] = { 0x00, 0x79, 0x54, 0x5E };  // "End"
            display.setSegments(sEnd, 4, 0); delay(1000);
            asm volatile("  jmp 0");   // restart pro čistý start s novými hodnotami
          } else {            
            display.setSegments(segAbt, 4, 0); delay(800);
          }
          calibrationModeAS = false;
        }
      } else {
        // POT: kalibrace potenciometru
        if (!calibrationMode) {
          calibrationMode = true;
          d[0] = 0x39; d[1] = 0x77; d[2] = 0x38;
          d[3] = display.encodeDigit(currentProfile + 1);  // "CaL1/2"
          display.setSegments(d, 4, 0); delay(1500);
          angleIndex = 0;
          Calibrate_display();
        } else {
          calibrationMode = false;
          d[0] = 0x79; d[1] = 0x54; d[2] = 0x5e; d[3] = 0x58;  // "Endc"
          display.setSegments(d, 4, 0); delay(1500);
          asm volatile("  jmp 0");
        }
        lastActivityTime = millis();
      }

    // --- Krátký stisk v kalibračním režimu (jen POT) ---
    } else if (calibrationMode) {
      switch (pressedButton) {
        case 1:
          angleIndex = (angleIndex + 1) % NumCalibrationPoints;
          Calibrate_display();
          break;
        case 2:
          saveAngleVoltage(angleIndex);
          d[0] = 0x6d; d[1] = 0x79; d[2] = 0x78; d[3] = 0x58;  // "SEtc"
          display.setSegments(d, 4, 0); delay(1500);
          angleIndex = (angleIndex + 1) % NumCalibrationPoints;
          Calibrate_display();
          break;
        case 3:
          saveMaxVoltage();
          d[0] = 0x71; d[1] = 0x1c; d[2] = 0x38; d[3] = 0x38;  // "FuLL"
          display.setSegments(d, 4, 0); delay(1500);
          asm volatile("  jmp 0");
          break;
      }
      delay(1500);
      lastActivityTime = millis();

    // --- Krátký stisk mimo kalibraci: volba snímače ---
    } else {
      switch (pressedButton) {
        case 1:
          currentProfile = 0; SENSOR_AS5600 = false;
          loadProfileFromEEPROM(0);   // zobrazí "pot1" + uloží profil
          Serial.println(F("==> pot1: Potenciometr"));
          break;
        case 2:
          currentProfile = 1; SENSOR_AS5600 = false;
          loadProfileFromEEPROM(1);   // zobrazí "pot2" + uloží profil
          Serial.println(F("==> pot2: Potenciometr"));
          break;
        case 3:
          currentProfile = 2; SENSOR_AS5600 = true; MaxAngle = 360.0;
          saveCurrentProfile();
          /*
          { uint8_t segAS[] = { 0x40, 0x77, 0x6d, 0x40 };  // "-AS-"
            display.setSegments(segAS, 4, 0); delay(1500); }
          */

// ----------- „scrollovací“ text -------------------
uint8_t segAnimFull[] = {
  0x00, 0x00, 0x00, 0x00,  // mezera (nájezd zprava)
  0x77, 0x54, 0x77, 0x38,  // AnAL
  0x5c, 0x3d, 0x00, 0x6d,  // oG S
  0x79, 0x54, 0x6d, 0x5c,  // EnSo
  0x50, 0x00, 0x77, 0x6d,  // r AS 
  0x40, 0x6d, 0x7d, 0x3f,  // -560
  0x3f, 0x00, 0x00, 0x00,  // 0
  0x00,0x00, 0x00, 0x00   // mezera (odjezd doleva)
};

const int len = sizeof(segAnimFull);
uint8_t window[4];

  for (int i = 0; i < len - 3; i++) {
    // posun okna o 4 znacích
    for (int j = 0; j < 4; j++) {
      window[j] = segAnimFull[i + j];
    }
    display.setSegments(window, 4, 0);
    delay(250);
  }

// ----------- „scrollovací“ text -------------------

          Serial.println(F("==> AS5600 analog"));
          break;
      }
    }
    pressedButton = 0;
  }

  // Auto-ukončení kalibrace po 5 min nečinnosti
  if (calibrationMode && (millis() - lastActivityTime > IDLE_TIMEOUT)) {
    calibrationMode = false;
    uint8_t d[] = { 0x79, 0x54, 0x5e, 0x58 };  // "Endc"
    display.setSegments(d, 4, 0); delay(1500);
    Angle_display(lastAngle);
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
            if (SENSOR_AS5600) {
              int absTarget = getAbsolutePosition(az);
              if (absTarget < endStopCCW || absTarget > endStopCW) {
                Serial.print(F("Hamlib: mimo END-stopy abs=")); Serial.println(absTarget);
                showEndstopWarning();
              } else {
                AutoRotate = az; lastElevation = el;
                ledPos = round(az / DegPerLED) % NumPixels;
                display.showNumberDec(az, false);
              }
            } else {
              AutoRotate = az; lastElevation = el;
              ledPos = round(az / DegPerLED) % NumPixels;
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
  fmt((int)lastAngle, azBuf);
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
      cur -= (encPos - lastEncPos) * stepVal;
      cur  = constrain(cur, minVal, maxVal);
      lastEncPos = encPos;
      display.showNumberDec(cur, false);
      if (maxVal == 360) {
        int lp = round((float)cur / 360.0 * NumPixels) % NumPixels;
        strip.clear(); strip.setPixelColor(lp, strip.Color(0, 0, 255)); strip.show();
      }
    }
  }
}

void scanStart() {
  uint8_t sA1[]   = { 0x77, 0x54, 0x00, 0x06 }; display.setSegments(sA1,   4, 0); delay(1000);
  int val = scanSetValue(scanAngle1, 0, 360, (int)round(DegPerLED));
  if (val < 0) return; scanAngle1 = val;

  uint8_t sA2[]   = { 0x77, 0x54, 0x00, 0x5B }; display.setSegments(sA2,   4, 0); delay(1000);
  val = scanSetValue(scanAngle2, 0, 360, (int)round(DegPerLED));
  if (val < 0) return; scanAngle2 = val;

  uint8_t sStep[] = { 0x6D, 0x78, 0x79, 0x73 }; display.setSegments(sStep, 4, 0); delay(1000);
  val = scanSetValue(scanStep, (int)round(DegPerLED), 90, (int)round(DegPerLED));
  if (val < 0) return; scanStep = val;

  uint8_t sInt[]  = { 0x06, 0x54, 0x78, 0x00 }; display.setSegments(sInt,  4, 0); delay(1000);
  val = scanSetValue(scanInterval, 1, 10, 1);
  if (val < 0) return; scanInterval = val;

  int startPos = constrain((int)round(lastAngle / (int)round(DegPerLED)) * (int)round(DegPerLED), scanAngle1, scanAngle2);
  scanCurrentTarget = startPos; scanDirectionUp = (startPos <= scanAngle2);
  scanRunning = true; scanWaitingForRotation = false; scanPauseUntil = 0;
  Scan_display(); delay(1500);

  if (lastAngle < scanAngle1 || lastAngle > scanAngle2)
    scanCurrentTarget = (abs(lastAngle - scanAngle1) < abs(lastAngle - scanAngle2)) ? scanAngle1 : scanAngle2;

  AutoRotate = (float)scanCurrentTarget;
  ledPos     = round((float)scanCurrentTarget / 360.0 * NumPixels) % NumPixels;
  scanWaitingForRotation = true;
}

void scanRestart() {
  int startPos = constrain((int)round(lastAngle / (int)round(DegPerLED)) * (int)round(DegPerLED), scanAngle1, scanAngle2);
  scanCurrentTarget = startPos; scanDirectionUp = (startPos <= scanAngle2);
  scanRunning = true; scanWaitingForRotation = false; scanPauseUntil = 0;
  scanAnimPos = round((float)startPos / 360.0 * NumPixels) % NumPixels;
  Scan_display(); delay(1000);
  if (lastAngle < scanAngle1 || lastAngle > scanAngle2)
    scanCurrentTarget = (abs(lastAngle - scanAngle1) < abs(lastAngle - scanAngle2)) ? scanAngle1 : scanAngle2;
  AutoRotate = (float)scanCurrentTarget;
  ledPos     = round((float)scanCurrentTarget / 360.0 * NumPixels) % NumPixels;
  scanWaitingForRotation = true;
}

void scanStop() {
  scanRunning = false; stop_AutoRotate();
  uint8_t d[] = { 0x6d, 0x78, 0x5c, 0x73 };  // "StoP"
  display.setSegments(d, 4, 0); delay(1500);
}

void scanRun() {
  if (!scanRunning) return;
  if (digitalRead(CW_BUTTON) == LOW || digitalRead(CCW_BUTTON) == LOW || digitalRead(PIN_SW) == LOW) {
    scanStop();
    while (digitalRead(CW_BUTTON) == LOW || digitalRead(CCW_BUTTON) == LOW || digitalRead(PIN_SW) == LOW);
    return;
  }
  if (scanWaitingForRotation) {
    if (AutoRotate != -1.0) return;
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
  AutoRotate = (float)scanCurrentTarget;
  ledPos     = round((float)scanCurrentTarget / 360.0 * NumPixels) % NumPixels;
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
    if (AutoRotate != -1.0) return;
    testWaitingForRotation = false;
    int pauseSec = (testDurationSetup == 1) ? 5 : random(5, 61);
    testPauseUntil = millis() + (unsigned long)pauseSec * 1000UL;
    unsigned long el  = (millis() - testStartTime) / 1000UL;
    unsigned long rem = (testEndTime > millis()) ? (testEndTime - millis()) / 1000UL : 0;
    Serial.print(F("  >> Dosazeno ")); Serial.print((int)lastAngle);
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
        uint8_t sT[] = { 0x78, 0x79, 0x6D, 0x78 }; display.setSegments(sT, 4, 0); delay(1500);
        uint8_t sE[] = { 0x00, 0x79, 0x54, 0x5E }; display.setSegments(sE, 4, 0); delay(1500);
        return;
      }
      testSeqCycle++; testSeqIndex = 1;
    }
    int tgt = testSequence[testSeqIndex++]; testStepCount++;
    AutoRotate = (float)tgt; ledPos = round((float)tgt / 360.0 * NumPixels) % NumPixels;
    testWaitingForRotation = true;
    Serial.print(F("Krok ")); Serial.print(testStepCount); Serial.print(F(" | Cil: ")); Serial.print(tgt);
    Serial.print(F("° | cyklus: ")); Serial.print(testSeqCycle); Serial.print(F("/")); Serial.println(testSequence[0]);
    return;
  }

  if (millis() >= testEndTime) {
    if (testRunning) {
      testRunning = false; testDurationSetup = 0; stop_AutoRotate();
      Serial.print(F("  KONEC TESTU | Celkem kroku: ")); Serial.println(testStepCount);
      uint8_t sT[] = { 0x78, 0x79, 0x6D, 0x78 }; display.setSegments(sT, 4, 0); delay(1500);
      uint8_t sE[] = { 0x00, 0x79, 0x54, 0x5E }; display.setSegments(sE, 4, 0); delay(1500);
    }
    return;
  }

  int tgt = random(11, 351); testStepCount++;
  AutoRotate = (float)tgt; ledPos = round((float)tgt / 360.0 * NumPixels) % NumPixels;
  testWaitingForRotation = true;
  Serial.print(F("Krok ")); Serial.print(testStepCount); Serial.print(F(" | Cil: ")); Serial.print(tgt); Serial.println(F("°"));
}


// --- Morse beeper ----
void beepMorse(const char* text) {  // např. beepMorse("K")
  const int DIT = 50;
  const int DAH = 150;
  const int GAP = 50;       // mezera mezi prvky znaku
  const int CHAR_GAP = 150; // mezera mezi znaky
  const int WORD_GAP = 350; // mezera mezi slovy

  struct MorseMap {
    char symbol;
    const char* code;
  };

  // Kompletní Morse tabulka (A-Z, 0-9, symboly)
  const MorseMap morseMap[] = {
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
    {'?', "..--.."},
    {'.', ".-.-.-"},
    {',', "--..--"},
    {'!', "-.-.--"},
    {'/', "-..-."},
    {'(', "-.--."},
    {')', "-.--.-"}
  };

  const int mapSize = sizeof(morseMap) / sizeof(morseMap[0]);

  for (int i = 0; text[i] != '\0'; i++) {
    char c = toupper(text[i]);

    if (c == ' ') {
      delay(WORD_GAP);
      continue;
    }

    const char* code = nullptr;

    // vyhledání znaku v tabulce
    for (int k = 0; k < mapSize; k++) {
      if (morseMap[k].symbol == c) {
        code = morseMap[k].code;
        break;
      }
    }
   
   // přehraje znak z tabulky pomocí "void beep()" funkce
    if (code) {
      for (int j = 0; code[j] != '\0'; j++) {
        if (code[j] == '.') beep(1, DIT, GAP);
        else                beep(1, DAH, GAP);
      }
      delay(CHAR_GAP);
    }
  }
}


// --- Debug ---
void debug_monitor() {
  Serial.println(F(" >>>"));
}
