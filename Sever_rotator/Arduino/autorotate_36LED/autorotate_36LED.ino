/*
 ###############  VERZE - 24.8.2025  #####################

 - Ovládání motoru rotátoru pomocí H-můstku 
 - Snímání azimutu pomocí potenciometru (napěťový dělič)
 - Zobrazení azimutu / úhlu na displeji TM1637
 - zobrazení arimutu pomocí LED na krukové mapě
 - AUTOROTACE pomocí nastavení encoderem


 Po zapnutí (resetu) dojde k testu všech LED a Displeje
 Po tomto testu se rozsvítí modráLED na pozisi 0 (sever),
 nyní můžeme stisknout encoder a rotátor se nastaví na pozici "sever".
 Pokud tak neučiníme LED po 3s zhasne.

 Pro nastavení AUTOROTACE otočíme encoderem na požadovaný "azimut"
 a stiskneme encoder. Tím dojde k otáčení antény.
 Přerušení rotace je možné stiskem tlačítka pro manuální rotaci.

 Manuální rotace je možná pomocí tlačítek, která jsou blokována proti
 současnému tisku a rychlému změně směru. Takže mezi změnou směru je
 prodleva 1s, toto zabrání k rázům a zmenší namáhání převodů rotátoru.

 Přidáno PWM řízení H-můstku BTS7960, možno definovat čas
 'rampTimeUp' a 'rampTimeDown' pro plynulý rozjezd a dojezd.


 Kalibrace potenciometru pomocí tlačítek M1/C, M2/S, M3/F
 Uložení / Načtení z EEPROM při restartu

 Funkce:
  Krátkým stiskem načteme 3 uložené předvolby kalibrace pro různé rotátory
   Tlačítko M1/C  - pro rot1 
   Tlačítko M2/S  - pro rot2
   Tlačítko M3/F  - pro rot3

  Dlouhý stisk libovolného tlačítka aktivuje/deaktivuje kalibraci - "CAL+číslo rotatoru" / "Endc" (při deaktivaci dojde k restartu MCU) 
   Tlačítko M1/C  - přepíná mezi kalibračnímy úhly (0–360) a zobrazuje např. "c 90"
   Tlačítko M2/S - uloží napětí (ve voltech) pro aktuální úhel → zobrazí "SEtc" a nastaví další úhel např. "c180"
   Tlačítko M3/F  - uloží napětí pro MAX úhel → zobrazuje "FuLL" ukončí kalibraci a restartuje MCU

  Po 5 minutách nečinnosti se kalibrace automaticky ukončí (bez restartu MCU)


  Propjení Tučňáka pomocí Hamlib protokolu

// na ATmega328P (Arduino Uno, Nano, Pro Mini) mohou fungovat:
// A0–A5   jako analogové vstupy i digitální výstupy
// A6 a A7 jsou pouze analogové vstupy (nefungují jako výstupy)
// D3, D5, D6, D9-11 jako PWM piny

*/

// #################### inicializace knihoven ###########################

#include <Adafruit_NeoPixel.h>
#include <TM1637Display.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <math.h>

// ############### zapojení  - Pinout  ########################

// KY-040 Rotary Encoder
#define PIN_CLK 2  // pin podporuje "interrupt"
#define PIN_DT 3   // pin podporuje "interrupt"
#define PIN_SW 4

// Displej TM1367
#define DIO 5
#define CLK 6

// Tlačítka rotace
#define CCW_BUTTON 7  // Tlačítko CCW
#define CW_BUTTON 8   // Tlačítko CW

// LED motor is Running
#define PIN_LED_MOTOR  9    

// Piny pro BTS7960 - motor rotátoru
#define CCW_RUN_PIN 10  // pin R_PWM
#define ENABLE_PWM 11   // pin EN_R + EN_L
#define CW_RUN_PIN 12   // pin L_PWM

// Neopixel Ring LED
#define PIN_LED 13

// Analogový vstup - azimut anteny
#define ANALOG_PIN A0

// Tlačítka pro kalibraci a předvolbu rotátoru 1-3 ( kalibrace napětí/úhel )
#define BUTTON_CAL_PIN A1
#define BUTTON_SET_PIN A2
#define BUTTON_FULL_PIN A3


/*
// volné piny
#define D9   // PWM pin
#define A4   // analogové vstupy i digitální výstupy
#define A5   // analogové vstupy i digitální výstupy
#define A6   // pouze analogový vstup  (nefunguje jako výstup)
#define A7   // pouze analogový vstup  (nefunguje jako výstup)
*/


// ################## definice proměnných  ############################

// KY-040 Rotary Encoder - AutoRotace
int lastPos = -1;                        // Předchozí pozice enkodéru
int ledPos = 0;                          // Počáteční pozice pro modrou LED
float AutoRotate = -1.0;                 // Proměnná pro uložený úhel, Počáteční hodnota -1 znamená, že úhel ještě není uložen
const float HYSTERESIS_END_ANGLE = 2.0;  // Hystereze úhlu pro zastavení Autorotace
unsigned long lastOutputChange = 0;
unsigned long lastChangeTime = 0;         // Čas poslední změny pozice enkodéru
const unsigned long InactiveTime = 4000;  // 4 sekund nečinnosti


// Displej TM1637
unsigned long lastUpdateTime = 0;          // Čas poslední změny
const unsigned long updateInterval = 50;   //  interval aktualizace v ms
unsigned long changeUpdateTime = 0;        // čas pro přepínání hodnot displeje
const unsigned long changeInterval = 600;  //  interval aktualizace v ms
int displayMode = 0;                       // 0 = zobrazení 'úhlu', 1 = zobrazení 'Auto'
int displayBrightness = 0x01;              // Jas displeje


// Neopixel ring LED
int NumPixels = 36;                   // počet LED
int LedBrightness = 2;                // JAS led pásku 10
float DegPerLED = 360.0 / NumPixels;  // Počet stupňů na jednu LED


// Output CW / CCW
unsigned long currentMillis = millis();


// Parametry řízení motoru BTS7960
const int maxSpeed = 255;        // max PWM
const float rampTimeUp = 0.5;    // čas pro zrychlení v sekundách
const float rampTimeDown = 0.5;  // čas pro zpomalení v sekundách
const int stepDelay = 10;        // zpoždění mezi kroky rampy v ms
bool isRunning = false;
bool direction = true;  // true = CW, false = CCW
int currentSpeed = 0;


// Analog input  (potenciometr)
const int ADC_MaxValue = 1023;
const float ADC_RefVoltage = 5.0;  // Referenční napětí ADC nebo 3.3 dle MCU
float lastAngle = -1;
int lastSensorValue = 0;            // Uloží poslední hodnotu potenciometru
const float HysteresisAngle = 3.0;  // Hranice hystereze nastavit dle citlivosti


// Kalibrace (potenciometr)
const int NumCalibrationPoints = 5;
float voltagePoints[NumCalibrationPoints];  // jsou v EEPROM
float anglePoints[NumCalibrationPoints] = { 0, 90, 180, 270, 360 };
float voltageAtMax = 0.00;
float MaxAngle = 365.00;
const unsigned int LONG_PRESS_DURATION = 1500;  // 1,5 sekundy
const unsigned long IDLE_TIMEOUT = 300000;      // 5 minut
unsigned long lastActivityTime = 0;
bool calibrationMode = false;
int angleIndex = 0;


// Profil kalibrace - pro více rotátorů
int currentProfile = 0;
int MaxProfiles = 3;

// EEPROM rozložení:
// [0] = currentProfile (1 bajt)
// [1..] = profily (každý 24 bajtů)


// parametry pro Rotator - Tučňák
int Hamlib_Azimuth = 0; // přijatý azimut
int Hamlib_Elevation = 0; // přijatá elevace
int lastElevation = 0; // aktuální elevace



// #################### inicializace zařízení ###########################

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NumPixels, PIN_LED, NEO_GRB + NEO_KHZ800);
TM1637Display display(CLK, DIO);
Encoder enc(PIN_CLK, PIN_DT);


// ####################  Inter(Extra)polace dráhy potenciometru  ####################

float voltageToAngleInterpolated(float voltage) {
  for (int i = 0; i < NumCalibrationPoints - 1; i++) {
    float v1 = voltagePoints[i];
    float v2 = voltagePoints[i + 1];
    float a1 = anglePoints[i];
    float a2 = anglePoints[i + 1];

    if ((v1 <= voltage && voltage <= v2) || (v1 >= voltage && voltage >= v2)) {
      float ratio = (voltage - v1) / (v2 - v1);
      float intrapolatedAngle = a1 + ratio * (a2 - a1);

      return intrapolatedAngle;
    }
  }
  // Extrapolace za poslední úsek
  float v1 = voltagePoints[NumCalibrationPoints - 2];
  float v2 = voltagePoints[NumCalibrationPoints - 1];
  float a1 = anglePoints[NumCalibrationPoints - 2];
  float a2 = anglePoints[NumCalibrationPoints - 1];

  float slope = (a2 - a1) / (v2 - v1);  // funguje i pro klesající v2 < v1
  float deltaV = voltage - v2;
  float extrapolatedAngle = a2 + slope * deltaV;

  return extrapolatedAngle;
}


// ####################  načtení posledního uloženého profilu MEM   ####################

// Adresa EEPROM pro profil (od adresy 1 – adresa 0 je currentProfile)
int getProfileBaseAddress(int profile) {
  return 1 + profile * (NumCalibrationPoints * sizeof(float) + sizeof(float));
}


// Načte hodnoty z EEPROM pro zvolený profil
void loadProfileFromEEPROM(int profile) {
  if (profile < 0 || profile >= MaxProfiles) return;

  int base = getProfileBaseAddress(profile);

  for (int i = 0; i < NumCalibrationPoints; i++) {
    EEPROM.get(base + i * sizeof(float), voltagePoints[i]);
  }

  EEPROM.get(base + NumCalibrationPoints * sizeof(float), voltageAtMax);

  // aktualizace MaxAngle
  float measuredVoltage = voltageAtMax;
  MaxAngle = voltageToAngleInterpolated(measuredVoltage);
  MaxAngle = floor(MaxAngle);


  Serial.print("Načten profil: ");
  Serial.println(profile);

  Serial.print("Rotátor č. ");
  Serial.println(currentProfile + 1);

  Serial.println("== Napětí podle úhlů ==");
  for (int i = 0; i < NumCalibrationPoints; i++) {
    Serial.print("Úhel ");
    Serial.print(anglePoints[i]);
    Serial.print("°: ");
    Serial.print(voltagePoints[i]);
    Serial.println(" V");
  }

  Serial.print("MAX  ");
  Serial.print(MaxAngle);
  Serial.print(" = ");
  Serial.print(voltageAtMax, 3);
  Serial.println(" V");
  
  // zobrazení profilu rotátoru (MEM)
  uint8_t digits[4] = { 0x50, 0x5c, 0x78, display.encodeDigit(int(profile + 1)) };  // rot1...rot3
  display.setSegments(digits, 4, 0);
  delay(1500);

  saveCurrentProfile();  // ulož aktualní rotátor   
}


// ################# test LED Neopixel + Displej při resetu MCU #############################

void test_LED_DISPLAY(int interval = 1) {
  // TEST Neopixel a displeje při startu MCU
  for (int ledIndex = 0; ledIndex < NumPixels; ledIndex++) {
    //for (int i = 0; i < NumPixels; i++) {
    //  strip.setPixelColor(i, strip.Color(0, 0, 0));
    // }

    int numPixelsPerSection = round(NumPixels / 4.0);  // Zaokrouhlí 1/4 na celé číslo
    if (ledIndex % numPixelsPerSection == 0) {
      strip.setPixelColor(ledIndex, strip.Color(255, 0, 0));
    } else {
      strip.setPixelColor(ledIndex, strip.Color(0, 255, 0));
    }
    strip.show();

    float angle = map(ledIndex, 0, NumPixels - 1, 0, 360);
    display.showNumberDec((int)angle, false);
    delay(interval);
  }

  delay(500);
  strip.clear();  // Nastaví všechny LED na černou (vypnuté)
  strip.show();   // Aktualizace LED pásku
  display.clear();
}

 
// ################################################################
// ############################# setup ############################
// ################################################################

void setup() {

  Serial.begin(9600);  // Inicializace sériového výstupu

  strip.begin();
  strip.setBrightness(LedBrightness);
  strip.show();  // Inicializace LED pásu

  display.setBrightness(displayBrightness);  //0x0a
  display.clear();

  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DT, INPUT_PULLUP);
  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(CW_BUTTON, INPUT_PULLUP);
  pinMode(CCW_BUTTON, INPUT_PULLUP);

  pinMode(ENABLE_PWM, OUTPUT);
  pinMode(CW_RUN_PIN, OUTPUT);
  pinMode(CCW_RUN_PIN, OUTPUT);
  
  digitalWrite(PIN_LED_MOTOR, LOW);

  digitalWrite(ENABLE_PWM, LOW);
  digitalWrite(CW_RUN_PIN, LOW);
  digitalWrite(CCW_RUN_PIN, LOW);
  
  pinMode(BUTTON_CAL_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SET_PIN, INPUT_PULLUP);
  pinMode(BUTTON_FULL_PIN, INPUT_PULLUP);

  // test LED a Displeje při startu MCU
  test_LED_DISPLAY();

  // načtení kalibrace potenciometru z EEPROM  
  loadCurrentProfile();                   // Načti číslo profilu z EEPROM
  loadProfileFromEEPROM(currentProfile);  // Načti data profilu

  // reset encoderu při startu na arimut 0 (sever)
  lastPos = 0;                // Uložení pozice sever
  lastChangeTime = millis();  // Uložení času poslední změny  
  
}

// ################################################################
// ############################# loop #############################
// ################################################################

void loop() {

  // ############################# ovladaní rotatoru pomocí Tučňáka -Hamlib ##############################
  Hamlib_Tucnak();

  // ############################# kalibrace potenciometru ##############################
  // pomocí tlačítek M1/C, M2/S, M3/F
  checkButtons();

  // ############################# Snímání azimutu (potenciometr) #############################

  int sensorValue = analogRead(ANALOG_PIN);
  lastSensorValue = sensorValue;  // poslední hodnota snimače

  // Převod hodnoty ADC na napětí
  float inputVoltage = (sensorValue / (float)ADC_MaxValue) * ADC_RefVoltage;

  // Převod napětí na úhel
  float angle = voltageToAngleInterpolated(inputVoltage);
  angle = constrain(angle, 0, MaxAngle);

  // Pokud je úhel blízko předchozímu (hystereze), ignoruj změnu
  if (lastAngle == -1 || abs(angle - lastAngle) >= HysteresisAngle) {
    lastAngle = angle;  // Uložíme stabilní úhel
  }

  //Serial.print(" | Analog input: ");Serial.print(inputVoltage, 2);Serial.print("V | Angle: ");Serial.print(lastAngle);Serial.print("° ");Serial.println(" >>>");


  // #############################  Zobrazení azimutu (Neopixel LED + displej)  #############################
 
  // Převod úhlu na index LED
  int ledIndex = round(lastAngle / DegPerLED) % NumPixels;  
  
   // Ignorovat první 5 sekund po restartu
   if (millis() - lastChangeTime > InactiveTime && AutoRotate == -1.0 ) {
       ledPos = ledIndex;  // pozice modré LED pro encoder - jako pozice anteny          
    }
    

  // Pokud uplynulo alespoň xxx ms od poslední aktualizace LED a TM1367
  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis();  // Uložíme čas poslední aktualizace

    strip.clear();  // Nastaví všechny LED na černou (vypnuté)

    // Definice barev pro Neopixel LED
    int numPixelsPerSection = round(NumPixels / 4.0);  // Zaokrouhlí 1/4 na celé číslo
    if (ledIndex % numPixelsPerSection == 0) {
      strip.setPixelColor(ledIndex, strip.Color(255, 0, 0));  // červeá barva pro 1/4 kruhu  ( 0, 90, 180 a 270 stupňů)
    } else {
      if (round(lastAngle) > 360) {
        for (int i = 0; i < ledIndex; i++) {
          strip.setPixelColor(i, strip.Color(255, 0, 0));  // 'vše' nad 360 stupňů Červená barva
        }
      } else {
        strip.setPixelColor(ledIndex, strip.Color(0, 255, 0));  // Zelená barva
      }
    }

    // displej TM1367
    if (AutoRotate != -1.0 && millis() - changeUpdateTime >= changeInterval) {
      changeUpdateTime = millis();  // Uložíme čas poslední aktualizace
      displayMode = !displayMode;   // Přepínáme mezi zobrazeními
    }

    if (displayMode) {
      Auto_display();  // Funkce pro zobrazení "Auto"
    } else if (AutoRotate != -1.0) {
      Angle_display(AutoRotate);  // Zobrazení úhlu autorotace
    } else if (!calibrationMode) {
      Angle_display(lastAngle);  // Zobrazení úhlu antény
    }
  }


  // ############################# Nastavení AUTOROTACE (KY-040 Rotary Encoder) #############################
  
 
long pos = enc.read() / 4;  // Zmenšení kroku

if (pos != lastPos) {   
    // Počítáme změnu enkodéru
    long encoderChange = pos - lastPos;
    
    // Aplikujeme změnu na ledPos (s inverzí směru)
    ledPos = ledPos - encoderChange;  // Mínus pro obrácení směru
    
    // Ošetříme přetečení
    if (ledPos < 0) {
        ledPos += NumPixels;
    } else if (ledPos >= NumPixels) {
        ledPos -= NumPixels;
    }
    
    lastPos = pos;              // Uložení nové pozice
    lastChangeTime = millis();  // Uložení času poslední změny    
   
}
  
  // Automatické zhasnutí modré LED po uplynutí intervalu od poslední změny pozice
  if (millis() - lastChangeTime > InactiveTime && AutoRotate == -1.0) {
    // zhasni modrou LED, pokud na dané pozici není jiná aktivní barva
    uint32_t currentColor = strip.getPixelColor(ledPos);
    // Pokud LED není jiná aktivní barva, zhasni ji
    if (currentColor == strip.Color(0, 0, 255)) {
      strip.setPixelColor(ledPos, strip.Color(0, 0, 0));  // zhasni Modrou LED
    }
  } else {
    strip.setPixelColor(ledPos, strip.Color(0, 0, 255));  // nastav LED Modrá barva
  }


  // Pokud je stisknuté tlačítko enkodéru (tlačítko se použije k uložení úhlu)
  if (millis() - lastChangeTime < InactiveTime) {
    if (digitalRead(PIN_SW) == LOW) {

      // Uložení aktuálního úhlu do proměnné AutoRotate
      AutoRotate = (float)ledPos / NumPixels * 360.0;
      //Serial.print("Úhel uložen do AutoRotate: ");Serial.println(AutoRotate, 2);

      if (AutoRotate > angle && !direction || AutoRotate < angle && direction) {
        stopMotor();  // stop motor pri zmene autorotace do protismeru
      }

      Auto_display();  // zobrazeni 'Auto' na displeji
      delay(500);      // Zpoždění pro debouncing tlačítka
    }
  }


  // ############################# Automatická Rotace anteny (motor) #############################

  if (AutoRotate != -1.0) {

    //Serial.print("AutoRotate aktivní > ");Serial.print(" antena: ");Serial.print(angle);Serial.print(" rotace: ");Serial.print(AutoRotate);Serial.println(" < ");
    float angleDiff = AutoRotate - angle;

    if (abs(angleDiff) > HYSTERESIS_END_ANGLE) {
      if (angleDiff > 0) {  // Cílový úhel je větší než aktuální
        CW_Motor();
      } else {  // Cílový úhel je menší než aktuální
        CCW_Motor();
      }
    } else {
      // Pokud jsme v tolerančním pásmu, zastavíme autorotaci
      stop_AutoRotate();
    }
  }


  // ############################# Manuální Rotace anteny (motor) #############################

  bool cwPressed = digitalRead(CW_BUTTON) == LOW;
  bool ccwPressed = digitalRead(CCW_BUTTON) == LOW;
  unsigned long currentMillis = millis();


  // Zastavení 'auto-rotace' při stisku tlačítka CW nebo CCW
  if (AutoRotate != -1.0) {
    if (cwPressed || ccwPressed) {
      lastOutputChange = currentMillis;
      stop_AutoRotate();
    }
  }


  // Ochrana proti aktivaci obou výstupů současně

  if (cwPressed && ccwPressed) {
    stopMotor();
  } else if (cwPressed && !ccwPressed) {
    if (!isRunning || !direction) {  // pokud nejede, nebo jede opačně
      if (currentMillis - lastOutputChange > 1000) {
        stopMotor();  // bezpečné zastavení před změnou směru
        delay(100);
        lastOutputChange = currentMillis;
        CW_Motor();
      }
    }
  } else if (ccwPressed && !cwPressed) {
    if (!isRunning || direction) {  // pokud nejede, nebo jede opačně
      if (currentMillis - lastOutputChange > 1000) {
        stopMotor();
        delay(100);
        lastOutputChange = currentMillis;
        CCW_Motor();
      }
    }
  } else if (AutoRotate != -1.0) {
    // provádí se autorotace
  } else {
    stopMotor();
  }


   // LED - motor is running
   if (isRunning) {  // pokud se motor rotátoru otáčí
       digitalWrite(PIN_LED_MOTOR, HIGH); 
      } else {  
       digitalWrite(PIN_LED_MOTOR, LOW); 
      }
   


  strip.show();  // Aktualizace LED pásku

  //debug_monitor();  // povolit sériový výstup pro ladění

}  // konec "void loop()"


// ################################################################
// #########################  funkce  #############################
// ################################################################


void clear_blue_LED() {
  lastChangeTime = InactiveTime + 1000;  // zhasne Modrou LED okamžitě
}


void Angle_display(int displayAngle) {
  // zobrazení "úhlu"
  if (displayAngle > 360) {
    int over360Angle = (int)displayAngle - 360;
    uint8_t digits[] = { display.encodeDigit(1), 0x00, display.encodeDigit(over360Angle / 10), display.encodeDigit(over360Angle % 10) };
    display.setSegments(digits, 4, 0);
  } else {
    display.showNumberDec((int)displayAngle, false);
  }
}


void Auto_display() {
  uint8_t digits[] = { 0x77, 0x1C, 0x78, 0x5C };  // Auto
  display.setSegments(digits, 4, 0);              //  obrazení na displeji 'Auto'
}


void stop_AutoRotate() {
  stopMotor();
  AutoRotate = -1.0;  // Uložení úhlu na -1 při přerušení
  clear_blue_LED();   // zhasne Modrou LED okamžitě
  displayMode = 0;    // přepne na zobrazení 'úhlu'
}


void CW_Motor() {
  direction = true;  // CW
  digitalWrite(CCW_RUN_PIN, LOW);
  digitalWrite(CW_RUN_PIN, HIGH);
  delay(10);
  accelerateMotor(maxSpeed, true);  // zrychlení (up)
  isRunning = true;
}

void CCW_Motor() {
  direction = false;  // CCW
  digitalWrite(CW_RUN_PIN, LOW);
  digitalWrite(CCW_RUN_PIN, HIGH);
  delay(10);
  accelerateMotor(maxSpeed, true);  // zrychlení (up)
  isRunning = true;
}


void stopMotor() {
  accelerateMotor(0, false);  // zpomalení (down)
  isRunning = false;
  digitalWrite(CCW_RUN_PIN, LOW);
  digitalWrite(CW_RUN_PIN, LOW);
  analogWrite(ENABLE_PWM, 0);
  delay(10);
}


void accelerateMotor(int targetSpeed, bool isUp) {
  float rampTime = isUp ? rampTimeUp : rampTimeDown;  // Vyber čas rampy podle směru
  int stepSize = max(1, maxSpeed / (rampTime * 1000.0 / stepDelay));


  if (currentSpeed < targetSpeed) {
    for (int s = currentSpeed; s <= targetSpeed; s += stepSize) {
      setMotorSpeed(min(s, targetSpeed));  // zajistí nepřekročení
      delay(stepDelay);
    }
  } else {
    for (int s = currentSpeed; s >= targetSpeed; s -= stepSize) {
      setMotorSpeed(min(s, targetSpeed));  // zajistí nepřekročení
      delay(stepDelay);
    }
  }
  currentSpeed = targetSpeed;
}


void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(ENABLE_PWM, speed);
}



// ##############################   Kalibrace potenciometru   ############################################

void checkButtons() {

  static unsigned long pressStart = 0;
  static bool buttonPressed = false;
  static uint8_t pressedButton = 0;

  bool bCal = !digitalRead(BUTTON_CAL_PIN);
  bool bSet = !digitalRead(BUTTON_SET_PIN);
  bool bFull = !digitalRead(BUTTON_FULL_PIN);

  // Detekce stisku tlačítka
  if ((bCal || bSet || bFull) && !buttonPressed) {
    buttonPressed = true;
    pressStart = millis();

    if (bCal && !bSet && !bFull) pressedButton = 1;
    else if (bSet && !bCal && !bFull) pressedButton = 2;
    else if (bFull && !bCal && !bSet) pressedButton = 3;
    else pressedButton = 0;  // více tlačítek současně – ignorovat

    //Serial.print("Pressed button: ");
    //Serial.println(pressedButton);
  }

  // Tlačítko uvolněno
  if (!bCal && !bSet && !bFull && buttonPressed) {
    buttonPressed = false;
    unsigned long pressDuration = millis() - pressStart;

    // Serial.print("Button released, duration: ");
    // Serial.println(pressDuration);

    uint8_t digits[4];  // Pole pro zobrazení

    // --- Dlouhý stisk ---
    if (pressDuration >= LONG_PRESS_DURATION) {
      if (!calibrationMode) {
        // Vstup do kalibrace
        calibrationMode = true;
        digits[0] = 0x39;
        digits[1] = 0x77;
        digits[2] = 0x38;
        digits[3] = display.encodeDigit(int(currentProfile + 1));  // CaL + číslo profilu
        display.setSegments(digits, 4, 0);
        //Serial.println("Entering calibration mode");
        delay(1500);
        angleIndex = 0;       // nastaví pozici 0
        Calibrate_display();  // Funkce pro zobrazení "úhlu pri kalibraci"


      } else {
        // Ukončení kalibrace a reset MCU
        calibrationMode = false;
        digits[0] = 0x79;
        digits[1] = 0x54;
        digits[2] = 0x5e;
        digits[3] = 0x58;  // Endc
        display.setSegments(digits, 4, 0);
        //Serial.println("Exiting calibration mode");
        delay(1500);
        asm volatile("  jmp 0");  // Skok na adresu 0, což způsobí restart MCU
        //Angle_display(lastAngle);  // Funkce pro zobrazení "úhlu anteny"
      }

      lastActivityTime = millis();
    }

    // --- Krátký stisk v kalibračním režimu ---
    else if (calibrationMode && pressDuration < LONG_PRESS_DURATION) {
      switch (pressedButton) {
        case 1:  // Set
          angleIndex = (angleIndex + 1) % NumCalibrationPoints;
          Calibrate_display();  // Funkce pro zobrazení "úhlu pri kalibraci"
          //Serial.println("Next angle selected");
          break;

        case 2:  // Save
          saveAngleVoltage(angleIndex);
          digits[0] = 0x6d;
          digits[1] = 0x79;
          digits[2] = 0x78;
          digits[3] = 0x58;  // SEtc
          display.setSegments(digits, 4, 0);
          delay(1500);
          angleIndex = (angleIndex + 1) % NumCalibrationPoints;  // nastaví další úhel
          Calibrate_display();                                   // Funkce pro zobrazení "úhlu pri kalibraci"
          //Serial.println("Angle saved");
          break;

        case 3:  // Max
          saveMaxVoltage();
          digits[0] = 0x71;
          digits[1] = 0x1c;
          digits[2] = 0x38;
          digits[3] = 0x38;  // FuLL
          display.setSegments(digits, 4, 0);
          delay(1500);
          asm volatile("  jmp 0");  // Skok na adresu 0, což způsobí restart MCU                                   
          break;

        default:
          //Serial.println("Short press ignored: invalid or multiple buttons");
          break;
      }

      delay(1500);
      lastActivityTime = millis();
    }

    // --- Krátký stisk mimo kalibraci:  nastaví sadu rot1 - rot3
    else {
      
      switch (pressedButton) {
        case 1:  // Set
          currentProfile = 0;                         
          loadProfileFromEEPROM(currentProfile);  // Načti data pro rot1                     
          break;

        case 2:  // Save
          currentProfile = 1;                    
          loadProfileFromEEPROM(currentProfile); // Načti data pro rot2
          break;

        case 3:  // Max
          currentProfile = 2;                                       
          loadProfileFromEEPROM(currentProfile); // Načti data pro rot3  
          break;

        default:
          //Serial.println("Short press ignored: invalid or multiple buttons");
          break;
      }
    }

    pressedButton = 0;  // reset
  }

  // autoatické ukončrní kalibrace po xxx min nečinosti bez resetu MCU
  if (calibrationMode && (millis() - lastActivityTime > IDLE_TIMEOUT)) {
    calibrationMode = false;
    uint8_t digits[4];  // Pole pro zobrazení
    digits[0] = 0x79;
    digits[1] = 0x54;
    digits[2] = 0x5e;
    digits[3] = 0x58;  // Endc
    display.setSegments(digits, 4, 0);
    delay(1500);
    Angle_display(lastAngle);  // Funkce pro zobrazení "úhlu anteny"
  }
}



void Calibrate_display() {
  int angle = (int)anglePoints[angleIndex];
  uint8_t digits[4];  // pole segmentových hodnot
  digits[0] = 0x58;   // např. nějaká značka, zůstává beze změny

  // Stovky
  if (angle >= 100)
    digits[1] = display.encodeDigit((angle / 100) % 10);
  else
    digits[1] = 0x00;  // vypnutí segmentu (prázdno)

  // Desítky
  if (angle >= 10)
    digits[2] = display.encodeDigit((angle / 10) % 10);
  else
    digits[2] = 0x00;  // prázdno

  // Jednotky - vždy se zobrazuje
  digits[3] = display.encodeDigit(angle % 10);

  display.setSegments(digits, 4, 0);
}


// Uloží jednu hodnotu z anglePoints[index]
void saveAngleVoltage(int index) {

  float voltage = analogRead(ANALOG_PIN) * (ADC_RefVoltage / ADC_MaxValue);
  voltagePoints[index] = voltage;

  if (index < 0 || index >= NumCalibrationPoints) return;

  int base = getProfileBaseAddress(currentProfile);
  int addr = base + index * sizeof(float);

  EEPROM.put(addr, voltagePoints[index]);
  Serial.print("Uloženo: voltagePoints[");
  Serial.print(index);
  Serial.print("] = ");
  Serial.println(voltagePoints[index]);
}


// Uloží hodnotu voltageAtMax
void saveMaxVoltage() {
  float voltage = analogRead(ANALOG_PIN) * (ADC_RefVoltage / ADC_MaxValue);
  voltageAtMax = voltage;

  int base = getProfileBaseAddress(currentProfile);
  int addr = base + NumCalibrationPoints * sizeof(float);

  EEPROM.put(addr, voltageAtMax);
  Serial.print("Uloženo voltageAtMax = ");
  Serial.println(voltageAtMax);
}



// Uloží číslo profilu na EEPROM[0]
void saveCurrentProfile() {
  EEPROM.update(0, currentProfile);

  uint8_t digits[] = { 0x50, 0x5c, 0x78, display.encodeDigit(int(currentProfile + 1)) };  // rot1...rot3
  display.setSegments(digits, 4, 0);
  delay(1500);

  Serial.print("Nastaven Rotátor č: ");
  Serial.println(currentProfile+1);
}


// Načte číslo profilu z EEPROM[0]
void loadCurrentProfile() {
  int stored = EEPROM.read(0);
  if (stored >= 0 && stored < MaxProfiles) {
    currentProfile = stored;
  } else {
    currentProfile = 0;  // Výchozí hodnota
  }  
}




// ##############################    Hamlib / Tučňák OK1ZIA  ############################################
/*
příjem příkazů na seriové lince na základě protokolu https://hamlib.sourceforge.net/html/rotctl.1.html#COMMANDS

Rotátor např.:  rotctl -l
 Rot #  Mfg         Model       Version         Status        Macro
   601  Yaesu       GS-232A     20201203.0      Beta          ROT_MODEL_GS232A


C:\Program Files (x86)\Tucnak> rotctl -vvvvv -m 601 -r COM3  

--------------------------------------------------------------------------------------

Rotator command: P
Azimuth: 65
Elevation: 0

rot_set_position called az=65.00 el=0.00
rot_set_position: south_zero=0
gs232a_rot_set_position called: 65.00 0.00
rig_flush: called for serial device
serial.c(642):serial_flush entered
tcflush
serial.c(674):serial_flush return(0)
write_block(): TX 9 bytes
0000    57 30 36 35 20 30 30 30 0d                          W065 000.

--------------------------------------------------------------------------------------

Rotator command: p

rot_get_position called
gs232a_rot_get_position called
rig_flush: called for serial device
serial.c(642):serial_flush entered
tcflush
serial.c(674):serial_flush return(0)
write_block(): TX 3 bytes
0000    43 32 0d                                            C2.
read_string called, rxmax=32
read_string(): RX 11 characters
0000    2b 30 30 38 31 2b 30 30 30 30 0d                    +0081+0000.
gs232a_rot_get_position: (az, el) = (81.0, 0.0)
rot_get_position: got az=81.00, el=0.00
Azimuth: 81.00
Elevation: 0.00

*/


void Hamlib_Tucnak() {
  static String lineBuffer = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r' || c == '\n') {
      lineBuffer.trim();

      if (lineBuffer.length() >= 2) {
        if (lineBuffer.startsWith("C2")) {
          send_LastPosition();
        }
        else if (lineBuffer.startsWith("W")) {
          String params = lineBuffer.substring(1);
          params.trim();

          int spaceIndex = params.indexOf(' ');
          if (spaceIndex > 0) {
            String azStr = params.substring(0, spaceIndex);
            String elStr = params.substring(spaceIndex + 1);
            azStr.trim();
            elStr.trim();

            int az = azStr.toInt();
            int el = elStr.toInt();

            // Převod azimutu z -180..+180 na 0..360
            if (az < 0) {
              az = 360 + az;
            }
            // (volitelně) omezit na 0..360 (pro jistotu)
            if (az >= 360) az -= 360;
            if (az < 0) az = 0;

            Hamlib_Azimuth = az;
            Hamlib_Elevation = el;


            // nastavení Azimutu a elevace z progamu Tučňák (Alt+R)
            AutoRotate = Hamlib_Azimuth;
            lastElevation = Hamlib_Elevation;
            

            display.showNumberDec(Hamlib_Azimuth, false);  // potlačí úvodní "0"
          }
        }
      }

      lineBuffer = "";
    }
    else {
      lineBuffer += c;
    }
  }
}


void send_LastPosition() {
  char buffer[12];

  auto formatSigned = [](int val, char *out) {
    char sign = (val < 0) ? '-' : '+';
    int absVal = abs(val);
    snprintf(out, 6, "%c0%03d", sign, absVal);  // Přidá "0" před číslo => +0xxx
  };

  char azBuf[6], elBuf[6];  // Potřeba místa pro '\0'
 
  formatSigned(lastAngle, azBuf);
  formatSigned(lastElevation, elBuf);

  snprintf(buffer, sizeof(buffer), "%s%s\r", azBuf, elBuf);  // Bez mezery mezi hodnotami
  Serial.print(buffer);
}



// ############################  Sériový výstup pro ladění  #######################################

void debug_monitor() {
  //Serial.print(">>> ");

  //Serial.print("Sensor Value: ");Serial.print(sensorValue);
  //Serial.print(" | Analog input: ");Serial.print(inputVoltage, 2);Serial.print("V | Angle: ");Serial.print(lastAngle);Serial.print("° ");
  //Serial.print("° | LED Index: ");Serial.print(ledIndex);
  //Serial.print(" | CCW Button: ");Serial.print(digitalRead(CCW_BUTTON));Serial.print(" > Output: ");Serial.print(digitalRead(ENABLE_PWM));
  //Serial.print("enc.read(): ");Serial.print(pos);Serial.print(" | lastPos: ");Serial.print(lastPos);Serial.print(" |  ledPos: ");Serial.print(ledPos);
  //Serial.print("Angle: ");
  //Serial.print(angle);
  //Serial.print("° | LED Index: ");
  //Serial.print(ledIndex);

  Serial.println(" >>>");  // odřádkování výpisu
}