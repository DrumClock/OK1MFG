/*
// --------------VERZE  17.5.2026 -------------

 5-Stupňový PTT Sequencer s HW volbou STEP_DELAY (binární DIP, sčítání)
 Watchdog provádí nouzovou sekvenci 50ms v reverzním pořadí
 HW volba STEP_DELAY pomocí DIP přepínače - hodnoty se SČÍTAJÍ:
   DIP1 = 10 ms
   DIP2 = 20 ms
   DIP3 = 40 ms
   DIP4 = 80 ms

   Rozsah 20-150 ms, krok 10 ms.
   Bezpečné minimum MIN_STEP_DELAY (20 ms = výchozí hodnota při nesepnutém DIP).

 Výstupy sekvenceru (PTT_x = stupeň x)
    PTT_1   - LNA napájení RX nebo PTT TX  
    PTT_2   - rezerva
    PTT_3   - TRX - PTT
    PTT_4   - rezerva
    PTT_5   - PA - PTT

// ---------------------------
*/

// Serial debug - zakomentuj řádek níže pro vypnutí veškerého debug výstupu
#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
  #define DBG_BEGIN(x)   Serial.begin(x)
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_BEGIN(x)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

// PTT vstup
#define PTT_IN 12

// Výstupy sekvenceru (PTT_x = stupeň x)
#define PTT_1   10   // stupeň 1 - LNA - napájení / PTT
#define PTT_2   9   // stupeň 2 - rezerva
#define PTT_3   8   // stupeň 3 - TRX - PTT
#define PTT_4   7   // stupeň 4 - rezerva
#define PTT_5   6   // stupeň 5 - rezerva

// Počet stupňů sekvenceru
#define NUM_STAGES 5

// Watchdog timeout - max čas pro celou TX_ON_SEQ nebo TX_OFF_SEQ
// Měl by být větší než NUM_STAGES * max STEP_DELAY s rezervou
// Max sekvence: 5 * 150 = 750 ms, watchdog 1000 ms = rezerva 250 ms
#define WATCHDOG_TIMEOUT 1000
#define EMERGENCY_STEP_DELAY 50  // ms - krok pro nouzové vypnutí

// Debounce nastavení
#define DEBOUNCE_DELAY 50  // 50 ms debounce pro PTT

// Bezpečné minimum STEP_DELAY - slouží zároveň jako default při nesepnutém DIP
// (chrání proti příliš krátkým intervalům)
#define MIN_STEP_DELAY 20

// HW DIP přepínač pro volbu STEP_DELAY - hodnoty se SČÍTAJÍ (binární)
// Rozsah 10-150 ms s krokem 10 ms, výsledek omezen zdola na MIN_STEP_DELAY
struct DelayOption {
  uint8_t pin;
  unsigned long value;
};

const DelayOption DELAY_OPTIONS[] = {
  { 2,  10},  // DIP1 - bit 0 (10 ms)
  { 3,  20},  // DIP2 - bit 1 (20 ms)
  { 4,  40},  // DIP3 - bit 2 (40 ms)
  { 5,  80}   // DIP4 - bit 3 (80 ms)
};
const uint8_t NUM_DELAY_OPTIONS = sizeof(DELAY_OPTIONS) / sizeof(DELAY_OPTIONS[0]);

// ---------------------------
// Globální proměnné
// ---------------------------
bool pttState = false;        // Stabilní (debounced) stav PTT
bool pttRaw = false;          // Aktuální surový stav PTT vstupu
bool lastPttRaw = false;      // Předchozí surový stav (pro debounce)
unsigned long lastDebounceTime = 0;  // Čas poslední změny surového vstupu

unsigned long seqStart = 0;
unsigned long watchdogTimer = 0;
unsigned long STEP_DELAY = MIN_STEP_DELAY;

int seqStep = 0;
enum SeqState {IDLE, TX_ON_SEQ, TX_ON_DONE, TX_OFF_SEQ};
SeqState seqState = IDLE;

// ---------------------------
// Pomocná funkce pro logování změn výstupů
// ---------------------------
void setOutput(uint8_t pin, bool value, const char* name) {
  digitalWrite(pin, value ? HIGH : LOW);
  DBG_PRINT("[");
  DBG_PRINT(millis());
  DBG_PRINT(" ms] ");
  DBG_PRINT(name);
  DBG_PRINT(" -> ");
  DBG_PRINTLN(value ? "HIGH" : "LOW");
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
  DBG_BEGIN(9600);
  DBG_PRINTLN("");
  DBG_PRINTLN("========================================");
  DBG_PRINTLN(" 5-Channel PTT Sequencer - Debug Mode");
  DBG_PRINTLN("========================================");

  pinMode(PTT_IN, INPUT_PULLUP);

  pinMode(PTT_1, OUTPUT);
  pinMode(PTT_2, OUTPUT);
  pinMode(PTT_3, OUTPUT);
  pinMode(PTT_4, OUTPUT);
  pinMode(PTT_5, OUTPUT);

  digitalWrite(PTT_1, LOW);
  digitalWrite(PTT_2, LOW);
  digitalWrite(PTT_3, LOW);
  digitalWrite(PTT_4, LOW);
  digitalWrite(PTT_5, LOW);

  // Inicializace HW pinů pro volbu STEP_DELAY
  for (uint8_t i = 0; i < NUM_DELAY_OPTIONS; i++) {
    pinMode(DELAY_OPTIONS[i].pin, INPUT_PULLUP);
  }

  watchdogTimer = millis();

  DBG_PRINTLN("Init OK, state = IDLE, all outputs LOW");
  DBG_PRINTLN("Waiting for PTT...");
  DBG_PRINTLN("");
}

// ---------------------------
// Hlavní smyčka
// ---------------------------
void loop() {
  // --- Debouncing PTT vstupu ---
  pttRaw = !digitalRead(PTT_IN); // aktivní LOW

  // Reset timeru pouze při skutečné změně surového stavu
  if (pttRaw != lastPttRaw) {
    lastDebounceTime = millis();
    lastPttRaw = pttRaw;
  }

  // Pokud je surový stav stabilní déle než DEBOUNCE_DELAY, akceptuj ho
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (pttState != pttRaw) {
      pttState = pttRaw;
      DBG_PRINT("[");
      DBG_PRINT(millis());
      DBG_PRINT(" ms] PTT_IN = ");
      DBG_PRINTLN(pttState ? "ACTIVE (TX)" : "RELEASED (RX)");
    }
  }

  // --- HW čtení DIP pinů pro STEP_DELAY (binární sčítání) ---
  unsigned long sumDelay = 0;
  for (uint8_t i = 0; i < NUM_DELAY_OPTIONS; i++) {
    if (!digitalRead(DELAY_OPTIONS[i].pin)) {  // aktivní LOW (DIP sepnut)
      sumDelay += DELAY_OPTIONS[i].value;
    }
  }
  // Ořezání na bezpečné minimum (slouží i jako default při sumDelay = 0)
  unsigned long newDelay = (sumDelay < MIN_STEP_DELAY) ? MIN_STEP_DELAY : sumDelay;

  // Log při změně STEP_DELAY
  if (newDelay != STEP_DELAY) {
    STEP_DELAY = newDelay;
    DBG_PRINT("[");
    DBG_PRINT(millis());
    DBG_PRINT(" ms] STEP_DELAY = ");
    DBG_PRINT(STEP_DELAY);
    DBG_PRINTLN(" ms");
  }

  // --- Watchdog - hlídá max čas celé sekvence (TX_ON_SEQ nebo TX_OFF_SEQ) ---
  // V TX_ON_DONE se resetuje (vysílání může trvat libovolně dlouho)
  if (seqState != IDLE && (millis() - watchdogTimer > WATCHDOG_TIMEOUT)) {
    DBG_PRINTLN("");
    DBG_PRINT("[");
    DBG_PRINT(millis());
    DBG_PRINTLN(" ms] *** WATCHDOG TIMEOUT - EMERGENCY SHUTDOWN ***");

    setOutput(PTT_5, LOW, "PTT_5"); delay(EMERGENCY_STEP_DELAY);
    setOutput(PTT_4, LOW, "PTT_4"); delay(EMERGENCY_STEP_DELAY);
    setOutput(PTT_3, LOW, "PTT_3 (TRX PTT - VF STOP)"); delay(EMERGENCY_STEP_DELAY);
    setOutput(PTT_2, LOW, "PTT_2"); delay(EMERGENCY_STEP_DELAY);
    setOutput(PTT_1, LOW, "PTT_1 (LNA ON)");

    DBG_PRINTLN("Emergency shutdown complete, state = IDLE");
    DBG_PRINTLN("");
    seqState = IDLE;
    seqStep = 0;
  }

  // --- Sekvencer ---
  // Detekce podle ÚROVNĚ pttState, ne hrany!
  switch (seqState) {
    case IDLE:
      if (pttState) {  // PTT aktivní → začni TX sekvenci
        DBG_PRINT("[");
        DBG_PRINT(millis());
        DBG_PRINT(" ms] >>> TX_ON_SEQ started (STEP_DELAY=");
        DBG_PRINT(STEP_DELAY);
        DBG_PRINTLN(" ms)");
        seqStart = millis();
        seqStep = 0;
        seqState = TX_ON_SEQ;
        watchdogTimer = millis();  // Start watchdog timer pro sekvenci
      }
      break;

    case TX_ON_SEQ:
      if (seqStep < NUM_STAGES && (millis() - seqStart >= STEP_DELAY)) {
        switch(seqStep) {
          case 0: setOutput(PTT_1, HIGH, "PTT_1 (LNA OFF)"); break;
          case 1: setOutput(PTT_2, HIGH, "PTT_2"); break;
          case 2: setOutput(PTT_3, HIGH, "PTT_3 (TRX PTT)"); break;
          case 3: setOutput(PTT_4, HIGH, "PTT_4"); break;
          case 4: setOutput(PTT_5, HIGH, "PTT_5 (PA PTT)"); break;
        }
        seqStep++;
        seqStart = millis();
        // POZN: watchdogTimer SE NERESETUJE při krocích - hlídá max čas celé sekvence
        if (seqStep >= NUM_STAGES) {
          seqState = TX_ON_DONE;
          DBG_PRINT("[");
          DBG_PRINT(millis());
          DBG_PRINTLN(" ms] === TX_ON_DONE (transmitting) ===");
        }
      }
      break;

    case TX_ON_DONE:
      // Vysílání může trvat libovolně dlouho - watchdog se resetuje
      watchdogTimer = millis();

      if (!pttState) {  // PTT uvolněn → začni vypínací sekvenci
        DBG_PRINT("[");
        DBG_PRINT(millis());
        DBG_PRINTLN(" ms] <<< TX_OFF_SEQ started");
        seqStart = millis();
        seqStep = 0;
        seqState = TX_OFF_SEQ;
        watchdogTimer = millis();  // Start watchdog timer pro vypínací sekvenci
      }
      break;

    case TX_OFF_SEQ:
      if (seqStep < NUM_STAGES && (millis() - seqStart >= STEP_DELAY)) {
        switch(seqStep) {
          case 0: setOutput(PTT_5, LOW, "PTT_5 (PA PTT OFF)"); break;
          case 1: setOutput(PTT_4, LOW, "PTT_4"); break;
          case 2: setOutput(PTT_3, LOW, "PTT_3 (TRX PTT OFF)"); break;
          case 3: setOutput(PTT_2, LOW, "PTT_2"); break;
          case 4: setOutput(PTT_1, LOW, "PTT_1 (LNA ON)"); break;
        }
        seqStep++;
        seqStart = millis();
        // POZN: watchdogTimer SE NERESETUJE při krocích - hlídá max čas celé sekvence
        if (seqStep >= NUM_STAGES) {
          seqState = IDLE;
          DBG_PRINT("[");
          DBG_PRINT(millis());
          DBG_PRINTLN(" ms] === IDLE (RX, watchdog off) ===");
          DBG_PRINTLN("");
        }
      }
      break;
  }
}
