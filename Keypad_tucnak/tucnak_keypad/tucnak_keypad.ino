#include <Keyboard.h>
#include "button_sequences.h"

#define MOD_CTRL  KEY_LEFT_CTRL
#define MOD_SHIFT KEY_LEFT_SHIFT
#define MOD_ALT   KEY_LEFT_ALT


/*
  2, 3, 4, 5, 6, 7, 8, 9, 10,    - 9x standardní piny
  14, 15, 16,                    - 3x SPI piny
  18, 19, 20, 21                 - 4x analogové A0–A3 jako digitální 
*/  

// Buttons - max. 16 - Arduino Pro Micro
constexpr uint8_t BUTTON_PINS[] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16, 18, 19, 20, 21 };
constexpr uint8_t NUM_BUTTONS = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);

// -------------------------------------------------------

void setup() {
  Keyboard.begin();
  Serial.begin(9600);
  delay(1500);

  // všechny piny BUTTON na PULLUP mód
  for (const uint8_t pin : BUTTON_PINS) {
    pinMode(pin, INPUT_PULLUP);

    Serial.print("pinMode(");
    Serial.print(pin);
    Serial.println(", INPUT_PULLUP)");
    delay(50);
  }
}

// -------------------------------------------------------

void loop() {    
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    if (digitalRead(BUTTON_PINS[i]) == LOW) {
      sendSequence(SEQUENCES[i]);
      while (digitalRead(BUTTON_PINS[i]) == LOW)
        ;         // Wait for release
      delay(50);  // Debounce delay

      Serial.print("stisknuto tlačítko: ");
      Serial.print(i+1);
      Serial.print(", sekvence: ");
      Serial.println(SEQUENCES[i]);
    }
  }
}

// -------------------------------------------------------

void sendSequence(const char* seqstr) {
  char buffer[128];
  strncpy(buffer, seqstr, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char* token = strtok(buffer, ",");
  while (token != NULL) {
    while (*token == ' ') token++;  // trim left space

    if (strncmp(token, "#", 1) == 0) {
      int pause = atoi(token + 1);
      delay(pause);
    } else if (strncmp(token, "ALT+", 4) == 0 && strlen(token) == 5) {
      char c = token[4];
      Keyboard.press(MOD_ALT);
      Keyboard.press(c);
      delay(100);
      Keyboard.release(c);
      Keyboard.release(MOD_ALT);
    } else if (strncmp(token, "CTRL+", 5) == 0 && strlen(token) == 6) {
      char c = token[5];
      Keyboard.press(MOD_CTRL);
      Keyboard.press(c);
      delay(100);
      Keyboard.release(c);
      Keyboard.release(MOD_CTRL);
    } else if (strncmp(token, "SHIFT+", 6) == 0 && strlen(token) == 7) {
      char c = token[6];
      Keyboard.press(MOD_SHIFT);
      Keyboard.press(c);
      delay(100);
      Keyboard.release(c);
      Keyboard.release(MOD_SHIFT);
    } else if (strcmp(token, "ALT+F4") == 0) {
      Keyboard.press(MOD_ALT);
      Keyboard.press(KEY_F4);
      delay(100);
      Keyboard.release(KEY_F4);
      Keyboard.release(MOD_ALT);
    } else if (strcmp(token, "CTRL+ENTER") == 0) {
      Keyboard.press(MOD_CTRL);
      Keyboard.press(KEY_RETURN);
      delay(100);
      Keyboard.release(KEY_RETURN);
      Keyboard.release(MOD_CTRL);
    } else if (strcmp(token, "ALT+TAB") == 0) {
      Keyboard.press(MOD_ALT);
      Keyboard.press(KEY_TAB);
      delay(100);
      Keyboard.release(KEY_TAB);
      Keyboard.release(MOD_ALT);
    } else if (strcmp(token, "DEL") == 0) {
      Keyboard.press(KEY_DELETE);
      delay(100);
      Keyboard.release(KEY_DELETE);
    } else if (strcmp(token, "ENTER") == 0) {
      Keyboard.press(KEY_RETURN);
      delay(100);
      Keyboard.release(KEY_RETURN);
    } else if (strcmp(token, "ESC") == 0) {
      Keyboard.press(KEY_ESC);
      delay(100);
      Keyboard.release(KEY_ESC);
    } else if (strcmp(token, "TAB") == 0) {
      Keyboard.press(KEY_TAB);
      delay(100);
      Keyboard.release(KEY_TAB);
    } else if (strcmp(token, "BACKSPACE") == 0) {
      Keyboard.press(KEY_BACKSPACE);
      delay(100);
      Keyboard.release(KEY_BACKSPACE);
    } else if (strncmp(token, "END", 3) == 0) {
      break;
    } else if (strcmp(token, "NUM0") == 0) {
      Keyboard.press(KEY_KP_0);
      delay(100);
      Keyboard.release(KEY_KP_0);
    } else if (strcmp(token, "NUM1") == 0) {
      Keyboard.press(KEY_KP_1);
      delay(100);
      Keyboard.release(KEY_KP_1);
    } else if (strcmp(token, "NUM2") == 0) {
      Keyboard.press(KEY_KP_2);
      delay(100);
      Keyboard.release(KEY_KP_2);
    } else if (strcmp(token, "NUM3") == 0) {
      Keyboard.press(KEY_KP_3);
      delay(100);
      Keyboard.release(KEY_KP_3);
    } else if (strcmp(token, "NUM4") == 0) {
      Keyboard.press(KEY_KP_4);
      delay(100);
      Keyboard.release(KEY_KP_4);
    } else if (strcmp(token, "NUM5") == 0) {
      Keyboard.press(KEY_KP_5);
      delay(100);
      Keyboard.release(KEY_KP_5);
    } else if (strcmp(token, "NUM6") == 0) {
      Keyboard.press(KEY_KP_6);
      delay(100);
      Keyboard.release(KEY_KP_6);
    } else if (strcmp(token, "NUM7") == 0) {
      Keyboard.press(KEY_KP_7);
      delay(100);
      Keyboard.release(KEY_KP_7);
    } else if (strcmp(token, "NUM8") == 0) {
      Keyboard.press(KEY_KP_8);
      delay(100);
      Keyboard.release(KEY_KP_8);
    } else if (strcmp(token, "NUM9") == 0) {
      Keyboard.press(KEY_KP_9);
      delay(100);
      Keyboard.release(KEY_KP_9);
    } else if (strlen(token) == 1) {
      char c = token[0];
      // Pouze pro písmena a ostatní znaky (ne číslice)
      Keyboard.write(c);
      delay(100);
    }

    token = strtok(NULL, ",");
  }
}