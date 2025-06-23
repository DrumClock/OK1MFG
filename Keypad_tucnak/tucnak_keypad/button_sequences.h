/*
Example:
#define BUT_1 "ALT+M, #300, CTRL+V, ALT+F4, END"

Supported:
- ALT+X,  CTRL+X, SHIFT+X, ALT+F4, CTRL+ENTER, ALT+TAB
- #xxx (pause in ms)
- Single char keys ('x')
- END, ESC, TAB, BACKSPACE, ENTER, DEL
- NUM0 ... NUM9

*/

#pragma once

/*
klávesové zkratky pro VHF log Tučňák - OK1ZIA
https://tucnak.nagano.cz/wiki/Hotkeys

 MAX 16 - klávesových zkratek - Arduino Pro Micro

*/

#define BUT_1  "ALT+M"    // přepíná režimy CW/SSB
#define BUT_2  "ALT+B"    // přepne pásmo
#define BUT_3  "ALT+C"    // prohodí značku v nedokončeném spojení
#define BUT_4  "ALT+V"    // prohodí lokátor v nedokončeném spojení
#define BUT_5  "CTRL+A"   // otočí první rotátor na aktuální azimut
#define BUT_6  "CTRL+B"   // otočí druhý rotátor na aktuální azimut

// nevyužité funkce

#define BUT_7  "ALT+R, #300"   // Rotátor - azimut
#define BUT_8  "ALT+1"         // Okno Spojení 
#define BUT_9  "ALT+7"         // Okno KST chat
#define BUT_10 "ALT+9"         // Okno Mapy
#define BUT_11 "ALT+O, #300, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, O, P, T, "         // změna operátora 
#define BUT_12 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, M, F, G, ENTER"     // změna operátora MFG
#define BUT_13 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, J, O, C, ENTER"     // změna operátora JOC
#define BUT_14 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, A, P, A, ENTER"     // změna operátora APA
#define BUT_15 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, Z, J, H, ENTER"     // změna operátora ZJH
#define BUT_16 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, V, I, G, ENTER"     // změna operátora VIG


const char* SEQUENCES[] = {
  BUT_1, BUT_2, BUT_3, BUT_4,
  BUT_5, BUT_6, BUT_7, BUT_8,
  BUT_9, BUT_10, BUT_11, BUT_12,
  BUT_13, BUT_14, BUT_15, BUT_16
};

