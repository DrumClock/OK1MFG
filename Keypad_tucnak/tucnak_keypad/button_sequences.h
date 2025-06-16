/*
Example:
#define BUT_1 "ALT+M, #300, CTRL+V, ALT+F4, END"

Supported:
- ALT+X
- CTRL+X
- SHIFT+X
- ALT+F4
- CTRL+ENTER
- ALT+TAB
- #xxx (pause in ms)
- Single char keys ('x')
- END
- ESC
- TAB
- BACKSPACE
- ENTER
- DEL
- NUM0 ... NUM9

*/

#pragma once

/*
klávesové zkratky pro VHF log Tučňák - OK1ZIA
https://tucnak.nagano.cz/wiki/Hotkeys
*/ 

// MAX 16 - klávesových zkratek - Arduino Pro Micro

/*
#define BUT_1  "ALT+M, END"      // přepíná režimy CW/SSB
#define BUT_2  "ALT+B, c, END"   // přepíne pásmo 145 MHz
#define BUT_3  "ALT+B, e, END"   // přepíne pásmo 435 MHz
#define BUT_4  "ALT+B, g, END"   // přepíne pásmo 1,3 GHz
#define BUT_5  "ALT+C, END"      // prohodí volací znaky v nedokončeném spojení
#define BUT_6  "ALT+V, END"      // prohodí lokátor v nedokončeném spojení
#define BUT_7  "CTRL+A, END"     // otočí první rotátor na aktuální azimut
#define BUT_8  "CTRL+B, END"     // otočí druhý rotátor na aktuální azimut
#define BUT_9  "ALT+1, END"      // Okno Spojení 
#define BUT_10 "ALT+7, END"      // Okno KST chat
#define BUT_11 "ALT+O, #500"     // změna operátora 
#define BUT_12 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, M, F, G, ENTER, END"     // změna operátora MFG
#define BUT_13 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, J, O, C, ENTER, END"     // změna operátora JOC
#define BUT_14 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, A, P, A, ENTER, END"     // změna operátora APA
#define BUT_15 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, Z, J, H, ENTER, END"     // změna operátora ZJH
#define BUT_16 "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, V, I, G, ENTER, END"     // změna operátora VIG

// "ALT+9, END"        // Okno Mapy
// "ALT+R, #300, END"      // Rotátor - azimut


const char* SEQUENCES[] = {
  BUT_1, BUT_2, BUT_3, BUT_4,
  BUT_5, BUT_6, BUT_7, BUT_8,
  BUT_9, BUT_10, BUT_11, BUT_12,
  BUT_13, BUT_14, BUT_15, BUT_16
};

*/

#define BUT_1  "ALT+M, END"      // přepíná režimy CW/SSB
#define BUT_2  "ALT+O, BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE,BACKSPACE, O, K, NUM1, O, P, T, END"     // změna operátora OK1OPT
#define BUT_3  "ALT+B, c, END"   // přepne na pásmo 145 MHz
#define BUT_4  "ALT+B, e, END"   // přepne na pásmo 435 MHz
#define BUT_5  "CTRL+A, END"     // otočí první rotátor na aktuální azimut
#define BUT_6  "ALT+V, END"      // prohodí lokátor v nedokončeném spojení

const char* SEQUENCES[] = {
  BUT_1, BUT_2, BUT_3,
  BUT_4, BUT_5, BUT_6
};

