# Keypad - VHF log Tučňák

![Rotator_frame](https://github.com/DrumClock/OK1MFG/blob/main/Keypad_tucnak/IMG_keypad.jpg)

# Popis

 - spouštění Hotkey logu Tučňák (nebo jiného) pomocí jednoho tlačítka

 # Ovládání

 - tlačítko **`MODE`** simuluje stisk Alt+m 
 - tlačítko **`BAND`** simuluje stisk Alt+b 
 - tlačítko **`CAL`**  simuluje stisk Alt+c 
 - tlačítko **`LOC`**  simuluje stisk Alt+v 
 - tlačítko **`ROT1`** simuluje stisk CTRL+a 
 - tlačítko **`ROT2`** simuluje stisk CTRL+b 


 # Programování
 
  Je použita deska **Arduino Promicro** z čípem **Atmega32U4** pro simulaci klávesnice.
  U této desky je možno nastavit max 16 tlačítek.
  Programování tlačítek se provádí v souboru **`button_sequences.h`** například:
  
<pre> ``` 

#define BUT_1  "ALT+m"    // přepíná režimy CW/SSB
#define BUT_2  "ALT+b"    // přepne pásmo
#define BUT_3  "ALT+c"    // prohodí značku v nedokončeném spojení
#define BUT_4  "ALT+v"    // prohodí lokátor v nedokončeném spojení
#define BUT_5  "CTRL+a"   // otočí první rotátor na aktuální azimut
#define BUT_6  "CTRL+b"   // otočí druhý rotátor na aktuální azimut   

``` </pre>
 
# složka CAD
- obsahuje soubor krabičky pro 3D tisk 
- k otevření je potřeba program FreeCAD : https://www.freecad.org

 
