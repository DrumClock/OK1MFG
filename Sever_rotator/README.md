# Ovládání rotátoru "SEVER"

![Rotator_frame](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/IMG_panel.jpg)

# Popis

 - Ovládání DC motoru rotátoru pomocí H-můstku (24V)
 - Snímání azimutu pomocí potenciometru (napěťový dělič)
 - Zobrazení azimutu na displeji TM1637
 - zobrazení arimutu pomocí LED na krukové mapě (Neopixel 60 LED)
 - nastavení AUTOROTACE encoderem KY-040
 - kalibrace snímače a uložení do EEPROM (max. 3 rotátory)

# Ovládání

 Po zapnutí (resetu) dojde k testu všech LED a Displeje.
 Po testu se rozsvítí na kruhové mapě modrá LED na pozici 0 (sever),
 nyní můžeme stisknout encoder a rotátor se nastaví na pozici "sever".
 Pokud tak neučiníme LED po 3s zhasne.

 AUTO<br>
 Pro nastavení automatické rotace otočíme **`encoderem`** na požadovaný "azimut" a stiskneme encoder. <br>
 Tím dojde k otáčení antény.<br>
 Přerušení rotace je možné stiskem tlačítka pro manuální rotaci.

 ROTACE<br>
 Manuální rotace je možná pomocí tlačítek **`CW`** a **`CCW`**, která jsou blokována proti
 současnému tisku a rychlému změně směru. <br>
 Takže mezi změnou směru je prodleva 1s, toto zabrání k rázům a zmenší namáhání převodů rotátoru.


 # tlačítka "Kalibrace"
  Kalibrace nelinearity potenciometru pomocí tlačítek  **`M1/C`**, **`M2/S`**, **`M3/F`** a uložení do EEPROM 

 ![Scheme_frame](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/IMG_kalibrace.jpg)
 
  Krátkým stiskem načteme 3 uložené předvolby kalibrace pro různé rotátory <br>
  
  - Tlačítko **`M1`**  - displej zobrazí "rot1" 
  - Tlačítko **`M2`**  - displej zobrazí "rot2"
  - Tlačítko **`M3`**  - displej zobrazí "rot3"

  Dlouhý stisk libovolného tlačítka aktivuje kalibraci.  <br>
  displej zobrazí "CAL+číslo rotatoru" <br>
  
  - Tlačítko **`C`**  - přepíná mezi kalibračnímy úhly (0,90,180,270,360) a displejv zobrazuje např. "c 90"
  - Tlačítko **`S`**  - uloží napětí (ve voltech) pro aktuální úhel displej zobrazí "SEtc" a nastaví další úhel např. "c180"
  - Tlačítko **`F`**  - uloží napětí pro MAX úhel displej zobrazuje "FuLL" ukončí kalibraci a restartuje MCU

  Opětovný dlouhý stisk libovolného tlačítka deaktivuje kalibraci.  <br>
  displej zobrazí  "Endc" a dojde k restartu MCU  <br> 

 Po 5 minutách nečinnosti se kalibrace automaticky ukončí (bez restartu MCU)<br>

 # Nastavení VHF logu "Tučňák" 
![Tucnak_frame](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/Tu%C4%8D%C5%88%C3%A1k%20-%20nastaven%C3%AD%20rot%C3%A1toru.png)

Po tomto nastavení je aktivní odeslání azimutu:

**`Alt+R`** - zadání azimutu ručně a odeslání <br><br>
![alt_r](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/alt_r.png)
 
**`Ctrl+A`** - odešle např. azimut 195 st. ze zadaného čtverce JN51AD <br><br>
![ctrl_a](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/ctrl_a.png) 

<br><br>

# Video:  https://youtu.be/hEBSZyiuYrg

<br><br>
# Schéma:
![Scheme_frame](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/Arduino_Sever_2.png)

# MAPA z JN69NX
- přiložený soubor **`mapa JN69NX.cdr`** je pro grafický editor **`CorelDraw`** 

![mapa_frame](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/mapa.png) 

# složka CAD
- obsahuje CAD soubory pro ohybaní plechů a 3D tisk dílů
- k otevření je potřeba program FreeCAD : https://www.freecad.org

 
