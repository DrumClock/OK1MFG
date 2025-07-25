# CW Paddle-Key

<p align="center">
  <img src="https://github.com/DrumClock/OK1MFG/raw/main/Keyer_paddle/IMG_2.jpg" alt="Ukázka" width="800">
</p>

# Popis

  Umožňuje tréning Morseovky a to jak vysílání tak příjem.
    
  
  Při vysílání Morseovy abecedy v akustické podobě se používají následující pravidla:<br> 

 - základní časovou jednotkou je délka tečky
 - čárka má stejnou dobu trvání jako tři tečky
 - zvuková pauza uvnitř značky má stejnou dobu trvání jako jedna tečka
 - zvuková pauza mezi značkami má stejnou dobu trvání jako jedna čárka
 - zvuková pauza mezi slovy má stejnou dobu trvání jako sedm teček	
 <br>
 
# Tempo „PARIS"
<br>

<p align="center">
  <img src="https://github.com/DrumClock/OK1MFG/blob/main/Keyer_paddle/paris.png" alt="Ukázka" width="600">
</p>
<br> 

<div align="center">
<table>
  <tr>
    <th>Písmeno</th>
    <th>Morse</th>
    <th>Časování</th>
    <th>Počty jednotek</th>    
  </tr>
  <tr>
    <td>P</td>
    <td>di da da di</td>
    <td>1 1 3 1 3 1 1 (3)</td>
    <td>14</td>    
  </tr>
  <tr>
    <td>A</td>
    <td>di da</td>
    <td>1 1 3 (3)</td>
    <td>8</td>    
  </tr>
  <tr>
    <td>R</td>
    <td>di da di</td>
    <td>1 1 3 1 1 (3)</td>
    <td>10</td>    
  </tr>
  <tr>
    <td>I</td>
    <td>di di</td>
    <td>1 1 1 (3)</td>
    <td>6</td>   
  </tr>
  <tr>
    <td>S</td>
    <td>di di di</td>
    <td>1 1 1 1 1 (3)</td>
    <td>8</td>
   
  </tr>
  <tr>
    <td> </td>
    <td> </td>
    <td> [4]</td>
    <td>4</td>
   
  </tr>
  <tr>
    <td><b>Celkem</b></td>
    <td>—</td>
    <td>—</td>
    <td><b>50</b></td>   
  </tr>
</table>
<p><i>( ) = mezera mezi znaky &nbsp;&nbsp; [ ] = mezera mezi slovy</i></p>
</div>

<br> 
<br> 
	Pro určení rychlosti vysílání se bere jako reference pětipísmenné slovo „PARIS".
	Celková doba jeho odvysílání včetně mezery za slovem je tedy 50 základních jednotek (délek teček). 
	Pokud tedy například hovoříme o tempu vysílání 12 slov za minutu (WPM, words per minute), 
	odpovídá to právě rychlosti průměrně jednoho písmene (znaku) za sekundu (12*5 = 60 znaků za minutu), 
	délka tečky je 60/(12*50) = 0,1 sekundy.
 <br>
 <br>
	Tyto poměry umožňují sluchem zcela spolehlivě rozlišit tečku od čárky i druh akustické pauzy.
	Za další, poměr celkové doby trvání vysílání tečky nebo čárky včetně pauzy uvnitř značky je 2:1,
	což usnadňuje udržet konstantní rychlost vysílání. 
 <br>
 
 # Ovládání

 - červené tlačítko **`X`**  <br>
  Krátký stisk smaže přijaté znaky na displeji. <br>
  Dlouhý stisk provede reset.<br>  
 
 - zelené tlačítko **`PLAY/STOP`** přehrávání Morse<br>
   Spustí nebo zastaví přehrávání  posledních 60-ti přijatých znaků na displeji<br>
   
 - encoder **`WPM`**  nastavení rychlosti a pádla <br>
   Otáčením se nastaví WPM rychlost pro vysílání i příjem znaků.<br>
   Krátký stisk uloží nastavení WPM a MODE do EEPROM <br>
   Dlouhý stisk přehodí MODE **`.-`** nebo **`-.`** pro paddle<br>

 # Programování 
  Je použita deska **Arduino Nano** a umí dekédévat tyto znaky:
  
  **`ABCDEFGHIJKLMNOPQRSTUVWXYZ 1234567890 ?/=,.`**
  
  Znaky je možno přidat zde:
  
  <pre> 
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
   </pre>
  
  Při vysílání se znaky dekodují dle pravidel PARIS, takže je dobré doržet mezeru mezi znaky.
  Pokud je na displeji zobrazen zanak "hvězdičky" znamená to, že znak není v tabulce nebo je špatně zahraný.
  Bohužel dekodování není 100% takže znak může být správny ale mezera mezi znaky je příliš krátká aby se 
  dekédoval správně. Znaky se nevypisují ihned ale po pauze cca 1s bez stisku paddle.
  <br>
  V této části definice pro dekodování je nastavena na parametr **`ditLength * 0.8`** pro lepší dekodování
  ale správně by měla být **`ditLength * 1`**
  
  <pre> 
    // Detekce konce znaku  →  mezera mezi znaky je minimálně ditLength * 1
    if (!tonePlaying && !elementPause && currentMorse.length() > 0 && (now - lastKeyTime > ditLength * 0.8 ) ) {   
      char decoded = decodeMorse(currentMorse);
      receivedText += decoded;
      if (receivedText.length() > 60) {
        receivedText = receivedText.substring(receivedText.length() - 60);
      }      
      currentMorse = "";
      endDecode = true;
      endDecodeTime = now;  // Uložit čas kdy byl nastaven endDecode
	  
  </pre>
 
# složka CAD
- obsahuje soubor krabičky pro 3D tisk 
- k otevření je potřeba program FreeCAD : https://www.freecad.org

 
