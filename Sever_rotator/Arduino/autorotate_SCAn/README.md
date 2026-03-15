Nová funkce pro rotátor LED 60

Pro jiný počet LED Neopixel změňit:  
  ```
  // Neopixel ring LED
  int NumPixels = 60;  // počet LED
  ```
<!-- 
# Funkce AutoTest:
 
   Stisknou a držet při "zapnutí" dokud se neoběví na displeji "tESt"
   
    Encoder        - mode 1 (opakování sekvence úhlů)
    Tlačítko M1/C  - mode 2 (náhodný 30 min)
    Tlačítko M2/S  - mode 3 (náhodný 1 hod)
    Tlačítko M3/F  - mode 4 (náhodný 2 hod)

  Po dokončení testu je nastaven normální režim.
  AutoTest je možné ukončit tlačítky **`CW / CCW → "End" → normální provoz`**

-->  
  
 # Funkce SCAn:
 
   Dlouhý stisk encoderu 1,5s a více
   
   Nastavení:
   
      "An 1" → enkodér → krátký stisk   - nastavení prvního úhlu
      "An 2" → enkodér → krátký stisk   - nastavení druhého úhlu
      "StEP" → enkodér → krátký stisk   - nastavení kroku ve stupních (def. 12 stupňů)
      "Int " → enkodér → krátký stisk   - nastavení intervalu v minutách (pauzy def. 2 min)

   Krátký stisk encoderu   **` start od aktuální pozice	`**  
	  	 
   Displej:	 **`střídá "SCAn" ↔ aktuální úhel`**
	  
   LED Mapa: **` modrá LED uržuje rozsah úhlů + animace radaru `**

   Zrušení funkce SCAn:	**`CW / CCW / encoder → "End" → normální provoz`**

   Přidán vstup "PTT" na pin A4 pro přerušení SCAn modu (aktivuje se LOW)
   
   ![Scheme_frame](https://github.com/DrumClock/OK1MFG/blob/main/Sever_rotator/Arduino/autorotate_SCAn/Arduino_Sever_PTT.png)

   # Video funkce    
   [![SCAn](https://img.youtube.com/vi/xlwmkpu4vwc/0.jpg)](https://www.youtube.com/watch?v=xlwmkpu4vwc)
