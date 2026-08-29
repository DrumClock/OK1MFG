# 5-WAY PTT Sequencer

Sequencer pro řízení postupného zapínání a vypínání jednotlivých částí zařízení.

![PTT Sequencer – přední strana](predni.jpg)


# [Arduino](https://github.com/DrumClock/OK1MFG/tree/main/Sequencer/Arduino/5CH_sequencer.ino) - program
Po aktivaci PTT sequencer postupně aktivuje jednotlivé výstupy s nastaveným časovým odstupem.

```text
PTT IN - ON

PTT 1 → PTT 2 → PTT 3 → PTT 4 → PTT  5
```

Po ukončení PTT proběhne vypnutí v opačném pořadí:

```text
PTT IN - OFF

PTT 5 → PTT 4 → PTT 3 → PTT 2 → PTT 1
```
Tím je zajištěno, že jednotlivé části zařízení jsou při přechodu mezi RX a TX zapínány a vypínány v definovaném pořadí.

## Nastavení DELAY SEQ TIME

Časový odstup mezi jednotlivými kroky se nastavuje pomocí DIP přepínačů na spodní straně krabičky.
Nastavením kombinace přepínačů lze zvolit požadovanou rychlost sekvence podle použitého zařízení.

![Nastavení časového kroku](spodek.jpg)


# [KiCAD](https://github.com/DrumClock/OK1MFG/tree/main/Sequencer/KiCAD) - schéma kanálu LNA / PTT

![Zapojení](/Sequencer/KiCAD/schema.png)

![PCB](/Sequencer/KiCAD/sequencer.jpg)


# Zadní panel

Na zadním panelu jsou umístěny vstupy a výstupy pro připojení zarizeni.

![Zadní panel – vstupy a výstupy](zadni.jpg)

### PTT IN
Vstup pro ovládací PTT signál (šlapka proti GND).
Po aktivaci vstupu se spustí zapínací sekvence. 
Po jeho uvolnění proběhne vypínací sekvence v opačném pořadí.

###  PTT-TX
- RX je pin konektoru RCA ve stavu "otevřeného kolektoru" 
- TX je uzemněn pin konektoru RCA

### LNA 
- RX je přivedeno napájení 12V na pin 1 (+LNA RX) konektoru GX
- TX je je uzemněn pin 2 (PTT TX) konektoru GX

### 12V DC
Vstup napájení 12V DC / LNA 1 je společný i pro a elektroniku sequenceru.
Další vstupy DC napájí jen tento kanál LNA. ( na obrátku vstup 12V DC napájí jak elektroniku tak LNA 1 a 2 )

# Přední panel

## Vypínač LNA 

Na předním panelu je samostatný vypínač, který
**odpojuje napájení pouze pro předzesilovače LNA**.
Nemá vliv na napájení ostatních částí zařízení.

![Přední panel – ovládání LNA](predni.jpg)

## Indikace

LED indikace na předním panelu informuje o aktivním stavu RX/TX sekvence.
Během přechodu do TX a návratu do RX probíhá sekvence automaticky podle nastaveného časového kroku.


# [FreeCAD](https://github.com/DrumClock/OK1MFG/tree/main/Sequencer/FreeCAD) - přední a zadní panel

Sequencer je umístěn v plastové krabičce KM60 159x60x140mm, prodejce [HADEX](https://www.hadex.cz/p/o208b-krabicka-plastova-km60-159x60x140mm?searchLogID=1625557)

Přední a zadní panel k této krabičce je možné parametrovat v programu FreeCAD.

![parametrování](/Sequencer/FreeCAD/VarSet.jpg)


# Provedení
# Popisky - přiložené soubor **`sequencer_label.cdr`** je pro grafický editor **`CorelDraw`** 

![Pohled shora](vrsek.jpg)

![Izometrický pohled](iso1.jpg)

![Izometrický pohled](iso2.jpg)

---
