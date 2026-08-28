# PTT Sequencer

PTT sequencer pro řízení postupného zapínání a vypínání jednotlivých částí zařízení.

![PTT Sequencer – přední strana](predni.jpg)

## Princip činnosti

Po aktivaci PTT sequencer postupně aktivuje jednotlivé výstupy s nastaveným časovým odstupem.

```text
PTT ON

PTT 1 → PTT 2 → PTT 3 → PTT 4 → PTT 5
          │         │         │
          └──── nastavený časový krok ────┘
```

Po ukončení PTT proběhne vypnutí v opačném pořadí:

```text
PTT OFF

PTT 5 → PTT 4 → PTT 3 → PTT 2 → PTT 1
```

Tím je zajištěno, že jednotlivé části zařízení jsou při přechodu mezi RX a TX zapínány a vypínány v definovaném pořadí.

## Zapojení 1 kanálu PTT

![Zapojení](/Sequencer/KiCAD/schema.png)
![PCB](/Sequencer/KiCAD/sequencer.jpg)


## Nastavení časového kroku

Časový odstup mezi jednotlivými kroky se nastavuje pomocí DIP přepínačů na zadním panelu.

Nastavením kombinace přepínačů lze zvolit požadovanou rychlost sekvence podle použitého zařízení.

![Nastavení časového kroku](spodek.jpg)

## Zadní panel

Na zadním panelu jsou umístěny vstupy a výstupy pro připojení sequenceru.

![Zadní panel – vstupy a výstupy](zadni.jpg)

### PTT IN

Vstup pro ovládací PTT signál.

Po aktivaci vstupu se spustí zapínací sekvence. Po jeho uvolnění proběhne vypínací sekvence v opačném pořadí.

### PTT OUT

Výstupy sequenceru pro jednotlivé řízené stupně.

Jednotlivé výstupy jsou aktivovány postupně podle nastaveného časového kroku.

## Ovládání LNA

Na předním panelu je samostatný vypínač **LNA**.

Vypínač **LNA odpojuje napájení pouze pro předzesilovače (LNA)**. Nemá vliv na napájení ostatních částí zařízení.

![Přední panel – ovládání LNA](predni.jpg)

## Indikace

LED indikace na předním panelu informuje o aktivním stavu sekvence.

Během přechodu do TX a návratu do RX probíhá sekvence automaticky podle nastaveného časového kroku.

## Konstrukce

Sequencer je umístěn v kompaktní skříňce s ovládacími prvky na předním panelu a konektory pro připojení na zadním panelu.

![Pohled shora](vrsek.jpg)

## Přehled projektu

Součástí projektu jsou také konstrukční podklady PCB a čelního panelu.

![Izometrický pohled](iso1.jpg)

![Izometrický pohled](iso2.jpg)

---

**PTT Sequencer – OK1MFG**
