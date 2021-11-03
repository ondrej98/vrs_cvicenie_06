# Náplň cvičenia
- zoznámenie sa s perifériou USART
- komunikácia MCU <=> PC
- funkcia "callback" a jej využitie

# Konfigurácia USART

<p align="center">
    <img src="https://raw.githubusercontent.com/VRS-Predmet/vrs_cvicenie_6/master/images/usart_config.PNG" width="850">
</p>

- na cvičení sa bude využivať periféria USART2: PA2 - Tx, PA15 - Rx

- USART2 treba zapnúť/povoliť v asynchrónnom režime

- v nastavení parametrov nie je potrebne meniť žiaden parameter okrem "Baud rate" (rovnaký musí byť nastavený aj na strane prijímača)

- prijímanie dát bude obslúžené v prerušení - v NVIC je potrebné povoliť prerušenie pre USART2

- vo vygenerovanom kóde je nutné doplniť povolenie prerušenia ak je prijatý znak - USART prerušenie "RXNE"

# Komunikácia s PC

<p align="center">
    <img src="https://raw.githubusercontent.com/VRS-Predmet/vrs_cvicenie_6/master/images/terminal.PNG" width="850">
</p>

- Nucleo doska ma fyzicky prepojený USART (konkrétne USART2) s ST-LINK/V2-1 a tak vytvára virtuálny COM port. Vďaka tomu dokážeme cez USART posielať dáta priamo do PC bez potreby USB/USART prevodníka (viac info v datasheet pre "STM32 Nucleo-32 boards")

- na strane PC sa môže využiť serial port terminál (PuTTY, Terminal by Br@y ...) s ktorým vieme sledovať komunikáciu na sériovom porte a zároveň posielať dáta

- na strane PC aj MCU je nutné mať nastavený rovnaký "Baud rate"! Taktiež nastavenia ako stop bit, parita ... musia byť rovnako nastavené ak sa náhodou pri konfigurácii MCU menili

# Callback funkcia

- "callback" je časť kódu (funkcia A), ktorá je predaná inej časti kódu (funkcii B) pričom sa očakáva, že funkcia B niekedy naspäť zavolá funkciu A, ktorá jej bola predaná - od toho je aj názov "callback"  

- v jayzku C je "callback" realizovaný ako smerník na funkciu

```python
void A() 
{ 
    printf("I am function A\n"); 
} 
  
void B(void (*ptr)()) 
{ 
    (*ptr) (); // callback to A 
} 
  
int main() 
{ 
   void (*ptr)() = A; 
   B(ptr); 
   
   return 0; 
} 
```

- pri programovaní vnorených systémov sa callback funkcie využívajú na oddelenie aplikačnej časti programu (vyššia úroveň) od hardvérovej časti programu, kde sa konfiguruje MCU (nízka úroveň)

- využitie to má v prípade, ak sa chce FW nejakého typu MCU portovať na iný typ MCU - FW je jednoduchšie prenášaťelný na iné zariadenia, je prehľadnejší a jednoduchší na údržbu

- príklad na callback funkciu je v ukážkovom projekte pre toto cvičenie (obsluhuje prerušenie od USART)

# Zadanie
- naprogramujte MCU tak, aby ste prikazmi prijatými cez USART vedeli ovladat LED
- ak MCU prijme cez USART príkaz 'ledON' - LED sa vypne
- ak MCU prijme cez USART príkaz 'ledOFF' - LED sa zapne
- s periodou 1s posielajte cez USART aktualny stav LED spet do PC
