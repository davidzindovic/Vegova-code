# Nalaganje programov
1. Odprite ```Arduino IDE``` (lahko tudi prenesete program in ga neposredno odprete)
2. Kopirajte kodo s CTRL+C (npr. osnove.ino) v Arduino IDE okno, tako da CTRL+A (izberete vse) in CTRL+V (prilepite kodo)
3. Ploščo povežite z USB priključkom, ki je dostopen na mizi.
4. Na vrhu Arduino IDE okna najdite zavihtek ```Orodja``` in izberite med možnostmi ```Plošča:```, nato pa najdite ```ESP32-WROOM-DA Module``` in ga z levim klikom izberite
5. Ponovno odprite zavihek ```Orodja``` in izberite možnost ```Vrata:```, nato izberite ```COM6```. V kolikor "COM6" ni na voljo, izberite tistega, ki NI COM1!
6. Na vrhu oknu najdite krogec s puščico, ki kaže desno. Če postavite miško na ta krogec, bi se moral pojaviti napis "Naloži". Z levim klikom se začne prevajanje kode. V kolikor je prevajanje uspešno, se nato izvede še nalaganje. Če pa opazite na dnu okna veliko teksta (rdeče s črnim ozadjem) ima vaš program napake, ki jih morate odpraviti in nato ponoviti korak št. 6.

# Spremljanje izpisa
```V zgornjem desnem vogalu Arduino IDE``` najdite povečevalno steklo s pluskom. Z levim klikom odprete ```Serijski monitor```.
Alternativna opcija je uporaba programa Putty, kjer izberete možnost "Serial" in vpišete pravilno številko COM vrat  (npr. COM6),

# Reševanje težav
Za namen te delavnice se izognite uporabit umetne inteligence in poskusite z uporabo brksalnika Google najti primer, kjer je nekdo že imel podobno težavo. Znan forum je StackOverflow.

# Preizkusite se v eni izmed nalog
Naloge se nahajajo v datoteki ```izzivi.md```.

# Dodaten izziv
Na računalnik namestite Python in zaženite "procesiranje_na_racunalniku.py", ki se nahaja v tej mapi.
Za dodatek lahko skripto pretvorite v .exe datoteko, da jo lahko kdorkoli zažene le z dvojnim klikom.
