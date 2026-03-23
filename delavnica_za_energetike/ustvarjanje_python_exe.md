# Navodila: Ustvarjanje samostojne .exe aplikacije iz Python skripte
Ta priročnik vas bo vodil skozi postopek pretvorbe vaše Python skripte (ki beleži podatke in izrisuje graf) v izvršljivo datoteko (.exe). To omogoča zagon programa na katerem koli Windows računalniku brez nameščenega Pythona.

## 1. Priprava okolja
Preden začnete, se prepričajte, da imate nameščeno orodje PyInstaller. Odprite ukazni poziv (CMD) ali terminal v VS Code in vnesite:


```pip install pyinstaller```

Opomba: Če uporabljate knjižnice, kot sta matplotlib in pyserial, morajo biti te že nameščene v vašem okolju.

## 2. Postopek pretvorbe
Odprite mapo, kjer se nahaja vaša Python datoteka (npr. procesiranje_na_racunalniku.py).

V naslovno vrstico mape kliknite in vpišite cmd ter pritisnite Enter, da se terminal odpre neposredno v tej mapi.

Vnesite naslednji ukaz:

```pyinstaller --noconfirm --onefile --windowed  procesiranje_na_racunalniku.py```

Razlaga uporabljenih stikal (zastavic):

--onefile: Vse knjižnice, ikone in kodo zapakira v eno samo datoteko .exe.

--windowed: Ob zagonu se ne odpre črno konzolno okno (primerno za programe z grafičnim vmesnikom/grafi).

--noconfirm: Samodejno prepiše stare datoteke, če postopek ponavljate.

## 3. Kje najdem svojo aplikacijo?
Po končanem postopku (ki lahko traja minuto ali dve) se bodo v vaši mapi pojavile nove mape. Vaš program se nahaja v mapi:

📂 dist/ ->  📄 graf_monitor.exe

Mape build/ in datoteko .spec, ki so nastale med postopkom, lahko po uspešni pretvorbi izbrišete.

⚠️ Pomembna opozorila za dijake
Vrata (COM Port): Preden ustvarite .exe, preverite, ali je v kodi nastavljen pravilen port (npr. COM3). Če se na drugem računalniku Arduino poveže na drug port, program ne bo deloval, razen če v kodi implementirate samodejno zaznavanje.

Zaprite Serial Monitor: Program .exe ne bo mogel dostopati do Arduina, če imate hkrati odprt Serial Monitor v Arduino IDE.

Antivirusni programi: Nekateri antivirusi lahko blokirajo .exe datoteke, ki jih ustvari PyInstaller, ker niso digitalno podpisane. Če se to zgodi, datoteko dodajte med izjeme.
