# Digitalni števec dogodkov (Spremenljivke in Inkrementacija)

Opis: Simuliraj industrijski števec, ki beleži število izdelkov na tekočem traku.

Spremenljivke: int stevecIzdelkov, bool senzorZaseden.

Izziv: Napiši funkcijo void zaznajIzdelek(). Vsakič, ko ročno spremeniš senzorZaseden iz false v true, naj se števec poveča za 1.

Izpis na Serial: "Trenutno stanje skladišča: [vrednost] kosov."

# Branje "elektronskega šuma" (Analogni vhod)

Opis: ESP32 ima zelo občutljive vhodne pine. Če pin ni povezan nikamor (plavajoči vhod), bere šum iz okolice.

Spremenljivke: int suroviSum, const int pin = 34; (uporabi ADC pin na ESP32).

Izziv: V loop() branju uporabi analogRead(pin). Ker je ESP32 12-biten, bo vrednost med 0 in 4095.

Izpis na Serial: Prikaži surov podatek v obliki: "Surovi šum (0-4095): [vrednost]".

# Pametni voltmeter (Matematično skaliranje)

Opis: Pretvori surove podatke šuma v dejansko napetost (V).

Spremenljivke: float napetost, int vhodnaVrednost.

Izziv: Ustvari funkcijo float pretvoriVNapetost(int surova). Upoštevaj, da pri ESP32 vrednost 4095 ustreza približno 3.3V. Formula: V= surovax3,3/4095​

Izpis na Serial: "Izmerjena napetost na pinu 34: [vrednost] V".

# 4. Krožni medpomnilnik zadnjih meritev (Seznami/Arrays)

Opis: Naprava mora shraniti zadnjih 5 vrednosti šuma, da lahko izloči napake.

Spremenljivke: int zgodovina[5], int indeks = 0.

Izziv: Ustvari funkcijo void dodajVArhiv(int nova). Ko prideš do konca seznama (indeks 4), naj naslednja meritev ponovno prepiše prvi element (indeks 0).

Izpis na Serial: Po vsakem novem zapisu izpiši celoten seznam v eni vrstici: "[V1, V2, V3, V4, V5]".

# 5. Iskanje ekstrema v šumu (Algoritmi na seznamih)

Opis: Naprava mora v seznamu zadnjih 5 meritev poiskati najvišji zaznan "špico" šuma.

Spremenljivke: Uporabi seznam iz prejšnje naloge.

Izziv: Napiši funkcijo int najdiMaksimum(), ki s for zanko pregleda seznam in vrne največjo vrednost.

Izpis na Serial: "Največja zaznana konica v zadnjih 5 meritvah: [vrednost]".

# 6. Detektor stabilnosti signala (Logika)

Opis: Ugotovi, ali je signal šuma stabilen ali močno niha.

Spremenljivke: int razlika, int pragNihanja = 500.

Izziv: Primerjaj trenutno meritev s prejšnjo. Če je absolutna razlika med njima večja od pragNihanja, sproži opozorilo.

Izpis na Serial: Če niha, izpiši: "OPOZORILO: Nestabilen signal!", sicer "Signal je stabilen.".

# 7. Indikator "Baterije" (Mapiranje vrednosti)

Opis: Simuliraj prikaz napolnjenosti baterije glede na napetost šuma.

Spremenljivke: int nivoBaterije (v odstotkih).

Izziv: Uporabi funkcijo map(), da vrednost šuma (0-4095) preslikaš v odstotke (0-100).

Izpis na Serial: Uporabi simbole za vizualizacijo: "Baterija: [##########] 100%" (število lojtric naj bo sorazmerno z odstotki).

# 8. Ukazni sistem preko Serial Monitorja (Dvosmerna komunikacija)

Opis: Arduino naj se odziva na ukaze, ki jih dijak vpiše v vrstico Serial Monitorja.

Spremenljivke: char ukaz.

Izziv: Uporabi Serial.read(). Če dijak vpiše 'P', naj se izračuna povprečje seznama, če vpiše 'I', naj se izbriše (ponastavi na 0) celoten seznam.

Izpis na Serial: "Prejet ukaz [P]: Povprečje je [vrednost]" ali "Sistem ponastavljen!".
