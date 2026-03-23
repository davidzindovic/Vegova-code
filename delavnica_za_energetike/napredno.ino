/*
 * PREDLOGA ZA ARDUINO - SEZNAMI IN FUNKCIJE
 */




// --- 1. ODSEK: POLJA (ARRAYS) ---
const int VELIKOST_SEZNAMA = 5;      // Določimo fiksno velikost polja
float meritve[VELIKOST_SEZNAMA];     // Seznam za shranjevanje zadnjih 5 temperatur
int trenutniIndeks = 0;              // Števec, kje v seznamu smo


void setup() {
  Serial.begin(9600);
  // Ponastavimo seznam na 0
  for (int i = 0; i < VELIKOST_SEZNAMA; i++) {
    meritve[i] = 0.0;
  }
}


void loop() {
  // Simulacija nove meritve (npr. senzor)
  float novaVrednost = random(200, 300) / 10.0; 

  // Klic funkcije za dodajanje v seznam
  dodajVSeznam(novaVrednost);

  // Izračun povprečja s funkcijo
  float povprecje = izracunajPovprecje();

  // Pošiljanje podatkov v Python (trenutna vrednost in povprečje)
  Serial.print(novaVrednost);
  Serial.print(",");
  Serial.println(povprecje);

  delay(500); // Hitrejše osveževanje za graf (0.5 sekunde)
}





// --- 2. ODSEK: FUNKCIJE ZA DELO S SEZNAMI ---

/*
 * Opis: Doda novo vrednost v polje na trenutno mesto.
 * Zahteva: float vrednost
 * Vrne: nič
 */
void dodajVSeznam(float v) {
  meritve[trenutniIndeks] = v;
  trenutniIndeks++;
  
  // Če pridemo do konca seznama, gremo nazaj na začetek (krožni medpomnilnik)
  if (trenutniIndeks >= VELIKOST_SEZNAMA) {
    trenutniIndeks = 0;
  }
}



/*
 * Opis: Sešteje vse elemente v polju in vrne povprečno vrednost.
 * Zahteva: nič (uporablja globalno polje meritve)
 * Vrne: float (povprečje)
 */
float izracunajPovprecje() {
  float vsota = 0;
  for (int i = 0; i < VELIKOST_SEZNAMA; i++) {
    vsota += meritve[i];
  }
  return vsota / VELIKOST_SEZNAMA;
}
