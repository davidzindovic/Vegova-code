/*
 * PREDLOGA ZA ARDUINO PROJEKT
 * Namen: Učenje spremenljivk, podatkovnih vrst in funkcij
 */

// --- 1. ODSEK: SPREMENLJIVKE IN PODATKOVNE VRSTE ---
// Tukaj dijaki definirajo svoje podatke
int celoStevilo = 25;          // int: za cela števila (npr. število meritev)
float decimalnoStevilo = 22.5; // float: za decimalna števila (npr. temperatura)
char znak = 'A';               // char: za posamezne znake
String besedilo = "Meritev";   // String: za nize znakov
bool stanje = true;            // bool: logična vrednost (true/false)

// --- 2. ODSEK: NASTAVITVE (Setup) ---
void setup() {
  // Inicializacija serijske komunikacije (hitrost 9600 baudov)
  Serial.begin(9600);
}

// --- 3. ODSEK: GLAVNA ZANKA (Loop) ---
void loop() {
  // Primer uporabe funkcije, ki izračuna kvadrat števila
  int rezultat = izracunajKvadrat(5);
  
  // Klic funkcije za pošiljanje podatkov v Python (v CSV formatu)
  posljiPodatke(decimalnoStevilo, celoStevilo);

  // Premor med meritvami (1 sekunda)
  delay(1000);
  
  // Simulacija spreminjanja podatkov za testiranje
  decimalnoStevilo += 0.5; 
}

// --- 4. ODSEK: PRIMERI FUNKCIJ ---

/*
 * Opis: Funkcija sprejme celo število in vrne njegov kvadrat.
 * Zahteva: int x (vhodni parameter)
 * Vrne: int (rezultat izračuna)
 */
int izracunajKvadrat(int x) {
  return x * x;
}

/*
 * Opis: Funkcija pripravi podatke za Python v formatu: vrednost1,vrednost2
 * Zahteva: float v1, int v2
 * Vrne: nič (void) - samo izpiše na serijski port
 */
void posljiPodatke(float v1, int v2) {
  Serial.print(v1);
  Serial.print(","); // Ločilo za CSV format
  Serial.println(v2);
}

/*
 * IZZIV ZA DIJAKE:
 * Ustvari funkcijo "pretvoriVCelzij", ki sprejme Fahrenheit (float) 
 * in vrne Celzij (float).
 */
