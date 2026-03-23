import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import time

# --- NASTAVITVE ---
PORT = 'COM3'  # Spremeni glede na svoj računalnik
BAUD_RATE = 9600
IME_DATOTEKE = "podatki.csv"

# Priprava podatkovnih list za graf
x_podatki = []
y_trenutno = []
y_povprecje = []

# Inicializacija serijske povezave
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
except:
    print("Napaka: Preveri port!")
    exit()

# Priprava grafičnega okna
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='Trenutno', color='blue')
line2, = ax.plot([], [], label='Povprečje (5)', color='red', linestyle='--')
ax.legend()
ax.set_title("Arduino Meritve v Realnem Času")
ax.set_xlabel("Čas (s)")
ax.set_ylabel("Vrednost")

def posodobi_graf(frame):
    if ser.in_waiting > 0:
        vrstica = ser.readline().decode('utf-8').strip()
        deli = vrstica.split(',')
        
        if len(deli) == 2:
            try:
                v1 = float(deli[0])
                v2 = float(deli[1])
                
                # Dodajanje v sezname za graf
                x_podatki.append(len(x_podatki))
                y_trenutno.append(v1)
                y_povprecje.append(v2)
                
                # Omejimo prikaz na zadnjih 50 točk
                if len(x_podatki) > 50:
                    x_podatki.pop(0)
                    y_trenutno.pop(0)
                    y_povprecje.pop(0)

                # Posodabljanje linij
                line1.set_data(range(len(x_podatki)), y_trenutno)
                line2.set_data(range(len(x_podatki)), y_povprecje)
                
                ax.relim()
                ax.autoscale_view()
                
                # Zapis v CSV
                with open(IME_DATOTEKE, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([time.strftime("%H:%M:%S"), v1, v2])
                    
            except ValueError:
                pass
    return line1, line2

# Zagon animacije
ani = FuncAnimation(fig, posodobi_graf, interval=100, cache_frame_data=False)
plt.show()
ser.close()
