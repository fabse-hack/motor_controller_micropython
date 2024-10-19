import threading
import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# Funktion zum Lesen der seriellen Daten
def read_serial(ser, data_queue):
    while True:
        line = ser.readline()
        if line:
            decoded_line = line.decode('utf-8').strip()
            print(decoded_line)
            data_queue.append(decoded_line)

# Funktion zum Aktualisieren des Diagramms
def update_plot(data_queue, rpm_data, duty_data, ax, line1, line2):
    while True:
        if data_queue:
            data = data_queue.pop(0)
            if data:
                try:
                    # Extrahiere RPM und Duty-Cycle aus den Daten
                    rpm, duty = map(float, data.split(','))
                    
                    # Füge die Daten in die Deques ein
                    rpm_data.append(rpm)
                    duty_data.append(duty)
                    
                    # Aktualisiere die Daten für die Linien
                    line1.set_ydata(rpm_data)
                    line2.set_ydata(duty_data)

                    # Aktualisiere die X-Achse und Y-Achsen
                    ax.set_xlim(0, len(rpm_data))
                    ax.set_ylim(min(min(rpm_data), min(duty_data)), max(max(rpm_data), max(duty_data)))

                    plt.draw()
                    plt.pause(0.01)
                except ValueError:
                    pass
            time.sleep(0.1)


def wait_for_serial_port(port):
    while True:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"Verbindung zu {port} hergestellt.")
            return ser
        except (serial.SerialException, OSError) as e:
            print(f"Fehler beim Verbinden mit {port}: {e}. Warte auf das Gerät...")
            time.sleep(2)

ser = wait_for_serial_port('COM6')

data_queue = []

# Thread zum Lesen der seriellen Daten
serial_thread = threading.Thread(target=read_serial, args=(ser, data_queue))
serial_thread.daemon = True
serial_thread.start()

# Matplotlib-Einstellungen
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))

# Initialisiere die Deques für die RPM- und Duty-Daten
rpm_data = deque([0]*100, maxlen=100)
duty_data = deque([0]*100, maxlen=100)

# Erstelle die Linien für die Diagramme
line1, = ax.plot(rpm_data, label='RPM', color='blue')
line2, = ax.plot(duty_data, label='Duty Cycle', color='orange')

# Achsenbeschriftungen und Titel
ax.set_title('RPM und Duty Cycle')
ax.set_ylabel('Werte')
ax.set_xlabel('Zeit')
ax.legend()

# Thread zum Aktualisieren des Diagramms
plot_thread = threading.Thread(target=update_plot, args=(data_queue, rpm_data, duty_data, ax, line1, line2))
plot_thread.daemon = True
plot_thread.start()

# Hauptschleife
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Programm beendet.")

