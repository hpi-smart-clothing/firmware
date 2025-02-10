from vpython import *
from time import sleep
import numpy as np
import math
import serial
import os
import csv
from datetime import datetime
import serial.tools.list_ports

def find_device_port():
    ports = serial.tools.list_ports.comports()
    print("Available COM ports:")
    for port in ports:
        print(f"Port: {port.device}, Description: {port.description}")
    for port in ports:
        if 'Arduino' in port.description or 'ESP32' in port.description or 'Serielles USB-Gerät' in port.description:
            return port.device
    return None

serial_com = find_device_port()

if serial_com is None:
    raise Exception("No Arduino or ESP32 device found")

print(f"Using COM port: {serial_com}")

# Serial-Port öffnen
ad = serial.Serial(serial_com, 115200, timeout=1)
sleep(1)

if ad.isOpen():
    print(f"Port {serial_com} ist geöffnet.")
else:
    print(f"Port {serial_com} konnte nicht geöffnet werden.")
    exit()


# Szene initialisieren
scene.range = 5
scene.background = color.black
toRad = 2 * np.pi / 360
scene.forward = vector(-1, -1, -1)
scene.width = 1200
scene.height = 1080
xarrow = arrow(length=2, shaftwidth=.1, color=color.red, axis=vector(1, 0, 0))
yarrow = arrow(length=2, shaftwidth=.1, color=color.green, axis=vector(0, 1, 0))
zarrow = arrow(length=4, shaftwidth=.1, color=color.blue, axis=vector(0, 0, 1))

#Positionen der BNOs festlegen
positions = [
    vector(3, 4, 0),
    vector(-3, 4, 0),
    vector(0, 6, 0),
    vector(2, -2, 0),
    vector(-2, -2, 0)
]

BNOs = []
arrows = []

#BNOs und Pfeile erstellen
for i in range(len(positions)):
    frontArrow = arrow(length=1, shaftwidth=.1, color=color.purple, axis=vector(1, 0, 0), pos=positions[i])
    upArrow = arrow(length=1, shaftwidth=.1, color=color.magenta, axis=vector(0, 1, 0), pos=positions[i])
    sideArrow = arrow(length=1, shaftwidth=.1, color=color.orange, axis=vector(0, 0, 1), pos=positions[i])
    arrows.append([frontArrow, upArrow, sideArrow])
    bno = box(length=2, width=1, height=.1, opacity=.8, pos=positions[i])
    BNOs.append(bno)

#Ausrichtung der BNOs lesen und anpassen
import time
csv_folder = os.path.join(os.getcwd(), "data")
if not os.path.exists(csv_folder):
    os.makedirs(csv_folder)
csv_path = os.path.join(csv_folder, f"sensor_data{time.time()}.csv")
file_exists = os.path.isfile(csv_path)
with open(csv_path, 'a', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow([
                    "timestamp",
                    "q0_1", "q1_1", "q2_1", "q3_1",
                    "q0_2", "q1_2", "q2_2", "q3_2",
                    "q0_3", "q1_3", "q2_3", "q3_3",
                    "q0_4", "q1_4", "q2_4", "q3_4",
                    "q0_5", "q1_5", "q2_5", "q3_5"
                    ])
    while True:
        try:
            while ad.inWaiting() == 0:
                pass
            dataPacket = ad.readline()
            dataPacket = str(dataPacket, 'utf-8').strip()
            splitPacket = dataPacket.split(",")

            # Annahme: Datenpaket enthält 20 Werte (fünf 4er-Pakete)
            if len(splitPacket) == 20:
                quaternions = []
                for i in range(5):
                    quaternions.append([float(splitPacket[j]) for j in range(i*4, (i+1)*4)])
            
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                row = [now] + [value for quat in quaternions for value in quat]
                writer.writerow(row)

            for i, q in enumerate(quaternions):
                    
                    #Ausrichtung der BNOs berechnen
                    roll = -math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
                    sin_pitch = 2 * (q[0] * q[2] - q[3] * q[1])
                    # Sicherstellen, dass der Wert im Bereich [-1, 1] liegt
                    sin_pitch = max(-1, min(1, sin_pitch))
                    pitch = math.asin(sin_pitch)
                    yaw = -math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) - np.pi / 2

                    rate(50)
                    k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
                    y = vector(0, 1, 0)
                    s = cross(k, y)
                    v = cross(s, k)
                    vrot = v * cos(roll) + cross(k, v) * sin(roll)
                    
                    # Ausrichtung der BNOs anpassen
                    BNOs[i].axis = k
                    BNOs[i].up = vrot
                    arrows[i][0].axis = k
                    arrows[i][1].axis = vrot
                    arrows[i][2].axis = cross(k,vrot)
                    arrows[i][0].length = 1
                    arrows[i][1].length = 1
                    arrows[i][2].length = 1

        except Exception as e:
            print(e)
            break
