from vpython import *
from time import sleep
import numpy as np
import math
import serial

serial_com = 'COM7'

# Serial-Port öffnen
ad = serial.Serial(serial_com, 9600, timeout=1)
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
pos1 = vector(3, 4, 0)
pos2 = vector(-3, 4, 0)

# Pfeile und Objekte erstellen
xarrow = arrow(length=2, shaftwidth=.1, color=color.red, axis=vector(1, 0, 0))
yarrow = arrow(length=2, shaftwidth=.1, color=color.green, axis=vector(0, 1, 0))
zarrow = arrow(length=4, shaftwidth=.1, color=color.blue, axis=vector(0, 0, 1))

frontArrow1 = arrow(length=1, shaftwidth=.1, color=color.purple, axis=vector(1, 0, 0), pos = pos1)
upArrow1 = arrow(length=1, shaftwidth=.1, color=color.magenta, axis=vector(0, 1, 0), pos = pos1)
sideArrow1 = arrow(length=1, shaftwidth=.1, color=color.orange, axis=vector(0, 0, 1), pos = pos1)

frontArrow2 = arrow(length=1, shaftwidth=.1, color=color.purple, axis=vector(1, 0, 0), pos = pos2)
upArrow2 = arrow(length=1, shaftwidth=.1, color=color.magenta, axis=vector(0, 1, 0), pos = pos2)
sideArrow2 = arrow(length=1, shaftwidth=.1, color=color.orange, axis=vector(0, 0, 1), pos = pos2)

bno1 = box(length=2, width=1, height=.1, opacity=.8, pos = pos1)
bno2 = box(length=2, width=1, height=.1, opacity=.8, pos = pos2)

while True:
    try:
        while ad.inWaiting() == 0:
            pass
        dataPacket = ad.readline()
        dataPacket = str(dataPacket, 'utf-8').strip()
        splitPacket = dataPacket.split(",")
        q10 = float(splitPacket[0])
        q11 = float(splitPacket[1])
        q12 = float(splitPacket[2])
        q13 = float(splitPacket[3])
        q20 = float(splitPacket[4])
        q21 = float(splitPacket[5])
        q22 = float(splitPacket[6])
        q23 = float(splitPacket[7])

        
        roll1 = -math.atan2(2 * (q10 * q11 + q12 * q13), 1 - 2 * (q11 * q11 + q12 * q12))
        pitch1 = math.asin(2 * (q10 * q12 - q13 * q11))
        yaw1 = -math.atan2(2 * (q10 * q13 + q11 * q12), 1 - 2 * (q12 * q12 + q13 * q13)) - np.pi / 2

        rate(50)
        k1 = vector(cos(yaw1) * cos(pitch1), sin(pitch1), sin(yaw1) * cos(pitch1))
        y1 = vector(0, 1, 0)
        s1 = cross(k1, y1)
        v1 = cross(s1, k1)
        vrot1 = v1 * cos(roll1) + cross(k1, v1) * sin(roll1)
        
        roll2 = -math.atan2(2 * (q20 * q21 + q22 * q23), 1 - 2 * (q21 * q21 + q22 * q22))
        pitch2 = math.asin(2 * (q20 * q22 - q23 * q21))
        yaw2 = -math.atan2(2 * (q20 * q23 + q21 * q22), 1 - 2 * (q22 * q22 + q23 * q23)) - np.pi / 2

        rate(50)
        k2 = vector(cos(yaw2) * cos(pitch2), sin(pitch2), sin(yaw2) * cos(pitch2))
        y2 = vector(0, 1, 0)
        s2 = cross(k2, y2)
        v2 = cross(s2, k2)
        vrot2 = v2 * cos(roll2) + cross(k2, v2) * sin(roll2)

        frontArrow1.axis = k1
        sideArrow1.axis = cross(k1, vrot1)
        upArrow1.axis = vrot1
        bno1.axis = k1
        bno1.up = vrot1
        sideArrow1.length = 2
        frontArrow1.length = 4
        upArrow1.length = 1
        
        frontArrow2.axis = k2
        sideArrow2.axis = cross(k2, vrot2)
        upArrow2.axis = vrot2
        bno2.axis = k2
        bno2.up = vrot2
        sideArrow2.length = 2
        frontArrow2.length = 4
        upArrow2.length = 1
        
    except Exception as e:
        print(f"Fehler: {e}")
        pass