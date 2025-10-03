from machine import Pin
import time
import cv2
from ulab import numpy as np

def cambiar_voltaje (p):
    print ("definir")

continuo = Pin (1, Pin.IN) # Define que siga activo el código
detectado = Pin (2, Pin.IN) # Input que define si detectó una falla o no
senal_encoder = Pin (3, Pin.IN) # Se reciben la cantidad de vueltas entregadas por el encoder
cant_senal_vuelta = 1 # Se define la cantidad de señales del encoder por vuelta
velocidad = Pin (4, Pin.OUT) # Se define la  velocidad
tiempo = time.time ()

while continuo:
    if detectado.value () == 1 and (time.time () - tiempo) < 0.5:
        if senal_encoder:
            print ("hoola")
        velocidad = 0
    else:
        velocidad = 132

    tiempo = time.time ()