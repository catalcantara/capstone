from machine import Pin
from machine import PWM
import time
import os
from ulab import numpy as np

# --- Función auxiliar ---
def vel_a_pwm(v_cm_s):
    max_v = 20
    return int((v_cm_s / max_v) * 65535)

# Definir pins
detectado = Pin (1, Pin.IN) # Input que define si detectó una falla o no
senal_encoder = Pin (2, Pin.IN) # Se reciben la cantidad de vueltas entregadas por el encoder
vel_pwm = PWM (Pin (3, Pin.OUT)) # Se define la  velocidad
vel_pwm.freq (100)
c_senal_vuelta = 1 # Cantidad de señales del encoder por vuelta
tiempo = time.ticks_ms()
i = 0 # Se define la variable sumativa de los encorders
d_rueda = 3 # Diametro rueda [cm]
vision = 8 # Rango de vision de la camara [cm]
largo = 250 # Largo recorrido en 1 minuto si no hay fallas [cm]
perimetro = 2 * np.pi * d_rueda
c_vlta = vision / perimetro # Se define la cantidad de vueltas de una rueda que se necesitan para recorrer 8 cm
t = 60 / (largo / vision) # Tiempo en que debe avanzar el rango de visión para alcanzar a recorrer 2.5 [m] en 60 [s]
p = 0.75 # Porcentaje del tiempo dedicado a avanzar (el resto es para que la cámara vuelva a su lugar)
t_avance = t * p # Tiempo que utilizará en avanzar los 8 [cm] mientras captura imágenes
t_acomodo = t * (1 - p) # Tiempo que utilizará para reacomodar la cámara
velocidad = vision / t_avance # Velocidad máxima del robot
v_acomodo = velocidad * 0.1 # Velocidad de las ruedas mientras la cámara se acomoda
posicion = 0

while (i / c_senal_vuelta) * perimetro < 250:
    # Se define la velocidad de las ruedas
    if senal_encoder.value () == 1: # Se suma la cantidad de señales del encoder para poder definir en dónde estamos
        print ("hoola")
        i += 1
    if detectado.value () == 1: # Si se detectó una falla, se detiene por 2 [s]
        vel_pwm.duty_u16 (0)
        time.sleep (2)
        print ("Enviar señal de reinicio de posición de cámara y capturar imagenes")
        posicion = (i / c_senal_vuelta) * perimetro # Se define la posición con respecto al cable en la que se encontró la falla
        print ("Entregarle a la raspy la posición de la falla")
        print ("Falla detectada en posición:", posicion, "cm")
        vel_pwm.duty_u16 (vel_a_pwm (velocidad))
        time.sleep (t_avance) # Se avanza para no leer nuevamente el mismo sector
        tiempo = time.ticks_ms()
    elif time.ticks_diff(time.ticks_ms() - tiempo) > t:
        tiempo = time.ticks_ms() # Se reinicia el tiempo
    elif time.ticks_diff(time.ticks_ms() - tiempo) > t_avance:
        print ("Se está definiendo la velocidad en [cm/s], falta pasarla a PWM")
        vel_pwm = PWM (vel_a_pwm (v_acomodo)) # Se define la velocidad de acomodo
    else:
        print ("Se está definiendo la velocidad en [cm/s], falta pasarla a PWM")
        vel_pwm = PWM (vel_a_pwm (velocidad)) # Se define la velocidad de avance