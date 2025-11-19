from machine import UART, Pin
from machine import PWM
import time
import os
from ulab import numpy as np

# Configuración UART para Sabertooth
uart = UART(39, baudrate=115200, tx=Pin(5))  # Cambiar el pin para que tenga sentido con el microcontrolador

# Función para definir y enviar el comando de velocidad
def set_motor (motor, velocidad): # Falta ver cómo funciona el driver, qué recibe
    # motor: 1 o 2
    # velocidad: valor entre -127 (reversa) y +127 (adelante)
    # Evitar valores fuera de rango
    velocidad = max(-127, min(127, int(velocidad)))
    # Se asumirá la velocidad va entre 1 y 255, pero se debe corroborar el estándar para ver si es eso o 64 - 192
    if motor == 1:
        comando = 128 + velocidad # Definir el inicio de los rangos 1 a 127 de la velocidad
    elif motor == 2:
        comando = 128 + velocidad # Definir inicio de los rangos 
    else:
        return

    uart.write(bytes([comando & 0xFF]))

# Función auxiliar
def vel_a_com (v_cm_s):
    max_v = 5000 # Definir la velocidad máxima del motor
    pon = 127 # Definir valor de ponderación de la entrada del driver
    return int((v_cm_s / max_v) * pon)

# Definir pins
detectado = Pin (2, Pin.IN) # Input que define si detectó una falla o no
senal_encoder = Pin (1, Pin.IN) # Se reciben la cantidad de vueltas entregadas por el encoder

tiempo = time.ticks_ms()
# Se definen las constantes
i = 0 # Se define la variable sumativa de los encorders
c_senal_vuelta = 1 # Cantidad de señales del encoder por vuelta
posicion = 0
vision = 8 # Rango de vision de la camara [cm]

# Se definen la estructura de las ruedas 
r_rueda = 4 # Radio rueda [cm]
v = 10000 / (40 * 60) # Velocidad esperada [cm/s]
largo = (v) * 60 # Largo recorrido en 1 minuto si no hay fallas, según v [cm]
perimetro = 2 * np.pi * r_rueda # Perimetro rueda

# Se definen las variables de tiempo
t = 60 / (largo / vision) # Tiempo que se necesita para recorrer 8 [cm] si se recorren 250 [cm] en 60 [s]
p = 0.75 # Porcentaje del tiempo dedicado a avanzar (el resto es para que la cámara vuelva a su lugar)
t_avance = t * p # Tiempo que utilizará en avanzar los 8 [cm] mientras captura imágenes
t_acomodo = t * (1 - p) # Tiempo que utilizará para reacomodar la cámara
velocidad = vision / t_avance # Velocidad estándar del robot [cm/s]
v_acomodo = velocidad * 0.1 # Velocidad de las ruedas mientras la cámara se acomoda [cm/s]

# Se definen las rpm
rpm_v = (velocidad / perimetro) * 60
rpm_v_a = (v_acomodo / perimetro) * 60

try:
    while (i / c_senal_vuelta) * perimetro < largo:
        # Se define la velocidad de las ruedas
        # pos = (i / c_senal_vuelta) * perimetro
        if senal_encoder.value () == 1: # Se suma la cantidad de señales del encoder para poder definir en dónde estamos
            print ("hoola")
            i += 1
        if detectado.value () == 1: # Si se detectó una falla, se detiene por 2 [s]
            vel = vel_a_com (-rpm_v_a)
            set_motor (1, vel)
            set_motor (2, vel)
            retraso = 1 # Tiempo en que se demora en procesar la imágen el robot
            time.sleep (retraso)
            set_motor (1, 0)
            set_motor (2, 0)
            time.sleep (2)
            print ("Enviar señal de reinicio de posición de cámara y capturar imagenes")
            posicion = (i / c_senal_vuelta) * perimetro # Se define la posición con respecto al cable en la que se encontró la falla
            print ("Entregarle a la raspy la posición de la falla")
            print ("Falla detectada en posición:", posicion, "cm")
            vel = vel_a_com (rpm_v)
            set_motor (1, vel)
            set_motor (1, -vel)
            time.sleep (t_avance) # Se avanza para no leer nuevamente el mismo sector
            tiempo = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms() - tiempo) > t: # Cambiar a control por avance, pero hay que revisar la presición que puede encontrar el encoder
            tiempo = time.ticks_ms() # Se reinicia el tiempo
        elif time.ticks_diff(time.ticks_ms() - tiempo) > t_avance:
        # elif pos > vision:
            print ("Se está definiendo la velocidad en [cm/s], falta pasarla a PWM")
            vel = vel_a_com (rpm_v_a) # Se define la velocidad de acomodo
            set_motor (1, vel)
            set_motor (2, -vel)
        else:
            print ("Se está definiendo la velocidad en [cm/s], falta pasarla a PWM")
            vel = vel_a_com (rpm_v) # Se define la velocidad de avance
            set_motor (1, vel)
            set_motor (2, -vel)

except KeyboardInterrupt:
    print("Deteniendo motores...")
    set_motor(1, 0)
    set_motor(2, 0)