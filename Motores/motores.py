from machine import Pin, UART, PWM
import time
import _thread
import sys
import select
import math 

# ============================================================
# ===================== PARÁMETROS ===========================
# ============================================================
# Sabertooth
baudrate = 115200 #[baudios]
max_vel = 1800 # 2047 real
real_max_vel = 2047
rpm_max = 330 # Aprox según datasheet y razón de radios
max_vel_rpm = max_vel * rpm_max / real_max_vel # Velocidad máxima en rpm (290)

# Servo
servo_freq = 50 #[Hz]
srv_stop = int (1.5 * 1e6)
srv_max = int (2.4 * 1e6)
srv_min = int (0.6 * 1e6)
vel_rot = int (0.5 * 1e6)

# Tiempo
dt = 0.02 #[ms]
blink_interval_ms = 500

# Encoder
cant_e = 1920 / 4

# ----------- Pines ------------
uart_id = 0 # Id Sabertooth
tx_pin = 0 # Pin 1 del Sabertooth
rx_pin = 1 # Pin 2 del Sabertooth
servo_pin = 16 # Pin del Servomotor
led_pin = "LED" # Pin del LED
p_enc_a1 = 2
p_enc_b1 = 3
p_enc_a2 = 4
p_enc_b2 = 5
p_hall = 22
uart_im_id = 1
tx_im = 8
rx_im = 9

# --- Señales externas ---
# uart_im_det =  UART(uart_im_id, baudrate=baudrate, tx=Pin(tx_im), rx=Pin(rx_im))  # HIGH cuando hay imagen detectada

hall = Pin(p_hall, Pin.IN, Pin.PULL_UP)

enc_1_A = Pin (p_enc_a1, Pin.IN)
enc_1_B = Pin (p_enc_b1, Pin.IN)
enc_2_A = Pin (p_enc_a2, Pin.IN)
enc_2_B = Pin (p_enc_b2, Pin.IN)

uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
imagen_detectada = 0

servo_pwm = PWM(Pin(servo_pin))
servo_pwm.freq(servo_freq)

led = Pin(led_pin, Pin.OUT)

# Avance ruedas
m = 100 * 100 # 100 [m] en [mm]
t_m = 40 * 60 # 40 [min] en [s]
d_r = 80 # 80 [mm] diametro rueda de tracción
v_min = m / t_m # Velocidad necesaria para cumplir requisito (4.1 [mm/s])
a = math.radians (66) # ° de visión de la cámara
d = (7.5 - 2) * 10 # Distancia entre la cámara y la superficie del cable
d_c = d * math.tan (a / 2) * 2 # Distancia vista por la cámara (71.4 [mm])
rec_r = math.pi * d_r # Distancia recorrida por una revolución de la rueda (251 [mm])
t_max_d_c = d_c / v_min # Tiempo máximo que se debe demorar en realizar un recorrido completo de la cámara (ida y vuelta) (17.4 [s])
p_v_d_c = d_c / rec_r # Porcentaje de una vuelta que se debe dar para recorrer la distancia vista (28.45%)
rpm_min = 60 * p_v_d_c / t_max_d_c # Velocidad en RPM mínimo (en promedio) para alcanzar la meta
senal_vel = rpm_min * max_vel / max_vel_rpm # Señal proporcional a la velocidad (6.08)
cant_enc_d_c = int (cant_e * p_v_d_c / 2) # Cantidad de pulsos del encoder para lograr la meta (546.17)

# ----- Controlador -----
K = 0.12
K_D = 0.5
K_I = 0.1

# ---- VARIABLES COMPARTIDAS ENTRE CORES ----
hall_count = 0
hall_lock = _thread.allocate_lock()

dir_rot = int (1)   # 1 → horario, -1 → antihorario
enc_count = int (0)
enc_tot = int (0)
err_prev = int (0)
sum_err = float (0)

# ============================================================
# ======================= FUNCIONES ==========================
# ============================================================
# Enviar señal a sabertooth
def send_sabertooth_com(speed: int) -> None:
    if speed > int (100): speed = int (100)
    if speed < int (-100): speed = int (-100)

    sabertooth_speed = int((speed / 100) * max_vel)
    com_1 = f"M1:{sabertooth_speed}\r\n"
    com_2 = f"M2:{int (-sabertooth_speed)}\r\n"
    uart.write(com_1.encode('utf-8'))
    uart.write(com_2.encode('utf-8'))

# Enviar señal servo
def set_duty(duty_ns_f: float, servo: PWM) -> None:
    global srv_min, srv_max

    duty_ns = int (duty_ns_f)
    if duty_ns > srv_max: duty_ns = srv_max
    if duty_ns < srv_min: duty_ns = srv_min

    servo.duty_ns(int(duty_ns))


# ============================================================
# == TAREA EN CORE 1 → CONTROL DEL MOTOR ROTACIONAL Y HALL ===
# ============================================================
def tarea_rotacion():
    global hall_count, dir_rot, srv_stop, imagen_detectada

    # Activar velocidad inicial
    set_duty ((srv_stop + vel_rot) * dir_rot, servo_pwm)

    while True:
        # --- Detectar imagen ---
        # if uart_im_det.value () == 1:
        # if imagen_detectada == 1:
        #     set_duty (srv_stop, servo_pwm)
        #     time.sleep (0.1)
        #     set_duty (srv_min * (- dir_rot) + srv_stop, servo_pwm)
        #     time.sleep (0.1)
        #     set_duty (srv_stop, servo_pwm)
        #     time.sleep (1)

        # --- Detectar imán ---
        if hall.value() == 1:
            # Contador seguro (zona crítica)
            with hall_lock:
                hall_count += 1
                n = hall_count

            print("Imán detectado, total:", n)

            # Reducir velocidad 10% durante 0.5 s
            set_duty(srv_stop, servo_pwm)
            time.sleep(1)

            # Si es el 4° imán: detener e invertir dirección
            if n % 4 == 0:
                print("4° imán → invertir rotación")

                # Stop breve
                set_duty(srv_stop, servo_pwm)
                time.sleep(0.1)

                # Cambiar dirección
                dir_rot *= -1

            # Restaurar velocidad normal
            set_duty(vel_rot * dir_rot + srv_stop, servo_pwm)

            # Esperar a que el sensor vuelva a LOW (evitar doble conteo)
            while hall.value() == 1:
                time.sleep(0.01)

        # ============================================================
        # Se le podría agregar un else que haga un control de velocidad con el encoder
        # ============================================================

        time.sleep(0.005)


# ============================================================
# ===== TAREA EN CORE 0 → CONTROL DE MOTORES DE TRACCIÓN =====
# ============================================================
def tarea_traccion():
    global imagen_detectada, cant_enc_d_c, enc_count, senal_vel, sum_err, rec_r, max_vel, max_vel_rpm, enc_tot, dt
    print ("Hello world")
    # time.sleep (1.5)
    send_sabertooth_com (50)
    err_prev = float (0)
    error = (cant_enc_d_c - enc_count) * d_c / dt
    time.sleep (0.01)
    while True:
        print (f"valor encoder {enc_1_A.value ()}")
        if enc_1_A.value () == 1:
            enc_count += 1
            enc_tot += 1
            print (f"Avanzó lo juro {enc_count}")

        # if uart_im_id.value () == 1:
        # if imagen_detectada == 1:
        #     print("Imagen detectada → modificar comportamiento")
        #     send_sabertooth_com (0)
        #     time.sleep (0.1)

        #     # Se retrocede un poco
        #     send_sabertooth_com (int (-senal_vel))
        #     time.sleep (0.1)
        #     # Se avanza sin girar la cámara
        #     send_sabertooth_com (int (senal_vel))
        #     time.sleep (0.5)
        #     # Se retrocede sin girar la cámara
        #     send_sabertooth_com (0)
        #     time.sleep (0.1)
        #     send_sabertooth_com (int (- senal_vel))
        #     time.sleep (0.5)

        #     # Se continúa el avance
        #     send_sabertooth_com (int(senal_vel))

        print (f"Conteo encoder {enc_count} y la cantidad de meta es: {cant_enc_d_c}")
        while enc_count < cant_enc_d_c:
            if enc_1_A.value () == 1:
                enc_count += 1
                enc_tot += 1
                print (f"Avanzó lo juro {enc_count}")
            err_prev = error
            error = (cant_enc_d_c - enc_count) * d_c / dt
            sum_err += error

            # Control proporcional de velocidad
            c_prop = K * error
            # Control derivativo
            c_der = K_D * (error - err_prev) / dt
            # Control integral
            c_int = K_I * (sum_err) * dt
            
            # Velocidad controlada
            vel = c_prop + c_der + c_int
            vel_rpm = (vel / rec_r)
            vel_por = vel_rpm * max_vel / max_vel_rpm
            # vel = senal_vel

            send_sabertooth_com (int (vel_por))



        # META ALCANZADA
        send_sabertooth_com (int (0))
        time.sleep(0.1)

        # Reiniciar tramo
        enc_count = int (0)
        err_prev = float (0)
        sum_err = float (0)
        error = float (0)
        time.sleep(0.05)
        time.sleep(0.01)



poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

led_on = True

last_blink_time = time.ticks_ms()

try:
    if poller.poll(0):
        cmd = sys.stdin.readline().strip()
        print('\n')
        imagen_detectada = int (cmd)

    # ============================================================
    # ================ INICIAR THREAD EN CORE 1 ==================
    # ============================================================
    _thread.start_new_thread(tarea_rotacion, ())

    # ============================================================
    # ================ CORE 0 EJECUTA TRACCIÓN ===================
    # ============================================================
    tarea_traccion()


except KeyboardInterrupt:
    print("\nPrograma detenido")

finally:
    send_sabertooth_com(int (0))
    set_duty (srv_stop, servo_pwm)
    led.off()