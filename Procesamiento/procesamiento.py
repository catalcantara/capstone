import cv2
import numpy as np
import time
import serial
import re
import select
import sys
from ultralytics import YOLO

puerto = "/dev/ttyACM0"  # cambia si es necesario /dev/ttyUSB0
baudrate = 115200

# Serial USB
# ser = serial.Serial(puerto, baudrate, timeout=1)

# Webcam
cap = cv2.VideoCapture(0)

# Función para tomar foto
def tomar_foto(cap, n, modelo, modelo_2, pos, archivo):
    ret, frame = cap.read()
    ruta_im = "./Procesamiento/Resultados/Con_Fallas/img_" + str(n) + ".jpg"
    resultados, det = evaluar_fisura (frame, modelo, modelo_2, ruta_im)
    if ret and resultados[0]:
        cv2.imshow("Camara", frame)
        almacenar_info (pos, "F", archivo)
    return resultados

def cargar_pic (ruta, n, modelo, modelo_2, pos, archivo):
    frame = cv2.imread (ruta, cv2.IMREAD_UNCHANGED)
    ruta_im = "./capstone/Procesamiento/Resultados/Con_Fallas/img_" + str(n) + ".jpg"
    resultados = evaluar_fisura (frame, modelo, modelo_2, ruta_im)
    print (resultados)
    if resultados[0]:
        cv2.imshow("Camara", frame)
        almacenar_info (pos, ruta_im, resultados[1][-1]["clase"], archivo)

# Función para evaluar fisura
def evaluar_fisura(frame, modelo, modelo_2, ruta_im, umbral_conf=0.30):
    """
    Evalúa una imagen y determina si hay fisura u óxido usando YOLOv8.

    Parámetros:
    - imagen: ruta a imagen o frame de OpenCV (numpy array)
    - umbral_conf: confianza mínima para considerar detección válida
    - guardar_resultado: si True, guarda imagen con bounding boxes dibujados
    - ruta_guardado: ruta donde guardar el archivo final

    Retorna:
    - hay_fisura (bool)
    - detecciones: lista de diccionarios con bounding boxes y metadata
    """

    # Corre el modelo
    resultados = modelo(frame)[0]

    detecciones = []
    hay_cable = False

    for falla in resultados.boxes:
        print (f"{falla}")
        confianza = float(falla.conf[0])
        clase  = int(falla.cls[0])
        x1, y1, x2, y2 = falla.xyxy[0].cpu().numpy()

        if confianza < 0.5:
            continue

        nombre_clase = modelo.names[clase]

        detecciones.append({
            "clase": nombre_clase,
            "confianza": confianza,
            "bbox": [float(x1), float(y1), float(x2), float(y2)]
        })
        print (clase, type (clase))
        if clase == 0 or hay_cable:
            hay_cable = True

    # Guardado opcional con bbox dibujados
    print (f"Terminó el for, se supone que hay cable {hay_cable}")
    print (detecciones)
    if hay_cable:
        x_1, x_2, y_1, y_2 = [10000, 0, 10000, 0]
        for det in detecciones:
            x1, y1, x2, y2 = map(int, det["bbox"])
            x_1 = min (x1, x2, x_1)
            y_1 = min (y1, y2, y_1)
            x_2 = max (x1, x2, x_2)
            y_2 = max (y1, y2, y_2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            texto = f"{det['clase']} {det['confianza']:.2f}"
            cv2.putText(frame, texto, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        print ("Hay cable")
        
        im_c = frame[y_1:y_2, x_1:x_2]
        # cv2.imshow("im_c", im_c)
        # cv2.waitKey(0)  # Espera hasta que presiones una tecla
        # cv2.destroyAllWindows()
        resultados_2 = modelo_2(im_c)[0]
        detecciones = []
        hay_falla = False

        for falla in resultados_2.boxes:
            print (f"{falla}")
            confianza = float(falla.conf[0])
            clase  = int(falla.cls[0])
            x1, y1, x2, y2 = falla.xyxy[0].cpu().numpy()

            if confianza < umbral_conf:
                continue

            nombre_clase = modelo_2.names[clase]

            detecciones.append({
                "clase": nombre_clase,
                "confianza": confianza,
                "bbox": [float(x1), float(y1), float(x2), float(y2)]
            })
            print (clase, type (clase))
            if clase != 0 or hay_falla:
                hay_falla = True

        detecciones = []
        if hay_falla:
            print ("Hay falla")
            for det in detecciones:
                x1, y1, x2, y2 = map(int, det["bbox"])
                cv2.rectangle(im_c, (x1, y1), (x2, y2), (0,255,0), 2)
                texto = f"{det['clase']} {det['confianza']:.2f}"
                cv2.putText(im_c, texto, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        
            cv2.imwrite(ruta_im, im_c)
            print ("Se guardó la imagen")

    return hay_falla, detecciones


# Función para almacenar info en el archivo csv
def almacenar_info (pos, ruta_im, tipo, archivo):
    if tipo == "O":
        tip = "Óxido"
    elif tipo == "F":
        tip = "Fisura"
    else:
        tip = "No identificado"
    with open (archivo, "a") as tex:
        tex.writelines ([f"\n{pos}, {tip}, {ruta_im}"])

def definir_latex (parte, ruta):
    if parte == 0:
        f = "w"
        lineas = [r"\documentclass[11pt,letterpaper,twoside]{report}\n",
                  r"\usepackage[utf8]{inputenc}",
                  r"\usepackage[english, spanish]{babel}",
                  r"\usepackage[T1]{fontenc}",
                  r"\usepackage{amsmath}",
                  r"\usepackage{siunitx}",
                  r"\usepackage{graphicx}",
                  r"\usepackage{float}",
                  r"\usepackage{geometry}",
                  r"\geometry{margin=2.5cm}",
                  r"\usepackage{booktabs}",
                  r"\usepackage{array}",
                  r"\newcolumntype{M}[1]{>{\centering\arraybackslash}m{#1}}",
                  r"\usepackage{fancyhdr}",
                  r"\pagestyle{fancy}",
                  r"\fancyhead[L]{Reporte de fallas\\ Cable de alta tensión}",
                  r"\fancyhead[R]{\thepage}",
                  r"\thispagestyle{plain}",
                  r"\def\doctitle{Tarea 4}",
                  r"\def\docsubtitle{Diseño de Bajo Nivel}",
                  r"\def\docdate{10.09.2025}",
                  r"\def\coursename{IRB2002 -- Diseñoo de Sistemas Robóticos}",
                  r"\def\authornamea{Catalina Alcántara}",
                  r"\def\authornameb{Ignacio Campillay}" ,
                  r"\def\authornamec{Vittorio Cherubini}",
                  r"\def\authornamed{Sebastián Madrid}",
                  r"\def\groupnum{Grupo N$^\circ$ 3}",
                  r"\def\deptname{Escuela de Ingeniería / Major en Ingeniería Robótica}",
                  r"\def\orgname{Pontificia Universidad Católica de Chile}",
                  r"\inputencoding{utf8}",
                  r"\begin{document}",
                  r"\noindent",
                  r"\begin{titlepage}",
                  r"\mbox{}\\[-0.99\baselineskip]",
                  r"\setlength{\unitlength}{1mm}",
                  r"\parbox{30mm}{\includegraphics[scale=0.47]{figs/logo_uc.pdf}\\[-0.55\baselineskip]}",
                  r"\parbox{\textwidth}{",
                  r"{\large \coursename}\\",
                  r"{\sc \deptname  }\\",
                  r"{\sc \orgname}",
                  r"}",
                  r"\rule[0ex]{\textwidth}{.4pt}",
                  r"",
                  r"\vspace{50mm}",
                  r"\parbox{\textwidth}{",
                  r"{\centering",
                  r"{\Huge \bf \doctitle}\\",
                  r"\vspace{10mm}",
                  r"{\LARGE \bf \docsubtitle}\\",
                  r"\vspace{10mm}",
                  r"{\Large \groupnum\\}",
                  r"\vspace{10mm}",
                  r"{\Large",
                  r"\authornamea\\",
                  r"\vspace{1mm}",
                  r"\authornameb\\",
                  r"\vspace{1mm}",
                  r"\authornamec\\",
                  r"\vspace{2mm}",
                  r"\authornamed}\\",
                  r"\vspace{10mm}",
                  r"{\Large  \docdate}\\",
                  r"}}",
                  r"\end{titlepage}",
                  r"\def\contentsname{\normalfont \Large Tabla de Contenidos\vspace*{-1cm}}",
                  r"\vspace*{-2cm}",
                  r"\parbox[t]{\textwidth}{",
                  r"{\tableofcontents}}",
                  r"\newpage",
                  r"\section{Reporte fallas}",
                  r"\begin{table}[H]",
                  r"\centering",
                  r"\begin{tabular}{|M{1cm}|M{2cm}|M{2cm}|M{5cm} |}",
                  r"\hline",
                  r"\textbf{ID} & \textbf{Posición} & \textbf{Tipo} & \textbf{Imagen}\\",
                  r"\hline"]
    else:
        f = "a"
        lineas = [r"\end{tabular}",
                  r"\end{table}",
                  r"\end{document}"]
        
    with open (ruta, f, encoding="utf-8") as tex:
        for linea in lineas:
            tex.write (linea + "\n")


def main (num):
    # 1. Accede a la cámara (0 es el índice de la cámara predeterminada)
    cap = cv2.VideoCapture(0)

    # 2. Verifica si la cámara se abrió correctamente
    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara.")
        exit()

    fotos = 0
    foto_error = 0
    ruta = "./capstone/Procesamiento/Resultados/Reportes/Reporte_Fallas" + num + ".tex"
    definir_latex (0, ruta)
    # Cargar modelo YOLO entrenado (usa tu best.pt)
    modelo = YOLO("./capstone/Procesamiento/Imagenes/best.pt")   # <= AJUSTA la ruta a donde guardes el modelo
    modelo_2 = YOLO("./capstone/Procesamiento/Cables/best.pt")   # <= AJUSTA la ruta a donde guardes el modelo

    # 3. Bucle para capturar y mostrar fotogramas
    ser = 0
    while True:
        # Lee un fotograma de la cámara
        if ser.in_waiting > 0:
            mensaje = ser.readline().decode().strip()
            pos_str, bool_str = mensaje.split(",")

            posicion = float(pos_str)
            capturar = bool_str.strip() == "True"

            if capturar:

                print(f"Imán detectado: {fotos%4}")
                print(f"Posición detectada: {posicion} m")

                det,  = tomar_foto (cap, foto_error, modelo, modelo_2, posicion, ruta)
                fotos += 1
                if det == True: foto_error += 1

        # Si se presiona la tecla 'q', sal del bucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def main_temporal (num):
    ruta = "./capstone/Procesamiento/Imagenes/images/val/"

    fotos = 0
    foto_error = 0
    ruta_tex = "./capstone/Procesamiento/Resultados/Reportes/Reporte_Fallas_" + num + ".tex"
    definir_latex (0, ruta_tex)
    # Cargar modelo YOLO entrenado (usa tu best.pt)
    modelo = YOLO("./capstone/Procesamiento/Imagenes/best.pt")   # <= AJUSTA la ruta a donde guardes el modelo
    modelo_2 = YOLO("./capstone/Procesamiento/Cables/best.pt")   # <= AJUSTA la ruta a donde guardes el modelo

    while True:
        imagen = sys.stdin.readline().strip()
        posicion = sys.stdin.readline().strip()
        ruta_final = ruta + imagen
        det = cargar_pic (ruta_final, foto_error, modelo, modelo_2, posicion, ruta_tex)
        fotos += 1
        if det == True: foto_error += 1

        # Si se presiona la tecla 'q', sal del bucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break




# poller = select.poll()
# poller.register(sys.stdin, select.POLLIN)
cmd = 0
try:
    # if poller.poll(0):
        cmd = sys.stdin.readline().strip()
        print('\n')
        main_temporal (cmd)



except KeyboardInterrupt:
    print("\nPrograma detenido")
    ruta = "./capstone/Procesamiento/Resultados/Reportes/Reporte_Fallas_" + cmd + ".tex"
    definir_latex (1, ruta)
