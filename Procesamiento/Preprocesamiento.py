import cv2
import os

# Clases YOLO (modifica si quieres)
CLASES = {
    "C": 0,  # Cable
    "S": 1,  # Scotch
    "F": 2,  # Fisura
    "D": 3,  # Desgaste
    "R": 4,  # Rotura
    "H": 5   # Hendidura
}

drawing = False
ix, iy = -1, -1
bbox = None


def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, bbox

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        bbox = None

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            bbox = (ix, iy, x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        bbox = (ix, iy, x, y)


def normalizar_bbox(x1, y1, x2, y2, img_w, img_h):
    # Corregir inversión al arrastrar de derecha a izquierda
    xmin = min(x1, x2)
    xmax = max(x1, x2)
    ymin = min(y1, y2)
    ymax = max(y1, y2)

    w = xmax - xmin
    h = ymax - ymin

    x_center = xmin + w / 2
    y_center = ymin + h / 2

    # Normalizar entre 0 y 1
    return x_center / img_w, y_center / img_h, w / img_w, h / img_h


def encontrar_cable(ruta_img):
    global bbox
    bbox = None

    img_prev = cv2.imread(ruta_img)
    if img_prev is None:
        print(f"Error: no se pudo cargar la imagen {ruta_img}")
        return
    h, w = img_prev.shape[:2]
    m = max (h, w)
    factor = 600 / m
    img = cv2.resize (img_prev, (0, 0), fx = factor, fy = factor)
    H, W = img.shape[:2]

    cv2.namedWindow(ruta_img)
    cv2.setMouseCallback(ruta_img, draw_rectangle)

    print("\n==============================")
    print(f"Imagen: {ruta_img}")
    print("Dibuja un bounding box. Presiona 'g' para guardar o 'n' si NO HAY falla.")
    print("==============================")
    print("\nTipo de falla:")
    print("[C] Cable")
    print("[S] Scotch")
    print("[F] Fisura")
    print("[R] Rotura")
    print("[D] Desgaste")
    print("[H] Hendidura")

    while True:
        clone = img.copy()

        if bbox:
            cv2.rectangle(clone, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,255,0), 2)

        cv2.imshow(ruta_img, clone)
        k = cv2.waitKey(1)

        if k == ord('g') and bbox is not None:

            x1, y1, x2, y2 = bbox
            print (x1, y1, x2, y2)
            if x1 < 5: x1 = 0
            if y1 < 5: y1 = 0
            if x2 > W - 5: x2 = W
            if y2 > H - 5: y2 = H
            guardar_recorte (img, ruta_img, x1, y1, x2, y2)
            xc, yc, ww, hh = normalizar_bbox(x1, y1, x2, y2, W, H)
            # Exportar .txt
            escribir_im_txt (ruta_img, xc, yc, ww, hh)

            break

        if k == ord('q'):
            print("❌ Cancelado")
            break

    cv2.destroyAllWindows()

def etiquetar_imagen(ruta_img):
    global bbox
    bbox = None

    img = cv2.imread(ruta_img)
    if img is None:
        print(f"Error: no se pudo cargar la imagen {ruta_img}")
        return
    H, W = img.shape[:2]

    cv2.namedWindow(ruta_img)
    cv2.setMouseCallback(ruta_img, draw_rectangle)

    print("\n==============================")
    print(f"Imagen: {ruta_img}")
    print("Dibuja un bounding box. Presiona 'g' para guardar o 'n' si NO HAY falla.")
    print("==============================")
    print("\nTipo de falla:")
    print("[C] Cable")
    print("[S] Scotch")
    print("[F] Fisura")
    print("[R] Rotura")
    print("[D] Desgaste")
    print("[H] Hendidura")

    while True:
        clone = img.copy()

        if bbox:
            cv2.rectangle(clone, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,255,0), 2)

        cv2.imshow(ruta_img, clone)
        k = cv2.waitKey(1)

        if k == ord('g') and bbox is not None:
            # Preguntar tipo de falla
            while True:
                k_2 = cv2.waitKey(1)
                tipo = 0
                if k_2 == ord('c'): tipo = "C"
                if k_2 == ord('s'): tipo = "S"
                if k_2 == ord('f'): tipo = "F"
                if k_2 == ord('r'): tipo = "R"
                if k_2 == ord('d'): tipo = "D"
                if k_2 == ord('h'): tipo = "H"
                if tipo != 0: break

            if tipo not in CLASES:
                print("Tipo inválido. Se cancela.")
                return

            class_id = CLASES[tipo]

            x1, y1, x2, y2 = bbox
            xc, yc, ww, hh = normalizar_bbox(x1, y1, x2, y2, W, H)

            # Exportar .txt
            ruta_txt = ruta_img.replace(".jpg", ".txt").replace(".png", ".txt").replace(".jpeg", ".txt").replace("images", "label")
            with open(ruta_txt, "w") as f:
                f.write(f"{class_id} {xc:.6f} {yc:.6f} {ww:.6f} {hh:.6f}\n")

            print(f"✔ Etiqueta guardada en {ruta_txt}")
            break

        if k == ord('n'):  # No hay falla
            ruta_txt = ruta_img.replace(".jpg", ".txt").replace(".png", ".txt")
            open(ruta_txt, "w").close()
            print(f"✔ Imagen sin falla → archivo vacío generado: {ruta_txt}")
            break

        if k == ord('q'):
            print("❌ Cancelado")
            break

    cv2.destroyAllWindows()


"""Errores:
Cable_24 (aprox)
"""
def guardar_recorte (img, ruta, x1, y1, x2, y2):
        # Recortar imagen
    crop = img[y1:y2, x1:x2]
    r_im = os.path.split (ruta)[-1]

    # Guardar imagen recortada
    out_img_path = os.path.join("Procesamiento", "Cables", "images", r_im)
    cv2.imwrite(out_img_path, crop)

def escribir_im_txt (ruta, xc, yc, ww, hh):
    # Generar label en formato YOLO (normalizado)

    # Guardar archivo .txt con misma base del nombre
    txt_name = os.path.split (ruta)[-1].split (".")[0] + ".txt"
    txt_path = os.path.join ("Procesamiento", "Imagenes", "label", txt_name)

    with open(txt_path, "w") as f:
        f.write(f"0 {xc:.6f} {yc:.6f} {ww:.6f} {hh:.6f}\n")

    print(f"[OK] Procesado: {txt_name} → recortado + etiqueta guardada.")

def procesar_carpeta(ruta_carpeta):
    imagenes = [f for f in os.listdir(ruta_carpeta)]

    for img in imagenes:
        etiquetar_imagen(os.path.join(ruta_carpeta, img))

def cambiar_clase (ruta):
    textos = [f for f in os.listdir(ruta)]

    for t in textos:
        path = os.path.join (ruta, t)
        new_lines = []
        with open(path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 5:
                    print(f"Línea inválida en {path}: {line}")
                    continue

                cls = int(parts[0])
                x, y, w, h = parts[1:]

                # Restar 1 a la clase
                # new_cls = cls + 1

                new_lines.append(f"{cls} {x} {y} {w} {h}")

        # Escribir el resultado
        with open(path, "w") as f:
            f.write(new_lines[0])

# ===============================
#       EJECUCIÓN
# ===============================

if __name__ == "__main__":
    carpeta = os.path.join("Procesamiento", "Cables", "label")
    r_1 = os.path.join (carpeta, "train")
    r_2 = os.path.join (carpeta, "val")

    cambiar_clase (r_1)
    cambiar_clase (r_2)
