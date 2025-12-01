import cv2

def funcion ():
    # Definir los nombres de los archivos de entrenamiento
    arr_train = ["Cable_1.jpeg",
                "Cable_2.jpeg",
                "Cable_3.jpeg",
                "Cable_4.jpeg",
                "Cable_5.jpeg",
                "Cable_6.jpeg",
                "Cable_7.jpeg",
                "Cable_11.jpeg",
                "Cable_12.jpeg",
                "Cable_13.jpeg",
                "Cable_14.jpg",
                "Cable_16.jpg",
                "Cable_17.jpg",
                "Cable_18.jpg",
                "Cable_19.jpg",
                "Cable_21.png",
                "Cable_24.png",
                "Cable_25.png",
                "Cable_26.png",
                "Cable_27.png",
                "Cable_28.png",
                "Cable_29.png",
                "Cable_30.png",
                "Cable_31.png",
                "Cable_32.png",
                "Cable_falla_1.jpeg",
                "Cable_falla_2.jpeg",
                "Cable_falla_3.jpeg",
                "Cable_falla_4.jpeg",
                "Cable_falla_6.jpeg",
                "Cable_falla_7.jpeg",
                "Cable_falla_10.jpeg",
                "Cable_falla_11.jpeg",
                "Cable_falla_12.png",
                "Cable_falla_13.png",
                "Cable_falla_14.png",
                "Cable_falla_16.png",
                "Cable_falla_17.png"]

    # Definir los archivos de prueba
    arr_val = ["Cable_8.jpeg",
            "Cable_9.jpeg",
            "Cable_15.jpg",
            "Cable_20.png",
            "Cable_22.png",
            "Cable_23.png",
            "Cable_falla_5.jpeg",
            "Cable_falla_8.jpeg",
            "Cable_falla_9.jpeg",
            "Cable_falla_15.png"]

    # Definir los paths principales
    array = [arr_val]
    path = "./capstone/Procesamiento/Imagenes/"
    path_1 = "label/"
    path_2 = "images/"

    for a in array:
        for file in a:
            # Definir si corresponde a train o val
            if a == arr_train:
                path_3 = "train/"
            else:
                path_3 = "val/"
            # Definir el resto de los paths
            file_txt = file.split (".")[0]
            path_im = path + path_2 + path_3 + file
            path_txt = path + path_1 + path_3 + file_txt + ".txt"

            img_prev = cv2.imread(path_im)
            if img_prev is None:
                print(f"Error: no se pudo cargar la imagen {path_im}")
                continue
            H, W = img_prev.shape[:2]
            m = max (H, W)
            factor = 600 / m
            img = cv2.resize (img_prev, (0, 0), fx = factor, fy = factor)
            H, W = img.shape[:2]
            clone = img.copy()
            roi = cv2.selectROI("Selecciona la falla", img, showCrosshair=True, fromCenter=False)
            cv2.destroyAllWindows()

            x_min = int(roi[0])
            y_min = int(roi[1])
            w = int(roi[2])
            h = int(roi[3])

            # Si no seleccionaste nada → archivo vacío
            if w == 0 or h == 0:
                with open (path_txt, "w") as txt:
                    print(">> No se seleccionó ninguna falla → creando archivo .txt vacío")
                continue

            x_max = x_min + w
            y_max = y_min + h

            print("Bounding box:")
            print(f"x_min={x_min}, y_min={y_min}, x_max={x_max}, y_max={y_max}")

            # Conversión YOLO
            cx = ((x_min + x_max) / 2) / W
            cy = ((y_min + y_max) / 2) / H
            bw = (x_max - x_min) / W
            bh = (y_max - y_min) / H

            print(f"YOLO line:")
            print(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

            with open (path_txt, "w") as txt:
                txt.write (f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

path = "./capstone/Procesamiento/Imagenes/label/train/Cable_1.txt"
with open(path, 'r', encoding='utf-8') as f:
    print(repr(f.read()))