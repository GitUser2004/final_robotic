# funcionan colores y circulos 
import cv2
import requests
import numpy as np
import time

CAM_URL = "http://192.168.100.101/capture"
ESP32_IP = "192.168.100.105"

TH_DETECCION = 1000
TH_DISTANCIA = 6000

ESPERANDO_MISION = 0
BUSCANDO_COLOR   = 1
ACERCANDOSE      = 2

estado = ESPERANDO_MISION
color_objetivo = "NINGUNO"
fase = "IDA"

kernel = np.ones((5,5), np.uint8)

# PARÁMETROS CÍRCULOS 
DP = 1.2
MIN_DIST = 60
P1 = 120          
P2 = 18    
MIN_R = 12
MAX_R = 180

print("[SISTEMA] Iniciado")
print("[ESTADO] Esperando misión")

# OBTENER COLOR OBJETIVO
def obtener_color_objetivo():
    try:
        r = requests.get(f"http://{ESP32_IP}/target", timeout=2)
        if r.status_code == 200:
            return r.text.strip().upper()
    except:
        pass
    return "NINGUNO"

# LOOP PRINCIPAL
while True:
    try:
        # ESPERAR MISIÓN (SIN CÁMARA)
        if estado == ESPERANDO_MISION:
            nuevo = obtener_color_objetivo()

            if fase == "IDA" and nuevo in ["ROJO", "VERDE", "AZUL"]:
                color_objetivo = nuevo
                estado = BUSCANDO_COLOR
                print(f"[ESTADO] Buscando círculo: {color_objetivo}")

            elif fase == "RETORNO" and nuevo == "AMARILLO":
                color_objetivo = "AMARILLO"
                estado = BUSCANDO_COLOR
                print("[ESTADO] Buscando círculo: AMARILLO")

            time.sleep(0.5)
            continue

        # CAPTURA (NO TOCAR)
        r = requests.get(CAM_URL, timeout=5)
        if r.status_code != 200:
            continue

        img = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue

        # PROCESAMIENTO
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5,5), 0)

        if color_objetivo == "ROJO":
            m1 = cv2.inRange(hsv, (0,120,70), (10,255,255))
            m2 = cv2.inRange(hsv, (170,120,70), (180,255,255))
            mask = m1 + m2

        elif color_objetivo == "VERDE":
            mask = cv2.inRange(hsv, (36,50,70), (89,255,255))

        elif color_objetivo == "AZUL":
            mask = cv2.inRange(hsv, (90,50,70), (128,255,255))

        elif color_objetivo == "AMARILLO":
            mask = cv2.inRange(hsv, (27,30,100), (38,255,255))

        else:
            continue

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # DETECTAR SOLO CÍRCULOS EN LA MÁSCARA
        # Hough necesita una imagen "suave"
        mask_blur = cv2.GaussianBlur(mask, (9, 9), 2)

        circles = cv2.HoughCircles(
            mask_blur,
            cv2.HOUGH_GRADIENT,
            dp=DP,
            minDist=MIN_DIST,
            param1=P1,
            param2=P2,
            minRadius=MIN_R,
            maxRadius=MAX_R
        )

        # Convertimos a "área" basada en el círculo detectado
        area = 0
        best_circle = None

        if circles is not None:
            circles = np.uint16(np.around(circles[0]))
            # elegir el círculo más grande (mejor para el objetivo)
            best_circle = max(circles, key=lambda c: c[2])
            x, y, rad = best_circle
            area = int(np.pi * (rad ** 2))

            # dibujar para debug
            cv2.circle(img, (x, y), rad, (255, 255, 255), 2)
            cv2.circle(img, (x, y), 2, (255, 255, 255), 3)

        # FSM
        if estado == BUSCANDO_COLOR and area > TH_DETECCION:
            estado = ACERCANDOSE
            print("[EVENTO] Círculo encontrado → acercándose")

        if estado == ACERCANDOSE and area > TH_DISTANCIA:
            print(f"[EVENTO] Misión cumplida: {color_objetivo}")

            try:
                requests.get(
                    f"http://{ESP32_IP}/color",
                    params={"c": color_objetivo},
                    timeout=2
                )
            except:
                pass

            cv2.destroyAllWindows()

            if color_objetivo == "AMARILLO":
                fase = "IDA"
            else:
                fase = "RETORNO"

            estado = ESPERANDO_MISION
            color_objetivo = "NINGUNO"
            print("[ESTADO] Esperando misión")
            time.sleep(1)
            continue

        # VISUAL
        cv2.putText(
            img,
            f"{fase} | {estado} | {color_objetivo} | circleArea {int(area)}",
            (20,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255,255,255),
            2
        )
        cv2.imshow("Vision Robot", img)

    except Exception as e:
        print("[ERROR]", e)

    if cv2.waitKey(1) & 0xFF == 27:
        print("[SISTEMA] Finalizado por usuario")
        break

    time.sleep(0.4)

cv2.destroyAllWindows()