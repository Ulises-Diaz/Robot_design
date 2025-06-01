import numpy as np
import cv2
import snap7
from snap7.util import set_bool
import time

# Initialize webcam (device 1)
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Video incorrecto")
    exit()

# PLC connection
IP = "192.168.1.105"
rack = 0 
slot = 1

plc = snap7.client.Client()

try:
    print("Attempting to connect to PLC...")
    plc.connect(IP, rack, slot)
    print("Successfully connected to PLC!")
except Exception as e:
    print(f"Failed to connect to PLC: {e}")
    plc = None

def write_bool_to_plc(client, db_number, byte_idx, bit_idx, value): 
    if client is None:
        print("PLC not connected, skipping write operation.")
        return
    try:
        data = bytearray(1)
        set_bool(data, 0, bit_idx, value)
        client.db_write(db_number, byte_idx, data)
    except Exception as e:
        print(f"Failed to write to PLC: {e}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        height, width, _ = frame.shape

        # Región de Interés (ROI)
        roi_y_start = int(height * 0.1)
        roi_y_end = int(height * 0.9)
        roi_x_start = int(width * 0.7)
        roi_x_end = int(width * 0.87)
        roi = frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        # Convertir a HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Rojo
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        # Blanco
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Gris fuerte
        lower_gray = np.array([0, 0, 100])
        upper_gray = np.array([180, 50, 200])
        mask_gray = cv2.inRange(hsv, lower_gray, upper_gray)

        # Combinar máscaras
        combined_mask = cv2.bitwise_or(mask_red, mask_white)
        combined_mask = cv2.bitwise_or(combined_mask, mask_gray)

        # Detectar bordes
        edges = cv2.Canny(combined_mask, 100, 200)

        # Hough
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)

        output = roi.copy()
        PLC_flag = False

        if lines is not None:
            for r_theta in lines[:1]:
                r, theta = r_theta[0]
                angle_deg = np.rad2deg(theta) - 90
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * r
                y0 = b * r
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
                print(f"Línea detectada con θ = {angle_deg:.2f}°")

                if -95 <= angle_deg <= -85:
                    PLC_flag = True
                    print("Ángulo correcto")
                else:
                    print("Ángulo erróneo")
        else:
            print("No se detectó ninguna línea")

        print(f"PLC Flag: {PLC_flag}")

        # PLC call
        write_bool_to_plc(plc, db_number=10, byte_idx=0, bit_idx=0, value=PLC_flag)

        time.sleep(0.1)

        # Mostrar resultados
        cv2.imshow("ROI con línea", output)
        cv2.imshow("Video Original", frame)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    if plc is not None:
        plc.disconnect()