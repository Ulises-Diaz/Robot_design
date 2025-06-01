import numpy as np
import cv2

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("video incorrecto")
    exit()

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
    if lines is not None:
        for r_theta in lines[:1]:  # Solo una línea
            r, theta = r_theta[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * r
            y0 = b * r
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            print(f"Línea detectada con θ = {np.rad2deg(theta):.2f}°")
    else:
        print("No se detectan líneas.")

    # Mostrar resultados
    cv2.imshow("ROI con línea", output)
    cv2.imshow("Video Original", frame)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()