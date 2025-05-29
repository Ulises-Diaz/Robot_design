import cv2
import numpy as np
import matplotlib.pyplot as plt

# Leer imagen
image = cv2.imread(r"C:\Users\ivand\OneDrive\Escritorio\FRED\FRED_VISION\Images\Photo-1 (1).jpeg")

# ROI automático centrado
height, width, _ = image.shape

roi_y_start = int(height*0.57)
roi_y_end = int(height*0.90)  

# Coordenadas horizontales (ancho)
roi_x_start = int(width*0.22)
roi_x_end = int(width*0.65) 

cropped_image = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

# Procesamiento
gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

# Aplicar filtro gaussiano para reducir ruido
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Detección de bordes con parámetros ajustados
edges = cv2.Canny(blurred, 30, 100, apertureSize=3)

# Reducir umbral para detectar más líneas
lines = cv2.HoughLines(edges, 1, np.pi/180, 150)  # Umbral reducido de 200 a 80

output_image = cropped_image.copy()

horizontal_lines = []
vertical_lines = []

if lines is not None:
    for r_theta in lines:
        r, theta = r_theta[0]
        
        # Convertir theta a grados para mejor comprensión
        angle_deg = np.degrees(theta)
        
        # Filtrar líneas horizontales (ángulo cerca de 0° o 180°)
        # Tolerancia de ±15 grados para líneas horizontales
        if (angle_deg < 15) or (angle_deg > 165):
            horizontal_lines.append((r, theta))
        
        # Filtrar líneas verticales (ángulo cerca de 90°)
        # Tolerancia de ±15 grados para líneas verticales
        elif 75 < angle_deg < 105:
            vertical_lines.append((r, theta))

# Dibujar líneas horizontales en rojo
for r, theta in horizontal_lines:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * r
    y0 = b * r
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    cv2.line(output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Rojo

# Dibujar líneas verticales en azul
for r, theta in vertical_lines:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * r
    y0 = b * r
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    cv2.line(output_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Azul

# Convertir BGR a RGB para matplotlib
image_rgb = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)
output_rgb = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)
edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)

# Mostrar con matplotlib
plt.figure(figsize=(18, 6))

plt.subplot(1, 3, 1)
plt.imshow(image_rgb)
plt.title("Imagen original (ROI)")
plt.axis("off")

#plt.imshow(edges_rgb)
#plt.title("Bordes detectados")
#plt.axis("off")

plt.subplot(1, 3, 2)
plt.imshow(output_rgb)
plt.title(f"Líneas detectadas\nHorizontales: {len(horizontal_lines)} (Rojo)\nVerticales: {len(vertical_lines)} (Azul)")
plt.axis("off")

plt.tight_layout()
plt.show()

print(f"Líneas horizontales detectadas: {len(horizontal_lines)}")
print(f"Líneas verticales detectadas: {len(vertical_lines)}")