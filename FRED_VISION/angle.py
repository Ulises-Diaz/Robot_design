import cv2
import numpy as np
import matplotlib.pyplot as plt

# Leer imagen
image = cv2.imread(r"C:\Users\ivand\OneDrive\Escritorio\FRED\Screenshot 2025-05-27 100040.png")

# ROI automático centrado
height, width, _ = image.shape

roi_y_start = int(height * 0.1)
roi_y_end = int(height * 0.9)  

# Coordenadas horizontales (ancho)
roi_x_start = int(width * 0.68)
roi_x_end = int(width * 0.99) 

cropped_image = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

# Procesamiento
gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

edges = cv2.Canny(gray, 50, 150, apertureSize=3)

lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

output_image = cropped_image.copy()

if lines is not None:
    for r_theta in lines:
        r, theta = r_theta[0]
        
        angle_deg = np.rad2deg(theta)
        angle_inclination = angle_deg - 90  # Ángulo con respecto al eje horizontal
    
        print(f"Ángulo (theta): {angle_deg:.2f}° | Inclinación: {angle_inclination:.2f}°")
        
        a = np.cos(theta)
        
        b = np.sin(theta)
        
        x0 = a * r
        
        y0 = b * r
        
        x1 = int(x0 + 1000 * (-b))
        
        y1 = int(y0 + 1000 * (a))
        
        x2 = int(x0 - 1000 * (-b))
        
        y2 = int(y0 - 1000 * (a))
        
        cv2.line(output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

# Convertir BGR a RGB para matplotlib
image_rgb = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)
output_rgb = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

# Mostrar con matplotlib
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.imshow(image)
plt.title("Imagen original (ROI)")
plt.axis("off")

plt.subplot(1, 2, 2)
plt.imshow(output_rgb)
plt.title("Líneas detectadas")
plt.axis("off")

plt.tight_layout()
plt.show()
