import cv2 
import numpy as np

image = cv2.imread(r"C:\Users\ivand\OneDrive\Escritorio\FRED\Photo-1 (1).jpeg")

roi = cv2.selectROI("select the area", image)
cropped_image = image[int(roi[1]):int(roi[1]+roi[3]), 
                      int(roi[0]):int(roi[0]+roi[2])]

gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
#edges = cv2.Canny(gray, 50, 150, apertureSize=3)
#lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

# Dibuja sobre una copia de la imagen original
#output_image = cropped_image.copy()

# if lines is not None:
#     for r_theta in lines:
#         arr = np.array(r_theta[0], dtype=np.float64)
#         r, theta = arr

#         a = np.cos(theta)
#         b = np.sin(theta)
#         x0 = a * r
#         y0 = b * r

#         x1 = int(x0 + 1000 * (-b))
#         y1 = int(y0 + 1000 * (a))
#         x2 = int(x0 - 1000 * (-b))
#         y2 = int(y0 - 1000 * (a))

#         cv2.line(output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

cv2.imshow("Cropped", cropped_image)
#cv2.imwrite("linesDetected.jpg", output_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
