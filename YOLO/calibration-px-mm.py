import cv2
import numpy as np

# --- Dimensiones reales de tarjeta (mm) ---
RECT_WIDTH_MM  = 85.60
RECT_HEIGHT_MM = 53.98

cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

print("Mostrá la tarjeta a la cámara. Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- Preprocesamiento robusto ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)

    # Umbral adaptativo para zonas con iluminación irregular
    thresh = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        21, 5
    )

    # Morfología → elimina ruido pequeño
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Canny basado en el binario
    edges = cv2.Canny(closed, 30, 120)

    # Contornos
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_rect = None
    best_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:   # filtro fuerte para ignorar ruido
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            # Medir bounding box para descartar figuras alargadas
            x, y, w, h = cv2.boundingRect(approx)
            aspect = w / float(h)

            # Tarjeta: aspect ratio ≈ 85.6 / 53.98 ≈ 1.586
            if 1.3 < aspect < 1.9:
                if area > best_area:
                    best_area = area
                    best_rect = approx

    if best_rect is not None:
        cv2.drawContours(frame, [best_rect], -1, (0, 255, 0), 2)

        pts = best_rect.reshape(4,2)

        # Reordenar puntos: [top-left, top-right, bottom-right, bottom-left]
        def order_points(pts):
            rect = np.zeros((4, 2), dtype="float32")
            s = pts.sum(axis=1)
            rect[0] = pts[np.argmin(s)]
            rect[2] = pts[np.argmax(s)]

            diff = np.diff(pts, axis=1)
            rect[1] = pts[np.argmin(diff)]
            rect[3] = pts[np.argmax(diff)]
            return rect

        ordered = order_points(pts)

        # Distancias en px
        width_px  = np.linalg.norm(ordered[0] - ordered[1])
        height_px = np.linalg.norm(ordered[0] - ordered[3])

        mm_per_px_x = RECT_WIDTH_MM  / width_px
        mm_per_px_y = RECT_HEIGHT_MM / height_px

        cv2.putText(frame, f"mm/px X: {mm_per_px_x:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(frame, f"mm/px Y: {mm_per_px_y:.3f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    # --- IMPORTANTE: solo una ventana ---
    cv2.imshow("Calibracion", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

