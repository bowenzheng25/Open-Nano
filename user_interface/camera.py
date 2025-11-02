import cv2

cap = cv2.VideoCapture(0)
zoom = 1  # zoom factor (>1 for zoom in, <1 for zoom out)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    centerX, centerY = w // 2, h // 2
    radiusX, radiusY = int(w/(2*zoom)), int(h/(2*zoom))

    minX, maxX = centerX - radiusX, centerX + radiusX
    minY, maxY = centerY - radiusY, centerY + radiusY

    cropped = frame[minY:maxY, minX:maxX]
    frame_zoomed = cv2.resize(cropped, (w, h))

    cv2.imshow('Zoomed Feed', frame_zoomed)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()