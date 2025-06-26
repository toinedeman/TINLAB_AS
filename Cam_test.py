# Code om vertraging te meten (toevoegen)
import cv2
import time
cap = cv2.VideoCapture(0)  # CAMO als bron
while True:
    start = time.time()
    ret, frame = cap.read()
    cv2.imshow('Frame', frame)
    print(f"Latency: {(time.time() - start):.2f}s")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break