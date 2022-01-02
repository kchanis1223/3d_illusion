import numpy as np
import cv2

cap=cv2.VideoCapture(0)

while(cap.isOpened()):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    cv2.imshow('frame1', frame[200:400, -1])
    if cv2.waitKey(1) &0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
