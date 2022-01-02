import numpy as np
import cv2
import time

wW=640
hH=480

cap = cv2.VideoCapture(0)
eyes=[]

class eye(object):
    def __init__(self, _frame, _haarcascade):
        frame= _frame
        haarcascade = _haarcascade
        detected = haarcascade.detectMultiScale(frame, 1.3, 5)
        x, y, w, h=detected
    def savecnt(self):
        pupilFrame=frame

        windowClose = np.ones((5,5), np.uint8)
        windowOpen = np.ones((2, 2), np.uint8)
        windowErode = np.ones((2, 2), np.uint8)

        pupilFrame = cv2.equalizeHist(frame[int(y+(h*.25)):(y+h), x:(x+w)])
        ret, pupilFrame = cv2.threshold(pupilFrame, 55, 255, cv2.THRESH_BINARY)
        pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_CLOSE, windowClose)
        pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_ERODE, windowErode)
        pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_OPEN, windowOpen)

        threshold = cv2.inRange(pupilFrame, 250, 255)

        _, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) >= 2:
            maxArea = 0
            MAindex = 0
            distanceX = []
            currentIndex = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                center = cv2.moments(cnt)
                try:
                    cx, cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
                except:
                    continue
                distanceX.append(cx)
                if area > maxArea:
                    maxArea = area
                    MAindex = currentIndex
                currentIndex = currentIndex + 1

            del contours[MAindex]
            del distanceX[MAindex]

        if len(contours) >= 1:
            maxArea = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > maxArea:
                    maxArea = area
                    largeBlob = cnt

        if len(largeBlob) > 0:
            center = cv2.moments(largeBlob)
            cx, cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
        return (cx, cy)

    def xy(self):
        return (x, y)

    def CheckData(self):
        if len(detected):
            return True
        else:
            return False


while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        img = np.zeros((200, 400, 3), np.uint8)
        lehr = cv2.CascadeClassifier('haarcascade_lefteye_2splits.xml')
        rehr = cv2.CascadeClassifier('haarcascade_righteye_2splits.xml')

        leftEye=eye(frame, lehr)
        if leftEye.CheckData():
            x, y=leftEye.xy
            rightEye=eye(frame[

    cv2.imshow('frame', frame)
    #cv2.imshow('frame2', pupilO)
    cv2.imshow('frame3', img)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
       break

cap.release()
cv2.destroyAllWindows()


