import numpy as np
import cv2
import time
import math as ma

focal_length=469.4610
cap = cv2.VideoCapture(0)
eyes=[]
pd=6.2
img = np.zeros((250, 450, 3), np.uint8)

class Line(object):
    def __init__(self, coor1, coor2):
        self.coor1 = coor1
        self.coor2 = coor2

    def distance(self):
        x1, y1 = self.coor1
        x2, y2 = self.coor2

        return float(ma.sqrt(ma.pow((x2-x1), 2) + ma.pow((y2-y1), 2)))


while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        try:
            eyes=[]
            fr=frame
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            left_eye = cv2.CascadeClassifier('haarcascade_lefteye_2splits.xml')
            right_eye = cv2.CascadeClassifier('haarcascade_righteye_2splits.xml')
            detected=[0, 0]
            detected[0]= left_eye.detectMultiScale(frame, 1.3, 5)
            detected[1] = right_eye.detectMultiScale(frame, 1.3, 5)
            print("###")
            print(detected[0])
        

            pupilFrame=frame
            pupilO=frame

            windowClose = np.ones((5,5), np.uint8)
            windowOpen = np.ones((2, 2), np.uint8)
            windowErode = np.ones((2, 2), np.uint8)

            for i in range (2):
                for(x, y, w, h) in detected[i]:
                    pupilFrame = cv2.equalizeHist(frame[int(y+(h*.25)):(y+h), x:(x+w)])
                    pupilO=pupilFrame
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
                                cx, cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
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
                    eyes.append([cx+x, cy+y+int(h*.25), x, y, w, h])
            if len(eyes)==2 :
                #if distance_object > 1000:
                #    continue
                #if (eyes[1][0]-eyes[0][0]) < 30:
                #    continue
                img = np.zeros((250, 450, 3), np.uint8)
                coor1=eyes[0]
                coor2=eyes[1]
                line=Line(coor1, coor2)
                distance=float(line.distance())
                distance_object = (pd*focal_length)/distance

                #for i in range(2):
                #    cv2.rectangle(fr, (eyes[i][2], eyes[i][3]), ((eyes[i][2]+eyes[i][4]), (eyes[i][3]+eyes[i][5])), (0, 0, 0), 1)
                #    cv2.line(fr, (eyes[i][2], eyes[i][3]), ((eyes[i][2]+eyes[i][4]), (eyes[i][3]+eyes[i][5])), (0, 0, 255), 1)
                #    cv2.line(fr, ((eyes[i][2]+eyes[i][4]), eyes[i][3]), (eyes[i][2], (eyes[i][3]+eyes[i][5])), (0, 0, 255), 1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(img, 'left_eye - x: '+ str(eyes[0][0])+ '  y: '+str(eyes[0][1]), (10, 50), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(img, 'right_eyes -x: '+str(eyes[1][0])+'  y: '+str(eyes[1][1]), (10, 100), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(img, 'distance between camera and eyes: ', (10, 150), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(img, str(distance_object) + ' cm', (10, 200), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.circle(fr, (eyes[0][0], eyes[0][1]), 5, 255, -1)
                cv2.circle(fr, (eyes[1][0], eyes[1][1]), 5, 255, -1)
        except:
            continue
    cv2.imshow('frame', fr)
    cv2.imshow('frame2', pupilO)
    cv2.imshow('frame3', img)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
       break

cap.release()
cv2.destroyAllWindows()

