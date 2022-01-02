import numpy as np
import cv2
import time
import math as ma

FOCAL_LENGTH=469.4610
PD=6.2
cap = cv2.VideoCapture(0)
#width=420
#height=320
left_eye_classifier = cv2.CascadeClassifier('haarcascade_lefteye_2splits.xml')
right_eye_classifier = cv2.CascadeClassifier('haarcascade_righteye_2splits.xml')
def Distance(coor1, coor2):
    x1, y1 = coor1
    x2, y2 = coor2
    return float(ma.sqrt(ma.pow((x2-x1), 2)) + ma.pow((y2-y1), 2))

class eye(object):
    def __init__(self, classifier):
        self.x, self.y, self.w, self.h =[0, 0, 0, 0]
        self.center_x, self.center_y= 0,0
        self.eye_frame=0
        self.classifier=classifier
    def FindingLocation(self, frame): #두 눈 중심을 찾아 eye 클래스에 저장한다.
        detected=self.classifier.detectMultiScale(frame, 1.3, 5)
        if (type(detected) is not tuple):
            self.x, self.y, self.w, self.h= detected[0]
            return True

    def FindingCenterOfEye(self, frame):
        windowClose = np.ones((5,5), np.uint8)
        windowOpen = np.ones((2, 2), np.uint8)
        windowErode = np.ones((2, 2), np.uint8)

        pupilFrame = cv2.equalizeHist(frame[int(self.y+(self.h*.25)):(self.y+self.h), self.x:(self.x+self.w)])
        self.eye_frame=pupilFrame
        ret, pupilFrame = cv2.threshold(pupilFrame, 55, 255, cv2.THRESH_BINARY)
        #사진의 잡음 제거 과정

        pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_CLOSE, windowClose)
        pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_ERODE, windowErode)
        pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_OPEN, windowOpen)

        threshold = cv2.inRange(pupilFrame, 250, 255)

        #등고선 작업으로 눈의 중심을 찾기
        _, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) >= 2:
            maxArea = 0
            MAindex = 0
            distanceX=[]
            currentIndex = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                center = cv2.moments(cnt)
                cx, cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
                distanceX.append(cx)
                if area > maxArea:
                    maxArea = area
                    MAindex = currentIndex
                currentIndex = currentIndex+1
            del contours[MAindex]
            del distanceX[MAindex]

        if len(contours) >=1:
            maxArea = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area>maxArea:
                    maxArea= area
                    largeBlob = cnt

        if len(largeBlob) > 0:
            center = cv2.moments(largeBlob)
            cx, cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
        self.center_x=cx+self.x
        self.center_y=cy+self.y+int(self.h*.25)

    def ShowLocation(self):
        return [self.x, self.y, self.w, self.h]

class eyes(object):
    def ShowInfo(self, left_eye, right_eye):
        return [left_eye.ShowLocation, right_eye.ShowLocation]

    def Checking(self, left_eye, right_eye): #값이 비정상적이게 나왔는지 판단하기 위해서
        return True

    def FindingCenterOfEyes(self, left_eye, right_eye, frame):
        left_eye.FindingCenterOfEye(frame)
        right_eye.FindingCenterOfEye(frame)

    def CalculateBetweenCameraAndEyes(self, left_eye, right_eye):
        global PD, FOCAL_LENGTH
        return (PD*FOCAL_LENGTH)/Distance([left_eye.center_x, left_eye.center_y], [right_eye.center_x, right_eye.center_y])

def main():
    img = np.zeros((250, 450 , 3), np.uint8)
    global left_eye_classifier, right_eye_classifier
    while (cap.isOpened()):
        ret, frame = cap.read()
        cv2.imshow("frame", frame)
        if ret == True:
            k=cv2.waitKey(1)
            if k==27:
                break
            try:
                frame_its_me=frame
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                left_eye=eye(left_eye_classifier)
                right_eye = eye(right_eye_classifier)
                if(not(left_eye.FindingLocation(frame) and right_eye.FindingLocation(frame))):
                    continue
                optic = eyes()
                if(optic.Checking(left_eye, right_eye)):
                    optic.FindingCenterOfEyes(left_eye, right_eye, frame)
                    #print(optic.CalculateBetweenCameraAndEyes(left_eye, right_eye)+'cm')
                    cv2.circle(frame_its_me, (left_eye.center_x, left_eye.center_y), 5, 255, -1)
                    cv2.circle(frame_its_me, (right_eye.center_x, right_eye.center_y), 5, 255, -1)
                    print(eyes.CalculateBetweenCameraAndEyes(eyes, left_eye, right_eye))
                else:
                    continue
            except Exception as e:
                continue

            cv2.imshow("frame", frame)
            cv2.imshow("its me", frame_its_me)
            cv2.imshow("left_eye", left_eye.eye_frame)
            cv2.imshow("right_eye", right_eye.eye_frame)


    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
