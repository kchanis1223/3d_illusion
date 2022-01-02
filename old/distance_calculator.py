
import numpy as np
import cv2 as cv
import math as ma
a=[0, 0]
i=0
knownW=29.7
cap=cv.VideoCapture(0)

class Line(object):
    def __init__(self, coor1, coor2):
        self.coor1 = coor1
        self.coor2 = coor2

    def distance(self):
        x1, y1 = self.coor1
        x2, y2 = self.coor2

        return ma.sqrt(ma.pow((x2-x1), 2) + ma.pow((y2-y1), 2))

def draw_circle(event,x,y,flags,param):
    if event == cv.EVENT_LBUTTONDBLCLK:
        cv.circle(img,(x,y),3,(255,0,0),-1)
        global i
        a[i]=(x, y)
        i+=1

def calfocal(pixel, knownD, knownW):
    return (pixel*knownD)/knownW
        


cv.namedWindow('image')
cv.setMouseCallback('image',draw_circle)
img = None
while(1):
    ret, frame = cap.read()
    if cv.waitKey(1) & 0xFF == ord('c'):
        ret, img = cap.read()
    cv.imshow('frame',frame)
    if img is not None:
        cv.imshow('image', img)
    if i==2:
        coor1=(a[0][0], a[0][1])
        coor2=(a[1][0], a[1][1])
        line = Line(coor1, coor2)
        print(line.distance())
        print(calfocal(line.distance(), 100, knownW))
        i=0
    if cv.waitKey(20) & 0xFF == 27:
        break
cv.destroyAllWindows()
#first: 438.0
#second: 447.0100670007332
#thirst: 460.2445002387318
#4: 450.9988913511872
#5: 468.0 6: 478 7: 474.7952 8: 464.0820 9: 478.7878
#10: 464.7562 // 11: 479.9426 12:480.8138 13: 480.9419  14:469.8533  15:461.3286  16: 468.4492

#average=469.4610
