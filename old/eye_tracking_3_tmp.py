import numpy as np
import cv2
import time
import math as ma

focal_length=469.4610
pd=6.2
#width=420
#height=320
left_eye_classifier = cv2.CascadeClassifier('haarcascade_lefteye_2splits.xml')
right_eye_classifier = cv2.CascadeClassifier('haarcascade_righteye_2splits.xml')
frame=NULL

def Distance([x1, y1], [x2, y2])	
	return float(ma.sqart(ma.pow((x2-x1), 2) + ma.pow((y2-y1), 2)))

class eye(object):
	"""docstring for eye"""
	def __init__(self, classifier):
		x, y, w, h =[0, 0, 0, 0]
		center_x, center_y= 0,0
	def FindingLocation(self):
		x, y, w, h=lassifier.detectMultiScale(frame, 1.3, 5)
	def ShowLocation(self):
		return [x, y, w, h]

		
		
class eyes(object):
	"""docstring for eyes"""
	def __init__(self, left_eye, right_eye):
		self.left_eye=left_eye
		self.right_eye = right_eye

	def ShowInfo(self):
		return [left_eye.ShowLocation, right_eye.ShowLocation]

	def Checking(self): #값이 비정상적이게 나왔는지 판단하기 위해서
		return True

	def FindingCenterOfEyes(): #두 눈 중심을 찾아 eye 클래스에 저장한다.
		eyes = [right_eye, left_eye]
		eyes_center=[]

		windowClose = np.ones((5, 5), np.uint8)
		windowOpen = np.ones((2, 2), np.uint8)
		windowErode = np.ones((2, 2), np.uint8)

		for eye in eyes:
			x, y, w, h = eye.ShowLocation()
			pupilFrame = cv2.equalizeHist(frame[int(y+(h*.25)): (y+h), x:(x+w)]) # 히스트로그램 균일화로 좀 더 잘 보이게 한다.
			eye.frame = pupilFrame
			ret, pupilFrame = cv2.threshold(pupilFrame, 55, 255, cv2.THRESH_BINARY) #frame을 하양색과 검은색으로 나눈다.
			#사진의 잡음 제거 과정
			pupilFrame = cv2.morphologyEX(pupilFrame, v2.MORPH_CLOSE, windowClose)
			pupilFrame = cv2.morphologyEX(pupilFrame, v2.MORPH_ERODE, windowErode)
			pupilFrame = cv2.morphologyEX(pupilFrame, v2.MORPH_OPEN, windowOpen)

			threshold = cv2.inRange(pupilFrame, 250, 255)

			#등고선 작업으로 눈의 중심을 찾기
			_, contours, hierarchy = cv2.findContours(threshold, cv2, RETR_LIST, cv2.CHAIN_APPRIX_SIMPLE)

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
				maxArea = o
				for cnt in contours:
					area = cv2.contourArea(cnt)
					if area>maxArea:
						maxArea= area
						largeBlob = cnt

			if len(largeBlob) > 0:
				center = cv2.moments(largeBlob)
				cx, cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
			eye.cx=cx+x, eye.cy=cy+y+int(h*.25)
	
	def CalculateBetweenCameraAndEyes:
		return (pd*focal_length)/Distance([left_eye.cx, left_eye.cy], [right_eye.cx, right_eye.cy])

def main():
	cap = cv2.VideoCapture(0)
	img = np.zeros((250, 450 , 3), np.uint8)
	left_eye=eye(left_eye_classifier)
	right_eye = eye(right_eye_classifier)
	eyes=(left_eye, right_eye)
	global frame
	while (cap.isOpened()):
		ret, frame = cap.read()
		if ret == True:
			try:
				frame_its_me=frame
				frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
				global left_eye_classifier, right_eye_classifier
				right_eye = eye.FindingLocation(left_eye_classifier, frame)
				left_eye = eye.FindingLocation(right_eye_classifier, frame)
				eyes=eyes(left_eye, right_eye, frame)
				if(eyes.Checking):
					eyes.FindingCenterOfEyes()
					print(eyes.CalculateBetweenCameraAndEyes()+'cm')
			except:
				continue
		cv2.imshow("its me", frame_its_me)
		cv2.imshow("left_eye", left_eye.frame)
		cv2.imshow("right_eye", right_eye.frame)
		if cv2.waitKey(1)& 0xFF == ord('q'):
			break
	cap.release()
	cv2.destroyAllWindows()




if __name__ == '__main__':
	main()
