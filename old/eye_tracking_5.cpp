#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <Windows.h>

using namespace std;
using namespace cv;

#define FOCAL_LENGTH 469.4610
#define PD 6.2
#define CAM_FPS 30
#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define DOUBLE_THRESH 55 //40
#define CLIP_LIMIT 4 //4

void applyCLAHE(Mat& srcArry, Mat& strArry);
void defMorphology(Mat& _thresholded_frame);
bool checking(Rect left, Rect right);
float calculateBetweenCameraAndEyes(Point left, Point right);

class Eye           //눈 정보를 담기 위한 클래스 설정
{
private:
    Rect eye_pos;
    int center_x, center_y;
    Mat eye_frame;
    CascadeClassifier eye_cascade;

public:
    bool FindingLocation(Mat& frame);       //눈 위치를 찾기 위한 함수
    void FindingCenterOfEye();              //눈 위치 찾고 난 뒤 동공 중심을 찾는 함수
    Point ShowCenter() {
        Point pts(center_x, center_y);
        return pts;
    }
    Rect ShowLocation()
    {
        return eye_pos;
    }

    Eye(String haar_cascade_name)
    {
        if (!eye_cascade.load(haar_cascade_name)) {
            cout << "ERROR: error - loading cascade" << endl;
            return;
        }
    }
};

void applyCLAHE(Mat& srcArry, Mat& dstArry)
        //CLAHE기술 적용, srcArry가 대상 프레임, dstArry가 결과물
{
    Mat lab_image;
    //이미지 형식을 BGR에서 Lab으로 변환
    cvtColor(srcArry, lab_image, COLOR_BGR2Lab);

    vector<Mat> lab_planes(3);
    split(lab_image, lab_planes);

    //CLAHE 기술 적용
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(CLIP_LIMIT);    //CLIP_LIMIT = 4
    Mat dst;
    clahe->apply(lab_planes[0], dst);

    dst.copyTo(lab_planes[0]);
    merge(lab_planes, lab_image);
    cvtColor(lab_image, dstArry, COLOR_Lab2BGR);
}

void defMorphology(Mat& _thresholded_frame) // 닫힘 연산, 침식 연산, 열림 연산 순으로 모폴로지 연산 실행
{
    //모폴로지 연산을 위해 마스크 구성
    Mat element_Close(5, 5, CV_8U, Scalar(1));
    Mat element_Erode(2, 2, CV_8U, Scalar(1));
    Mat element_Open(2, 2, CV_8U, Scalar(1));

    //모폴로지 연산 실행
    morphologyEx(_thresholded_frame, _thresholded_frame, MORPH_CLOSE, element_Close);
    morphologyEx(_thresholded_frame, _thresholded_frame, MORPH_ERODE, element_Erode);
    morphologyEx(_thresholded_frame, _thresholded_frame, MORPH_OPEN, element_Open);
}

bool checking(Rect left, Rect right)
    //눈 위치 정보가 제대로 저장되었는지 확인하기 위한 함수
{
    if (right.x+right.width > left.x)
        return false;
    else
        return true;
}

float calculateBetweenCameraAndEyes(Point left, Point right)
    //카메라와 눈 사이의 거리를 구하는 함수
{
    float distance_eye;
    float distance_eye_and_camera;
    distance_eye = sqrt(pow(left.x - right.x, 2) + pow(left.y - right.y, 2));
    distance_eye_and_camera = (PD*FOCAL_LENGTH) / distance_eye;
    return distance_eye_and_camera;
}

bool Eye::FindingLocation(Mat& frame)
{
    vector<Rect> eyes;
    Mat dst_frame;
    Mat frame_gray;

    applyCLAHE(frame, dst_frame);
    cvtColor(dst_frame, frame_gray, COLOR_RGB2GRAY); //이미지를 그레이스케일로 변환
    eye_cascade.detectMultiScale(frame_gray, eyes, 1.3, 5, 0, Size(30, 30), Size(70, 60));
        //눈 위치 정보들을 Rect형 백터 eyes에 저장

    if (eyes.empty())
        return false;

    eye_pos = eyes[0];
    eye_frame = dst_frame(eye_pos);

    return true;
}

void Eye::FindingCenterOfEye()
{
    Mat pupilFrame;
    vector<vector<cv::Point>> contours;;
    vector<Vec4i> hierarchy;
    int maxArea, maxIndex, area;
    cvtColor(eye_frame, pupilFrame, COLOR_RGB2GRAY);

    //임계값을 DOUBLE_THRESH로 설정하여 이진화를 실시
    threshold(pupilFrame, pupilFrame, DOUBLE_THRESH, 255, THRESH_BINARY);
    defMorphology(pupilFrame);

    //이진화된 이미지를 기반으로 등고선을 찾기
    findContours(pupilFrame, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    //등고선에서 닫힌 도형의 영역이 두번째로 큰 도형의 중심을 찾기
    //그 중심이 동공의 중심점
    if ( contours.size() >= 2)
    {
        maxArea = 0, maxIndex = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            area = contourArea(contours[i]);
            if (area > maxArea)
            {
                maxArea = area;
                maxIndex = i;
            }
        }
        contours.erase(contours.begin()+maxIndex);      //가장 큰 영역의 등고선 지우기
    }

    if (contours.size() >= 1)
    {
        maxArea = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            area = contourArea(contours[i]);
            if (area > maxArea)
            {
                maxArea = area;
                maxIndex = i;
            }
        }
    }

    if (maxIndex >= 0)
    {
        //Moments를 이용하여 기하학적 방법으로 무게중심 찾기
        Moments moment = moments(contours[maxIndex]);
        center_x = eye_pos.x+int(moment.m10 / moment.m00);
        center_y = eye_pos.y+int(moment.m01 / moment.m00);
    }
}

int main(int argc, const char ** argv)
{
    VideoCapture cam(0); //웹캠 열기
    Mat frame;

    String left_eye_cascade_name = "haarcascade_lefteye_2splits.xml";
    String right_eye_cascade_name = "haarcascade_righteye_2splits.xml";

    if (!cam.isOpened())
    {
        cout << "ERROR: can't open the cam" << endl;
        return -1;
    }

    //출력 비디오 설정 fps: 30, 640x480
    cam.set(CAP_PROP_FPS, CAM_FPS);
    cam.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    cam.set(CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);

    Eye left_eye(left_eye_cascade_name);
    Eye right_eye(right_eye_cascade_name);


    while(cam.read(frame))
    {
        if (frame.empty())
        {
            cout << "ERROR: can't grap frame from cam" << endl;
            return -1;
        }

        if (left_eye.FindingLocation(frame) && right_eye.FindingLocation(frame))
        {

            if (checking(left_eye.ShowLocation(), right_eye.ShowLocation()))
            {
                left_eye.FindingCenterOfEye();
                right_eye.FindingCenterOfEye();
                cout << float(calculateBetweenCameraAndEyes(left_eye.ShowCenter(), right_eye.ShowCenter())) << endl;

            }
        }
        imshow("me", frame);
        char c = (char)waitKey(10);
        if (c == 27) { break; }
    }
    return 0;
}
