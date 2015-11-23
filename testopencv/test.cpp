/*
#include <stdio.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "lib1.h"
#include "lib2.h"
using namespace cv;
using namespace std;

int main(int argc,char* argv[])
{
    CvCapture *capture;
    IplImage *frame;
    capture=cvCreateCameraCapture(0);
    cvNamedWindow("Webcam",1);
    Matrix3d m = Matrix3d::Random();
    PrintMatrix(m);
    Matrix3d n = Matrix3d::Random();
    PrintMatrix(n);
    MatrixMultiply(m,n);
    
    while(true)
    {
        frame = cvQueryFrame(capture);
        if(!frame)break;
        cvShowImage("Webcam",frame);
        if(cvWaitKey(20)>0)
            break;
    }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Webcam");
    return 0;
}
*/

#include <opencv/cv.hpp>
#include <opencv/cxcore.hpp>
#include <opencv/highgui.h>
#include <cmath>
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;
const int winHeight = 600;
const int winWidth = 800;
CvPoint mousePosition = cvPoint(winWidth >> 1, winHeight >> 1);
void mouseEvent(int event, int x, int y, int flags, void *param)
{
    if (event == CV_EVENT_MOUSEMOVE) 
    {
        mousePosition = cvPoint(x, y);
    }
}

int main(void)
{
    const int stateNum = 4;
    const int measureNum = 2;
    CvKalman* kalman = cvCreateKalman(stateNum, measureNum, 0);//state(x,y,detaX,detaY)
    CvMat* process_noise = cvCreateMat(stateNum, 1, CV_32FC1);
    CvMat* measurement = cvCreateMat(measureNum, 1, CV_32FC1);//measurement(x,y)
    CvRNG rng = cvRNG(-1);
    float A[stateNum][stateNum] = 
    {//transition matrix
        {1, 0, 1, 0},
        {0, 1, 0, 1},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    memcpy(kalman->transition_matrix->data.fl, A, sizeof(A));
    cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1));
    cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-5));
    cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(1e-1));
    cvSetIdentity(kalman->error_cov_post, cvRealScalar(1));
    cvRandArr(&rng, kalman->state_post, CV_RAND_UNI, cvRealScalar(0), cvRealScalar(winHeight>winWidth ? winWidth : winHeight));
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);
    while (1){
        const CvMat* prediction = cvKalmanPredict(kalman, 0);
        CvPoint predict_pt = cvPoint((int)prediction->data.fl[0], (int)prediction->data.fl[1]);
        measurement->data.fl[0] = (float)mousePosition.x;
        measurement->data.fl[1] = (float)mousePosition.y;
        cvKalmanCorrect(kalman, measurement);
        cvSet(img, cvScalar(255, 255, 255, 0));
        cvCircle(img, predict_pt, 5, CV_RGB(0, 255, 0), 3);//predicted point with green
        cvCircle(img, mousePosition, 5, CV_RGB(255, 0, 0), 3);//current position with red
        char buf[256];
        snprintf(buf, 256, "predicted position:(%3d,%3d)", predict_pt.x, predict_pt.y);
        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));
        snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
        cvPutText(img, buf, cvPoint(10, 60), &font, CV_RGB(0, 0, 0));
        cvShowImage("kalman", img);
        int key = cvWaitKey(3);
        if (key == 27){  
            break;
        }
    }
    cvReleaseImage(&img);
    cvReleaseKalman(&kalman);
    return 0;
}
