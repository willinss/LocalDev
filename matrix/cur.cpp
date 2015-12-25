
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cxcore.hpp>
#include <opencv/cv.hpp>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
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
int main()
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);

    MatrixXd A(6,6);
    A << 1, 0, 1, 0, 1, 0,
         0, 1, 0, 1, 0, 1,
         0, 0, 1, 0, 1, 0,
         0, 0, 0, 1, 0, 1,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    
    VectorXd predict_x(6);//state
    predict_x(0) = winWidth >> 1;
    predict_x(1) = winHeight >> 1;
    VectorXd optimize_x(6);
    MatrixXd H(2,6); //observationI
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0;
    MatrixXd Q(6,6); //process_noise
    Q(0,0)=Q(1,1)=Q(2,2)=Q(3,3)=Q(4,4)=Q(5,5)=1e-3;
    Matrix2d R; //measure_noise
    R(0,0)=R(1,1)=1e-1;
    
    MatrixXd P(6,6); //cov
    Matrix2d S; //temp matrix
    MatrixXd K(6,2); //
    Vector2d z; //measure value
    Vector2d y;
    MatrixXd I(6,6);
    I = MatrixXd::Identity(6,6);

    while(1)
    {
        cvSet(img, cvScalar(255, 255, 255, 0));
        cvCircle(img, mousePosition, 5, CV_RGB(255, 0, 0), 3);//current position with red
        char buf[256];
        snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
        cvPutText(img, buf, cvPoint(10, 60), &font, CV_RGB(0, 0, 0));
        z(0) = mousePosition.x;
        z(1) = mousePosition.y;
        optimize_x = A * predict_x;
        P = A * P * A.transpose() + Q;
        y = z - H * optimize_x;
        S = H * P * H.transpose() + R;
        
        K = P * H.transpose() * S.inverse();
        predict_x = optimize_x + K * y;
        //cout << predict_x << endl;
        P = (I - K * H) * P;

        CvPoint predictpos = cvPoint((int)predict_x(0),(int)predict_x(1));
        snprintf(buf, 256, "predicted position:(%3d,%3d)", predictpos.x, predictpos.y);
        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));
        cvCircle(img, predictpos , 5, CV_RGB(0, 0, 255), 3);
        cvShowImage("kalman", img);
        int key = cvWaitKey(30);
        if (key == 27)
            break;
    }
    return 0;
}
