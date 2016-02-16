#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cxcore.hpp>
#include <opencv/cv.hpp>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace cv;
const int winHeight = 300;
const int winWidth = 400;
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

    MatrixXd A_x(2,2);
    A_x << 1, 0.55,
		   0, 1;   
    VectorXd predict_x_x(2);//state
    predict_x_x(0) = winWidth >> 1;
	predict_x_x(1) = 0;
    VectorXd optimize_x_x(2);
    RowVector2d H_x; //observation
    H_x(0) = 1;
	H_x(1) = 0;
    MatrixXd Q_x(2,2); //process_noise
    Q_x(0,0) = Q_x(1,1) = 1e-2;
    double R_x; //measure_noise
    R_x = 1e-1;
    
    MatrixXd P_x(2,2); //cov
    double S_x; //temp matrix
    Vector2d K_x; //
    double z_x; //measure value
    double y_x;
    double I_x;
    I_x = 1;
	

	MatrixXd A_y(2,2);
    A_y << 1, 0.55,
		   0, 1;   
    VectorXd predict_x_y(2);//state
    predict_x_y(0) = winHeight >> 1;
	predict_x_y(1) = 0;
    VectorXd optimize_x_y(2);
    RowVector2d H_y; //observation
    H_y(0) = 1;
	H_y(1) = 0;
    MatrixXd Q_y(2,2); //process_noise
    Q_y(0,0) = Q_y(1,1) = 1e-2;
    double R_y; //measure_noise
    R_y = 1e-1;
    
    MatrixXd P_y(2,2); //cov
    double S_y; //temp matrix
    Vector2d K_y; //
    double z_y; //measure value
    double y_y;
    double I_y;
    I_y = 1;


	
    while(1)
    {
        cvSet(img, cvScalar(255, 255, 255, 0));
        cvCircle(img, mousePosition, 5, CV_RGB(255, 0, 0), 3);//current position with red
        char buf[256];
        snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
        cvPutText(img, buf, cvPoint(10, 60), &font, CV_RGB(0, 0, 0));
        z_x = mousePosition.x;
        optimize_x_x = A_x * predict_x_x;
        P_x = A_x * P_x * A_x.transpose() + Q_x;
        y_x = z_x - (H_x * optimize_x_x)(0,0);
        S_x = (H_x * P_x * H_x.transpose())(0,0) + R_x;
        K_x = P_x * H_x.transpose()/ S_x;
        predict_x_x = optimize_x_x + K_x * y_x;
        P_x = (I_x - (K_x * H_x)(0,0)) * P_x;

        z_y = mousePosition.y;
        optimize_x_y = A_y * predict_x_y;
        P_y = A_y * P_y * A_y.transpose() + Q_y;
        y_y = z_y - (H_y * optimize_x_y)(0,0);
        S_y = (H_y * P_y * H_y.transpose())(0,0) + R_y;
        K_y = P_y * H_y.transpose()/ S_y;
        predict_x_y = optimize_x_y + K_y * y_y;
        P_y = (I_y - (K_y * H_y)(0,0)) * P_y;

        CvPoint predictpos = cvPoint((int)predict_x_x(0),(int)predict_x_y(0));
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
