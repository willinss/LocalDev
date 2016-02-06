#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <Eigen/Dense>
#include <sys/time.h>
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
	double deltaT = 0.06;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);
	timeval starttime,curtime;
    MatrixXd A_x(3,3);
    A_x << 
	1, 0.7, 0.3,
	0, 1, 0.6,
	0, 0, 1;    
    VectorXd predict_x_x(3);//state
    predict_x_x(0) = winWidth >> 1;
	predict_x_x(1) = 0;
	predict_x_x(2) = 0;
    VectorXd optimize_x_x(3);
    RowVectorXd H_x(3); //observation
    H_x(0) = 1;
	H_x(1) = 0;
	H_x(2) = 0;
    MatrixXd Q_x(3,3); //process_noise
    Q_x(0,0) = Q_x(1,1) = Q_x(2,2) = 0.001;
    double R_x; //measure_noise
    R_x = 0.01;
    
    MatrixXd P_x(3,3); //cov
    double S_x; //temp matrix
    VectorXd K_x(3); //
    double z_x; //measure value
    double y_x;
    MatrixXd I_x(3,3);
    I_x(0,0) = I_x(1,1) = I_x(2,2) = 1;

    MatrixXd A_y(3,3);
    A_y << 
	1, 0.7, 0.3,
	0, 1, 0.6,
	0, 0, 1;   
    VectorXd predict_x_y(3);//state
    predict_x_y(0) = winWidth >> 1;
	predict_x_y(1) = 0;
	predict_x_y(2) = 0;
    VectorXd optimize_x_y(3);
    RowVectorXd H_y(3); //observation
    H_y(0) = 1;
	H_y(1) = 0;
	H_y(2) = 0;
    MatrixXd Q_y(3,3); //process_noise
    Q_y(0,0) = Q_y(1,1) = Q_y(2,2) = 0.001;
    double R_y; //measure_noise
    R_y = 0.01;
    
    MatrixXd P_y(3,3); //cov
    double S_y; //temp matrix
    VectorXd K_y(3); //
    double z_y; //measure value
    double y_y;
    MatrixXd I_y(3,3);
    I_y(0,0) = I_y(1,1) = I_y(2,2) = 1;
	
	gettimeofday(&starttime,NULL);

    while(1)
    {
		gettimeofday(&curtime,NULL);
		printf("%4.4f\n",curtime.tv_sec - starttime.tv_sec + (curtime.tv_usec - starttime.tv_usec) / 1000000.0);
		starttime = curtime;
        cvSet(img, cvScalar(255, 255, 255, 0));
        cvCircle(img, mousePosition, 5, CV_RGB(255, 0, 0), 3);//current position with red
        char buf[256];
        snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
        cvPutText(img, buf, cvPoint(10, 60), &font, CV_RGB(0, 0, 0));

        optimize_x_x = A_x * predict_x_x;
        P_x = A_x * P_x * A_x.transpose() + Q_x;

        S_x = (H_x * P_x * H_x.transpose())(0,0) + R_x;
        K_x = P_x * H_x.transpose() / S_x;
        z_x = mousePosition.x;
        y_x = z_x - (H_x * optimize_x_x)(0,0);
        predict_x_x = optimize_x_x + K_x * y_x;
        P_x = (I_x - K_x * H_x) * P_x;


        optimize_x_y = A_y * predict_x_y;
        P_y = A_y * P_y * A_y.transpose() + Q_y;

        S_y = (H_y * P_y * H_y.transpose())(0,0) + R_y;
        K_y = P_y * H_y.transpose() / S_y;
        z_y = mousePosition.y;
        y_y = z_y - (H_y * optimize_x_y)(0,0);
        predict_x_y = optimize_x_y + K_y * y_y;
        P_y = (I_y - K_y * H_y) * P_y;

        CvPoint predictpos = cvPoint((int)predict_x_x(0),(int)predict_x_y(0));
        snprintf(buf, 256, "predicted position:(%3d,%3d)", predictpos.x, predictpos.y);
        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));
        cvCircle(img, predictpos , 5, CV_RGB(0, 0, 255), 3);
        cvShowImage("kalman", img);
        int key = cvWaitKey(1000*deltaT);
        if (key == 27)
            break;
    }
    return 0;
}
