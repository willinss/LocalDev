#include <iostream>
#include <fstream>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <Eigen/Dense>
#include <sys/time.h>
#include <stdio.h>
#include <vector>
using namespace std;
using namespace Eigen;
using namespace cv;
const int winHeight = 300;
const int winWidth = 400;
bool run_f = false;

CvPoint mousePosition = cvPoint(winWidth >> 1, winHeight >> 1);
void mouseEvent(int event, int x, int y, int flags, void *param)
 {
    if (event == CV_EVENT_MOUSEMOVE)
	{
        mousePosition = cvPoint(x, y);
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		run_f = !run_f;
	}
}

int main()
{

	CvPoint measurement;
	double deltaT = 0.030;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);
	timeval starttime,curtime;
	ifstream fin;
	ofstream fout_MCA, fout_MCV, fout_CV, fout_CA;
	ofstream f_RMSE_x,f_RMSE_y;
	f_RMSE_x.open("RMSE_x.txt", ios::app);
	f_RMSE_y.open("RMSE_y.txt", ios::app);
/**********read data***********/
	fin.open("1604data.dat",ios::in);
	vector<int> p_x;
	vector<int> p_y;
	while(fin >> measurement.x >> measurement.y)
	{
		p_x.push_back(measurement.x);
		p_y.push_back(measurement.y);
	}

	fout_MCV.open("1604MCV.dat", ios::out);	

/*********Modified CV**********/	
    MatrixXd A_x_MCV(2,2);
    A_x_MCV << 1, 0.55,
		   	   0, 1;   
    VectorXd predict_x_x_MCV(2);//state
    predict_x_x_MCV(0) = winWidth >> 1;
	predict_x_x_MCV(1) = 0;
    VectorXd optimize_x_x_MCV(2);
    RowVector2d H_x_MCV; //observation
    H_x_MCV(0) = 1;
	H_x_MCV(1) = 0;
    MatrixXd Q_x_MCV(2,2); //process_noise
    Q_x_MCV(0,0) = Q_x_MCV(1,1) = 1e-4;
    double R_x_MCV; //measure_noise
    R_x_MCV = 1e-2;
    
    MatrixXd P_x_MCV(2,2); //cov
    double S_x_MCV; //temp matrix
    Vector2d K_x_MCV; //
    double z_x_MCV; //measure value
    double y_x_MCV;
    MatrixXd I_x_MCV(2,2);
    I_x_MCV(0,0) = I_x_MCV(1,1) = 1;	

	MatrixXd A_y_MCV(2,2);
    A_y_MCV << 1, 0.55,
			   0, 1;   
    VectorXd predict_x_y_MCV(2);//state
    predict_x_y_MCV(0) = winHeight >> 1;
	predict_x_y_MCV(1) = 0;
    VectorXd optimize_x_y_MCV(2);
    RowVector2d H_y_MCV; //observation
    H_y_MCV(0) = 1;
	H_y_MCV(1) = 0;
    MatrixXd Q_y_MCV(2,2); //process_noise
    Q_y_MCV(0,0) = Q_y_MCV(1,1) = 1e-4;
    double R_y_MCV; //measure_noise
    R_y_MCV = 1e-2;
    
    MatrixXd P_y_MCV(2,2); //cov
    double S_y_MCV; //temp matrix
    Vector2d K_y_MCV; //
    double z_y_MCV; //measure value
    double y_y_MCV;
    MatrixXd I_y_MCV(2,2);
    I_y_MCV(0,0) = I_y_MCV(1,1) = 1;

	gettimeofday(&starttime,NULL);

//    while(1)
	for(int i = 0;i < p_x.size(); i++)
    {
		measurement.x = p_x[i];
		measurement.y = p_y[i];
/*
		gettimeofday(&curtime,NULL);
		starttime = curtime;
        cvSet(img, cvScalar(255, 255, 255, 0));
//      cvCircle(img, mousePosition, 5, CV_RGB(0, 0, 0), 3);//current position with red
        cvCircle(img, measurement, 5, CV_RGB(0, 0, 0), 3);//current position with red
//		measurement = mousePosition;
        char buf[256];
        snprintf(buf, 256, "current position :(%3d,%3d)", measurement.x, measurement.y);
        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));
*/

		if(!run_f)
		{
/*********Modified CV**********/
        	optimize_x_x_MCV = A_x_MCV * predict_x_x_MCV;
        	P_x_MCV = A_x_MCV * P_x_MCV * A_x_MCV.transpose() + Q_x_MCV;

        	S_x_MCV = (H_x_MCV * P_x_MCV * H_x_MCV.transpose())(0,0) + R_x_MCV;
        	K_x_MCV = P_x_MCV * H_x_MCV.transpose() / S_x_MCV;
        	z_x_MCV = measurement.x;
        	y_x_MCV = z_x_MCV - (H_x_MCV * optimize_x_x_MCV)(0,0);
        	predict_x_x_MCV = optimize_x_x_MCV + K_x_MCV * y_x_MCV;
        	P_x_MCV = (I_x_MCV - K_x_MCV * H_x_MCV) * P_x_MCV;

        	optimize_x_y_MCV = A_y_MCV * predict_x_y_MCV;
        	P_y_MCV = A_y_MCV * P_y_MCV * A_y_MCV.transpose() + Q_y_MCV;

		    S_y_MCV = (H_y_MCV * P_y_MCV * H_y_MCV.transpose())(0,0) + R_y_MCV;
		    K_y_MCV = P_y_MCV * H_y_MCV.transpose() / S_y_MCV;
		    z_y_MCV = measurement.y;
		    y_y_MCV = z_y_MCV - (H_y_MCV * optimize_x_y_MCV)(0,0);
		    predict_x_y_MCV = optimize_x_y_MCV + K_y_MCV * y_y_MCV;
		    P_y_MCV = (I_y_MCV - K_y_MCV * H_y_MCV) * P_y_MCV;
			
			fout_MCV << (int)predict_x_x_MCV(0) << " " << (int)predict_x_y_MCV(0) << endl;
		}

/*********************
		CvPoint predictpos_MCV = cvPoint((int)predict_x_x_MCV(0),(int)predict_x_y_MCV(0));

        snprintf(buf, 256, "MCV:(%3f,%3f)", predict_x_x_MCV(0), predict_x_y_MCV(0));
        cvPutText(img, buf, cvPoint(10, 70), &font, CV_RGB(0, 0, 0));

		cvCircle(img, predictpos_MCV , 5, CV_RGB(0, 134, 139), 3);
        cvShowImage("kalman", img);

        int key = cvWaitKey(30);
        if (key == 27)
            break;
/********************/
    }
    return 0;
}
