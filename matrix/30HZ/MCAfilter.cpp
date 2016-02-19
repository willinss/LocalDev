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
	ofstream fout_MCA;
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

	fout_MCA.open("1604MCA.dat", ios::out);		

/*********modified CA**********/
    MatrixXd A_x_MCA(3,3);
    A_x_MCA << 1, 0.6, 0.3,
		   	   0, 1, 0.55,
			   0, 0, 1;    
    VectorXd predict_x_x_MCA(3);//state
    predict_x_x_MCA(0) = winWidth >> 1;
	predict_x_x_MCA(1) = 0;
	predict_x_x_MCA(2) = 0;
    VectorXd optimize_x_x_MCA(3);
    RowVectorXd H_x_MCA(3); //observation
    H_x_MCA(0) = 1;
	H_x_MCA(1) = 0;
	H_x_MCA(2) = 0;
    MatrixXd Q_x_MCA(3,3); //process_noise
    Q_x_MCA(0,0) = Q_x_MCA(1,1) = Q_x_MCA(2,2) = 0.001;
    double R_x_MCA; //measure_noise
    R_x_MCA = 0.01;
    
    MatrixXd P_x_MCA(3,3); //cov
    double S_x_MCA; //temp matrix
    VectorXd K_x_MCA(3); //
    double z_x_MCA; //measure value
    double y_x_MCA;
    MatrixXd I_x_MCA(3,3);
    I_x_MCA(0,0) = I_x_MCA(1,1) = I_x_MCA(2,2) = 1;

    MatrixXd A_y_MCA(3,3);
    A_y_MCA << 1, 0.6, 0.3,
		   	   0, 1, 0.55,
		       0, 0, 1;   
    VectorXd predict_x_y_MCA(3);//state
    predict_x_y_MCA(0) = winHeight >> 1;
	predict_x_y_MCA(1) = 0;
	predict_x_y_MCA(2) = 0;
    VectorXd optimize_x_y_MCA(3);
    RowVectorXd H_y_MCA(3); //observation
    H_y_MCA(0) = 1;
	H_y_MCA(1) = 0;
	H_y_MCA(2) = 0;
    MatrixXd Q_y_MCA(3,3); //process_noise
    Q_y_MCA(0,0) = Q_y_MCA(1,1) = Q_y_MCA(2,2) = 0.0001;
    double R_y_MCA; //measure_noise
    R_y_MCA = 0.01;
    
    MatrixXd P_y_MCA(3,3); //cov
    double S_y_MCA; //temp matrix
    VectorXd K_y_MCA(3); //
    double z_y_MCA; //measure value
    double y_y_MCA;
    MatrixXd I_y_MCA(3,3);
    I_y_MCA(0,0) = I_y_MCA(1,1) = I_y_MCA(2,2) = 1;

	gettimeofday(&starttime,NULL);


	for(int i = 0;i < p_x.size(); i++)
    {
		measurement.x = p_x[i];
		measurement.y = p_y[i];
/*
		gettimeofday(&curtime,NULL);
//		printf("%4.4f\n",curtime.tv_sec - starttime.tv_sec + (curtime.tv_usec - starttime.tv_usec) / 1000000.0);
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
/*********modified CA**********/
        	optimize_x_x_MCA = A_x_MCA * predict_x_x_MCA;
        	P_x_MCA = A_x_MCA * P_x_MCA * A_x_MCA.transpose() + Q_x_MCA;

        	S_x_MCA = (H_x_MCA * P_x_MCA * H_x_MCA.transpose())(0,0) + R_x_MCA;
        	K_x_MCA = P_x_MCA * H_x_MCA.transpose() / S_x_MCA;
        	z_x_MCA = measurement.x;
        	y_x_MCA = z_x_MCA - (H_x_MCA * optimize_x_x_MCA)(0,0);
        	predict_x_x_MCA = optimize_x_x_MCA + K_x_MCA * y_x_MCA;
        	P_x_MCA = (I_x_MCA - K_x_MCA * H_x_MCA) * P_x_MCA;

        	optimize_x_y_MCA = A_y_MCA * predict_x_y_MCA;
        	P_y_MCA = A_y_MCA * P_y_MCA * A_y_MCA.transpose() + Q_y_MCA;

        	S_y_MCA = (H_y_MCA * P_y_MCA * H_y_MCA.transpose())(0,0) + R_y_MCA;
        	K_y_MCA = P_y_MCA * H_y_MCA.transpose() / S_y_MCA;
        	z_y_MCA = measurement.y;
        	y_y_MCA = z_y_MCA - (H_y_MCA * optimize_x_y_MCA)(0,0);
        	predict_x_y_MCA = optimize_x_y_MCA + K_y_MCA * y_y_MCA;
        	P_y_MCA = (I_y_MCA - K_y_MCA * H_y_MCA) * P_y_MCA;
	
			fout_MCA << (int)predict_x_x_MCA(0) << " " << (int)predict_x_y_MCA(0) << endl;
		}
/*********************
        CvPoint predictpos_MCA = cvPoint((int)predict_x_x_MCA(0),(int)predict_x_y_MCA(0));
        snprintf(buf, 256, "MCA:(%3f,%3f)", predict_x_x_MCA(0), predict_x_y_MCA(0));
        cvPutText(img, buf, cvPoint(10, 50), &font, CV_RGB(0, 0, 0));

        cvCircle(img, predictpos_MCA , 5, CV_RGB(0, 0, 255), 3);
        cvShowImage("kalman", img);

        int key = cvWaitKey(30);
        if (key == 27)
            break;
/********************/
    }
    return 0;
}
