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
FILE *fp = NULL;

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

int main(int argc, char* argv[])
{

	CvPoint measurement;
	double deltaT = 0.030;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);
	timeval starttime,curtime;
/**********open file***********/
	ifstream fin;
	ofstream fout_CA;
	ofstream f_RMSE_x,f_RMSE_y;
	f_RMSE_x.open("RMSE_x.txt", ios::app);
	f_RMSE_y.open("RMSE_y.txt", ios::app);

	fin.open(argv[1], ios::in);
	vector<int> p_x;
	vector<int> p_y;
	while(fin >> measurement.x >> measurement.y)
	{
		p_x.push_back(measurement.x);
		p_y.push_back(measurement.y);
	}
	char fileout[50] ="CV";
	strcat(fileout,argv[1]);
	fout_CA.open(fileout, ios::out);	
/*********Combined CA**********/
    MatrixXd A_CA(6,6);
    A_CA << 1, 0, 0.7, 0, 0.2, 0,
	        0, 1, 0, 0.7, 0, 0.2,
       		0, 0, 1, 0, 0.5, 0,
	        0, 0, 0, 1, 0, 0.5,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1;
    
    VectorXd predict_x_CA(6);
    predict_x_CA(0) = winWidth >> 1;
 	predict_x_CA(1) = winHeight >> 1;
	predict_x_CA(2) = predict_x_CA(3) = predict_x_CA(4) = predict_x_CA(5) = 0;
    VectorXd optimize_x_CA(6);
    MatrixXd H_CA(2,6); //observation
    H_CA << 1, 0, 0, 0, 0, 0,
       		0, 1, 0, 0, 0, 0;
    MatrixXd Q_CA(6,6); //process_noise
    Q_CA(0,0) = Q_CA(1,1) = Q_CA(2,2) = Q_CA(3,3) = Q_CA(4,4) = Q_CA(5,5) = 1e-4;
    Matrix2d R_CA; //measure_noise
    R_CA(0,0) = R_CA(1,1) = 1e-2;
    
    MatrixXd P_CA = MatrixXd::Identity(6,6); //cov
    Matrix2d S_CA; //temp matrix
    MatrixXd K_CA(6,2); //
    Vector2d z_CA; //measure value
    Vector2d y_CA;
    MatrixXd I_CA = MatrixXd::Identity(6,6);

	gettimeofday(&starttime,NULL);


	double sum_x = 0, sum_y = 0;
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
/*********Combined CA**********/
		    z_CA(0) = measurement.x;
		    z_CA(1) = measurement.y;
		    optimize_x_CA = A_CA * predict_x_CA;
		    P_CA = A_CA * P_CA * A_CA.transpose() + Q_CA;
		    y_CA = z_CA - H_CA * optimize_x_CA;
		    S_CA = H_CA * P_CA * H_CA.transpose() + R_CA;
		    K_CA = P_CA * H_CA.transpose() * S_CA.inverse();
		    predict_x_CA = optimize_x_CA + K_CA * y_CA;
		    P_CA = (I_CA - K_CA * H_CA) * P_CA;
			fout_CA << (int)predict_x_CA(0) << " " << (int)predict_x_CA(1) << endl;
/********************************/

//			fprintf(fp, "%d,%d\n", measurement.x, measurement.y);
		}
/*********************
		CvPoint predictpos_CA = cvPoint((int)predict_x_CA(0),(int)predict_x_CA(1));

        snprintf(buf, 256, "CA:(%3f,%3f)", predict_x_CA(0), predict_x_CA(1));
        cvPutText(img, buf, cvPoint(10, 90), &font, CV_RGB(0, 0, 0));

		cvCircle(img, predictpos_CA , 5, CV_RGB(100, 100, 0), 3);

        cvShowImage("kalman", img);
        int key = cvWaitKey(30);
        if (key == 27)
            break;
/********************/
    }

	f_RMSE_x << sqrt(sum_x / (p_x.size() - 1)) << " ";
	f_RMSE_y << sqrt(sum_y / (p_y.size() - 1)) << " ";

	fin.close();
	fout_CA.close();
	f_RMSE_x.close();
	f_RMSE_y.close();
    return 0;
}
