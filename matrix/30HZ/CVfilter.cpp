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

int main(int argc, char* argv[])
{
	if(argc != 2) 
	{	
		cerr << "No such file\n";
		return -1;
	}

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
	ofstream fout_CV;
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
	fout_CV.open(fileout, ios::out);	
/*********Combined CV**********/
    Matrix4d A_CV;
    A_CV << 1, 0, 1, 0,
        	0, 1, 0, 1,
        	0, 0, 1, 0,
        	0, 0, 0, 1;
    
    Vector4d predict_x_CV;
	predict_x_CV(0) = winWidth >> 1;
	predict_x_CV(1) = winHeight >> 1;
    Vector4d optimize_x_CV;
    MatrixXd H_CV(2,4); //observation
    H_CV << 1, 0, 0, 0,
       		0, 1, 0, 0;
    Matrix4d Q_CV; //process_noise
    Q_CV(0,0) = Q_CV(1,1) = Q_CV(2,2) = Q_CV(3,3) = 0.0001;
    Matrix2d R_CV; //measure_noise
    R_CV(0,0) = R_CV(1,1) = 0.01;
    
    Matrix4d P_CV = MatrixXd::Identity(4,4); //cov
    Matrix2d S_CV; //temp matrix
    MatrixXd K_CV(4,2); //
    Vector2d z_CV; //measure value
    Vector2d y_CV;
    Matrix4d I_CV = MatrixXd::Identity(4,4);

	gettimeofday(&starttime,NULL);

//    while(1)
	double sum_x = 0, sum_y = 0;
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

		if(i > 0)
		{
			sum_x += (predict_x_CV(0) - measurement.x)*(predict_x_CV(0) - measurement.x);
			sum_y += (predict_x_CV(1) - measurement.y)*(predict_x_CV(1) - measurement.y);
		}
		if(!run_f)
		{

/*********Combined CV**********/
		    z_CV(0) = measurement.x;
		    z_CV(1) = measurement.y;
		    optimize_x_CV = A_CV * predict_x_CV;
		    P_CV = A_CV * P_CV * A_CV.transpose() + Q_CV;
		    y_CV = z_CV - H_CV * optimize_x_CV;
		    S_CV = H_CV * P_CV * H_CV.transpose() + R_CV;
		    K_CV = P_CV * H_CV.transpose() * S_CV.inverse();
		    predict_x_CV = optimize_x_CV + K_CV * y_CV;
//			cout << predict_x_CV << endl << endl;
		    P_CV = (I_CV - K_CV * H_CV) * P_CV;
			fout_CV << (int)predict_x_CV(0) << " " << (int)predict_x_CV(1) << endl;
		}


/*********************
		CvPoint predictpos_CV = cvPoint((int)predict_x_CV(0),(int)predict_x_CV(1));

        snprintf(buf, 256, "CV:(%3d,%3d)", predictpos_CV.x, predictpos_CV.y);
        cvPutText(img, buf, cvPoint(10, 50), &font, CV_RGB(0, 0, 0));

		cvCircle(img, predictpos_CV , 5, CV_RGB(0, 255, 0), 3);

        cvShowImage("kalman", img);
        int key = cvWaitKey(3000);
        if (key == 27)
            break;
/********************/
    }
	f_RMSE_x << sqrt(sum_x / (p_x.size() - 1)) << " ";
	f_RMSE_y << sqrt(sum_y / (p_y.size() - 1)) << " ";

	fin.close();
	fout_CV.close();
	f_RMSE_x.close();
	f_RMSE_y.close();
    return 0;
}
