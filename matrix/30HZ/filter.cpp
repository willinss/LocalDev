#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <Eigen/Dense>
#include <sys/time.h>
#include <stdio.h>
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
		if(run_f == true)
		{
			struct tm *newtime;
			time_t long_time;
			time(&long_time);
			char filename[50];
			newtime = localtime(&long_time);
			strftime(filename, sizeof(filename), "%H%Mdata.dat", newtime);
			if(fp = fopen(filename, "w+") == NULL)
				printf("file open error\n");
		}
		else
		{
			fclose(fp);
			fp = NULL;
		}
	}
}

int main()
{

	double deltaT = 0.030;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);
	timeval starttime,curtime;


/****record data****/

	

/*********modified CA**********/
    MatrixXd A_x_MCA(3,3);
    A_x_MCA << 1, 0.6, 0.3,
		   	   0, 1, 0.5,
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
		   	   0, 1, 0.5,
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
    Q_y_MCA(0,0) = Q_y_MCA(1,1) = Q_y_MCA(2,2) = 0.001;
    double R_y_MCA; //measure_noise
    R_y_MCA = 0.01;
    
    MatrixXd P_y_MCA(3,3); //cov
    double S_y_MCA; //temp matrix
    VectorXd K_y_MCA(3); //
    double z_y_MCA; //measure value
    double y_y_MCA;
    MatrixXd I_y_MCA(3,3);
    I_y_MCA(0,0) = I_y_MCA(1,1) = I_y_MCA(2,2) = 1;

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
    Q_x_MCV(0,0) = Q_x_MCV(1,1) = 1e-3;
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
    Q_y_MCV(0,0) = Q_y_MCV(1,1) = 1e-3;
    double R_y_MCV; //measure_noise
    R_y_MCV = 1e-2;
    
    MatrixXd P_y_MCV(2,2); //cov
    double S_y_MCV; //temp matrix
    Vector2d K_y_MCV; //
    double z_y_MCV; //measure value
    double y_y_MCV;
    MatrixXd I_y_MCV(2,2);
    I_y_MCV(0,0) = I_y_MCV(1,1) = 1;

/*********Combined CV**********/
    Matrix4d A_CV;
    A_CV << 1, 0, 1, 0,
        	0, 1, 0, 1,
        	0, 0, 1, 0,
        	0, 0, 0, 1;
    
    Vector4d predict_x_CV(winWidth >> 1, winHeight >> 1, 0, 0);//state
    Vector4d optimize_x_CV;
    MatrixXd H_CV(2,4); //observation
    H_CV << 1, 0, 0, 0,
       		0, 1, 0, 0;
    Matrix4d Q_CV; //process_noise
    Q_CV(0,0) = Q_CV(1,1) = Q_CV(2,2) = Q_CV(3,3) = 1e-4;
    Matrix2d R_CV; //measure_noise
    R_CV(0,0) = R_CV(1,1) = 1e-2;
    
    Matrix4d P_CV; //cov
    Matrix2d S_CV; //temp matrix
    MatrixXd K_CV(4,2); //
    Vector2d z_CV; //measure value
    Vector2d y_CV;
    Matrix4d I_CV = MatrixXd::Identity(4,4);

/*********Combined CA**********/
    MatrixXd A_CA(6,6);
    A_CA << 1, 0, 0.5, 0, 0.2, 0,
	        0, 1, 0, 0.5, 0, 0.2,
       		0, 0, 1, 0, 0.4, 0,
	        0, 0, 0, 1, 0, 0.4,
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
    Q_CA(0,0) = Q_CA(1,1) = Q_CA(2,2) = Q_CA(3,3) = Q_CA(4,4) = Q_CA(5,5) = 1e-3;
    Matrix2d R_CA; //measure_noise
    R_CA(0,0) = R_CA(1,1) = 1e-2;
    
    MatrixXd P_CA(6,6); //cov
    Matrix2d S_CA; //temp matrix
    MatrixXd K_CA(6,2); //
    Vector2d z_CA; //measure value
    Vector2d y_CA;
    MatrixXd I_CA = MatrixXd::Identity(6,6);





	gettimeofday(&starttime,NULL);

    while(1)
    {
		gettimeofday(&curtime,NULL);
//		printf("%4.4f\n",curtime.tv_sec - starttime.tv_sec + (curtime.tv_usec - starttime.tv_usec) / 1000000.0);
		starttime = curtime;
        cvSet(img, cvScalar(255, 255, 255, 0));
        cvCircle(img, mousePosition, 5, CV_RGB(0, 0, 0), 3);//current position with red

		CvPoint measurement = mousePosition;
        char buf[256];
        snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));


		if(run_f)
		{
/*********modified CA**********
        	optimize_x_x_MCA = A_x_MCA * predict_x_x_MCA;
        	P_x_MCA = A_x_MCA * P_x_MCA * A_x_MCA.transpose() + Q_x_MCA;

        	S_x_MCA = (H_x_MCA * P_x_MCA * H_x_MCA.transpose())(0,0) + R_x_MCA;
        	K_x_MCA = P_x_MCA * H_x_MCA.transpose() / S_x_MCA;
        	z_x_MCA = mousePosition.x;
        	y_x_MCA = z_x_MCA - (H_x_MCA * optimize_x_x_MCA)(0,0);
        	predict_x_x_MCA = optimize_x_x_MCA + K_x_MCA * y_x_MCA;
        	P_x_MCA = (I_x_MCA - K_x_MCA * H_x_MCA) * P_x_MCA;

        	optimize_x_y_MCA = A_y_MCA * predict_x_y_MCA;
        	P_y_MCA = A_y_MCA * P_y_MCA * A_y_MCA.transpose() + Q_y_MCA;

        	S_y_MCA = (H_y_MCA * P_y_MCA * H_y_MCA.transpose())(0,0) + R_y_MCA;
        	K_y_MCA = P_y_MCA * H_y_MCA.transpose() / S_y_MCA;
        	z_y_MCA = mousePosition.y;
        	y_y_MCA = z_y_MCA - (H_y_MCA * optimize_x_y_MCA)(0,0);
        	predict_x_y_MCA = optimize_x_y_MCA + K_y_MCA * y_y_MCA;
        	P_y_MCA = (I_y_MCA - K_y_MCA * H_y_MCA) * P_y_MCA;



/*********Modified CV**********
        	optimize_x_x_MCV = A_x_MCV * predict_x_x_MCV;
        	P_x_MCV = A_x_MCV * P_x_MCV * A_x_MCV.transpose() + Q_x_MCV;

        	S_x_MCV = (H_x_MCV * P_x_MCV * H_x_MCV.transpose())(0,0) + R_x_MCV;
        	K_x_MCV = P_x_MCV * H_x_MCV.transpose() / S_x_MCV;
        	z_x_MCV = mousePosition.x;
        	y_x_MCV = z_x_MCV - (H_x_MCV * optimize_x_x_MCV)(0,0);
        	predict_x_x_MCV = optimize_x_x_MCV + K_x_MCV * y_x_MCV;
        	P_x_MCV = (I_x_MCV - K_x_MCV * H_x_MCV) * P_x_MCV;

        	optimize_x_y_MCV = A_y_MCV * predict_x_y_MCV;
        	P_y_MCV = A_y_MCV * P_y_MCV * A_y_MCV.transpose() + Q_y_MCV;

		    S_y_MCV = (H_y_MCV * P_y_MCV * H_y_MCV.transpose())(0,0) + R_y_MCV;
		    K_y_MCV = P_y_MCV * H_y_MCV.transpose() / S_y_MCV;
		    z_y_MCV = mousePosition.y;
		    y_y_MCV = z_y_MCV - (H_y_MCV * optimize_x_y_MCV)(0,0);
		    predict_x_y_MCV = optimize_x_y_MCV + K_y_MCV * y_y_MCV;
		    P_y_MCV = (I_y_MCV - K_y_MCV * H_y_MCV) * P_y_MCV;



/*********Combined CV**********
		    z_CV(0) = mousePosition.x;
		    z_CV(1) = mousePosition.y;
		    optimize_x_CV = A_CV * predict_x_CV;
		    P_CV = A_CV * P_CV * A_CV.transpose() + Q_CV;
		    y_CV = z_CV - H_CV * optimize_x_CV;
		    S_CV = H_CV * P_CV * H_CV.transpose() + R_CV;
		    K_CV = P_CV * H_CV.transpose() * S_CV.inverse();
		    
		    predict_x_CV = optimize_x_CV + K_CV * y_CV;
		    P_CV = (I_CV - K_CV * H_CV) * P_CV;


/*********Combined CA**********
		    z_CA(0) = mousePosition.x;
		    z_CA(1) = mousePosition.y;
		    optimize_x_CA = A_CA * predict_x_CA;
		    P_CA = A_CA * P_CA * A_CA.transpose() + Q_CA;
		    y_CA = z_CA - H_CA * optimize_x_CA;
		    S_CA = H_CA * P_CA * H_CA.transpose() + R_CA;
		    K_CA = P_CA * H_CA.transpose() * S_CA.inverse();
		    
		    predict_x_CA = optimize_x_CA + K_CA * y_CA;
		    P_CA = (I_CA - K_CA * H_CA) * P_CA;
/********************************/

			fprintf(fp, "%d,%d\n", measurement.x, measurement.y);
		}

        CvPoint predictpos_MCA = cvPoint((int)predict_x_x_MCA(0),(int)predict_x_y_MCA(0));
		CvPoint predictpos_MCV = cvPoint((int)predict_x_x_MCV(0),(int)predict_x_y_MCV(0));
		CvPoint predictpos_CV = cvPoint((int)predict_x_CV(0),(int)predict_x_CV(1));
		CvPoint predictpos_CA = cvPoint((int)predict_x_CA(0),(int)predict_x_CA(1));

        snprintf(buf, 256, "MCA:(%3f,%3f)", predict_x_x_MCA(0), predict_x_y_MCA(0));
        cvPutText(img, buf, cvPoint(10, 50), &font, CV_RGB(0, 0, 0));

        snprintf(buf, 256, "MCV:(%3f,%3f)", predict_x_x_MCV(0), predict_x_y_MCV(0));
        cvPutText(img, buf, cvPoint(10, 70), &font, CV_RGB(0, 0, 0));

        snprintf(buf, 256, "CA:(%3f,%3f)", predict_x_CA(0), predict_x_CA(1));
        cvPutText(img, buf, cvPoint(10, 90), &font, CV_RGB(0, 0, 0));

        snprintf(buf, 256, "CV:(%3f,%3f)", predict_x_CV(0), predict_x_CV(1));
        cvPutText(img, buf, cvPoint(10, 110), &font, CV_RGB(0, 0, 0));

		cvCircle(img, predictpos_CA , 5, CV_RGB(100, 100, 0), 3);
		cvCircle(img, predictpos_CV , 5, CV_RGB(0, 255, 0), 3);
		cvCircle(img, predictpos_MCV , 5, CV_RGB(0, 134, 139), 3);
        cvCircle(img, predictpos_MCA , 5, CV_RGB(0, 0, 255), 3);
        cvShowImage("kalman", img);
        int key = cvWaitKey(1000*deltaT);
        if (key == 27)
            break;
    }
    return 0;
}
