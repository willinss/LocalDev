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
	double deltaT = 0.03;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);
	timeval starttime,curtime;

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
    predict_x_y_MCA(0) = winWidth >> 1;
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
    Q_CV(0,0)=Q_CV(1,1)=Q_CV(2,2)=Q_CV(3,3)= 1e-3;
    Matrix2d R_CV; //measure_noise
    R_CV(0,0)=R_CV(1,1)=1e-1;
    
    Matrix4d P_CV; //cov
    Matrix2d S_CV; //temp matrix
    MatrixXd K_CV(4,2); //
    Vector2d z_CV; //measure value
    Vector2d y_CV;
    Matrix4d I_CV = MatrixXd::Identity(4,4);

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
    Q_x_MCV(0,0) = Q_x_MCV(1,1) = 1e-2;
    double R_x_MCV; //measure_noise
    R_x_MCV = 1e-1;
    
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
    Q_y_MCV(0,0) = Q_y_MCV(1,1) = 1e-2;
    double R_y_MCV; //measure_noise
    R_y_MCV = 1e-1;
    
    MatrixXd P_y_MCV(2,2); //cov
    double S_y_MCV; //temp matrix
    Vector2d K_y_MCV; //
    double z_y_MCV; //measure value
    double y_y_MCV;
    MatrixXd I_y_MCV(2,2);
    I_y_MCV(0,0) = I_y_MCV(1,1) = 1;


	gettimeofday(&starttime,NULL);

    while(1)
    {
		gettimeofday(&curtime,NULL);
//		printf("%4.4f\n",curtime.tv_sec - starttime.tv_sec + (curtime.tv_usec - starttime.tv_usec) / 1000000.0);
		starttime = curtime;
        cvSet(img, cvScalar(255, 255, 255, 0));
        cvCircle(img, mousePosition, 5, CV_RGB(255, 0, 0), 3);//current position with red
        char buf[256];
        snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));

/*********modified CA**********/
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

        CvPoint predictpos_MCA = cvPoint((int)predict_x_x_MCA(0),(int)predict_x_y_MCA(0));

/*********Combined CV**********/
        z_CV(0) = mousePosition.x;
        z_CV(1) = mousePosition.y;
        optimize_x_CV = A_CV * predict_x_CV;
        P_CV = A_CV * P_CV * A_CV.transpose() + Q_CV;
        y_CV = z_CV - H_CV * optimize_x_CV;
        S_CV = H_CV * P_CV * H_CV.transpose() + R_CV;
        K_CV = P_CV * H_CV.transpose() * S_CV.inverse();
        
        predict_x_CV = optimize_x_CV + K_CV * y_CV;
        P_CV = (I_CV - K_CV * H_CV) * P_CV;
        CvPoint predictpos_CV = cvPoint((int)predict_x_CV(0),(int)predict_x_CV(1));

/*********Modified CV**********/
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
		CvPoint predictpos_MCV = cvPoint((int)predict_x_x_MCV(0),(int)predict_x_y_MCV(0));




//        snprintf(buf, 256, "predicted position:(%3d,%3d)", predictpos.x, predictpos.y);
//        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));
		cvCircle(img, predictpos_CV , 5, CV_RGB(0, 255, 0), 3);
		cvCircle(img, predictpos_MCV , 5, CV_RGB(0, 100, 100), 3);
        cvCircle(img, predictpos_MCA , 5, CV_RGB(0, 0, 255), 3);
        cvShowImage("kalman", img);
        int key = cvWaitKey(1000*deltaT);
        if (key == 27)
            break;
    }
    return 0;
}
