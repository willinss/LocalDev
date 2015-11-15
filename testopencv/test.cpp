#include <stdio.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
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
