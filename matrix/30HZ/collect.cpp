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
const int winHeight = 800;
const int winWidth = 800;
bool run_f = false;
FILE *fp = NULL;
int num;
int c;
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
/*********
			struct tm *newtime;
			time_t long_time;
			time(&long_time);
			newtime = localtime(&long_time);
/**************/
			char filename[50] = "data1/data";
			char filenum[50];
			sprintf(filenum, "%d", num); 
			strcat(filename, filenum);
			strcat(filename, ".txt");
			if(!(fp = fopen(filename, "w+")))
				printf("file open error\n");
			num++;
			c = 0;
		}
		else
		{
			fclose(fp);
			fp = NULL;
		}
	}
}

int main(int argv, char* argc[])
{
	CvPoint measurement;
	double deltaT = 0.030;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvNamedWindow("kalman");
    cvSetMouseCallback("kalman", mouseEvent);
    IplImage* img = cvCreateImage(cvSize(winWidth, winHeight), 8, 3);
	timeval starttime,curtime;
	c = 0;
	num = atoi(argc[1]);
	
	
	gettimeofday(&starttime,NULL);

    while(1)
    {


		gettimeofday(&curtime,NULL);
		starttime = curtime;
        cvSet(img, cvScalar(255, 255, 255, 0));
        cvCircle(img, measurement, 5, CV_RGB(0, 0, 0), 3);//current position with red
		measurement = mousePosition;
        char buf[256];
        snprintf(buf, 256, "current position :(%3d,%3d)  %3d", measurement.x, measurement.y, c);
        cvPutText(img, buf, cvPoint(10, 30), &font, CV_RGB(0, 0, 0));


		if(run_f)
		{
			c++;
			fprintf(fp, "%d %d\n", measurement.x, measurement.y);
		}

        cvShowImage("kalman", img);

        int key = cvWaitKey(10);
        if (key == 27)
            break;
    }
    return 0;
}
