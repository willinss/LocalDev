#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "DJI_LIB/DJI_Pro_App.h"
#include <time.h>
#include <sys/time.h>
using namespace cv;

CvVideoWriter *writer;
VideoWriter writer1;
FILE *fp;
FILE *fvp;
bool record_flag = false;
struct timeval currenttime;
