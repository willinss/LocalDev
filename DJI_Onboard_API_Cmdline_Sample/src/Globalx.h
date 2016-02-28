#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "DJI_LIB/DJI_Pro_App.h"
#include <sys/time.h>
#include <time.h>
using namespace cv;

extern CvVideoWriter *writer;
extern VideoWriter writer1;
extern FILE *fp;
extern FILE *fvp;
extern bool record_flag;
extern struct timeval currenttime;
