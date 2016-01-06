#include "Tracker.h"
#include "Config.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Tracker.h"
#ifndef IMAGETHREAD_H_
#define IMAGETHREAD_H_
typedef struct{
    int x;
    int y;
}Imagepoint;
using namespace cv;
void rectangle(Mat&,const FloatRect&,const Scalar&);
void onMouse(int,int,int,int,void*);
void ImageThread();
extern Imagepoint target_location;
extern Imagepoint target_error;
extern int target_width;
#endif
