#include "Tracker.h"
#include "Config.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Tracker.h"


using namespace cv;
void rectangle(Mat&,const FloatRect&,const Scalar&);
void onMouse(int,int,int,int,void*);
void ImageThread();
