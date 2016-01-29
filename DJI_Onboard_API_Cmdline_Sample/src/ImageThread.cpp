#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <time.h>
#include "Tracker.h"
#include "Config.h"
#include "ImageThread.h"
#include "Globalx.h"
using namespace std;
using namespace cv;

static const int kLiveBoxWidth = 80;
static const int kLiveBoxHeight = 80;

void rectangle (Mat& rMat, const FloatRect& rRect, const Scalar& rColour)
{
    IntRect r(rRect);
    rectangle(rMat, Point(r.XMin(), r.YMin()), Point(r.XMax(),r.YMax()),rColour);
}

vector<int> myvector;
bool selectObject = false;
IntRect selectionRegion;
Point originMouse;
bool newregion = false;
int trackObject = 0;
bool startTrack = false;
Imagepoint target_location;

Imagepoint target_error;
int target_width;
void onMouse(int event, int x, int y, int, void*)
{
    if(selectObject)
    {
        selectionRegion.SetXMin(MIN(x,originMouse.x));
        selectionRegion.SetYMin(MIN(y, originMouse.y));
        selectionRegion.SetWidth(std::abs(x - originMouse.x));
        selectionRegion.SetHeight(std::abs(y - originMouse.y));                     
    }
        switch (event)
        {
            case CV_EVENT_LBUTTONDOWN:
                printf("buttondown:    x:%d,y:%d\n",x,y);                
                originMouse = Point(x, y);
                selectionRegion = IntRect(x, y, 0, 0);
                
                selectObject = true;                
                break;
            case CV_EVENT_LBUTTONUP:
                selectObject = false; 
                newregion = true;
                trackObject = -1;
                printf("buttonup:    x:%d,y:%d,width:%d,height:%d\n", selectionRegion.XMin(), selectionRegion.YMin(), selectionRegion.Height(), selectionRegion.Width());
                break;
        }
}

void ImageThread()
{
    /*
    //write the location of the target to the file named by time 
    struct tm *newtime;
    char filename[30];
    time_t long_time;
    time(&long_time);
    newtime = localtime(&long_time);
    
    strftime(filename,sizeof(filename),"%H%M%pdata.out",newtime);
    FILE *fp = fopen(filename,"w+");
    if(!fp) 
        cout << "File open fails" << endl;
    */

    string configPath = "../src/config.txt";
    Config conf(configPath);
    VideoCapture cap;
    FloatRect initBB;
    string imgFormat;
    
    if(!cap.open(1))
    {
        printf("error when opening camera\n");
        pthread_exit(0);
        return;
    }
    
    Mat tmp;
    cap >> tmp;
    initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2,conf.frameHeight/2-kLiveBoxHeight/2,kLiveBoxWidth,kLiveBoxHeight);

    Tracker tracker(conf);
    namedWindow("result");
    setMouseCallback("result", onMouse, 0);

    Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
    bool paused = false;
    bool doInitialise = false;
    srand(conf.seed);
    while(true)
    {
        Mat frame;
        Mat frameOrig;
        cap >> frameOrig;
        resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
        frame.copyTo(result);
	
		
        if(doInitialise)
        {
            if(tracker.IsInitialised())
            {
                tracker.Reset();
            }
            else
            {
                tracker.Initialise(frame, initBB);
            }
            doInitialise = false;
        }
        else
        {
            if(trackObject < 0)
            {
                initBB = selectionRegion;
                trackObject = 1;
            }
            if(!startTrack)
            {
                rectangle(result, initBB, CV_RGB(255, 255, 255));
            }
        }

        if(tracker.IsInitialised())
        {
            tracker.Track(frame);
            
            if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
            
            //fprintf(fp,"%.1f,%.1f\n", tracker.GetBB().XCentre(), tracker.GetBB().YCentre());
            target_location.x = tracker.GetBB().XCentre();
            target_location.y = tracker.GetBB().YCentre();
            target_error.x = target_location.x - conf.frameWidth / 2;
            target_error.y = - target_location.y + conf.frameHeight / 2;
            target_width = tracker.GetBB().Width();
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
        }
		
		if (!conf.quietMode)  //keyboard event
		{
			imshow("result", result);
			int key = waitKey(paused ? 0 : 1);
			if (key != -1)
			{
				if (key == 27 || key == 113) // esc q
				{
					break;
				}
				else if (key == 112) // p
				{
					paused = !paused;
				}
				else if (key == 105) // i
				{
					doInitialise = true;
					startTrack = !startTrack;
				}
			}
			if(record_flag)	
			{
				writer1 << result;
				gettimeofday(&currenttime, NULL);
				fprintf(fvp, "%3.3f\n",currenttime.tv_sec + currenttime.tv_usec / 1000000.0);
			}
        }
	}
    cvDestroyWindow("result");
}














