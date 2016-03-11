#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <ctime>
#include "Tracker.h"
#include "Config.h"
#include "ImageThread.h"
#include "Globalx.h"
#include "DJI_LIB/DJI_Pro_App.h"
using namespace std;
using namespace cv;

static const int kLiveBoxWidth = 80;
static const int kLiveBoxHeight = 80;

api_common_data_t gimbal_info;
sdk_std_msg_t msg1_info;
//double Camera_angle_yaw;
//double Camera_angle_pitch;
//double angle_x;
//double angle_y;
//double v1;
//double v2;
//double length_x;
//double length_y;
//double Pi = 3.1415;
int bins = 256;
float range[] = {0,255};
float* ranges[] = {range};
int hist_count = 1;
double Compare = 0;
double Compare_rgb[3] = {0,0,0};
IplImage frame_hist;
IplImage* frame_gray;
IplImage* channels[3];
clock_t start,finish;

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
    
    if(!cap.open(0))
    {
        printf("error when opening camera\n");
        pthread_exit(0);
        return;
    }

	CvHistogram* gray_hist = cvCreateHist(1,&bins,CV_HIST_ARRAY,ranges,1);
	CvHistogram* rgb_hist[3];
	for(int i = 0;i < 3;i++)
	{
		rgb_hist[i] =  cvCreateHist(1,&bins,CV_HIST_ARRAY,ranges,1);
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

			frame_hist = IplImage(frame);
			frame_gray = cvCreateImage(cvGetSize(&frame_hist), IPL_DEPTH_8U, 1);
			
			cvCvtColor(&frame_hist,frame_gray,CV_BGR2GRAY);
            
            if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
            
            //fprintf(fp,"%.1f,%.1f\n", tracker.GetBB().XCentre(), tracker.GetBB().YCentre());
			//DJI_Pro_Get_Broadcast_Data(&msg1_info);
			//gimbal_info = msg1_info.gimbal;
			
			const FloatRect& bo = tracker.GetBB();
			cvSetImageROI(frame_gray,
						  cvRect(bo.XMin(),
								 bo.YMin(),
								 bo.XMax() - bo.XMin(),
							     bo.YMax() - bo.YMin())
					     );
			cvSetImageROI(&frame_hist,
						  cvRect(bo.XMin(),
								 bo.YMin(),
								 bo.XMax() - bo.XMin(),
							     bo.YMax() - bo.YMin())
					     );

			IplImage* img_hist = cvCreateImage(
						  cvSize(bo.XMax() - bo.XMin(),
                                 bo.YMax() - bo.YMin()),
                                 frame_gray->depth, frame_gray->nChannels);
			IplImage* img_hist_rgb = cvCreateImage(
						  cvSize(bo.XMax() - bo.XMin(),
                                 bo.YMax() - bo.YMin()),
                                 *(&frame_hist.depth), *(&frame_hist.nChannels));

			cvCopy(frame_gray,img_hist,0);
			cvCopy(&frame_hist,img_hist_rgb,0);

			for(int i = 0;i < 3 ;i++)
			{
				channels[i] = cvCreateImage(cvGetSize(&frame_hist), IPL_DEPTH_8U, 1);
			}
            cvSplit(img_hist_rgb,channels[0],channels[1],channels[2],0);
			cvResetImageROI(frame_gray);

			start = clock();

			if(hist_count == 1)
			{
				cvCalcHist(&img_hist,gray_hist,0,0);
				cvNormalizeHist(gray_hist,1.0);
				hist_count ++;
			}
			else
			{
				CvHistogram* gray_hist_tracker = cvCreateHist(1,&bins,CV_HIST_ARRAY,ranges,1);
				cvCalcHist(&img_hist,gray_hist_tracker,0,0);
				cvNormalizeHist(gray_hist_tracker,1.0);
				Compare = cvCompareHist(gray_hist,gray_hist_tracker,CV_COMP_BHATTACHARYYA);
				if(Compare >= 0.3)
				{
					cout << Compare << endl;
					doInitialise = true;
					startTrack = !startTrack;
				}
				gray_hist = gray_hist_tracker;
			}

			/*if(hist_count == 1)
			{
				for(int i = 0;i < 3;i++)
				{
					cvCalcHist(&channels[i],rgb_hist[i],0,0);
				}
				hist_count ++;
			}
			else
			{
				CvHistogram* rgb_hist_tracker[3];
				for(int i = 1; i < 3 ; i++)
				{
					rgb_hist_tracker[i] = cvCreateHist(1,&bins,CV_HIST_ARRAY,ranges,1);
				}
				for(int i = 1; i < 3 ; i++)
				{
					cvCalcHist(&channels[i],rgb_hist_tracker[i],0,0);
					cvNormalizeHist(rgb_hist_tracker[i],1.0);
					Compare_rgb[i] = cvCompareHist(rgb_hist[i],rgb_hist_tracker[i],CV_COMP_BHATTACHARYYA);
					rgb_hist[i] = rgb_hist_tracker[i];
				}
				double result = (Compare_rgb[0] + Compare_rgb[1] + Compare_rgb[2])/3;
				if(result >= 0.3)
				{
					cout << result << endl;
					doInitialise = true;
					startTrack = !startTrack;
				}
			}*/
			
			finish = clock();
			cout << "                       Time: " << double(finish - start)/CLOCKS_PER_SEC << endl;

            target_location.x = tracker.GetBB().XCentre();
            target_location.y = tracker.GetBB().YCentre();
            target_error.x = target_location.x - conf.frameWidth / 2;
            target_error.y = - target_location.y + conf.frameHeight / 2;
            target_width = tracker.GetBB().Width();
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));

			//Camera_angle_yaw = gimbal_info.z;
			//Camera_angle_pitch = 90 + gimbal_info.y;
			//v1 = ((double)target_location.y - 120)/120;
			//v2 = ((double)target_location.x - 160)/160;
			//if(v2 < 0)
				//v2 = -1 * v2;
			//double yy_A = atan(v1 * tan(Pi * 17.65/180));
			//double xx_A = atan(v2 * tan(Pi * 21.8/180));

			//angle_y = Camera_angle_pitch * Pi / 180 - yy_A;

			//double PT1 = 195 / cos(angle_y);
			//double PN = PT1 * cos(-1 * yy_A);
			//double NT2 = PN * tan(xx_A);
			//double OT1 = 195 * tan(angle_y);
			//double angle_xx = atan(NT2 / OT1);

			//angle_x = angle_xx + abs(Camera_angle_yaw + 113) * Pi / 180;

			//double OT = OT1 / cos(angle_xx);
			//length_y = OT * cos(angle_x);
			//length_x = OT * sin(angle_x);
			//cout<< "OT: "<< OT <<" length_x: "<< length_x << " length_y: " << length_y << endl;
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














