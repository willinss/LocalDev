//============================================================================
// Name        : main.cpp
// Author      : wuyuwei
// Version     :
// Copyright   : DJI Inc
// Description : DJI Onboard API test in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <string.h>
#include "DJI_Pro_Sample.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>
#include "control_timer.h"
#include "DJI_LIB/DJI_Pro_App.h"
#include "ImageThread.h"
#include "Globalx.h"

using namespace std;
using namespace cv;


char filename[50];
char videoname[50];
char filevideoname[50];
int main_operate_code = 0;
int temp32 = 0;
bool valid_flag = false;
bool err_flag = false;
activate_data_t user_act_data; 

pthread_t t_control;
pthread_t t_camera;
pthread_t t_serialport;

static void Display_Main_Menu(void)
{
	printf("\r\n");
	printf("-----------   < Main menu >  ----------\r\n\r\n");

	printf("[a]  Request activation\n");
	printf("[b]  Request to obtain control\n");
	printf("[c]  Release control\n");
	printf("[d]  Takeoff\n");
	printf("[e]  Landing\n");
	printf("[f]  Go home\n");
	printf("[g]  Attitude control sample\n");
	printf("[h]  Gimbal right 1 degree //Draw circle sample\n");
	printf("[i]  Gimbal left 1 degree //Draw square sample\n");
	printf("[j]  Gimbal control sample\n");
	printf("[k]  Gimbal down 10 degree //Take a picture\n");
	printf("[l]  Gimbal up 5 degree //Start video\n");
	printf("[m]  Stop video\n");
	printf("[n]  Query UAV current status\n");
	if(record_flag) 
		printf("[o]  Stop and Save\n");
	else
		printf("[o]  Start gimbal control and record data\n");
	printf("\ninput a/b/c etc..then press enter key\r\n");
	printf("---------------------------------------\r\n");
	printf("input: ");
}

void user_function()
{
    ;
}

void *CameraShow(void *argc)
{ 
    ImageThread();
    printf("\ncamera thread quit\n");
    pthread_exit(0);
}

void *ControlLoop(void *argc)
{
    controlTimer();
    printf("\ncontrol thread quit\n");
    pthread_exit(0);
}

void open_file()
{
//*
	struct tm *newtime;
    time_t long_time;
    time(&long_time);
    newtime = localtime(&long_time);
    
    strftime(filename, sizeof(filename), "%H%M%p data.txt", newtime);
    fp = fopen(filename,"w+");
    if(!fp) 
        cout << "File open fails" << endl;

	strftime(filevideoname, sizeof(filevideoname), "%H%M%p videotime.txt", newtime);
	fvp = fopen(filevideoname, "w+");
	if(!fvp)
		cout << "Filevideo open fails" << endl;

	strftime(videoname, sizeof(videoname), "%H%M%pvideo.avi", newtime);
	writer1.open(videoname, CV_FOURCC('M', 'J', 'P', 'G'), 15, cvSize(320,240), 1);
	if(!writer1.isOpened())
		cout << "Video open fails" << endl;
    /*
     * create a thread to send control msg every 100ms
     */ 
   int ret_control;
    ret_control = pthread_create(&t_control, NULL, ControlLoop, NULL);
    if(ret_control != 0)
        printf("create control pthread error\n");
}

void close_file()
{
//*
	cvReleaseVideoWriter(&writer);
	fclose(fp);
	fclose(fvp);
//*/
}


void *Serialport(void *argc)
{   
	if(DJI_Sample_Setup() < 0)
	{
		printf("Serial Port Open ERROR\n");
		pthread_exit(0);
	}
}



int main(int argc,char **argv)
{
//	int main_operate_code = 0;
//    int temp32 = 0;
//	bool valid_flag = false;
//	bool err_flag = false;
//	activate_data_t user_act_data; 

	char temp_buf[65];
	char app_bundle_id[32] = "1234567890";

	if(argc == 2 && strcmp(argv[1],"-v") == 0)
	{
		printf("\nDJI Onboard API Cmdline Test,Ver 1.0.0\n\n");
		return 0;
	}
	printf("\nDJI Onboard API Cmdline Test,Ver 1.1.0\n\n");

    /*
     * create a thread to process image from camera
     */
   int ret_camera;
    ret_camera = pthread_create(&t_camera, NULL, CameraShow, NULL);
    if(ret_camera != 0)
        printf("create camera pthread error\n");

	/* 
     *  create a thread to maintain serialport
     */ 
	int ret_serialport;
	ret_serialport = pthread_create(&t_serialport, NULL, Serialport, NULL);
	if(ret_serialport != 0)
		printf("create serialport pthread error\n");

	
    sigset_t blockSet;
    sigemptyset(&blockSet);
    sigaddset(&blockSet, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &blockSet, NULL);

	user_act_data.app_key = temp_buf;
	user_act_data.app_ver = SDK_VERSION;
	strcpy((char*)user_act_data.app_bundle_id, app_bundle_id);
    if(DJI_Pro_Get_Cfg(NULL,NULL,&user_act_data.app_id,&user_act_data.app_api_level,user_act_data.app_key) == 0)
	{
		/* user setting */
		printf("--------------------------\n");
		printf("app id=%d\n",user_act_data.app_id);
		printf("app api level=%d\n",user_act_data.app_api_level);
		printf("app key=%s\n",user_act_data.app_key);
		printf("--------------------------\n");
	}
	else
	{
		printf("ERROR:There is no user account\n");	
		return 0;
	}

    Display_Main_Menu();

	while(1)
	{
		temp32 = getchar();
		if(temp32 != 10)
		{
			if(temp32 >= 'a' && temp32 <= 'o' && valid_flag == false)
			{
				main_operate_code = temp32;
				valid_flag = true;
			}
			else
			{
                printf("input: %d", temp32);
				err_flag = true;
			}
			continue;
		}
		else
		{
			if(err_flag == true)
			{
				printf("input: ERROR\n");
				Display_Main_Menu();
				err_flag = valid_flag = false;
				continue;
			}
		}

		switch(main_operate_code)
		{
		case 'a':
			/* api activation */
			DJI_Pro_Activate_API(&user_act_data,NULL);
			break;
		case 'b':
			/* get controller */
			DJI_Pro_Control_Management(1,NULL);
			break;
		case 'c':
			/* release controller */
			DJI_Pro_Control_Management(0,NULL);
			break;
		case 'd':
			/* takeoff */
			DJI_Pro_Status_Ctrl(4,0);
			break;
		case 'e':
			/* landing */
			DJI_Pro_Status_Ctrl(6,0);
			break;
		case 'f':
			/* go home */
			DJI_Pro_Status_Ctrl(1,0);
			break;
		case 'g':
			/* attitude ctrl */
			if(DJI_Sample_Atti_Ctrl()< 0)
    		{
        		printf("Please waiting current sample finish\n");
   			}
			break;
		case 'h':
			DJI_Sample_Gimbal_AngelCtrl(10, 0, 0, 5);  //right 1 degree
			/* draw circle 			
			if(DJI_Sample_Funny_Ctrl(DRAW_CIRCLE_SAMPLE)< 0)
    		{
        		printf("Please waiting current sample finish\n");
   			}
			*/
			break;
		case 'i':
			DJI_Sample_Gimbal_AngelCtrl(-10, 0, 0, 5);  //left 1 degree
			/* draw square
			if(DJI_Sample_Funny_Ctrl(DRAW_SQUARE_SAMPLE)< 0)
    		{
        		printf("Please waiting current sample finish\n");
   			}
*/
			break;
		case 'j':
			if(DJI_Sample_Gimbal_Ctrl() < 0)
    		{
        		printf("Please waiting current sample finish\n");
   			}
			break;
		case 'k':
		//	DJI_Sample_Camera_Shot();
            DJI_Sample_Gimbal_AngelCtrl(0, 0, -100, 5); //down 1 degree
			break;
		case 'l':
		//	DJI_Sample_Camera_Video_Start();
        //  DJI_Sample_Gimbal_AngelCtrl(0, 0, 10, 10);
        //  Start_flag = true;
            DJI_Sample_Gimbal_AngelCtrl(0, 0, 50, 5); //up 1 degree
			break;
		case 'm':
			DJI_Sample_Camera_Video_Stop();
			break;
		case 'n':
			/* status query */
			DJI_Sample_Drone_Status_Query();
			break;
		case 'o':
			record_flag = !record_flag;
			if(record_flag)
				open_file();
			else
				close_file();
			break;
		}
		main_operate_code = -1;
		err_flag = valid_flag = false;
		Display_Main_Menu();
	}


	return 0;
}
