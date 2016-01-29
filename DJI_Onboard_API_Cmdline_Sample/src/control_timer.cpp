#include <sys/time.h>
#include <signal.h>
#include <time.h>
#include <stdio.h>
#include "ImageThread.h"
#include "DJI_LIB/DJI_Pro_App.h"
#include "Globalx.h"
#include "control_timer.h"
#include "DJI_Pro_Sample.h"

gimbal_custom_control_angle_t gimbal_ctrl;

api_quaternion_data_t q_info;
api_vel_data_t v_info;
api_pos_data_t pos_info;
sdk_std_msg_t msg_info;

static volatile sig_atomic_t gotAlarm = 0;
Imagepoint target_error_pre;
Imagepoint target_error_last;

void display()
{
/*
    struct itimerval itv;
    static struct timeval start;
    struct timeval curr;
    static bool callStart = true;

    if(callStart)
        gettimeofday(&start,NULL);

    gettimeofday(&curr, NULL);

    printf("%s %6.4f\n", msg, curr.tv_sec - start.tv_sec + (curr.tv_usec - start.tv_usec) / 1000000.0);
//    getitimer(ITIMER_REAL, &itv);
//    printf("%6.2f   %6.2f",itv.it_value.tv_sec + itv.it_value.tv_usec / 1000000.0 , itv.it_interval.tv_sec + itv.it_interval.tv_usec / 1000000.0);
    callStart = false;
//	
	static int count = 0;
    if(count <= 50 && count >= 1)
    {
        DJI_Sample_Gimbal_AngelCtrl(33,0,0,1);
        printf("right\n");
    }
    else if(count <= 100 && count >= 51)
    {
        DJI_Sample_Gimbal_AngelCtrl(-33,0,0,1);    
        printf("left\n");
    }
//	printf("num: %d\n",count);
    if(Start_flag)
    {
        count++;
    }
//*/
//	printf("target_error: (%4d,%4d)   target_width: %4d\n",target_error.x, target_error.y, target_width);
//
    int pe_x = target_error.x - target_error_last.x;
    int pe_y = target_error.y - target_error_last.y;
    int ie_x = target_error.x;
    int ie_y = target_error.y;
    int de_x = target_error.x - 2 * target_error_last.x + target_error_pre.x;
    int de_y = target_error.y - 2 * target_error_last.y + target_error_pre.y;
	
	double p1 = 0.13, i1 = 0.23, d1 = 0.13;
	double p2 = 0.08, i2 = 0.2, d2 = 0.08;	

    gimbal_ctrl.yaw_angle = p1 * pe_x + i1 * ie_x + d1 * de_x;
//    gimbal_ctrl.yaw_angle = (int)(0.3 * target_error.x);
//	gimbal_ctrl.yaw_angle = 0;
    gimbal_ctrl.pitch_angle = p2 * pe_y + i2 * ie_y + d2 * de_y;
//    gimbal_ctrl.pitch_angle = (int)(1.25 * target_error.y);
//    gimbal_ctrl.pitch_angle = 0;
    gimbal_ctrl.roll_angle = 0;
    gimbal_ctrl.duration = 1;

	DJI_Pro_Get_Broadcast_Data(&msg_info);
	q_info = msg_info.q;
	v_info = msg_info.v;
	pos_info = msg_info.pos;

    DJI_Sample_Gimbal_AngelCtrl(gimbal_ctrl.yaw_angle, gimbal_ctrl.roll_angle, gimbal_ctrl.pitch_angle, gimbal_ctrl.duration);
//	printf("error :%4d,%4d\n", target_error.x, target_error.y);
//	gettimeofday(&currenttime, NULL);
//	fprintf(fp, "time[%3.3f] : %4d,%4d\n", currenttime.tv_sec + currenttime.tv_usec / 1000000.0, target_error.x, target_error.y);
//	printf("%4f,%4f,%4f,%4f\n",q_info.q0,q_info.q1,q_info.q2,q_info.q3);
/********四元数转欧拉角********
		float pitch,roll,yaw;
		roll = atan2(2 * (q_info.q0 * q_info.q1 + q_info.q2 * q_info.q3), 1 - 2 * (q_info.q1 * q_info.q1 + q_info.q2 * q_info.q2));
		pitch = asin(2 * (q_info.q0 * q_info.q2 - q_info.q3 * q_info.q1));
		yaw = atan2(2 * (q_info.q0 * q_info.q3 + q_info.q1 * q_info.q2), 1 - 2 * (q_info.q2 * q_info.q2 + q_info.q3 * q_info.q3));

		roll = roll * 180 / 3.1415;
		pitch = pitch * 180 / 3.1415;
		yaw  = yaw * 180 / 3.1415;
	
		printf("time[%3.3f]:	  roll:%4f,pitch:%4f,yaw:%4f   vx:%4f,vy:%4f,vz:%4f   height:%4f, alti:%4f\n", currenttime.tv_sec + currenttime.tv_usec / 1000000.0, roll, pitch, yaw, v_info.x, v_info.y, v_info.z, pos_info.height, pos_info.alti);		


		fprintf(fp,"time[%3.3f]:	  roll:%4f,pitch:%4f,yaw:%4f   vx:%4f,vy:%4f,vz:%4f   height:%4f, alti:%4f\n", currenttime.tv_sec + currenttime.tv_usec / 1000000.0, roll, pitch, yaw, v_info.x, v_info.y, v_info.z, pos_info.height, pos_info.alti);
//		*******************************/
    target_error_pre = target_error_last; 
    target_error_last = target_error;
}

void sigalrmHandler(int sig)
{
    gotAlarm = 1;
}

void controlTimer()
{
    struct itimerval itv;
    clock_t prevClock;
    struct sigaction sa;

    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sa.sa_handler = sigalrmHandler;
    sigaction(SIGALRM, &sa, NULL);

    itv.it_value.tv_sec = 0;
    itv.it_value.tv_usec = 100000;
    itv.it_interval.tv_sec = 0;
    itv.it_interval.tv_usec = 100000;

    setitimer(ITIMER_REAL, &itv, 0);

    prevClock = clock();
    

    while(1)
    {
        while(((clock() - prevClock) * 10 / CLOCKS_PER_SEC) < 5)
        {
            if(gotAlarm)
            {
                gotAlarm = 0;
                display();
            }
        }
        prevClock = clock();

		if(!record_flag) break;
    }
}
