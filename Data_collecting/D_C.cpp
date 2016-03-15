#include "DJI_LIB/DJI_Pro_App.h"
#include "DJI_Pro_Sample.h"
#include "tinyxml2.h"
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include <stdlib.h>
#include <ctime>

using namespace std;
using namespace cv;

int image_count = 1;
time_t time_now;
string filename = "/home/ncslab/Documents/Data_collecting/Data/image/Image";
api_quaternion_data_t q_info;
api_vel_data_t v_info;
sdk_std_msg_t msg1_info;

int main(int argc,char **argv)
{
	char buff;
	Mat frame;
	tm *p;
	ofstream fout("/home/ncslab/Documents/Data_collecting/Data/time.txt");

	if(DJI_Sample_Setup() < 0)
	{
		cout << "Serial Port Open ERROR\n";
		pthread_exit(0);	
	}

	VideoCapture cap;
	if(!cap.open(0))
	{
		cout << "Can not open the camera!" << endl;
		return 0;
	}
	
	while(true)
	{
		Mat frame_result;
		cap >> frame;
		resize(frame,frame_result,Size(320,240));
		imshow("Camera",frame_result);
		int key = waitKey(33);
			if (key != -1)
			{
				if (key == 27) // esc
				{
					fout.close();
					break;
				}
				else if (key == 113) // q
				{
					time_now = time(NULL);
					p = localtime(&time_now);
					DJI_Pro_Get_Broadcast_Data(&msg1_info);
					q_info = msg1_info.q;
					v_info = msg1_info.v;

					float pitch,roll,yaw;
					roll = atan2(2 * (q_info.q0 * q_info.q1 + q_info.q2 * q_info.q3), 1 - 2 * (q_info.q1 * q_info.q1 + q_info.q2 							  * q_info.q2));
					pitch = asin(2 * (q_info.q0 * q_info.q2 - q_info.q3 * q_info.q1));
					yaw = atan2(2 * (q_info.q0 * q_info.q3 + q_info.q1 * q_info.q2), 1 - 2 * (q_info.q2 * q_info.q2 + q_info.q3 						  * q_info.q3));

					roll = roll * 180 / 3.1415;
					pitch = pitch * 180 / 3.1415;
					yaw  = yaw * 180 / 3.1415;

					cout << ' ' << p->tm_hour
						 << ':' << p->tm_min 
						 << ':' << p->tm_sec 
						 << " Image" << image_count 
						 << " Roll: "  << roll
						 << " Pitch: " << pitch
					     << " Yaw: "   << yaw 
						 << " v_x: "   << v_info.x
                         << " v_y: "   << v_info.y << endl;

					gcvt(image_count,4,&buff);
					
					fout << "Image" << image_count 
						 << ' ' << "Roll: " << pitch
						 << ' ' << "Pitch: " << roll
  						 << ' ' << "Yaw: " << yaw
						 << ' ' << "v_x: " << v_info.x
						 << ' ' << "v_y: " << v_info.y
						 << ' ' << "Time:" 
						 << ' ' << p->tm_hour
						 << ':' << p->tm_min 
						 << ':' << p->tm_sec << '\n';

					string name = filename + buff + ".JPEG";
					imwrite(name,frame_result);
					image_count ++;					
				}

			}
	}
	
	return 0;
}
