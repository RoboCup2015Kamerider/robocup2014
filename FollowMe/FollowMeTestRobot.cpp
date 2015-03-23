#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <ViewImage.h>

//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.1415926535
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   

using namespace std;


class MyController : public Controller {  
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

private:
	RobotObj *m_robot;
	double v_left;
	double v_right;

private:
	// for control
	bool b_start;

public:
	ViewService *m_view;

public:
	int getPointDistance(int i, int j, int camID);
	CvPoint detect(char* src);

	bool avoidObstacles();
};  

void MyController::onInit(InitEvent &evt) 
{  
	m_robot = getRobotObj(myname());
	m_robot->setWheel(10.0, 10.0);
	v_left = 0.0;
	v_right = 0.0;

	b_start = false;

	m_view = (ViewService*)connectToService("SIGViewer");
}

double MyController::onAction(ActionEvent &evt)
{
	if(b_start == false)
	{
		std::string msg = "start";  
		broadcastMsg(msg);
		b_start = true;
	}
	else if(avoidObstacles() == false)
	{
		ViewImage *v_img = m_view->captureView(3, COLORBIT_24, IMAGE_320X240); 
		char *buf = v_img->getBuffer();

                IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);

                //memcpy(src->imageData, buf, src->imageSize);
		cout << "buf" << endl;
                CvPoint c_point = detect(buf);

		int dis = getPointDistance(c_point.x, c_point.y, 3);
		//int dis = 300;
		cout << "dis = " << dis << endl;
		if(dis > 200)
		{
			v_left = (dis - 200) / 1000 + 5;
			v_right = (dis - 200) / 1000 + 5;
		}

		
	}
	m_robot->setWheelVelocity(v_left, v_right);
	return 2.0;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
}

void MyController::onCollision(CollisionEvent &evt)
{
	
}

int MyController::getPointDistance(int i, int j, int camID) 
{
	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1024.0, camID, DEPTHBIT_8, IMAGE_320X240);
        char *dis_buf = dis_img->getBuffer(); 
 	int width = dis_img->getWidth();
	unsigned char distance = dis_buf[i * width + j]; 
	delete[] dis_buf;
	return int(distance) * 4;
}

CvPoint MyController::detect(char* src)
{
	cout << "In detection" << endl;
   	int x = 0, y = 0;
	int count = 0;
	unsigned char B, G ,R;
   	for(int i = 0; i < 240; i+=2)  
    	{  
		cout << "In i " << i << endl;
       		for(int j = 0; j < 320; j += 6)  
       		{  
           		B = src[i * 320 + j * 3];
           		G = src[i * 320 + j * 3 + 1];
           		R = src[i * 320 + j * 3 + 2];   
			//cout << "B" << B << "\t" << G << "\t" << R << endl;        
           		if( B > 130 && B < 150 && G > 160 && G < 190 && R > 170 && R < 210)
	   		{
               			count++;
               			x += i;
               			y += j / 3;
               
               			//CV_IMAGE_ELEM(img,unsigned char,i,j)=255;
               			//CV_IMAGE_ELEM(img,unsigned char,i,j+1)=255;
               			//CV_IMAGE_ELEM(img,unsigned char,i,j+2)=255;
                
	   		}
       		}
		//cout << i << endl;
    	}
    	CvPoint pt;
    	pt.x = y / count;
    	pt.y = x / count;
    	cout<< "pt = " << pt.x << " " << pt.y << endl;
	//delete[] src;
    	return pt;
}

bool MyController::avoidObstacles()
{
	ViewImage *dis_img = m_view->distanceSensor1D(0.0, 255.0, 3, DEPTHBIT_8, IMAGE_320X1);
        char *dis_buf = dis_img->getBuffer(); 
 	int width = dis_img->getWidth();
	//int height = dis_img->getHeight();
	int min_dis = 255;
	//for(int i = 0; i < height; i++)
	//{
		for(int j = 0; j < width; j++)
		{
			unsigned char distance = dis_buf[j];
			//cout << int(distance) << " ";
			if(min_dis > distance)
			{
				min_dis = distance;
			} 
		}
	//}
	if(min_dis < 80)
	{
		cout << "avoid = " << min_dis << endl;
		v_left = 0;
		v_right = 0;
		return true;
	}
	return false;
}

extern "C" Controller * createController() {  
	return new MyController;  
}


