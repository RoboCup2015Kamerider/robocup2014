#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include <vector>
#include <ViewImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.1415926535797
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )
#define _VEL_SCALE	1

using namespace std;

class SendController : public Controller
{
public:
  	void onInit(InitEvent &evt);
  	double onAction(ActionEvent &evt);
  	void onRecvMsg(RecvMsgEvent &evt);
	void onCollision(CollisionEvent &evt);

public:
	RobotObj *m_robot;

	ViewService *m_view;

  	double velocity;
  	int i;
 	int j;

 	double v_left;
 	double v_right;

	bool b_start;
	bool b_check1;
	bool b_check2;
	bool b_elevator_send;

	int getPointDistance(int i, int j, int camID);
	CvPoint detectPants(IplImage* src);
	CvPoint detectCoatColor(IplImage* src);
	bool elevatorDetect(IplImage* src);
	int checkPointDetect(int camID);

	void check1(int dis);
	int detectHuman(CvPoint& cpt, int camID);
	bool detectElevator(int camID);
	
	Vector3d* gethline();
	int findObstacles(int& index, int camID, double theta);
public:
	void _SET_VEL(double _vl, double _vr);

public:
	void MOVEFORWARD(double DIS);
	void MOVEBACKWARD(double DIS);
	void TURNLEFT();
	void TURNRIGHT();
	void TURNBACK();
	void STOP();
	void LOOKAROUND();

	void MOVERIGHT(double DIS);
	void MOVELEFT(double DIS);

public:
	bool START();
	bool DIRECTIONCORRECTION(double x);
	bool POSITIONCORRECTION(double x, int t);
};

void SendController::onInit(InitEvent &evt)
{

	b_start = false;
	b_check1 = false;
	b_check2 = false;
	b_elevator_send = false;
 	v_left = 0.0;
 	v_right = 0.0;

	m_robot = this->getRobotObj(this->myname());
	m_robot->setWheel(10.0, 10.0);
	
	m_view = (ViewService*)connectToService("SIGViewer");
}

double SendController::onAction(ActionEvent &evt)
{
	static int i_la = 0;
	if(START() == true)
	{
		//findObstacles(3);
		sleep(1.0);
		CvPoint cpt1, cpt2, cpt3;
		int dis3 = detectHuman(cpt3, 3);
		int dis2 = detectHuman(cpt2, 2);
		int dis1 = detectHuman(cpt1, 1);
		
		if(dis3 != -1 && dis3 > 80)
		{
			double v = double(dis3 - 100.0) / 20.0 + 2.0;
			v = v > 12.0 ? 12.0 : v;
			if(DIRECTIONCORRECTION(cpt3.x) == false)
			{
				MOVEFORWARD(v);
				i_la = 0;
			}
		}
		else if(dis3 != -1 && dis3 < 80 
			&& (detectElevator(1) || detectElevator(2)))
		{
			cout << "ELEVATOR" << endl;
			STOP();
			if(b_elevator_send == false)
			{
				std::string msg = "elevator";
				broadcastMsg(msg);
				b_elevator_send = true;
				sleep(5.0);
				MOVEBACKWARD(40);
				msg = "ok";
				broadcastMsg(msg);
				sleep(6.0);
				i_la = 0;
			}
		}
		else if(dis2 != -1 && dis2 > 100)
		{	
			if(POSITIONCORRECTION(cpt2.x, 1) == false)
			{
				TURNRIGHT();
				i_la = 0;
			}
		}
		else if(dis1 != -1 && dis1 > 100)
		{
			if(POSITIONCORRECTION(cpt1.x, -1) == false)
			{
				TURNLEFT();
				i_la = 0;
			}	
		}
		else
		{	
			//MOVEFORWARD(10.0);
			if(i_la != 0 && i_la % 8 == 0)
			{
				MOVEFORWARD(10.0);
			}
			cout << "lookaround" << endl;
			LOOKAROUND();
			i_la++;
		}
	}
  	return 0.1;
}



void SendController::onRecvMsg(RecvMsgEvent &evt)
{
 	std::string sender = evt.getSender();

  	std::string msg = evt.getMsg();
	if(msg == "check1")
	{
		b_check1 = true;
		cout << "#### check 1" << endl;
	}
	else if(msg == "check2")
	{
		b_check2 = true;
		cout << "#### check 2" << endl;
	}
  	LOG_MSG(("message : %s", msg.c_str()));
}

void SendController::onCollision(CollisionEvent &evt)
{

	typedef CollisionEvent::WithC C;
		// Get name of entity which is touched by the robot
	const std::vector<std::string> & with = evt.getWith();
		// Get parts of the robot which is touched by the entity
	const std::vector<std::string> & mparts = evt.getMyParts();

		// loop for every collided entities
	for(int i = 0; i < with.size(); i++) 
	{
		cout << "with = " << with[i] << endl;	
	}
}



int SendController::getPointDistance(int i, int j, int camID) 
{
	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1024.0, camID, DEPTHBIT_8, IMAGE_320X240);
        char *dis_buf = dis_img->getBuffer(); 
 	int width = dis_img->getWidth();
	unsigned char distance = dis_buf[i * width + j]; 
	delete[] dis_buf;
	return int(distance) * 4;
}

CvPoint SendController::detectPants(IplImage* src)
{
   	int x = 0, y = 0;
	int count = 0;
	unsigned char B, G ,R;
   	for(int i = 0; i < 240; i+=1)  
    	{  
       		for(int j = 0; j < src->widthStep; j += 3)  
       		{  
           		B = CV_IMAGE_ELEM(src, unsigned char, i, j);
           		G = CV_IMAGE_ELEM(src, unsigned char, i, j + 1);
           		R = CV_IMAGE_ELEM(src, unsigned char, i, j + 2);       
           		if( B > 120 && B < 160 && G > 150 && G < 190 && R > 160 && R < 200)
	   		{
               			count++;
               			y += i;
               			x += j / 3;
	   		}

       		}
    	}

    	CvPoint pt;
	pt.x = 0;
	pt.y = 0;
	if(count != 0)
	{
    		pt.x = x / count;
    		pt.y = y / count;
	}
    	return pt;
}

CvPoint SendController::detectCoatColor(IplImage* src)
{	
	int x = 0, y = 0;
	int count = 0;
	unsigned char B, G ,R;
   	for(int i = 0; i < src->height; i+=1)  
    	{  
       		for(int j = 0; j < src->widthStep; j += 3)  
       		{  
           		B = CV_IMAGE_ELEM(src, unsigned char, i, j);
           		G = CV_IMAGE_ELEM(src, unsigned char, i, j + 1);
           		R = CV_IMAGE_ELEM(src, unsigned char, i, j + 2); 
     
           		if( B > 100 && B < 140 && G > 50 && G < 90 && R > 70 && R < 110)
	   		{
                 		count++;
               			y += i;
               			x += j / 3;
               
               			//CV_IMAGE_ELEM(src, unsigned char, i, j) = 255;
               			//CV_IMAGE_ELEM(src, unsigned char, i, j + 1) = 255;
               			//CV_IMAGE_ELEM(src, unsigned char, i, j + 2) = 255;
                
	   		}
       		}
    	}
    	CvPoint pt;
	pt.x = 0;
	pt.y = 0;
	if(count != 0)
	{
    		pt.x = x / count;
    		pt.y = y / count;
	}
    	//cout<< "pt = " << pt.x << " " << pt.y << endl;
	//delete[] src;
	//cvReleaseImage(&src);
    	return pt;
}
int SendController::findObstacles(int& index, int camID, double theta = 0.0)
{
	ViewImage *dis_img = m_view->distanceSensor1D(0.0, 255.0, camID, DEPTHBIT_8, IMAGE_320X1);
        char *dis_buf = dis_img->getBuffer(); 
 	int width = dis_img->getWidth();

	double fovy = m_robot->getCamFOV() * PI / 180.0; 
  	double ar = m_robot->getCamAS(); 
        double fovx = 2 * atan(tan(fovy * 0.5) * ar);
	
	int min_dis = 255;

	for(int j = 0; j < width; j++)
	{
		unsigned char distance = dis_buf[j];
		//cout << int(distance) << "\t";
		distance = distance * cos(theta + fovx * j / (width - 1.0) - fovx / 2.0);
		//cout << int(distance) << endl;
		if(min_dis > distance)
		{
			min_dis = distance;
			index = j;
		} 
	}
	cout << "\t\t\tmin_dis = " << min_dis << " cam = " << camID << endl;
	return min_dis;
}

int SendController::checkPointDetect(int camID)
{
	ViewImage *v_img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = v_img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);

        memcpy(src->imageData, buf, src->imageSize);
	
	int x = 0, y = 0;
	int count = 0;
	unsigned char B, G ,R;
   	for(int i = 0; i < src->height; i+=1)  
    	{  
       		for(int j = 0; j < src->widthStep; j += 3)  
       		{  
           		B = CV_IMAGE_ELEM(src, unsigned char, i, j);
           		G = CV_IMAGE_ELEM(src, unsigned char, i, j + 1);
           		R = CV_IMAGE_ELEM(src, unsigned char, i, j + 2);       
           		if( B > 40 && B < 80 && G > 200 && G < 240 && R > 40 && R < 80)
	   		{
               			count++;
               			y += i;
               			x += j / 3;
	   		}
       		}
    	}
    	CvPoint c_point;
	c_point.x = 0;
	c_point.y = 0;
	if(count != 0)
	{
    		c_point.x = x / count;
    		c_point.y = y / count;
	}
	
	int dis = -1;
	if(c_point.x != 0 && c_point.y != 0)
	{
		cout << "check point detect cam " << camID  << "\t" << c_point.y << "\t" << c_point.x << endl;
		dis = getPointDistance(c_point.x, c_point.y, camID);
	}
	//cvReleaseImage(&src);
    	return dis;
}

int SendController::detectHuman(CvPoint& cpt, int camID)
{
	ViewImage *v_img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = v_img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);

        memcpy(src->imageData, buf, src->imageSize);

	CvPoint c_point = detectCoatColor(src);
	CvPoint p_point = detectPants(src);

	int c_dis, p_dis;
	if(c_point.x != 0 && c_point.y != 0)
	{
		c_dis = getPointDistance(c_point.y, c_point.x, camID);
	}
	else
	{
		return -1;
	}
	if(p_point.x != 0 && p_point.y != 0)
	{
		p_dis = getPointDistance(p_point.y, p_point.x, camID);
	}
	else
	{
		return -1;
	}
	cpt = c_point;
	cout << "detect human cam " << camID << endl;
	return c_dis;
}

void SendController::check1(int dis)
{
	m_robot->setWheelVelocity(0.0, 0.0);
	sleep(5.0);
	m_robot->setWheelVelocity(dis / 10, dis / 10);
	sleep(1.0);
	m_robot->setWheelVelocity(0.0, 0.0);
}

bool SendController::elevatorDetect(IplImage* src)
{
	int count = 0;
	unsigned char B, G ,R;
   	for(int i = 0; i < src->height; i+=2)  
    	{  
       		for(int j = 0; j < src->widthStep; j += 6)  
       		{  
           		B = CV_IMAGE_ELEM(src, unsigned char, i, j);
           		G = CV_IMAGE_ELEM(src, unsigned char, i, j + 1);
           		R = CV_IMAGE_ELEM(src, unsigned char, i, j + 2);       
           		if( B > 190 && B < 230 && G > 190 && G < 230 && R > 190 && R < 230)
	   		{
               			count++;
	   		}
       		}
    	}
	if(count > 20000)
	{
		cout << " EEEEEEEEEEEEEEEEEE" << endl;
		return true;
	}
	return false;
}
void SendController::MOVEFORWARD(double DIS)
{
	_SET_VEL(DIS * _VEL_SCALE, DIS * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	STOP();
}
void SendController::MOVEBACKWARD(double DIS)
{
	_SET_VEL(-DIS * _VEL_SCALE, -DIS * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	STOP();
}
void SendController::TURNLEFT()
{
	m_robot->setWheelVelocity((-PI / 4.0) * _VEL_SCALE, (PI / 4.0) * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	m_robot->setWheelVelocity(0.0, 0.0);
}
void SendController::TURNRIGHT()
{
	m_robot->setWheelVelocity((PI / 4.0) * _VEL_SCALE, (-PI / 4.0) * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	m_robot->setWheelVelocity(0.0, 0.0);
}
void SendController::TURNBACK()
{
	m_robot->setWheelVelocity((PI / 2.0) * _VEL_SCALE, (-PI / 2.0) * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	m_robot->setWheelVelocity(0.0, 0.0);
}
void SendController::STOP()
{
	m_robot->setWheelVelocity(0.0, 0.0);
}
void SendController::LOOKAROUND()
{
	double v = 0.0;
	_SET_VEL((PI / 8.0 + v) * _VEL_SCALE, (-PI / 8.0 + v) * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	STOP();
}
bool SendController::DIRECTIONCORRECTION(double x)
{
	double v = 5.0;
	if(x > 200)
	{
		_SET_VEL((0.1 + v) * _VEL_SCALE, (-0.1 + v) * _VEL_SCALE);
		sleep(1.0 / _VEL_SCALE);
		STOP();
		return true;
	}
	else if(x < 120)
	{
		_SET_VEL((-0.1 + v) * _VEL_SCALE, (0.1 + v) * _VEL_SCALE);
		sleep(1.0 / _VEL_SCALE);
		STOP();
		return true;
	}
	return false;
}
bool SendController::POSITIONCORRECTION(double x, int t)
{
	double v = 1.0;
	if(t != 1 && t != -1)
	{
		cout << "POSITIONCORRECTION t ERROR!" << endl;
		return true;
	}
	if(x > 200)
	{
		//MOVEBACKWARD(t * 2.0);
		m_robot->setWheelVelocity((0.0 + v) * t, (0.1 + v) * t);
		return true;
	}
	else if(x < 120)
	{
		//MOVEFORWARD(t * 2.0);
		m_robot->setWheelVelocity((0.1 + v) * t, (0.0 + v) * t);
		return true;
	}
	return false;
}
bool SendController::START()
{
	if(b_start == false)
	{
		std::string msg = "start";  
		broadcastMsg(msg);
		b_start = true;
	}
	if(b_start = true)
	{
		return true;
	}
	return false;
}
bool SendController::detectElevator(int camID)
{
	ViewImage *v_img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = v_img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);

        memcpy(src->imageData, buf, src->imageSize);

	int count = 0;
	unsigned char B, G ,R;
   	for(int i = 0; i < src->height; i++)  
    	{  
       		for(int j = 0; j < src->widthStep; j += 3)  
       		{  
           		B = CV_IMAGE_ELEM(src, unsigned char, i, j);
           		G = CV_IMAGE_ELEM(src, unsigned char, i, j + 1);
           		R = CV_IMAGE_ELEM(src, unsigned char, i, j + 2);       
           		if( B > 200 && B < 230 && G > 200 && G < 230 && R > 200 && R < 230)
	   		{
               			count++;
	   		}
       		}
    	}
	cout << "Elevator count = " << count << endl;
	if(count > 320 * 240 / 8)
	{
		return true;
	}
	return false;
}
Vector3d* SendController::gethline() 
{
	if(m_view != NULL) 
	{ 
        	ViewImage *dis_img = m_view->distanceSensor1D(0.0, 1023.0, 3, DEPTHBIT_8, IMAGE_320X1);

  		double fovy = m_robot->getCamFOV() * PI / 180.0; 
 
  		double ar = m_robot->getCamAS(); 
 
                double fovx = 2 * atan(tan(fovy * 0.5) * ar);

                char *dis_buf = dis_img->getBuffer(); 
 
                int width = dis_img->getWidth();

	        Vector3d *p = new Vector3d[width];
                double *theta = new double[width];  

	        unsigned char *distance = new unsigned char[width];

		for(int i = 0; i < width; i++)
		{
                	theta[i] = fovx * i / (width - 1.0) - fovx / 2.0;
		  	distance[i] = dis_buf[i] * 4;
		}
		for(int j = 0; j < width; j++)
		{
			p[j] = Vector3d(distance[j] * sin(theta[j]), 30.0, distance[j] * cos(theta[j]) + 20.0);
		}

		delete theta;
		delete distance;

		return p;
	}
}  
void SendController::_SET_VEL(double _vl, double _vr)
{
	_vl /= _VEL_SCALE;
	_vr /= _VEL_SCALE;
	int _obDis;
	int _mDis = findObstacles(_obDis, 3);
	double _v_left = 0;
	double _v_right = 0;
	double _v_same = 0;

	if(_vl == _vr && _vl > 0)
	{
		_v_same = _vl * 10.0 > _mDis - 50 ? (_mDis - 50) / 10.0 : _vl;
		
		if(_mDis <= 50)
		{
			if(_obDis < 160)
			{
				MOVERIGHT(2.0);
			}
			else
			{
				MOVELEFT(2.0);
			}
		}
		m_robot->setWheelVelocity(_v_same * _VEL_SCALE, _v_same * _VEL_SCALE);
		return;
	}
	else if(_vl > 0 && _vr > 0)
	{
		
		double _v_mid = (_vl + _vr) / 2.0;
		double theta = _vl - _vr;
		_mDis = findObstacles(_obDis, 3, theta);
		double _max_mid = _mDis - 50;
		_v_same = _max_mid > _v_mid ? _v_mid : _max_mid;
		double _dv = _v_mid - _v_same;
		
		if(_mDis <= 50)
		{
			if(_obDis < 160)
			{
				MOVERIGHT(2.0);
			}
			else
			{
				MOVELEFT(2.0);
			}
		}
		m_robot->setWheelVelocity((_vl - _dv) * _VEL_SCALE, (_vr - _dv) * _VEL_SCALE);
		return;
	}
	m_robot->setWheelVelocity(_vl * _VEL_SCALE, _vr * _VEL_SCALE);
}
void SendController::MOVERIGHT(double DIS)
{
	TURNRIGHT();
	_SET_VEL(DIS * _VEL_SCALE, (DIS + 0.01) * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	STOP();
	TURNLEFT();
}
void SendController::MOVELEFT(double DIS)
{
	TURNLEFT();
	_SET_VEL((DIS + 0.01) * _VEL_SCALE, DIS * _VEL_SCALE);
	sleep(1.0 / _VEL_SCALE);
	STOP();
	TURNRIGHT();
}

extern "C"  Controller * createController ()
{
  return new SendController;
}
