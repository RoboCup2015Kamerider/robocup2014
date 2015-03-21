#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
using namespace std;

#define DEBUG_MODE true
//#define SHOW_POINT

//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )


typedef unsigned char UC;
const double PI=3.1415926535897932384626;
const double SAFE_DIS=50;
enum ViewPosition
{
	HEAD=1,LEFT,MIDDLE,RIGHT
};
struct Point
{
	double X,Y;
};

class RobotController : public Controller
{
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 
	/**
	the robot's status
	*/
	double getSpeed();
	Point  getPosition();
	Vector3d* gethline();

	bool findTable(int& im, int& jm, int ID);
	int getPointDistance(int i, int j, int camID);

protected:

	/**
	all motion control of the wheel
	*/
	void stop(void);//set the speed to 0
	void turn(double theta);//turn left as theta
	void setSpeed(double speed);
	void go(double distance);//go foraward(+) or go back(-) distance

	/**
	control the robot's hands
	*/
	void throwTrash(void);
	double goGraspingObject(Vector3d &pos);
	void neutralizeArms(double evt_time);
	void prepareThrowing(double evt_time);

	/*
	*/
	double getMinDistance(ViewPosition camera_id);

	double	updateTime();
private:
	RobotObj *robot;
	ViewService *view;

	/**
	robot's pose
	*/
	Point P;//current position of robot
	Point T[4];//trash boxes
	Point D;//desk
	Point G[3];//goals

	double THETA;//current direction , DON"T CHANGE IT !!!
	double V;	//speed of wheel, DON"T CHANGE IT !!! 
	double R;           // radius of the wheel
	double L;         // length of wheel-track
	timeval LCST;//Last_Change_Speed_Time the time when the last speed changed

	timeval sleep_time;//
	bool is_grasp;//
	bool is_cmd;
	double refresh_time;//for onAction

public:
	IplImage* img_temp;
	void getROI(IplImage* dis_image,CvRect& ROI);
	IplImage* TemplateMatch(IplImage* img,IplImage* tp,CvPoint &pt,IplImage* dis_image,double &m,int flag);
	CvPoint findCup(IplImage *tp,int ID,float dis_t);
	void findTrashBox2(int ID);
};


void RobotController::onInit(InitEvent &evt)
{
	// get robot's name
	robot = getRobotObj(myname());
	view=(ViewService*)connectToService("SIGViewer");
	img_temp = cvLoadImage("./template/cup.png");
	//robot->setCamFOV(3.0*PI/8.0);

	// set wheel configuration
	R = 10.0;
	L = 10.0;
	robot->setWheel(R, L);

	P.X=0;
	P.Y=0;
	THETA=0;
	V=0;
	gettimeofday(&LCST,NULL);

	is_grasp=false;
	is_cmd = false;
	refresh_time=1.0;
	sleep_time.tv_sec=1;
	sleep_time.tv_usec=0;

	refresh_time=1.0;

	#ifdef DEBUG_MODE
	Vector3d  r3d;
	robot->getPosition(r3d);
	cout<<r3d.x()<<","<<r3d.y()<<","<<r3d.z()<<endl;
	#endif
}


double RobotController::onAction(ActionEvent &evt)
{
	if(is_cmd)
	{
		CvPoint cpt = findCup(img_temp, 1, 0);
		if(cpt.x != 0 && cpt.y != 0)
		{
			int dis = getPointDistance(cpt.y, cpt.x, 1);
			if(dis > 60)
			{
				double min_dis = getMinDistance(MIDDLE);
				if(min_dis > 50)
				{
					go(5);
				}
				else if(cpt.x < 160)
				{
					turn(-PI / 2);
					go(5);
					turn(PI / 2 + 0.01);
				}
				else if(cpt.x > 160)
				{
					turn(PI / 2);
					go(5);
					turn(-PI / 2 - 0.01);
				}
			}	
		}
		return 1.0;
	}
	int t_i, t_j;
	if(findTable(t_i, t_j, 3))
	{
		
		//int dis = getPointDistance(t_i, t_j, 3);
		int dis = getMinDistance(MIDDLE);
		cout << "dis = " << dis << endl;
		if(dis > 100)
		{
		robot->setWheelVelocity(3, 3);
		sleep(1.0);
		robot->setWheelVelocity(0, 0);
		}
		else
		{
			cout << "arrive" << endl;
			is_cmd = true;
		}
	}
	else if(findTable(t_i, t_j, 2))
	{
		robot->setWheelVelocity(-PI / 4, PI / 4);
		sleep(1.0);
		robot->setWheelVelocity(0, 0);
	}
	else if(findTable(t_i, t_j, 4))
	{
		robot->setWheelVelocity(PI / 4, -PI / 4);
		sleep(1.0);
		robot->setWheelVelocity(0, 0);
	}
	else
	{
		double min_dis=getMinDistance(MIDDLE);
		if(min_dis<SAFE_DIS)
		{
			double l_dis=getMinDistance(LEFT);
			double r_dis=getMinDistance(RIGHT);
			if(l_dis<r_dis)
				turn(PI/2.0);
			else
				turn(-PI/2.0);
			refresh_time=0.2;
		}
		else 
		{
			go(30);
		}
		refresh_time=0.2;
	// if(min_dis<120){
	// 	go(30);
	// 	refresh_time=0.2;
	// }else{
	// 	setSpeed(20);
	// 	refresh_time=1.0;
	// }
		cout<<"min distance:"<<min_dis<<endl;
	}
	return refresh_time;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();
//	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	if(strcmp(msg.c_str(), "go") == 0){
		go(30);
	}else if(strcmp(msg.c_str(), "360") == 0)	{
		turn(2*PI);
	}else if(strcmp(msg.c_str(), "180") == 0){
		turn(PI);
	}else if(strcmp(msg.c_str(), "90") == 0){
		turn(PI/2.0);
	}else if(strcmp(msg.c_str(), "45") == 0){
		turn(PI/4.0);
	}else if(strcmp(msg.c_str(), "cmd") == 0){
		is_cmd = !is_cmd;
	}
	else if(strcmp(msg.c_str(), "coco") == 0){
		findCup(img_temp, 1, 0);
	}
	if(sender == "moderator_0"){
		if(msg == "Task_start"){
			setSpeed(0);
		}else if(msg == "Time_over"){
			stop();
		}else if(msg == "Task_end"){
			stop();
		}
	}
}

void RobotController::onCollision(CollisionEvent &evt)
{
	if (is_grasp == false) {
		typedef CollisionEvent::WithC C;
		// Get name of entity which is touched by the robot
		const std::vector<std::string> & with = evt.getWith();
		// Get parts of the robot which is touched by the entity
		const std::vector<std::string> & mparts = evt.getMyParts();

		// loop for every collided entities
		for(int i = 0; i < with.size(); i++) {
			// if(goal == with[i]) {
			// 	// If the right hand touches the entity
			// 	if(mparts[i] == "RARM_LINK7") {
			// 		SimObj *my = getObj(myname());
			// 		CParts * parts = my->getParts("RARM_LINK7");
			// 		if(parts->graspObj(with[i])) 
			// 			is_grasp = true;
			// 	}
			// }
		}
	}
}

double RobotController::getSpeed(){
	return V;
}

Point RobotController::getPosition(){
	return P;
}

double RobotController::updateTime(){
	timeval old=LCST;
	gettimeofday(&LCST,NULL);
	return (double)(LCST.tv_sec-old.tv_sec)+(LCST.tv_usec-old.tv_usec)/1000000.0;
}

void RobotController::setSpeed(double speed){
	double w=speed/R;
	robot->setWheelVelocity(w,w);

	double t=updateTime();
	if(abs(V)>0.001){
		P.X+=V*t*cos(THETA);
		P.Y+=V*t*sin(THETA);
	}

	#ifdef DEBUG_MODE
	cout<<"\nchange speed from "<<V<<" to "<<speed<<" ,has run time : "<<t<<" and THETA= "<<THETA<<"\n curent position: "<<P.X<<" "<<P.Y<<endl;
	Vector3d  r3d;
	robot->getPosition(r3d);
	cout<<r3d.x()<<","<<r3d.y()<<","<<r3d.z()<<endl;
	#endif

	V=speed;//upadate V


}

void RobotController::stop(void) {
	setSpeed(0);
}

void RobotController::go(double distance)
{
	double t=1.0;//(double)sleep_time.tv_sec+sleep_time.tv_usec/1000000.0;
	sleep_time.tv_sec=(int)t;
	sleep_time.tv_usec=0;
	double v=distance/t;
	#ifdef DEBUG_MODE
	cout<<"set v= "<<v<<" ,t= "<<t<<endl;
	#endif
	setSpeed(v);
	select (0, NULL, NULL, NULL, &sleep_time);
	stop();
}

void RobotController::turn(double theta)
{
	stop();
	//double t=(double)sleep_time.tv_sec+sleep_time.tv_usec/1000000.0;
	double t=1;
	sleep_time.tv_sec=(int)t;
	sleep_time.tv_usec=0;
	double w=(theta*L)/(2.0*t*R);
	updateTime();
	robot->setWheelVelocity(w, -w);
	select (0, NULL, NULL, NULL, &sleep_time);
	robot->setWheelVelocity(0,0);
	t=updateTime();
	//THETA+=theta;
	THETA+=w*2.0*t*R/L;
	// if(THETA>2*PI){
	// 	THETA-=2*PI;
	// }else if(THETA<-2*PI){
	// 	THETA+=2*PI;
	// }
	//stop();
}

double RobotController::getMinDistance(ViewPosition camera_id=MIDDLE) 
{
	if(view != NULL) { 

		ViewImage *dis_img = view->distanceSensor2D(0.0, 255.0, camera_id,DEPTHBIT_8,IMAGE_320X240);
		double fovy = robot->getCamFOV() * PI / 180.0;  
		double ar = robot->getCamAS();  
		double fovx = 2 * atan(tan(fovy*0.5)*ar);
		char *dis_buf = dis_img->getBuffer();  

		int width = dis_img->getWidth();
		int h=dis_img->getHeight();

		double *theta=new double[width];  
		double *thetay=new double[h];
		for(int i=0;i<=(width+1)/2;i++)
		{
			theta[i] =fovx*i/(width-1.0)-fovx/2.0; //atan((i-159.5)/160.0*tan(fovx/2));  
			theta[width-i-1]=theta[i];
		}
		for (int i = 0; i <= h/2; ++i)
		{
			thetay[i]=fovy/2*(1-2*i/h);
			thetay[h-i-1]=thetay[i];
		}

		double* d=new double[width];
		for (int i = 0; i < width; ++i)//all -> 1 line
		{
			d[i]=(UC)dis_buf[i]*cos(thetay[0]);
			for (int j = 1; j < h/2+25; ++j)
			{
				double td=(UC)dis_buf[j*width+i]*cos(thetay[j]);
				if(td<d[i])
				{
					d[i]=td;
				}
			}
			d[i]*=cos(theta[i]);
			#ifdef SHOW_POINT
			cout<<d[i]<<" , ";
			#endif
		}
		double	distance=d[0];
		for(int i=1;i<width;++i)
		{
			if(d[i]<distance)
			{
				distance=d[i];
			}
		}
		delete[]theta;
		delete[]d;

		return distance;
	}else
	return -1;
}
bool RobotController::findTable(int& im, int& jm, int ID)
{
               ViewImage *img = view->captureView(ID, COLORBIT_24, IMAGE_320X240); 
	       char *buf = img->getBuffer();
                // the image
                IplImage *image=cvCreateImage(cvSize(320,240),8,3);
                memcpy(image->imageData,buf,image->imageSize);

                ViewImage *dis_img = view->distanceSensor2D(0.0, 255.0, ID,DEPTHBIT_8,IMAGE_320X240);  
                char *dis_buf = img->getBuffer();
                IplImage *dis_image=cvCreateImage(cvSize(320,240),8,1);
                memcpy(dis_image->imageData,dis_buf,dis_image->imageSize);
                int dt_count=0;
		int zz_count=0;
                CvPoint d_pt;
		CvPoint z_pt;
		
                d_pt.x=0;
                d_pt.y=0;

		z_pt.x=0;
                z_pt.y=0;

                for( int i = image->height/2; i < image->height; i++)  
                {  
                       for(int j = 0; j < image->widthStep; j+=3)  
                       {  
                           int B=CV_IMAGE_ELEM(image,unsigned char,i,j);
                           int G=CV_IMAGE_ELEM(image,unsigned char,i,j+1);
                           int R=CV_IMAGE_ELEM(image,unsigned char,i,j+2);
                           //鍖哄煙 鍦版
                              if(B<100&&(G>0&&G<80)&&(R>0&&R<90))
		              {
                               dt_count++;
                               d_pt.x+=j/3;
                               d_pt.y+=i;
                               //std::cout<<"find"<<std::endl;
                               CV_IMAGE_ELEM(image,unsigned char,i,j)=255;
                               CV_IMAGE_ELEM(image,unsigned char,i,j+1)=0;
                               CV_IMAGE_ELEM(image,unsigned char,i,j+2)=0;
                                
				
			       }
			        else
				{
					//鐧借壊
				        if(B>150&&(G>150)&&(R>150)&&fabs(double(R-G))<30&&fabs(double(R-B))<30&&fabs(double(B-G))<30)
				        {
					        zz_count++;
					        z_pt.x+=j/3;
					        z_pt.y+=i;
					        //std::cout<<"find"<<std::endl;
					        CV_IMAGE_ELEM(image,unsigned char,i,j)=0;
					        CV_IMAGE_ELEM(image,unsigned char,i,j+1)=0;
					        CV_IMAGE_ELEM(image,unsigned char,i,j+2)=255;
                                        
				        }
				        else
				        {
					         CV_IMAGE_ELEM(image,unsigned char,i,j)=255;
					        CV_IMAGE_ELEM(image,unsigned char,i,j+1)=255;
					        CV_IMAGE_ELEM(image,unsigned char,i,j+2)=255;
				        }
	                   
				}
                       }  
  
                } 
                if(dt_count>2000)
                { 
                	d_pt.x /= dt_count;
                	d_pt.y /= dt_count;
                	//cvCircle(image,d_pt,10,CV_RGB(255,0,0),3,8);
                }
		if(zz_count>10)
                { 
                	z_pt.x /= zz_count;
                	z_pt.y /= zz_count;
                	//cvCircle(image,z_pt,10,CV_RGB(0,0,255),3,8);
                }
                if((z_pt.y<d_pt.y-10)
			&&(fabs(z_pt.x-d_pt.x)<50)
			&& z_pt.y < 200)
                {	
			cvCircle(image,d_pt,10,CV_RGB(255,0,0),3,8);
			cvNamedWindow("DIS");
                	cvShowImage("DIS",image);
                	cvWaitKey(30);
                	cvReleaseImage(&image);
                	cvReleaseImage(&dis_image);  
			cout << "Yes " << ID << endl;
			im = d_pt.y;
			jm = d_pt.x;
			return true;
                }
                /*cvNamedWindow("DIS");
                cvShowImage("DIS",image);
                cvWaitKey(30);
                delete img;
                delete dis_img;
                cvReleaseImage(&image);
                cvReleaseImage(&dis_image);*/
		return false;
}


	/**
@param D position of goal

void  potentialControl(V2D D)//脢脝脛脺艗脝脣茫
{
	if(m_view==NULL) 
	{
		cout<<"No view"<<endl;
		return; 
	}

	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 255.0, 3,DEPTHBIT_8,IMAGE_320X240);
	double fovy = robot->getCamFOV() * PI / 180.0;  
	double ar = robot->getCamAS();  
	double fovx = 2 * atan(tan(fovy*0.5)*ar);
	char *dis_buf = dis_img->getBuffer();  

	int width = dis_img->getWidth();
	int h=dis_img->getHeight();

	double *theta=new double[width];  
	for(int i=0;i<=(width+1)/2;i++)
	{
		theta[i] =fovx*i/(width-1.0)-fovx/2.0; //atan((i-159.5)/160.0*tan(fovx/2));  
		theta[width-i-1]=theta[i];
	}

	double* d=new double[width];
	for (int i = 0; i < width; ++i)//all -> 1 line
	{
		d[i]=(UC)dis_buf[i];
		for (int j = 1; j < h/2+24; ++j)
		{
			if((UC)dis_buf[j*width+i]<d[i])
			{
				d[i]=(UC)dis_buf[j*width+i];
			}
		}
		cout<<d[i]<<" , ";
	}

	double k=1.0,kf=1.0;//
	int n=width;
	const double max_distance=200;
	double *f=new double[n];
	double fx=0,fy=0;

	for(int i=0;i<n;++i)
	{
		if(d[i]<max_distance)
		{
			f[i]=(1.0/d[i]-1.0/max_distance)/(d[i]*d[i])*(D.x*D.x+D.y*D.y);
			fx+=f[i]*cos(theta[i]);
			fy+=f[i]*sin(theta[i]);
		}
	}
	delete[]theta;
	delete[]d;
	delete[]f;

	fx=-kf*fx+k*D.x;
	fy=-kf*fy+k*D.y;

	// double angle=atan(fx/fy);
	// double v=sqrt(fx*fx+fy*fy);

	// turnTheta(angle);
	// setSpeed(0.001*v);
}
*/
void RobotController::throwTrash(void)
{
	// get the part info. 
	CParts *parts = robot->getParts("RARM_LINK7");

	// release grasping
	parts->releaseObj();
	// wait a bit
	select (0, NULL, NULL, NULL, &sleep_time);
	// set the grasping flag to neutral
	is_grasp = false;

	//sendMsg("moderator_0", "end of task");
}


double RobotController::goGraspingObject(Vector3d &pos)
{
	return 1;
}


void RobotController::neutralizeArms(double evt_time)
{
}
int RobotController::getPointDistance(int i, int j, int camID) 
{
	ViewImage *dis_img = view->distanceSensor2D(0.0, 1024.0, camID, DEPTHBIT_8, IMAGE_320X240);
        char *dis_buf = dis_img->getBuffer(); 
 	int width = dis_img->getWidth();
	unsigned char distance = dis_buf[i * width + j]; 
	delete[] dis_buf;
	return int(distance) * 4;
}
Vector3d* RobotController::gethline() 
{
	if(view != NULL) 
	{ 
        	ViewImage *dis_img = view->distanceSensor1D(0.0, 1024.0, 3, DEPTHBIT_8, IMAGE_320X1);

  		double fovy = robot->getCamFOV() * PI / 180.0; 
 
  		double ar = robot->getCamAS(); 
 
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

void RobotController::getROI(IplImage* dis_image,CvRect& ROI)
{
	//鏍规嵁dis_image锛屽垝瀹氭劅鍏磋叮鍖哄煙
	IplImage* ed=cvCreateImage(cvGetSize(dis_image),8,1);
	//cvAdaptiveThreshold(dis_image, ed, 255, CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_OTSU, 9, -10);
	cvThreshold(dis_image, ed, 100, 255, CV_THRESH_OTSU);
	
	int x_min=320,x_max=0;
	int y_min=240,y_max=0;
	
	for(int i=0;i<ed->height;i++)
	{
		for(int j=0;j<ed->width;j++)
		{
			if(CV_IMAGE_ELEM(ed,unsigned char,i,j)==0)
			{
				if(i<y_min)
					y_min=i;
				if(i>y_max)
					y_max=i;
				if(j<x_min)
					x_min=j;
				if(j>x_max)
					x_max=j;


			}
		}
	}
	CvPoint pt1;
	CvPoint pt2; 
	pt1.x=x_min;
	pt1.y=y_min;
	pt2.x=x_max;
	pt2.y=y_max;
	//cvRectangle(img,pt1,pt2,cvScalar(0,0,255),1,8);
	
	ROI.x=x_min;
        ROI.y=y_min;
        ROI.width=x_max-x_min;
        ROI.height=y_max-y_min;
        cvReleaseImage(&ed);


}
 IplImage* RobotController::TemplateMatch(IplImage* img,IplImage* tp,CvPoint &pt,IplImage* dis_image,double &m,int flag)
{
       //image-----------------input image
       //tp--------------------input template image
       //dis_image-------------input distance image
       //m---------------------output 
       //flag-------------------0--no ROI   1--ROI
       //return-----------------the new template image

        CvPoint     temp_minloc,temp_maxloc;
        CvPoint     minloc, maxloc; 

        double      temp_minval=0;
        double      temp_maxval=0;

        double      maxval=0;

        int         res_width, res_height; 
        int         mask;

        double      scale=(double(img->height)/(tp->height)<double(img->width)/(tp->width))?double(img->height)/(tp->height):double(img->width)/(tp->width);
        double      templateScale[20];
        for(int i=0;i<20;i++)
        {
                templateScale[i]=(i+1)/20.0*scale;
        }

	int img_height;
	int img_width;
	int x_off;
	int y_off;
	if(flag==1)
	{
		CvRect ROI;
		getROI(dis_image,ROI);
		cvSetImageROI(img,ROI);
		img_height=ROI.height;
		img_width=ROI.width;
		x_off=ROI.x;
		y_off=ROI.y;

	}
	else if(flag==0)
	{
		img_height=img->height;
		img_width=img->width;
		x_off=0;
		y_off=0;

	}
        for(int i=0;i<20;i++)  //瀵?0涓笉鍚屽昂瀵哥殑妯℃澘杩涜鍖归厤
        {
		CvSize dst_size;
		dst_size.height  = (int)((tp->height)* templateScale[i]);	
		dst_size.width   = (int)((tp->width)* templateScale[i]);
		IplImage *dst= cvCreateImage( dst_size, tp->depth, tp->nChannels );
		cvResize(tp,dst,CV_INTER_LINEAR );   //Resizes image 

		if(dst->height>img_height)
			break;

		res_width  = img_width - dst->width + 1;   //缁撴灉鍥剧殑灏哄
		res_height = img_height - dst->height + 1; 
		IplImage* res = cvCreateImage( cvSize( res_width, res_height ), IPL_DEPTH_32F, 1 ); 
	
		//cvMatchTemplate( img, dst, res, CV_TM_CCORR_NORMED);   //鍖归厤缁撴灉瀛樺湪res涓?
		//cvMatchTemplate( img, dst, res, CV_TM_SQDIFF_NORMED ); 
		//cvMatchTemplate( img, dst, res, CV_TM_CCORR ); 
		//cvMatchTemplate( img, dst, res, CV_TM_SQDIFF ); 
		//cvMatchTemplate( img, dst, res, CV_TM_CCOEFF ); 
		cvMatchTemplate( img, dst, res, CV_TM_CCOEFF_NORMED );

                //input single-channel array ,find the minimum and maximum element values and their positions.
		cvMinMaxLoc( res, &temp_minval, &temp_maxval, &temp_minloc, &temp_maxloc, 0 );
		//cout<<"绗?<<i<<"娆″尮閰嶏紝鍖归厤姣斾緥涓?<< templateScale[i]<<"妯℃澘澶у皬涓?<<dst_size.width<<"*"<<dst_size.height<<"鍖归厤绋嬪害涓?:"<<temp_maxval<<endl;
		
		if(temp_maxval>=maxval)  //瀵绘壘100骞呮ā鏉垮浘鍍忔渶鍖归厤鐨勭粨鏋?
		{
			maxval=temp_maxval;
			maxloc=temp_maxloc;
			mask=i;
		}
		cvReleaseImage( &res );
	        cvReleaseImage( &dst );
		
        }
	m=maxval;
	CvPoint pt1; 
	CvPoint pt2; 
	CvRect rect; 
	rect=cvRect(maxloc.x,maxloc.y,(tp->width)* templateScale[mask],(tp->height)* templateScale[mask]);//鏈€浣崇殑鍖归厤鍖哄煙 
	pt1=cvPoint(rect.x+x_off,rect.y+y_off); 
	pt2=cvPoint(rect.x+rect.width+x_off,rect.y+rect.height+y_off); 
	pt.x=x_off+(pt1.x+pt2.x)/2;
	pt.y=y_off+(pt1.y+pt2.y)/2;
	//鑾峰彇璇嗗埆鍖哄煙
	IplImage* tpp=cvCreateImage(cvSize(rect.width,rect.height),8,3);
	cvSetImageROI(img, rect);
	cvCopy(img, tpp);
	cvResetImageROI(img);

        if(flag==1)
	    cvResetImageROI(img);
	std::cout<<"match"<<std::endl;
 
	return tpp;
 
}


CvPoint RobotController::findCup(IplImage *tp,int ID,float dis_t)
{
        //tp----------------template image 
        //dis_t-------------the stop distance
        //index-------------the camera ID
        
        ViewImage *img = view->captureView(ID, COLORBIT_24, IMAGE_320X240); 	  
	char *buf = img->getBuffer();
        IplImage *image=cvCreateImage(cvSize(320,240),8,3);
        memcpy(image->imageData,buf,image->imageSize); 
                
        CvPoint pp;
                         
        //the distance image    
        ViewImage *dis_img = view->distanceSensor2D(0.0, 255.0, ID,DEPTHBIT_8,IMAGE_320X240);    
        char *dis_buf = dis_img->getBuffer(); 
        IplImage* dis_image=cvCreateImage(cvSize(320,240),8,1);
        memcpy(dis_image->imageData,dis_buf,dis_image->imageSize); 
        
        double m;      
        TemplateMatch(image,tp,pp,NULL,m,0);
	
        LOG_MSG((" x = %d",pp.x));
        LOG_MSG((" y = %d",pp.y));
                
        cvNamedWindow("dis");
	cvCircle(image, pp, 5, CV_RGB(255, 0, 0), 3, 8);
        cvShowImage("dis",image);
        cvWaitKey(100);  
                
        int width = dis_img->getWidth();
        int height = dis_img->getHeight();   
        int index = pp.y*width + pp.x;  
        int distance = dis_buf[index]; 
                  
        LOG_MSG((" distance = %d",distance));
        //LOG_MSG((" distance_pre = %d",distance_pre));
	cout << "m = " << m << endl;
	/*if(m > 0.5)
	{
		return pp;
	}
	return cvPoint(0, 0);*/
        return pp;
        //hold up the arm
	/*if(distance < 100 && distance > 60)
	{
		double larm_joint_angular = my->getJointAngle("LARM_JOINT1");
	        std::cout << "angular = " << larm_joint_angular << std::endl;
		if(my->getJointAngle("LARM_JOINT1") > DEG2RAD(-130.0) ||
			my->getJointAngle("LARM_JOINT1") < DEG2RAD(-140.0))
		{
			double velocity = DEG2RAD(-135.0) - larm_joint_angular;
			my->setJointVelocity("LARM_JOINT1", velocity, velocity);
			sleep(1.0);
			my->setJointVelocity("LARM_JOINT1", 0, 0);
		}
	}
		
	//move forward
        if(distance>dis_t)
        { 
                       
             //if((distance-distance_pre)>-10&&(distance-distance_pre)<10)
              {
                   Weel_one=(distance-dis_t)/100.0;
                   Weel_two=Weel_one;
                   my->setWheelVelocity(Weel_one,Weel_two);     
                   sleep(1.0);
                   my->setWheelVelocity(0.0,0.0);
              }

        }
        else
        {
              LOG_MSG((" stop distance = %d",distance));
        }                       
	//turn the body	
	//if((pp.x-pp_pre.x)>-20&&(pp.x-pp_pre.x)<20)
	{	
               Weel_one=(pp.x-120)/1000.0;
	       Weel_two=-(pp.x-120)/1000.0;		  
		my->setWheelVelocity(Weel_one,Weel_two);
		sleep(1.0);
		my->setWheelVelocity(0,0); 
	} 
		        
	//turn head	
	//if((pp.y-pp_pre.y)>-20&&(pp.y-pp_pre.y)<20)
	{	  
		float rad = (pp.y-120)/15.0/180.0 * PI;
		my->setJointVelocity(joint[1], rad, rad);
		sleep(1.0);
		my->setJointVelocity(joint[1], 0, 0);
	}
		
	//the conditions of stop
        if(((pp.x-120)>-10)&&((pp.x-120)<10)&&((120-pp.y)<10)&&((distance-dis_t)>-2)&&((distance-dis_t)<2))
        {
		flag = false;
		grasping = true;
		std::cout<<"stop"<<std::endl;
	}
	
        delete dis_img;
        distance_pre=distance;   
	delete img;   
        cvReleaseImage(&image);
        cvReleaseImage(&tp); 
        cvReleaseImage(&dis_image); */
        //cvDestroyAllWindows();  
}
void RobotController::findTrashBox2(int ID)
{
               ViewImage *img = view->captureView(ID, COLORBIT_24, IMAGE_320X240); 
	       char *buf = img->getBuffer();
                // the image
                IplImage *image=cvCreateImage(cvSize(320,240),8,3);
                memcpy(image->imageData,buf,image->imageSize);

                ViewImage *dis_img = view->distanceSensor2D(0.0, 255.0, ID,DEPTHBIT_8,IMAGE_320X240);  
                char *dis_buf = img->getBuffer();
                IplImage *dis_image=cvCreateImage(cvSize(320,240),8,1);
                memcpy(dis_image->imageData,dis_buf,dis_image->imageSize);
                
                int tb_count=0;
		int red_count=0;
		int blue_count=0;
		int green_count=0;
                CvPoint tb_pt;//涓績鐐?
		tb_pt.x=0;tb_pt.y=0;
		CvPoint red_pt;//绾㈣壊鍨冨溇妗朵腑蹇冪偣
		red_pt.x=0;red_pt.y=0;
		CvPoint blue_pt;//钃濊壊
		blue_pt.x=0;blue_pt.y=0;
		CvPoint green_pt;//缁胯壊
		green_pt.x=0;green_pt.y=0;
                for( int i = 0; i < image->height; i++)  
                {  
                       for(int j = 0; j < image->widthStep; j+=3)  
                       {  
                           int B=CV_IMAGE_ELEM(image,unsigned char,i,j);
                           int G=CV_IMAGE_ELEM(image,unsigned char,i,j+1);
                           int R=CV_IMAGE_ELEM(image,unsigned char,i,j+2);
                           //鍖哄煙鍨冨溇绠?
                              if((B>0&&B<150)&&(G>165&&G<190)&&(R>165&&R<190)&&(fabs(double(R-G))<10))
		              {
                               tb_count++;
                               tb_pt.x+=j/3;
                               tb_pt.y+=i;
                               //std::cout<<"find"<<std::endl;
                               CV_IMAGE_ELEM(image,unsigned char,i,j)=255;
                               CV_IMAGE_ELEM(image,unsigned char,i,j+1)=0;
                               CV_IMAGE_ELEM(image,unsigned char,i,j+2)=0;
                                
				
			       }
			       else
			       {
					//red
				        if((B<120)&&(G<100)&&(R>140)&&(fabs(double(B-G))<10))
				        {
				                red_count++;
                                                red_pt.x+=j/3;
                                                red_pt.y+=i;
					        
					        //std::cout<<"find"<<std::endl;
					        CV_IMAGE_ELEM(image,unsigned char,i,j)=0;
					        CV_IMAGE_ELEM(image,unsigned char,i,j+1)=0;
					        CV_IMAGE_ELEM(image,unsigned char,i,j+2)=255;
                                        
				        }
				        else
				        {
				                //blue
					        if((B>100)&&(G<150&&G>50)&&(R<50)&&(fabs(double(B-G))<50))
					        {
					                blue_count++;
                                                        blue_pt.x+=j/3;
                                                        blue_pt.y+=i;
					                CV_IMAGE_ELEM(image,unsigned char,i,j)=0;
							CV_IMAGE_ELEM(image,unsigned char,i,j+1)=255;
							CV_IMAGE_ELEM(image,unsigned char,i,j+2)=0;
					        }
					        else
					        {
					                //green
					                if((R>70&&R<90)&&(G>120&&G<140)&&(B>70&&B<90))
					                {
					                        green_count++;
                                                                green_pt.x+=j/3;
                                                                green_pt.y+=i;
					                        CV_IMAGE_ELEM(image,unsigned char,i,j)=255;
								CV_IMAGE_ELEM(image,unsigned char,i,j+1)=255;
								CV_IMAGE_ELEM(image,unsigned char,i,j+2)=0;
					                }
					                else
					                {
					                        CV_IMAGE_ELEM(image,unsigned char,i,j)=255;
					                        CV_IMAGE_ELEM(image,unsigned char,i,j+1)=255;
					                        CV_IMAGE_ELEM(image,unsigned char,i,j+2)=255;
					                
					                }
					        
					        }
				        }
	                   
				}
                       }  
  
                } 
                
               
                //cvCircle(image,d_pt,10,CV_RGB(255,0,0),3,8);  
                if(tb_count>2000)  //trashbox detected
                {
			tb_pt.x/=tb_count;
			tb_pt.y/=tb_count;	
                     if(red_count>20)
                     {
			red_pt.x/=red_count;
			red_pt.y/=red_count;
                        cvCircle(image,red_pt,10,CV_RGB(0,0,0),3,8);
                     }
                     if(green_count>20)
                     {
			green_pt.x/=green_count;
			green_pt.y/=green_count;
                        cvCircle(image,green_pt,10,CV_RGB(0,0,0),3,8);
                     }
                     if(blue_count>20)
                     {
			blue_pt.x/=blue_count;
			blue_pt.y/=blue_count;
                        cvCircle(image,blue_pt,10,CV_RGB(0,0,0),3,8);
						
                     }
                }
                
                cvNamedWindow("DIS");
                cvShowImage("DIS",image);
                cvWaitKey(30);
                delete img;
                delete dis_img;
                cvReleaseImage(&image);
                cvReleaseImage(&dis_image);

}
//********************************************************************
extern "C" Controller * createController()
{
	return new RobotController;  
}
