
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"

#include <sstream>
#include <math.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <ViewImage.h>

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )

class SendController : public Controller
{
public:
  	//銈枫儫銉ャ儸銉笺偡銉с兂闁嬪鏅傘伀涓€搴︺仩銇戝懠鍑恒仌銈屻倠闁㈡暟onInit銇埄鐢ㄣ倰瀹ｈ█銇椼伨銇?	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	void onRecvMsg(RecvMsgEvent &evt);
	void onCollision(CollisionEvent &evt);
        

public:
	//鎶撳彇鍑芥暟
	void graspObject(unsigned char distance);
	CvPoint detect(IplImage* img);
	void getROI(IplImage* dis_image,CvRect& ROI);
	IplImage* TemplateMatch(IplImage* img,IplImage* tp,CvPoint &pt,IplImage* dis_image,double &m,int flag);
	void findCup(IplImage *tp,int ID,float dis_t); //return the maxval(鍖归厤绋嬪害)
	IplImage* findTrashBox(IplImage *tp,int ID,float dis_t,double m_t,double &m);
	void findTable(int ID);
	
	

public:
	//SendController();
	RobotObj *my;

  	double velocity;
  	int i;
 
 	int j;
	int c;

 	double Weel_one;
 	double Weel_two;

	double Rarm_1;

	bool m_grasp;
	bool grasping;
	bool openHand;
        bool flag;   

	int rec;
	
	int joint_index;
	int joint_deg;

	ViewService* m_view;  //camera connect

	std::string msg;  // 鍛戒护妗嗗唴鐨勪俊鎭?public:
        char *joint[29];  //鍏宠妭鐨勫悕绉?    
	unsigned char distance_pre;
	unsigned char distance;  //璺濈
	IplImage* tpimage;       //template image
	
};

void SendController::onInit(InitEvent &evt)
{
	c = 9;
	
	joint_index = -1;
	joint_deg = 0;
	m_grasp = false;
	grasping = false;
	openHand=false;
        flag=true;

	m_view = (ViewService*)connectToService("SIGViewer");

 	Weel_one = 0.0;
 	Weel_two = 0.0;

	Rarm_1 = 0.0;
	rec = -1;
        distance_pre=88;
        distance=255;
	//flg = false;

	int argc =0;
	char** argv = NULL;
	my = getRobotObj(myname());
	my->setWheel(10.0, 10.0);
	
	velocity = 1;

        joint[0] =              "HEAD_JOINT0";
	joint[1] =		"HEAD_JOINT1";
	joint[2] =		"LARM_JOINT0";
	joint[3] =		"LARM_JOINT1";
	joint[4] =		"LARM_JOINT3";
	joint[5]=		"LARM_JOINT4";
	joint[6] =		"LARM_JOINT5";
	joint[7] =		"LARM_JOINT6";
	joint[8] =		"LARM_JOINT7";
	joint[9] =		"LBWHEEL_JOINT0";
	joint[10] =		"LFWHEEL_JOINT0";
	joint[11] =		"RARM_JOINT0";
	joint[12] =		"RARM_JOINT1";
	joint[13] =		"RARM_JOINT3";
	joint[14] =		"RARM_JOINT4";
	joint[15] =		"RARM_JOINT5";
	joint[16] =		"RARM_JOINT6";
	joint[17] =		"RARM_JOINT7";
	joint[18] =		"RBWHEEL_JOINT0";
	joint[19] =		"RFWHEEL_JOINT0";
	joint[20] =		"ROOT_JOINT0";
	joint[21] =		"ROOT_JOINT1";
	joint[22] =		"ROOT_JOINT2";
	joint[23] =		"WAIST_JOINT0";
	joint[24] =		"WAIST_JOINT1";
	joint[25] =		"WAIST_JOINT2";
        joint[26] =               "moveforward";
        joint[27] =               "turnleft";
	joint[28] =		"turnright";
	
        tpimage=cvLoadImage("/home/robocup/myRoboCup/template/33.png");
}

double SendController::onAction(ActionEvent &evt)
{
	
      /*if(rec == 1)
	{
		for(int i = 0; i < 26; i++)
		{
			Vector3d v3d;
			if(my->getJointPosition(v3d, joint[i]) == true)
			{
				std::cout << joint[i] << "\tx = " 
				<< v3d.x() << "\ty = " << v3d.y() 
				<< "\tz = " << v3d.z() << std::endl;  
			}
		}
		rec = 0;	
	}*/
	if(joint_index >= 0 && joint_index < 26)
	{
		float rad = joint_deg / 180.0 * PI;
		my->setJointVelocity(joint[joint_index], rad, rad);
		sleep(1.0);
		my->setJointVelocity(joint[joint_index], 0, 0);
		joint_index = -1;
	}
        if(joint_index ==26)
	{
                //鍚戝墠璧?		Weel_one = joint_deg/10.0;
                Weel_two = joint_deg/10.0;
                my->setWheelVelocity(Weel_one,Weel_two);
                sleep(1.0);
                my->setWheelVelocity(0,0);
		//joint_index = -1;
		
		
	}
        if(joint_index ==27)
	{
                //鍚戝乏杞?		Weel_one =joint_deg / 180.0 * PI;
                Weel_two=-joint_deg / 180.0 * PI;
                my->setWheelVelocity(Weel_one,Weel_two);
                sleep(1.0);
                my->setWheelVelocity(0,0);	
		//joint_index = -1;			
	}
        if(joint_index ==28)
	{
                //鍚戝彸杞?		Weel_two =joint_deg / 180.0 * PI;
                Weel_one=-joint_deg / 180.0 * PI;
                my->setWheelVelocity(Weel_one,Weel_two);
                sleep(1.0);
                my->setWheelVelocity(0,0);	
		//joint_index = -1			
	}
        
        
	if(m_grasp)
	{
		flag=false;
	}
	
        if(strcmp("capture", msg.c_str())==0)//flag
	{

                IplImage *tp=cvLoadImage("/home/robocup/catkin_ws/src/sig_ros/template/cup.png"); 
                findCup(tp,2,35);             	
                //msg="none";
                                           
        }
     
 	if(strcmp("capture", msg.c_str())==0)//flag)
	{
	
	  //std::cout<<"show"<<std::endl;
	    
	    double m;
	    IplImage* temp2= findTrashBox(tpimage,3,35,0.6,m);
	    if(m>0.6)
	    {
	        tpimage=temp2;
	    }
            cvNamedWindow("opencv");
            cvShowImage("opencv",tpimage);
            cvWaitKey(30);
         
            
                                           
        }
        if(flag)
        {
             findTable(3);
             Weel_one=0.2;
             Weel_two=-0.2;		  
             my->setWheelVelocity(Weel_one,Weel_two);
	     sleep(1.0);
	     my->setWheelVelocity(0,0); 
             
        }	
        
               
	if(grasping == true)
	{
		graspObject(distance);
		grasping = false;
	}
             
  	//1绉掋亰銇嶃伀onAction銇屽懠銇冲嚭銇曘倢銇俱仚
  	return 0.1;
}

void SendController::onRecvMsg(RecvMsgEvent &evt)
{
 	std::string sender = evt.getSender();

  	// 鏂囧瓧鍒椼倰鍙栧緱
  	msg = evt.getMsg();

  	LOG_MSG(("message : %s", msg.c_str()));

        //鍒嗚В鍛戒护锛屼娇鍏宠妭杩愬姩
	joint_index = atoi(msg.c_str());
	double d = atof(msg.c_str());
	joint_deg = (d - joint_index) * 100;
	std::cout << joint_index << "\t" << joint_deg << std::endl;

        
}

void SendController::onCollision(CollisionEvent &evt)
{
	if (m_grasp == false)
	{
		typedef CollisionEvent::WithC C;

		const std::vector<std::string> & with = evt.getWith();
		const std::vector<std::string> & mparts = evt.getMyParts();
		for(int i = 0; i < with.size(); i++)
		{
			if("can_0" == with[i])
			{

				if(mparts[i] == "LARM_LINK7")
				{
					SimObj *my = getObj(myname());
					CParts * parts = my->getParts("LARM_LINK7");
					if(parts->graspObj(with[i])) 
					{
						m_grasp = true;
					}
				}
			}
		}
	}
}
CvPoint SendController::detect(IplImage* img)
{
   int x=0,y=0;
   int count=0;
   for( int i = 0; i < img->height; i++)  
    {  
       for(int j = 0; j < img->widthStep; j+=3)  
       {  
           int B=CV_IMAGE_ELEM(img,unsigned char,i,j);
           int G=CV_IMAGE_ELEM(img,unsigned char,i,j+1);
           int R=CV_IMAGE_ELEM(img,unsigned char,i,j+2);
           //鍒ゆ柇绾㈣壊鍖哄煙
           if(B<100&&G<100&R>180)
	   {
               count++;
               x+=i;
               y+=j/3;
               //std::cout<<"find"<<std::endl;
               CV_IMAGE_ELEM(img,unsigned char,i,j)=255;
               CV_IMAGE_ELEM(img,unsigned char,i,j+1)=255;
               CV_IMAGE_ELEM(img,unsigned char,i,j+2)=255;
                
	   }
       }  
  
    }  
    CvPoint pt;
    pt.x=y/count;
    pt.y=x/count;
    std::cout<<pt.x<<" "<<pt.y<<std::endl;
    return pt;
   // cvCircle(img,pt,20,CV_RGB(255,0,0),2,8,0);
   // cvSaveImage("/home/robocup/catkin_ws/src/sig_ros/cam.bmp",img);
   // Weel_one=0;
   // Weel_two=0.1;
   // my->setWheelVelocity(Weel_one,Weel_two);
   //// sleep(1.0);
}

void SendController::getROI(IplImage* dis_image,CvRect& ROI)
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

void SendController::graspObject(unsigned char distance)
{
	//寰楀埌鍚勪釜鍙傛暟
	double head_angular = my->getJointAngle("HEAD_JOINT1");
	//std::cout << "head = " << head_angular << std::endl;
	double slope = 9.44 / tan(head_angular) + distance;
	//std::cout << "slope = " << slope << std::endl;
	double y = 96.0 + 9.44 / cos(head_angular) - slope * sin(head_angular);
	//std::cout << "y = " << y << std::endl;
	double z = slope * cos(head_angular);
	//std::cout << "z " << z << std::endl;
	double l = sqrt((84.0 - y) * (84.0 - y) + z * z);
	//std::cout << "l = " << l << std::endl;
		
	/*for(int i = 0; i < 26; i++)
	{
		Vector3d v3d;
		if(my->getJointPosition(v3d, joint[i]) == true)
		{
			std::cout << joint[i] << "\tx = " 
			<< v3d.x() << "\ty = " << v3d.y() 
			<< "\tz = " << v3d.z() << std::endl;  
		}
	}*/
	double theta1 = -((atan(z / (84 - y)) - acos((l * l + 17.6 * 17.6 - 28 * 28) / 2 / l / 17.6)));
	double theta2 = -(PI - acos((28 * 28 + 17.6 * 17.6 - l * l) / 2 / 28 / 17.6));
		
	//std::cout << theta1 * 180 / PI << ":" << theta2 * 180 / PI << std::endl;

	double old_theta0 = my->getJointAngle("LARM_JOINT0");
	double old_theta1 = my->getJointAngle("LARM_JOINT1");
	double old_theta2 = my->getJointAngle("LARM_JOINT4");

	double v_theta0 = -0.2 - old_theta0;
	double v_theta1 = theta1 - old_theta1;
	double v_theta2 = theta2 - old_theta2;

	my->setJointVelocity("LARM_JOINT0", v_theta0, v_theta0);
	my->setJointVelocity("LARM_JOINT1", v_theta1, v_theta1);
	my->setJointVelocity("LARM_JOINT4", v_theta2, v_theta2);
	sleep(1.0);
	my->setJointVelocity("LARM_JOINT0", 0, 0);
	my->setJointVelocity("LARM_JOINT1", 0, 0);
	my->setJointVelocity("LARM_JOINT4", 0, 0);
}

IplImage* SendController::TemplateMatch(IplImage* img,IplImage* tp,CvPoint &pt,IplImage* dis_image,double &m,int flag)
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
	
		//cvMatchTemplate( img, dst, res, CV_TM_CCORR_NORMED);   //鍖归厤缁撴灉瀛樺湪res涓?		//cvMatchTemplate( img, dst, res, CV_TM_SQDIFF_NORMED ); 
		//cvMatchTemplate( img, dst, res, CV_TM_CCORR ); 
		//cvMatchTemplate( img, dst, res, CV_TM_SQDIFF ); 
		//cvMatchTemplate( img, dst, res, CV_TM_CCOEFF ); 
		cvMatchTemplate( img, dst, res, CV_TM_CCOEFF_NORMED );

                //input single-channel array ,find the minimum and maximum element values and their positions.
		cvMinMaxLoc( res, &temp_minval, &temp_maxval, &temp_minloc, &temp_maxloc, 0 );
		//cout<<"绗?<<i<<"娆″尮閰嶏紝鍖归厤姣斾緥涓?<< templateScale[i]<<"妯℃澘澶у皬涓?<<dst_size.width<<"*"<<dst_size.height<<"鍖归厤绋嬪害涓?:"<<temp_maxval<<endl;
		
		if(temp_maxval>=maxval)  //瀵绘壘100骞呮ā鏉垮浘鍍忔渶鍖归厤鐨勭粨鏋?		{
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


void SendController::findCup(IplImage *tp,int ID,float dis_t)
{
        //tp----------------template image 
        //dis_t-------------the stop distance
        //index-------------the camera ID
        
        ViewImage *img = m_view->captureView(ID, COLORBIT_24, IMAGE_320X240); 	  
	char *buf = img->getBuffer();
        IplImage *image=cvCreateImage(cvSize(320,240),8,3);
        memcpy(image->imageData,buf,image->imageSize); 
                
        CvPoint pp;
                         
        //the distance image    
        ViewImage *dis_img = m_view->distanceSensor2D(0.0, 255.0, ID,DEPTHBIT_8,IMAGE_320X240);    
        char *dis_buf = dis_img->getBuffer(); 
        IplImage* dis_image=cvCreateImage(cvSize(320,240),8,1);
        memcpy(dis_image->imageData,dis_buf,dis_image->imageSize); 
        
        double m;      
        TemplateMatch(image,tp,pp,dis_image,m,1);
	
        LOG_MSG((" x = %d",pp.x));
        LOG_MSG((" y = %d",pp.y));
                
        //cvNamedWindow("dis");
        //cvShowImage("dis",dis_image);
        //cvWaitKey(0);  
                
        int width = dis_img->getWidth();
        int height = dis_img->getHeight();   
        int index = pp.y*width + pp.x;  
        distance = dis_buf[index]; 
                  
        LOG_MSG((" distance = %d",distance));
        LOG_MSG((" distance_pre = %d",distance_pre));
                
        //hold up the arm
	if(distance < 100 && distance > 60)
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
        cvReleaseImage(&dis_image); 
        //cvDestroyAllWindows();  
}

IplImage* SendController::findTrashBox(IplImage *tp,int ID,float dis_t,double m_t,double &m)
{
               ViewImage *img = m_view->captureView(ID, COLORBIT_24, IMAGE_320X240); 
	  
	       char *buf = img->getBuffer();
		
                // the image
                IplImage *image=cvCreateImage(cvSize(320,240),8,3);
                memcpy(image->imageData,buf,image->imageSize);

                CvPoint pp;

                IplImage* temp=TemplateMatch(image,tp,pp,NULL,m,0);
                
                std::cout<<" m=   "<<m<<std::endl;
                
                LOG_MSG((" x = %d",pp.x));
                LOG_MSG((" y = %d",pp.y));
          
                //the distance image    
                ViewImage *dis_img = m_view->distanceSensor2D(0.0, 255.0, ID,DEPTHBIT_8,IMAGE_320X240);    
                char *dis_buf = dis_img->getBuffer(); 
                            
                
                int width = dis_img->getWidth();
                int height = dis_img->getHeight();   
                int index = pp.y*width + pp.x;  
                distance = dis_buf[index]; 
                  
                LOG_MSG((" distance = %d",distance));
                
                if((m>m_t)&&(m<=1))
                {
                        //hold up the arm
		        if(distance < 100 && distance > 60)
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
                                       Weel_one=(distance-dis_t)/50.0;
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
                                Weel_one=(pp.x-160)/1000.0;
		                Weel_two=-(pp.x-160)/1000.0;		  
		                my->setWheelVelocity(Weel_one,Weel_two);
		                sleep(1.0);
		                my->setWheelVelocity(0,0); 
		        } 
		}
		else
		{
		     //if(distance>50)
		     {
		       //turn
		       Weel_one=0.2;
		       Weel_two=-0.2;		  
		       my->setWheelVelocity(Weel_one,Weel_two);
		       sleep(1.0);
		       my->setWheelVelocity(0,0); 
		     }
		     
		}       
		//the conditions of stop
                if(((pp.x-160)>-10)&&((pp.x-160)<10)&&((distance-dis_t)>-2)&&((distance-dis_t)<2))
                {
			flag = false;
			openHand = true;
			std::cout<<"stop"<<std::endl;
		}
		
		delete img; 
		delete dis_img;   
                cvReleaseImage(&image); 
                return temp;
                
}
void SendController::findTable(int ID)
{
               ViewImage *img = m_view->captureView(ID, COLORBIT_24, IMAGE_320X240); 
	       char *buf = img->getBuffer();
                // the image
                IplImage *image=cvCreateImage(cvSize(320,240),8,3);
                memcpy(image->imageData,buf,image->imageSize);

                ViewImage *dis_img = m_view->distanceSensor2D(0.0, 255.0, ID,DEPTHBIT_8,IMAGE_320X240);  
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
                if(dt_count>10)
                { 
                	d_pt.x /= dt_count;
                	d_pt.y /= dt_count;
                	cvCircle(image,d_pt,10,CV_RGB(255,0,0),3,8);
                }
		if(zz_count>10)
                { 
                	z_pt.x /= zz_count;
                	z_pt.y /= zz_count;
                	cvCircle(image,z_pt,10,CV_RGB(0,0,255),3,8);
                }
                cvNamedWindow("DIS");
                cvShowImage("DIS",image);
                cvWaitKey(30);
                delete img;
                delete dis_img;
                cvReleaseImage(&image);
                cvReleaseImage(&dis_image);

}

extern "C"  Controller * createController ()
{
  	return new SendController;
}
