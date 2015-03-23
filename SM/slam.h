// SLAM_H
#ifndef SLAM_H
#define SLAM_H

#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  

#include <iostream>
#include <iomanip>

#define PI 3.1415926535797

using namespace std;

class SlamController : public Controller
{  
public:  // basic fun
  	void onInit(InitEvent &evt);  
  	double onAction(ActionEvent&);  
  	void onRecvMsg(RecvMsgEvent &evt); 
  	void onCollision(CollisionEvent &evt); 

public:	// basic var
	RobotObj *m_robot;
	ViewService *m_view;

public:	// basic action
	void MOVEFORWARD(int distance);
	void MOVEBACKWARD(int distance);
	void TURNLEFT();
	void TURNRIGHT();
	void TURNBACK();
	void STOP();
	// advance action
	void MOVELEFT();
	void MOVERIGHT();

public:	// bottom fun
	void _SLEEP_();
	void _SET_VEL_(int distance);

public:	// test fun
	void getPosition();

public:
	// dis sense
	int* GETHLINE(int camID);
	Vector3d* GETHPOINTA(int* data);

public:
	// vision sense
};  
  
void SlamController::onInit(InitEvent &evt) 
{  
	m_robot = this->getRobotObj(this->myname());
	m_view = (ViewService*)connectToService("SIGViewer");

	m_robot->setWheel(10.0, 10.0);
	m_robot->setJointVelocity("LARM_JOINT4", -PI * 3.0 / 4.0, -PI * 3.0 / 4.0);
	_SLEEP_();
	m_robot->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
	m_robot->setJointVelocity("RARM_JOINT4", -PI * 3.0 / 4.0, -PI * 3.0 / 4.0);
	_SLEEP_();
	m_robot->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
}  
  
double SlamController::onAction(ActionEvent &evt) 
{  
	MOVEFORWARD(100);
	TURNLEFT();
	getPosition();
  	return 0.1;      
}  
  
void SlamController::onRecvMsg(RecvMsgEvent &evt) 
{  
}  

void SlamController::onCollision(CollisionEvent &evt) 
{ 
}

void SlamController::getPosition()
{ 
	Vector3d result;
	m_robot->getPosition(result);
	cout << "getPosition " << result.x() << "\t" 
		<< result.y() << "\t" 
		<< result.z() << endl;
}
#endif
