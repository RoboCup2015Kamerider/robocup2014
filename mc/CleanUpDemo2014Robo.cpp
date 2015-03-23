#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <algorithm>

//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class DemoRobotController : public Controller
{
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

	void stopRobotMove(void);
	double goToObj(Vector3d pos, double range);
	double rotateTowardObj(Vector3d pos);
	void recognizeObjectPosition(Vector3d &pos, std::string &name);
	void throwTrash(void);
	double goGraspingObject(Vector3d &pos);
	void neutralizeArms(double evt_time);
	void prepareThrowing(double evt_time);

private:
	RobotObj *m_robotObject;
	ViewService *m_view;

	int m_task;
	int m_state; 
	double refreshRateOnAction;

	std::string m_trashName1;
	std::string m_trashName2;
	std::string m_graspObjectName;

	std::string m_trashBoxName1;
	std::string m_trashBoxName2;

	double m_angularVelocity;  // rotation speed of the wheel
	double m_jointVelocity;    // rotation speed around the joint
	double m_radius;           // radius of the wheel
	double m_distance;         // length of wheel-track
	double m_movingSpeed;      // actual velocity of the moving robot

	// times wasted for moving, adjusting or driving
	double m_time;
	double m_time1;
	double m_time4;

	//positions
	Vector3d m_frontTrashBox1;
	Vector3d m_frontTrashBox2;
	Vector3d m_relayPoint1;
	Vector3d m_frontDesk1;

	// condition flag for grasping trash
	bool m_grasp;

  // angular parameter used to put robot's hands down
  double thetaA;
};


void DemoRobotController::onInit(InitEvent &evt)
{
	// get robot's name
	m_robotObject = getRobotObj(myname());

	// set wheel configuration
	m_radius = 10.0;
	m_distance = 10.0;
	m_robotObject->setWheel(m_radius, m_distance);

	m_time = 0.0;
	m_time1 = 0.0;
	m_time4 = 0.0;

	m_task = 0;		// number of taks
	m_state = 0;  // switch of initial behavior
	refreshRateOnAction = 0.1;     // refresh-rate for onAction proc.

	// angular velocity of wheel and moving speed of robot
	m_angularVelocity = 1.5;
	m_movingSpeed = m_angularVelocity*m_radius;  // conversion: rad/ms -> m/ms)

	// rotation speed of joint
	m_jointVelocity = 0.5;

	m_trashName1 = "petbottle_2";
	m_trashName2 = "can_1";

	m_trashBoxName1 = "trashbox_0";  // for recycle
	m_trashBoxName2 = "trashbox_2";  // for can

	// set positions;
	m_frontTrashBox1  = Vector3d(-80.0, 0.0, -90);  // for recycle material
	m_frontTrashBox2  = Vector3d( 120.0, 0.0, -90);  // for can material
	m_relayPoint1     = Vector3d(190.0, 0.0, -65.0);
	m_frontDesk1			= Vector3d(270.0, 0.0, -65.0);

	m_grasp = false;
}


double DemoRobotController::onAction(ActionEvent &evt)
{
	switch(m_state) {
		case 0: {
			break;
		}
		case 1: {
			this->stopRobotMove();
			break;
		}
		case 50: {  // detour: rotate toward relay point 1
			if(evt.time() >= m_time) {
				this->stopRobotMove();

				double l_moveTime = rotateTowardObj(m_relayPoint1);

				m_time = l_moveTime+evt.time();
				m_state = 60;
			}
			break;
		}
		case 60: {  // detour: go toward relay point 1
			if(evt.time() >= m_time) {
				this->stopRobotMove();

				double l_moveTime = goToObj(m_relayPoint1, 0.0);

				m_time = l_moveTime+evt.time();
				m_state = 70;
			}
			break;
		}
		case 70: {  // rotate toward the position in front of trash
			if(evt.time() >= m_time) {
				this->stopRobotMove();

				double l_moveTime = rotateTowardObj(m_frontDesk1);

				m_time = l_moveTime+evt.time();
				m_state = 80;
			}
			break;
		}
		case 80: {  // go toward the position in front of trash
			if(evt.time() >= m_time) {
				this->stopRobotMove();

				double l_moveTime = goToObj(m_frontDesk1, 0.0);

				m_time = l_moveTime+evt.time();
				m_state = 90;
			}
			break;
		}
		case 90: {  // rotate toward the trash
			if(evt.time() >= m_time) {
				this->stopRobotMove();

				Vector3d l_tpos;

				if(m_task == 1)
					this->recognizeObjectPosition(l_tpos, m_trashName1);
				else
					this->recognizeObjectPosition(l_tpos, m_trashName2);

				double l_moveTime = rotateTowardObj(l_tpos);

				m_time = l_moveTime+evt.time();
				m_state = 100;
			}
			break;
		}
		case 100: {  // prepare the robot arm to grasping the trash
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				this->neutralizeArms(evt.time());

				m_state = 105;
			}
			break;
		}
		case 105: {  // fix robot direction for grasping
			if(evt.time() >= m_time1) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4) {
				Vector3d l_tpos;
				if(m_task == 1)
					this->recognizeObjectPosition(l_tpos, m_trashName1);
				else
					this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = rotateTowardObj(l_tpos);

				m_time = l_moveTime+evt.time();

				m_state = 110;
			}
			break;
		}
		case 110: {  // approach to the trash
			if(evt.time() >= m_time) {
				this->stopRobotMove();

				Vector3d l_tpos;
				
				if(m_task == 1)
					this->recognizeObjectPosition(l_tpos, m_trashName1);
				else
					this->recognizeObjectPosition(l_tpos, m_trashName2);

				double l_moveTime = goToObj(l_tpos, 30.0);
				m_time = l_moveTime+evt.time();

				m_state = 120;
			}
			break;
		}
		case 120: {  // try to grasp trash
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				
				Vector3d l_tpos;

				if(m_task == 1)
					this->recognizeObjectPosition(l_tpos, m_trashName1);
				else
					this->recognizeObjectPosition(l_tpos, m_trashName2);
					
				double l_moveTime = goGraspingObject(l_tpos);
				m_time = l_moveTime+evt.time();

				m_state = 125;
			}
			break;
		}
		case 125: {
			if(evt.time() >= m_time) {
				m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				this->neutralizeArms(evt.time());

				m_state = 130;
			}
			break;
		}
		case 130: {
			if(evt.time() >= m_time1) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4) {

				m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 20./m_movingSpeed + evt.time();

				m_state = 150;
			}
			break;
		}
		case 150: {
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				double l_moveTime;
				if(m_task == 1)
					l_moveTime = rotateTowardObj(m_frontTrashBox1);
				else
					l_moveTime = rotateTowardObj(m_frontTrashBox2);

				m_time = l_moveTime + evt.time();
				m_state = 160;
			}
			break;
		}
		case 160: {
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				double l_moveTime;
				if(m_task == 1)
					l_moveTime = goToObj(m_frontTrashBox1,0.0);
				else
					l_moveTime = goToObj(m_frontTrashBox2,0.0);
				m_time = l_moveTime + evt.time();
				m_state = 161;
			}
			break;
		}
		case 161: {
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				this->prepareThrowing(evt.time());

				m_state = 165;
			}
			break;
		}
		case 165: {
			if(evt.time() >= m_time1) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4) {

				Vector3d l_tpos;
				if(m_task == 1)
					this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				else
					this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + evt.time();

				m_state = 170;
			}
			break;
		}
		case 170: {
			if(evt.time() >= m_time) {

				this->stopRobotMove();
				Vector3d l_tpos;
				if(m_task == 1)
					this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				else
					this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = goToObj(l_tpos, 50.0);
				m_time = l_moveTime + evt.time();

				m_state = 180;
			}
			break;
		}
		case 180: {
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				Vector3d l_tpos;
				if(m_task == 1)
					this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				else
					this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + evt.time();

				m_state = 200;
			}
			break;
		}
		case 200: {  // throw trash and get back a bit
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				this->throwTrash();

				sleep(1);

				m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 50.0/m_movingSpeed + evt.time();

				m_state = 225;
			}
			break;
		}
		case 225: {  // recover robot arms
			if(evt.time() >= m_time) {
				this->stopRobotMove();
				this->neutralizeArms(evt.time());

				m_state = 240;
			}
			break;
		}
		case 240: {  // go next
			if(evt.time() >= m_time1) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4) {
				this->stopRobotMove();

				broadcastMsg("Task_finished");
				//broadcastMsg("Give_up");
				m_state = 0;
			}
			break;
		}

	}
	return refreshRateOnAction;
}

void DemoRobotController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();
	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	if(sender == "moderator_0"){
		if(msg == "Task_start"){
			m_task++;
			m_time = 0.0;
			m_state = 50;
			if(m_task == 1)	m_graspObjectName = m_trashName1;
			else	m_graspObjectName = m_trashName2;
		}
		if(msg == "Time_over"){
			m_time = 0.0;
			m_state = 0;
		}
		if(msg == "Task_end"){
			m_time = 0.0;
			m_state = 0;
		}
	}
}

void DemoRobotController::onCollision(CollisionEvent &evt)
{
	if (m_grasp == false) {
		typedef CollisionEvent::WithC C;
		// Get name of entity which is touched by the robot
		const std::vector<std::string> & with = evt.getWith();
		// Get parts of the robot which is touched by the entity
		const std::vector<std::string> & mparts = evt.getMyParts();

		// loop for every collided entities
		for(int i = 0; i < with.size(); i++) {
			if(m_graspObjectName == with[i]) {
				// If the right hand touches the entity
				if(mparts[i] == "RARM_LINK7") {
					SimObj *my = getObj(myname());
					CParts * parts = my->getParts("RARM_LINK7");
					if(parts->graspObj(with[i])) m_grasp = true;
				}
			}
		}
	}
}


void DemoRobotController::stopRobotMove(void) {
	m_robotObject->setWheelVelocity(0.0, 0.0);
}

double DemoRobotController::goToObj(Vector3d pos, double range)
{
	// get own position
	Vector3d robotCurrentPosition;
	//m_robotObject->getPosition(robotCurrentPosition);
	m_robotObject->getPartsPosition(robotCurrentPosition,"RARM_LINK2");

	// pointing vector for target
	Vector3d l_pos = pos;
	l_pos -= robotCurrentPosition;

	// ignore y-direction
	l_pos.y(0);

	// measure actual distance
	double distance = l_pos.length() - range;

	// start moving
	m_robotObject->setWheelVelocity(m_angularVelocity, m_angularVelocity);

	// time to be elapsed
	double l_time = distance / m_movingSpeed;

	return l_time;
}


double DemoRobotController::rotateTowardObj(Vector3d pos)
{
	// "pos" means target position
	// get own position
	Vector3d ownPosition;
	m_robotObject->getPartsPosition(ownPosition,"RARM_LINK2");

	// pointing vector for target
	Vector3d l_pos = pos;
	l_pos -= ownPosition;

	// ignore variation on y-axis
	l_pos.y(0);

	// get own rotation matrix
	Rotation ownRotation;
	m_robotObject->getRotation(ownRotation);

	// get angles arround y-axis
	double qw = ownRotation.qw();
	double qy = ownRotation.qy();
	double theta = 2*acos(fabs(qw));

	if(qw*qy < 0) theta = -1.0*theta;

	// rotation angle from z-axis to x-axis
	double tmp = l_pos.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// direction
	if(l_pos.x() > 0) targetAngle = -1.0*targetAngle;
	targetAngle += theta;

	double angVelFac = 3.0;
	double l_angvel = m_angularVelocity/angVelFac;

	if(targetAngle == 0.0) {
		return 0.0;
	}
	else {
		// circumferential distance for the rotation
		double l_distance = m_distance*M_PI*fabs(targetAngle)/(2.0*M_PI);

		// Duration of rotation motion (micro second)
		double l_time = l_distance / (m_movingSpeed/angVelFac);

		// Start the rotation
		if(targetAngle > 0.0) {
			m_robotObject->setWheelVelocity(l_angvel, -l_angvel);
		}
		else{
			m_robotObject->setWheelVelocity(-l_angvel, l_angvel);
		}

		return l_time;
	}
}

void DemoRobotController::recognizeObjectPosition(Vector3d &pos, std::string &name)
{
	// get object of trash selected
	SimObj *trash = getObj(name.c_str());

	// get trash's position
	trash->getPosition(pos);
}


void DemoRobotController::prepareThrowing(double evt_time)
{
	double thetaJoint1 = 50.0;
	m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;

	double thetaJoint4 = 65.0;
	m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;

}


void DemoRobotController::throwTrash(void)
{
	// get the part info. 
	CParts *parts = m_robotObject->getParts("RARM_LINK7");

	// release grasping
	parts->releaseObj();

	// wait a bit
	sleep(1);

	// set the grasping flag to neutral
	m_grasp = false;

	//sendMsg("moderator_0", "end of task");
}


double DemoRobotController::goGraspingObject(Vector3d &pos)
{
	double l_time;
	double thetaJoint4 = 30.0;

	m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

	l_time = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity;

	return l_time;
}


void DemoRobotController::neutralizeArms(double evt_time)
{
	double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0/(M_PI);
	double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0/(M_PI);
	double thetaJoint1 = -15 - angleJoint1;
	double thetaJoint4 = -110 - angleJoint4;

	if(thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

	if(thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT1", m_jointVelocity, 0.0);

	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;
}


//********************************************************************
extern "C" Controller * createController()
{
	return new DemoRobotController;  
}
