#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <sys/time.h>

class MyController : public Controller {  
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	//void onCollision(CollisionEvent &evt);

private:
	BaseService *m_ref;   // Referee service
	double retValue;      // Refresh rate of the modification
	bool   colState;      // Collision state
	bool   pcolState;     // Collision state
	std::string roboName; // Robot's name
	std::string mdName;   // Moderator's name

	// position vectors and rotation matrices required to modify robot's behavior
	Vector3d crrPos;
	Vector3d prv1Pos;
	Rotation crrRot;
	Rotation prv1Rot;

	const static unsigned int jtnum=7;
	std::vector<std::string> jointName;
	double prv1JAng_r[jtnum];
	double crrJAng_r[jtnum];

	double rsLen;

	std::vector<std::string> m_entities;
	std::vector<std::string> m_entNames;
	int entNum;

	struct timeval t0, t1;

};

void MyController::onInit(InitEvent &evt)
{
	int i, size;

	retValue  = 0.08;
	roboName  = "robot_004";
	mdName    = "moderator_0";

	crrPos  = Vector3d(0,0,0);
	prv1Pos = Vector3d(0,0,0);

	getAllEntities(m_entities);
/*
	size = m_entities.size();
	for(i=0;i<size;i++){
		if((m_entities[i] != mdName) &&
		   (m_entities[i] != "score") &&
		   (m_entities[i] != "checkpoint_001") &&
		   (m_entities[i] != "checkpoint_002") &&
		   //(m_entities[i] != "man_000") && // for check
		   (m_entities[i] != roboName)){
			m_entNames.push_back(m_entities[i]);
		}
	}
	entNum = m_entNames.size();
*/

	gettimeofday(&t1, NULL);
}

double MyController::onAction(ActionEvent &evt)
{
	gettimeofday(&t0, NULL);
	int sec, msec;
	if (t0.tv_usec < t1.tv_usec) {
		sec = t0.tv_sec - t1.tv_sec - 1;
		msec= 1000000 + t0.tv_usec - t1.tv_usec;
	}
	else {
		sec = t0.tv_sec - t1.tv_sec;
		msec = t0.tv_usec - t1.tv_usec;
	}
	//LOG_MSG(("%d.%d",sec,msec));
	t1 = t0;

	// check whether Referee service is available or not
	bool available = checkService("FollowMeReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("FollowMeReferee");
	}

	// get information about the robot and renew it
	//----------------------------------
	SimObj *r_my = getObj(roboName.c_str());
	// for body configuration
	r_my->getPosition(crrPos);
	r_my->getRotation(crrRot);

	Vector3d rsLenVec(prv1Pos.x()-crrPos.x(), prv1Pos.y()-crrPos.y(), prv1Pos.z()-crrPos.z());
	rsLen = rsLenVec.length();

/*
	for(int k=0;k<entNum;k++){
		SimObj* locObj = getObj(m_entNames[k].c_str());
		CParts *parts = locObj->getMainParts();
		bool state = parts->getCollisionState();

		if(state){
			colState=true;       // collided with main body of robot
			if(rsLen == 0.0) {
				r_my->setRotation(prv1Rot);
			} else {
				r_my->setPosition(prv1Pos);
			}

			std::string msg = "FollowMeReferee/Collision with [" + m_entNames[k] + "]" "/-100";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
			break;
		}
	}
*/

	std::map<std::string, CParts *> partsm = r_my->getPartsCollection();
	for (SimObj::PartsM::const_iterator i=partsm.begin(); i!=partsm.end(); i++) {
		//LOG_MSG((i->first.c_str()));
		CParts *parts = r_my->getParts(i->first.c_str());
		bool state = parts->getCollisionState();

		if(state){
			colState=true;       // collided with main body of robot
			if(rsLen == 0.0) {
				r_my->setRotation(prv1Rot);
			} else {
				r_my->setPosition(prv1Pos);
			}

			if(!colState){
				prv1Pos=crrPos;
				prv1Rot=crrRot;
			} else if(pcolState){
				for(int i=0;i<jtnum;i++) prv1JAng_r[i]=crrJAng_r[i];
					pcolState = false;
				} else{
				colState = false;
			}

			std::string msg = "FollowMeReferee/Collision" "/-100";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
			break;
		}
	}

	if(!colState){
		prv1Pos=crrPos;
		prv1Rot=crrRot;
		//colState=false;     // reset collision condition
	} else if(pcolState){
		for(int i=0;i<jtnum;i++) prv1JAng_r[i]=crrJAng_r[i];
			pcolState = false;
		} else{
		//Do nothing on "collided" condition
		//LOG_MSG((colPtName.c_str()));
		colState = false;
	}

	return retValue;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
}


extern "C" Controller * createController() {
	return new MyController;
}
