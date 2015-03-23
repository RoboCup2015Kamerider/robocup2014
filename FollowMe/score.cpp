#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <sstream>
#include <sys/time.h>

using namespace std;

class MyController : public Controller {
public:
	void onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void onRecvMsg(RecvMsgEvent &evt);
	//void onCollision(CollisionEvent &evt);

private:
	BaseService *m_ref;   // Referee service
	double retValue;      // Refresh rate of the modification

	SimObj *m_my;
	bool check1;
	bool elevator;
	bool crowd;
	bool collision;
	bool check1_clear;
	bool elevator_clear;
	bool not_elevator;
	bool a;
	bool b;
	bool c;
	bool d;
	int total;

	//std::vector<std::string> m_entities;

	struct timeval t0, t1;
};

void MyController::onInit(InitEvent &evt)
{
	m_ref = NULL;
	retValue  = 0.05;

	check1 = false;
	elevator = false;
	crowd = false;
	collision = false;
	check1_clear = false;
	elevator_clear = false;
	not_elevator = false;
	a = false;
	b = false;
	c = false;
	d = false;
	total = 0;

	// ** Need to correctly connect to referee service
	m_my = getObj(myname());
	//getAllEntities(m_entities);
	//SimObj *obj = getObj("moderator_0");

}

double MyController::onAction(ActionEvent &evt)
{
	/*
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
	LOG_MSG(("%d.%d",sec,msec));
	t1 = t0;*/

	// check whether Referee service is available or not
	bool available = checkService("FollowMeReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("FollowMeReferee");
	}
	
	if (check1 == true && check1_clear == true && a == false){
		total = total + 300;
		stringstream ss;
		ss << total;
		string result = ss.str();
		//sendMsg("SIGViewer", result);
		std::string msg = "FollowMeReferee/Check point1 clear/300";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		a = true;
	}

	if (elevator_clear == false && elevator == true){
		total = total - 100;
		stringstream ss4;
		ss4 << total;
		string result4 = ss4.str();
		//sendMsg("SIGViewer", result4);
		std::string msg = "FollowMeReferee/Elevator" "/-100";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		LOG_MSG((msg.c_str()));
		}
		elevator = false;
	}

	if (elevator == true && elevator_clear == true && b == false){
		total = total + 300;
		stringstream ss2;
		ss2 << total;
		string result2 = ss2.str();
		sendMsg("SIGViewer", result2);
		std::string msg = "FollowMeReferee/Elevator clear" "/300";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		b = true;
		elevator_clear = false; //誤メッセージ判定のため
		elevator = false;
	}
	if (collision == true && c == false){
		total = 0;
		stringstream ss3;
		ss3 << total;
		string result3 = ss3.str();
		sendMsg("SIGViewer", result3);
		std::string msg = "FollowMeReferee/Collision" "/-100";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		c = true;
	}
	if (crowd == true && d == false){
		total = total + 400;
		stringstream ss5;
		ss5 << total;
		string result5 = ss5.str();
		sendMsg("SIGViewer", result5);
		std::string msg = "FollowMeReferee/Crowded loacation clear" "/400";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		d = true;
	}
	return retValue;
}

void MyController::onRecvMsg(RecvMsgEvent &evt) {
	string msg = evt.getMsg();
	if (msg == "checkpoint1_clear"){
		check1_clear = true;
	}
	else if (msg == "elevator_clear"){
		elevator_clear = true;
	}
	else if (msg == "Collision"){
		collision = true;
	}
	else if (msg == "check1"){
		check1 = true;
	}
	else if (msg == "elevator"){
		elevator = true;
	}
	else if (msg == "crowd"){
		crowd = true;
	}
}

/*void MyController::onCollision(CollisionEvent &evt) {
}*/

extern "C" Controller * createController() {
	return new MyController;
}


