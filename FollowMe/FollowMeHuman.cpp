#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"
#include <unistd.h>
#include <algorithm>

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   

bool start;
bool sw;
using namespace std;

class MyController : public Controller {
public:
	void onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void onRecvMsg(RecvMsgEvent &evt);
	void onCollision(CollisionEvent &evt);

	/* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
	* @param  pos 回転したい方向の位置
	* @param  vel 回転速度
	* @param  now 現在時間
	* @return 回転終了時間
	*/
	double rotateTowardObj1(Vector3d pos, double vel, double now);
	double rotateTowardObj2(Vector3d pos, double vel, double now);
	//avoid
	double rotateTowardObj3(Vector3d pos, double vel, double now);

	/* @brief  位置を指定しその方向に進みます
	* @param  pos   行きたい場所
	* @param  vel   移動速度
	* @param  range 半径range以内まで移動
	* @param  now   現在時間
	* @return 到着時間
	*/
	double goToObj(Vector3d pos, double vel, double range, double now);

private:
	SimObj *my;
	// ViewService *m_view;

};

void MyController::onInit(InitEvent &evt)
{
	my = getObj(myname());

	start = false;
	sw = false;
	// 左手を下に下げます  
	my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));

	// 右手をsageます  
	my->setJointAngle("RARM_JOINT2", DEG2RAD(90));

}

double MyController::onAction(ActionEvent &evt)
{
	int count = 0;
	Vector3d pos;

	if (start == true){
		while (count<20){
			if (sw == false){
				my->getPosition(pos);
				my->setPosition(pos.x(), pos.y(), pos.z() - 10);
				my->setJointAngle("LARM_JOINT1", DEG2RAD(-30));
				my->setJointAngle("RLEG_JOINT2", DEG2RAD(-20));
				my->setJointAngle("RLEG_JOINT4", DEG2RAD(10));
				usleep(100000);
				my->setJointAngle("LARM_JOINT1", DEG2RAD(0));
				my->setJointAngle("RLEG_JOINT2", DEG2RAD(0));
				my->setJointAngle("RLEG_JOINT4", DEG2RAD(0));
				sw = true;
			}
			else{
				my->getPosition(pos);
				my->setPosition(pos.x(), pos.y(), pos.z() - 10);
				my->setJointAngle("RARM_JOINT1", DEG2RAD(-30));
				my->setJointAngle("LLEG_JOINT2", DEG2RAD(-20));
				my->setJointAngle("LLEG_JOINT4", DEG2RAD(10));
				usleep(100000);
				my->setJointAngle("RARM_JOINT1", DEG2RAD(0));
				my->setJointAngle("LLEG_JOINT2", DEG2RAD(0));
				my->setJointAngle("LLEG_JOINT4", DEG2RAD(0));
				sw = false;
			}
			count++;

			if(count==7){
				usleep(3000000);
			}

			start = false;
		}
	}

	return 0.1;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
	if (msg == "point1"){
		start = true;
	}
}

void MyController::onCollision(CollisionEvent &evt)
{

}

extern "C" Controller * createController() {
	return new MyController;
}

