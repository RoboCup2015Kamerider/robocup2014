#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <iostream>
#include <math.h>
#include <unistd.h>

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

struct coordinate{
	double x[255];
	double y[255];
	double z[255];
	double w[255];
};


bool first;
bool start;
bool elevator;
bool crowd;
bool end;
bool stop;
bool flg1;
bool flg2;
bool flg3;
bool ok;
bool elevator_end;

double dx,dy,dz;
double x,y,z,w;
int i; 




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
	double rotateTowardObj(Vector3d pos);

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
	std::vector<std::string> m_entities;

	coordinate temp; 
};

void MyController::onInit(InitEvent &evt) 
{
	start = false;
	elevator = false;
	crowd = false;
	end = false;
	stop = false;
	flg1 = false;
	flg2 = false;
	flg3 = false;
	first = false;
	ok=false;
	elevator_end = false;
	i=0;

	my = getObj(myname());
 
	// 左手を下に下げます  
	my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));  

	// 右手を下に下げます  
	my->setJointAngle("RARM_JOINT2", DEG2RAD(90));  

	if(first==false){
		FILE* fp;
		x=0;
		y=0;
		z=0;
		w=0; //チェックポイント

		dx=0;
		dy=0;
		dz=0;

		if((fp = fopen("node.txt", "r")) == NULL) {
			printf("File do not exist.\n");
			exit(0);
		}
		while(fscanf(fp, "%lf,%lf,%lf,%lf", &x, &y, &z,&w) != EOF) {
			temp.x[i]=x;
			temp.y[i]=y;
			temp.z[i]=z;
			temp.w[i]=w;
			i++;
		}
		fclose(fp);
		first = true;
		i=0;
	}

	getAllEntities(m_entities);

}

double MyController::onAction(ActionEvent &evt)
{
	int    count = 0;
	int    count2 = 0; //何歩歩くか
	double r = 0.0;    //2点間の直線距離
	double r2 = 0.0;
	bool   sw = false; //歩行の左右切り替え  
	int    j = 0;

	if(end==false){
		if(start==true){

			Vector3d pos;
			Vector3d npos;
			double angle;


			if(stop==false){
				my->getPosition(pos);

				npos.x(temp.x[i]); 
				npos.y(temp.y[i]); 
				npos.z(temp.z[i]); 

				dx=(temp.x[i]-pos.x());
				dy=(temp.y[i]-pos.y());
				dz=(temp.z[i]-pos.z());

				//angle = rotateTowardObj(npos);
				angle = -atan2(dx,dz);

				//if(angle < 0.0){
				//	angle = -1.0 * angle;
				//}

				my->setAxisAndAngle(0,1.0, 0, -angle);

				r=sqrt(pow(dx,2)+pow(dz,2));

				count2 = (int)r / 10;

				dx = dx/count2;
				dz = dz/count2;

				while(count<count2){
					if(sw == false){
						my->getPosition(pos);
						my->setPosition(pos.x()+dx,pos.y(),pos.z()+dz);
						my->setJointAngle("LARM_JOINT1", DEG2RAD(-30)); 
						my->setJointAngle("RLEG_JOINT2", DEG2RAD(-20));
						my->setJointAngle("RLEG_JOINT4", DEG2RAD(10));   
						usleep(100000);
						my->setJointAngle("LARM_JOINT1", DEG2RAD(0));  
						my->setJointAngle("RLEG_JOINT2", DEG2RAD(0)); 
						my->setJointAngle("RLEG_JOINT4", DEG2RAD(0)); 
						sw = true;
					}else{
						my->getPosition(pos);
						my->setPosition(pos.x()+dx,pos.y(),pos.z()+dz);
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
				}

				count = 0;
				count2= 0;
				i++;
			}//stopのところ

			Vector3d myPos;
			my->getPosition(myPos);

			int entSize = m_entities.size();
			for(int j = 0; j < entSize; j++){
				if(m_entities[j] == "robot_004"){
					// エンティティ取得
					SimObj *ent = getObj(m_entities[j].c_str());

					// 位置取得
					Vector3d tpos;
					ent->getPosition(tpos);

					Vector3d vec(tpos.x()-myPos.x(), tpos.y()-myPos.y(), tpos.z()-myPos.z());

					r2=sqrt((vec.x()*vec.x())+(vec.z()*vec.z()));

					if(r2<=200){
						stop=false;

						if(temp.w[i-1] == 1.0){
							std::string msg = "point1";  
							//"man_001"にメッセージを送信します  
							sendMsg("man_001", msg);
							elevator =false; 
						}else if(temp.w[i-1] == 2.0){
							std::string msg3 = "check1";
							sendMsg("score",msg3);
							elevator = false;
						}else if(temp.w[i-1] == 3.0){
							stop=true;
							if(flg1==false){
								//broadcastMsgToSrv("Elevator");
								std::string msg1 = "elevator_close";
								sendMsg("wall_008", msg1);
								flg1=true;
								elevator = false;
							} 
							if(elevator==true){
								if(flg2==false){
									std::string msg2 = "elevator_open";
									sendMsg("wall_008", msg2);
									std::string msg4 = "elevator";
									sendMsg("score", msg4);
									flg2=true;
								}
								if(ok==true){
									elevator = false;
									stop=false;
								}
							}
						}else if(temp.w[i-1]==4.0){
							std::string msg5 = "crowd";
							sendMsg("score", msg5);
							elevator = false;
						}else if(temp.w[i-1]==5.0){
							end=true;
						}
					}else if(r2>200){
						if(ok==true && flg3==false){
							stop=false;
							flg3=true;
							elevator = false;
						}else{
							stop = true;
							elevator = false;
						}
					}
				}
			}
		}
	}
	return 0.1;
}


double MyController::rotateTowardObj(Vector3d pos)
{
	// 自分の位置の取得
	Vector3d myPos;
	my->getPosition(myPos);

	// 自分の位置からターゲットを結ぶベクトル
	Vector3d tmpp = pos;
	tmpp -= myPos;

	// y方向は考えない
	tmpp.y(0);

	// 自分の回転を得る
	Rotation myRot;
	my->getRotation(myRot);

	// エンティティの初期方向
	Vector3d iniVec(0.0, 0.0, 1.0);

	// y軸の回転角度を得る(x,z方向の回転は無いと仮定)
	double qw = myRot.qw();
	double qy = myRot.qy();

	double theta = 2*acos(fabs(qw));

	if(qw*qy < 0)
	theta = -1*theta;

	// z方向からの角度
	double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// 方向
	if(tmpp.x() > 0) targetAngle = -1*targetAngle;
	targetAngle += theta;

	if(targetAngle == 0.0){
	return 0.0;
	}
	return targetAngle;
}


void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
	if(msg == "start"){
		start = true;
	}else if(msg == "elevator"){
		std::string msg0 ="elevator";
		//sendMsg("score",msg0);
		elevator = true;
	}else if(msg == "Collision"){
		//end = true;
	}else if(msg == "ok"){
		ok = true;
	}
}

void MyController::onCollision(CollisionEvent &evt)
{

}

extern "C" Controller * createController() {
	return new MyController;  
}


