#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
  
class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);
	void onCheckCollision();
	void onCheckRoom();

private:
  BaseService *m_ref;   // Referee service
  double retValue;      // Refresh rate of the modification
  bool   colState;      // Collision state
  bool  pcolState;      // Collision state
  std::string roboName; // Robot's name
  std::string mdName;   // Moderator's name

  const static unsigned int jtnum=7;
  std::vector<std::string> jointName;
  double prv1JAng_r[jtnum];
  double crrJAng_r[jtnum];

  std::vector<std::string> m_entities;
  std::vector<std::string> m_entNames;
  int entNum;

  std::vector<std::string> m_rooms;
  int m_roomState;

  // position vectors and rotation matrices required to modify robot's behavior
  Vector3d crrPos;
  Vector3d prv1Pos;
  Rotation crrRot;
  Rotation prv1Rot;

  double rsLen;
};  
  

void MyController::onInit(InitEvent &evt) {  
  
	m_rooms.push_back("living room"); //0
	m_rooms.push_back("kitchen"); //1
	m_rooms.push_back("lobby"); //2
	m_rooms.push_back("bed room"); //3
	m_roomState = 0;

	int i, cnt;

  retValue = 0.08;
  colState = false;
  pcolState = false;
  roboName = "robot_000";
  mdName   = "moderator_0";

  crrPos  = Vector3d(0,0,0);
  prv1Pos = Vector3d(0,0,0);

  getAllEntities(m_entities);

  // select objects to be observed
  cnt = m_entities.size();
  for(i=0;i<cnt;i++){
    if((m_entities[i] != mdName) &&
       (m_entities[i] != roboName) &&
       (m_entities[i] != "trashbox_0") &&
       (m_entities[i] != "trashbox_1") &&
       (m_entities[i] != "trashbox_2") &&
       (m_entities[i] != "petbottle_0") &&
       (m_entities[i] != "petbottle_1") &&
       (m_entities[i] != "petbottle_2") &&
       (m_entities[i] != "petbottle_3") &&
       (m_entities[i] != "petbottle_4") &&
       (m_entities[i] != "banana") &&
       (m_entities[i] != "chigarette") &&
       (m_entities[i] != "chocolate") &&
       (m_entities[i] != "mayonaise_0") &&
       (m_entities[i] != "mayonaise_1") &&
       (m_entities[i] != "mugcup") &&
       (m_entities[i] != "can_0") &&
       (m_entities[i] != "can_1") &&
       (m_entities[i] != "can_2") &&
       (m_entities[i] != "can_3") &&
       (m_entities[i] != "apple") &&
       (m_entities[i] != "clock") &&
       (m_entities[i] != "kettle") ){
      m_entNames.push_back(m_entities[i]);
    }
  }
  entNum = m_entNames.size();
}  
  
double MyController::onAction(ActionEvent &evt) {
  // check whether Referee service is available or not
  bool available = checkService("EGPSRReferee");
  if(!available && m_ref != NULL) m_ref = NULL;
  else if(available && m_ref == NULL){
    m_ref = connectToService("EGPSRReferee");
  }

  // get information about the robot and renew it
  //----------------------------------
  SimObj *r_my = getObj(roboName.c_str());

  // for body configuration
  r_my->getPosition(crrPos);
  r_my->getRotation(crrRot);

  Vector3d rsLenVec(prv1Pos.x()-crrPos.x(), prv1Pos.y()-crrPos.y(), prv1Pos.z()-crrPos.z());
  rsLen = rsLenVec.length();

  // for arm configuration
  //for(int j=0;j<jtnum;j++) crrJAng_r[j] = r_my->getJointAngle(jointName[j].c_str());

	onCheckCollision();
	onCheckRoom();
  // 
  for(int k=0;k<entNum;k++){
    SimObj* locObj = getObj(m_entNames[k].c_str());
    CParts *parts = locObj->getMainParts();
    bool state = parts->getCollisionState();

    if(state){
      colState=true;       // collided with main body of robot
      if(rsLen == 0.0) {
//        pcolState=true;
        r_my->setRotation(prv1Rot);
      } else {
//        pcolState=false;
        r_my->setPosition(prv1Pos);
      }

      std::string msg = "EGPSRReferee/Collision with [" + m_entNames[k] + "]" "/-100";
      if(m_ref != NULL){
        m_ref->sendMsgToSrv(msg.c_str());
      }
      else{
        LOG_MSG((msg.c_str()));
      }
      break;
    }
  }

//  if(!colState && !pcolState){
  if(!colState){
    prv1Pos=crrPos;
    prv1Rot=crrRot;
    //colState=false;     // reset collision condition
  } else if(pcolState){
    for(int i=0;i<jtnum;i++) prv1JAng_r[i]=crrJAng_r[i];
		pcolState = false;
  } else{
    //Do nothing on "collided" condition
//    LOG_MSG((colPtName.c_str()));
		colState = false;
  }
  return retValue;      
}  

void MyController::onCheckCollision(){
}

void MyController::onCheckRoom(){
	int x, z;
	int num = 4;
	x =crrPos.x();
	z =crrPos.z();
	
	if(x>-100&&x<500&&z>-425&&z<75)	num=0; // living room
	if(x>100&&x<500&&z>75&&z<425){ // kitchen
		num=1;
		if(m_roomState==0){
			std::string msg = "EGPSRReferee/Robot is in [" + m_rooms[num] + "]" "/+500";
			m_roomState=1;
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}
	}		
	if(x>-500&&x<-100&&z>-425&&z<-75){ // lobby
	 num=2;
		if(m_roomState==1){
			std::string msg = "EGPSRReferee/Robot is in [" + m_rooms[num] + "]" "/+500";
			m_roomState=2;
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}
	}
	if(x>-500&&x<-100&&z>75&&z<425){ // bed room
	 num=3;
		if(m_roomState==2){
			std::string msg = "EGPSRReferee/Robot is in [" + m_rooms[num] + "]" "/+500";
			m_roomState=3;
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}
	}
}
extern "C" Controller * createController() {  
  return new MyController;  
}  

