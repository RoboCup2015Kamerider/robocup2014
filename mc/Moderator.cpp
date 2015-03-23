#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <sstream>
#include <fstream>
#include <iomanip>
#include <unistd.h>

enum Reachable{
  UP,
  DOWN,
  LEFT,
  RIGHT
};

typedef struct target{
  std::string name;

  float x;
  float y;
  float z;

  float height; //bb on y-axis
  float length; //bb on x-axis
  float width; //bb on z-axis
} Target;

typedef struct table{
  std::string name;

  float x;
  float y;
  float z;

  float length; //bb on x-axis
  float height; //bb on y-axis
  float width; //bb on z-axis

  int reachable[4];
} Table;

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

	const static unsigned int jtnum=7;
	std::vector<std::string> jointName;
	double prv1JAng_r[jtnum];
	double crrJAng_r[jtnum];

	std::vector<std::string> m_entities;
	std::vector<std::string> m_entNames;
	int entNum;

	// position vectors and rotation matrices required to modify robot's behavior
	Vector3d crrPos;
	Vector3d prv1Pos;
	Rotation crrRot;
	Rotation prv1Rot;

	double rsLen;

	int trialCount;
	int trialMax;

	bool   isCleaningUp;
	double startTime;
	double endTime;

	std::vector<std::string> targetEntities;
	std::vector<double>      targetEntitiesHeight;

	void startTask();
	void resetRobotCondition();
	void breakTask();
	void initObjects();

	std::vector<Target> parseFileTargets(std::string fileName);
	std::vector<Table> parseFileTables(std::string fileName);
	std::map< int, std::vector<std::string> > parseFileTrials(std::string filename);

	std::map< int, std::vector<std::string> > m_trialsObjects;
	std::map< int, std::vector<Target> > m_targetsOnTrial;
	std::vector<Target> m_targets;
	std::vector<Table> m_tables;

	bool checkAvailablePos(float posObj, Target obj, int indPosOnTable, std::vector<Target> vec);
	void reposObjects();
	int getNextPosOnTable(int indPosOnTable, Table table);
	void getNextTable(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable);
	void performChange(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable);
	float mapRange(float s, float a1, float a2, float b1, float b2);
	void takeAwayObjects();

	template<typename Obj>
	int contains(std::vector<Obj> vec, std::string key);

	double getCurrentTime();
};

void MyController::onInit(InitEvent &evt)
{
	int i, cnt, found = -1;

	retValue  = 0.08;
	colState  = false;
	pcolState = false;
	roboName  = "robot_000";
	mdName    = "moderator_0";

	crrPos  = Vector3d(0,0,0);
	prv1Pos = Vector3d(0,0,0);

	m_tables = parseFileTables("table_list.txt");
	m_targets = parseFileTargets("object_list.txt");
	m_trialsObjects = parseFileTrials("trials_object.txt");


	//for(int i=0; i<m_targets.size(); i++){
			//std::cout << "m_target: " << m_targets[i].name << std::endl;
		
	//}


	for(std::map< int, std::vector<std::string> >::iterator it = m_trialsObjects.begin(); it != m_trialsObjects.end(); ++it){

		std::cout << "it first: " << it->first << std::endl;
		std::cout << "it second size: " << it->second.size()<< std::endl;

		for(int i=0; i<it->second.size(); i++){
			m_targetsOnTrial[it->first].push_back(m_targets[contains(m_targets, it->second[i])]);
			//std::cout << "contains: " << contains(m_targets, it->second[i]) << std::endl;
			//std::cout << "it second[i]: " << it->second[i] << std::endl;
		}

	}
	getAllEntities(m_entities);

	// select objects to be observed
	cnt = m_entities.size();
	for(i = 0; i < cnt; i++){

		if( (found = contains(m_tables, m_entities[i]) ) != -1 ){
			SimObj* entity = getObj(m_entities[i].c_str());
			Vector3d pos;
			entity->getPosition(pos);

			m_tables[found].x = pos.x();
			m_tables[found].y = pos.y();
			m_tables[found].z = pos.z();
		}

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
		   (m_entities[i] != "petbottle_5") &&
		   (m_entities[i] != "banana") &&
		   (m_entities[i] != "mayonaise_0") &&
		   (m_entities[i] != "mayonaise_1") &&
		   (m_entities[i] != "mugcup") &&
		   (m_entities[i] != "can_0") &&
		   (m_entities[i] != "can_1") &&
		   (m_entities[i] != "can_2") &&
		   (m_entities[i] != "can_3") &&
		   (m_entities[i] != "apple") ){
			m_entNames.push_back(m_entities[i]);
		}
	}
	entNum = m_entNames.size();

	trialCount = 0;
	trialMax = 2;

	isCleaningUp = false;

	startTime =  0.0;
	endTime   = 480.0; // [sec]
	srand(2);
	//srand(time(NULL));
}

double MyController::onAction(ActionEvent &evt)
{
	if(trialCount >= trialMax){
		return retValue;
	}

	// check whether Referee service is available or not
	bool available = checkService("RoboCupReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("RoboCupReferee");
	}

	// reset clean-up task
	if(!isCleaningUp){
		double deadTime = 3.0;
		resetRobotCondition();
		reposObjects();

		usleep(deadTime * 1000000);

		// broadcast start message
		LOG_MSG(("Task_start"));
		broadcastMsg("Task_start");

		std::string msg = "RoboCupReferee/start/";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		else{
			LOG_MSG((msg.c_str()));
		}

		startTime = evt.time() + deadTime;
		isCleaningUp = true;
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

	//
	for(int k=0;k<entNum;k++){
		SimObj* locObj = getObj(m_entNames[k].c_str());
		CParts *parts = locObj->getMainParts();
		bool state = parts->getCollisionState();

		if(state){
			colState=true;       // collided with main body of robot
			if(rsLen == 0.0) {
				//pcolState=true;
				r_my->setRotation(prv1Rot);
			} else {
				//pcolState=false;
				r_my->setPosition(prv1Pos);
			}

			std::string msg = "RoboCupReferee/Collision with [" + m_entNames[k] + "]" "/-100";
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
			break;
		}
	}

	//if(!colState && !pcolState){
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

	std::stringstream time_ss;
	double elapsedTime = evt.time() - startTime;
	//if(evt.time() - startTime > endTime){
	if(elapsedTime > endTime){
		LOG_MSG(("Time_over"));
		broadcastMsg("Time_over");
		breakTask();
		//time_ss << "RoboCupReferee/time/00:00:00";
	}
	else{
		double remainedTime = endTime - elapsedTime;
		int min, sec, msec;
		sec = (int)remainedTime;
		min = sec / 60;
		sec %= 60;
		msec = (int)((remainedTime - sec) * 100);
		time_ss <<  "RoboCupReferee/time/";
		time_ss << std::setw(2) << std::setfill('0') << min << ":";
		time_ss << std::setw(2) << std::setfill('0') << sec;
	}
	if(m_ref != NULL){
		m_ref->sendMsgToSrv(time_ss.str().c_str());
	}
	else{
		LOG_MSG((time_ss.str().c_str()));
	}

	return retValue;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg    = evt.getMsg();
	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	if(sender == "robot_000" && msg == "Task_finished"){
		LOG_MSG(("Task_end"));
		broadcastMsg("Task_end");
		breakTask();
	}
	if(sender == "robot_000" && msg == "Give_up"){
		LOG_MSG(("Task_end"));
		broadcastMsg("Task_end");
		breakTask();
	}
}

void MyController::resetRobotCondition()
{
	RobotObj *robot;

	// reset robot condition
	robot = getRobotObj(roboName.c_str());
	robot->setWheelVelocity(0.0, 0.0);
	robot->setRotation(Rotation(1,0,0,0));
	robot->setPosition(100.0, 30.0, 10.0);
	robot->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
	robot->setJointAngle("RARM_JOINT1", 0.0);
	robot->setJointAngle("RARM_JOINT4", 0.0);

	//
	std::map<std::string, CParts *> partsm = robot->getPartsCollection();
	for (SimObj::PartsM::const_iterator i=partsm.begin(); i!=partsm.end(); i++) {
		CParts *parts = robot->getParts(i->first.c_str());
		parts->releaseObj();
	}
}

void MyController::startTask()
{
	// broadcast start message
	broadcastMsg("Task_start");
}

void MyController::breakTask()
{
	isCleaningUp = false;
	takeAwayObjects();
	trialCount++;
	std::string msg = "RoboCupReferee/end/";
	if(m_ref != NULL){
		m_ref->sendMsgToSrv(msg.c_str());
	}
	else{
		LOG_MSG((msg.c_str()));
	}
			
	if(trialCount == trialMax){
		resetRobotCondition();
		LOG_MSG(("Mission_complete"));
		broadcastMsg("Mission_complete");
	}
}

/*
  s: value to map
  a1: lower boundary of the departure range
  a2: upper boundary of the departure range
  b1: lower boundary of the arrival range
  b2: upper boundary of the arrival range
*/
float MyController::mapRange(float s, float a1, float a2, float b1, float b2){
	return b1 + ( (s-a1) * (b2-b1) ) / (a2 - a1);
}

/*
  check if the position, posObj, where we want to put the object, obj, is
  available compared to the already placed objects contained in the vector, vec
*/
bool MyController::checkAvailablePos(float posObj, Target obj, int indPosOnTable, std::vector<Target> vec){
	bool available = true;

	if(vec.size() > 0){
		if( indPosOnTable == DOWN || indPosOnTable == UP ){
			for(std::vector<Target>::iterator it = vec.begin(); it != vec.end() && available; ++it){
				float xInf = it->x - 0.5 * it->length - 20;
				float xSup = it->x + 0.5 * it->length + 20;
				float xInfObj = posObj - 0.5 * obj.length;
				float xSupObj = posObj + 0.5 * obj.length;

				available = (xInfObj < xInf && xSupObj < xInf ||
				xInfObj > xSup && xSupObj > xSup);
			}
		}

		else{
			for(std::vector<Target>::iterator it = vec.begin(); it != vec.end() && available; ++it){
				float zInf = it->z - 0.5 * it->width - 20;
				float zSup = it->z + 0.5 * it->width + 20;
				float zInfObj = posObj - 0.5 * obj.width;
				float zSupObj = posObj + 0.5 * obj.width;

				available = (zInfObj < zInf && zSupObj < zInf ||
				zInfObj > zSup && zSupObj > zSup);
			}
		}
	}

	return available;
}

int MyController::getNextPosOnTable(int indPosOnTable, Table table){
	int j;
	for(j=(indPosOnTable+1) % 4; j != indPosOnTable && table.reachable[j] == -1; j = (j + 1) % 4);

	j--;

	if(j != indPosOnTable){
		if(j == 0)
			j = 3;
		else
			j--;
	}

	else
		j = -1;

	return j;
}

void MyController::getNextTable(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable){
	*indTab = (*indTab + 1) % vecTable.size();
	*table = vecTable[*indTab];

	bool posAvailable = false;

	while( !posAvailable ) {
		*indPosOnTable = rand() % 4;
		posAvailable = table->reachable[*indPosOnTable] != -1;
	}
}

void MyController::performChange(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable){
	int randChange = rand() % 2; // rand to determine if we look for another position or another table

	if(randChange == 1){
		int j = getNextPosOnTable(*indPosOnTable, *table);

		if(j == -1){
			getNextTable(indTab, indPosOnTable, table, vecTable);
		}

		else{
			*indPosOnTable = j;
		}
	}

	else{
		getNextTable(indTab, indPosOnTable, table, vecTable);
	}
}

void MyController::reposObjects(){
	int nbTables = m_tables.size();
	std::vector<Target> placedObjects;
	float yObj;
	float xObj;
	float zObj;

	for(std::vector<Target>::iterator it = m_targetsOnTrial[trialCount].begin(); it != m_targetsOnTrial[trialCount].end(); ++it){
		int indTab = rand() % nbTables;
		Table table = m_tables[indTab];
		bool posAvailable = false;
		int indPosOnTable;
		int nbTries;

		/*
			we took a random table among the available ones and now we take a random
			position on this table (UP, DOWN, LEFT, RIGHT) if it's available
		*/
		while( !posAvailable ) {
			indPosOnTable = rand() % 4;
			posAvailable = table.reachable[indPosOnTable] != -1;
		}


		/*
			This loop is made mainly to avoid placing different objects at the same
			spot. If we try to place the object 10 times and it doesn't succeed in
			finding an available spot then we try to find another position on the
			current table or change the table directly
		*/

		do{
			//std::cout << "probleme here MOUZEURFOUQUEUR" << std::endl;
			nbTries = 0;

			if( indPosOnTable == DOWN || indPosOnTable == UP ){
				yObj = table.y + 0.5 * table.height + 0.5 * it->height + 1.75;
				float xOffset = it->length;
				//float xOffset = 20;
				float xInf = table.x - 0.5 * table.length + xOffset;
				float xSup = table.x + 0.5 * table.length - xOffset;
				float zOffset = it->width;
				//float zOffset = 20;

				do{
					xObj = mapRange(rand(), 0,  RAND_MAX, xInf, xSup);
					nbTries++;
				} while(!checkAvailablePos(xObj, *it, indPosOnTable, placedObjects) && nbTries < 10);

				if(nbTries >= 9){
					performChange(&indTab, &indPosOnTable, &table, m_tables);
				}

				else{

					if(indPosOnTable == DOWN){
						zObj = table.z + 0.5 * table.width - zOffset;
					}

					else{
					zObj = table.z - 0.5 * table.width + zOffset;
					}
				}
			}

			else{
				yObj = table.y + 0.5 * table.height + 0.5 * it->height + 2.0;
				float zOffset = it->width;
				float zSup = table.z - 0.5 * table.width + zOffset;
				float zInf = table.z + 0.5 * table.width - zOffset;
				float xOffset = it->length;

				do{
					zObj = mapRange(rand(), 0,  RAND_MAX, zInf, zSup);
					nbTries++;
				} while(!checkAvailablePos(zObj, *it, indPosOnTable, placedObjects) && nbTries < 10);

				if(nbTries >= 9){
					performChange(&indTab, &indPosOnTable, &table, m_tables);
				}

				else{
					if(indPosOnTable == RIGHT){
						xObj = table.x + 0.5 * table.length - xOffset;
					}

					else{
						xObj = table.x - 0.5 * table.length + xOffset;
					}
				}
			}
		}while(nbTries >= 9);

		SimObj* target = getObj(it->name.c_str());
		target->setPosition(Vector3d(xObj, yObj, zObj));

		it->x = xObj;
		it->y = yObj;
		it->z = zObj;

		placedObjects.push_back(*it);
	}
}

std::vector<Table> MyController::parseFileTables(std::string fileName){
	std::vector<Table> vecObj;
	Table obj;

	obj.reachable[UP] = 1;
	obj.reachable[DOWN] = 1;
	obj.reachable[LEFT] = 1;
	obj.reachable[RIGHT] = 1;

	std::fstream fin;
	fin.open(fileName.c_str());
	std::string buf;

	while(std::getline(fin, buf)){
		std::cout << "BUFF Tables: " << std::endl;
		std::cout << buf << std::endl;

		std::size_t found_name = buf.find(",");
		std::size_t found_height = buf.find(",", found_name + 1);
		std::size_t found_width = buf.find(",", found_height + 1);

		obj.name = buf.substr(0, found_name);
		obj.height = atof( buf.substr(found_name + 1, found_height - found_name - 1).c_str() );
		obj.width = atof( buf.substr(found_height + 1, found_width - found_height - 1).c_str() );
		obj.length = atof( buf.substr(found_width + 1, buf.size() - found_width - 2).c_str() );

		vecObj.push_back(obj);
	}

	fin.close();

	return vecObj;
}

std::vector<Target> MyController::parseFileTargets(std::string fileName){
	std::vector<Target> vecObj;
	Target obj;

	std::fstream fin;
	fin.open(fileName.c_str());
	std::string buf;

	while(std::getline(fin, buf)){
		std::size_t found_name = buf.find(",");
		std::size_t found_height = buf.find(",", found_name + 1);
		std::size_t found_width = buf.find(",", found_height + 1);

		obj.name = buf.substr(0, found_name);
		obj.height = atof( buf.substr(found_name + 1, found_height - found_name - 1).c_str() );
		obj.width = atof( buf.substr(found_height + 1, found_width - found_height - 1).c_str() );
		obj.length = atof( buf.substr(found_width + 1, buf.size() - found_width - 2).c_str() );

		vecObj.push_back(obj);
	}

	fin.close();

	return vecObj;
}


std::map< int, std::vector<std::string> > MyController::parseFileTrials(std::string fileName){
	std::map< int, std::vector<std::string> > trialsObj;
	std::vector<std::string> vec;
	int i=0;
	std::fstream fin;
	fin.open(fileName.c_str());
	std::string buf;

	while(std::getline(fin, buf)){
		vec.clear();

		std::size_t found_name1 = buf.find(",");
		std::size_t found_name2 = buf.find(",", found_name1 + 1);
		std::size_t found_name3 = buf.find(",", found_name2 + 1);
		std::size_t found_name4 = buf.find(",", found_name3 + 1);

		std::cout << "i: " << i << std::endl;
		std::cout << "\tBUFF: " << std::endl;
		std::cout << buf << std::endl;
		/*std::cout << "\tfound_name1: " << found_name1 << std::endl;
		std::cout << "\tfound_name2: " << found_name1 << std::endl;
		std::cout << "\tfound_name3: " << found_name1 << std::endl;
		std::cout << "\tfound_name4: " << found_name1 << std::endl;
		*/
		vec.push_back(buf.substr(0, found_name1));
		vec.push_back(buf.substr(found_name1 + 1, found_name2 - found_name1 - 1));
		vec.push_back(buf.substr(found_name2 + 1, found_name3 - found_name2 - 1));
		vec.push_back(buf.substr(found_name3 + 1, found_name4 - found_name3 - 1));
		vec.push_back(buf.substr(found_name4 + 1, buf.size() - found_name4 - 2));

		trialsObj[i] = vec;
		i++;
	}

	fin.close();

	return trialsObj;
}

template<typename Type>
int MyController::contains(std::vector<Type> vec, std::string key){
	bool found = false;
	int i;
	//std::cout << "\tcontains key: " << key << std::endl;
	//std::cout << "\tcontains key length: " << key.size() << std::endl;
	
	for(i = 0 ; i < vec.size() && !found; i++){
		//std::cout << "\tcontains vec[i]: " << vec[i].name << std::endl;
		//std::cout << "\tcontains vec[i] length: " << vec[i].name.size() << std::endl;
		
 		found = vec[i].name == key;
	}

	if(!found)
		i = -1;
	else
		i--;

	return i;
}

void MyController::takeAwayObjects(){
	for(std::vector<Target>::iterator it = m_targetsOnTrial[trialCount].begin(); it != m_targetsOnTrial[trialCount].end(); ++it){
		SimObj* target = getObj(it->name.c_str());
		target->setPosition(Vector3d(100000, 100000, 100000));
	}
}

extern "C" Controller * createController() {
	return new MyController;
}
