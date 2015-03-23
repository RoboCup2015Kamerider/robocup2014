#include "Controller.h"
#include "Logger.h"
#include <unistd.h>

#define PI 3.141592

//角度からラジアンに変換します。
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 ) 
#define RAD2DEG(RAD) ( (RAD) * 180.0 / (PI) )

class CameraController : public Controller
{
public:
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent &evt);
  void rotateTowardRobot(Vector3d pos);
 
private:
  // カメラ関係
  SimObj *m_my;
  Vector3d m_pos;
  std::string m_name;  

  // ロボット関係
  RobotObj *r_my;
  Vector3d r_pos;  

}; 

void CameraController::onInit(InitEvent &evt) 
{  
  // カメラ初期方向　1:45 2:135 3:315 4:225
  // ロボット初期位置取得
  r_my = getRobotObj("robot_000");
  r_my->getPosition(r_pos);

  // カメラ番号取得
  m_my = getObj(myname());  
  m_name = m_my->name();

  // 定点カメラをロボットの方向に回転
  rotateTowardRobot(r_pos);
}

//定期的に呼び出される関数です。
double CameraController::onAction(ActionEvent &evt)
{
  // ロボット位置取得
  r_my->getPosition(r_pos);

  // カメラ番号取得
  m_my = getObj(myname());  
  m_name = m_my->name();

  // 定点カメラをロボットの方向に回転
  rotateTowardRobot(r_pos);
  return 0.05;
}

void CameraController::rotateTowardRobot(Vector3d r_pos)
{
  // カメラの位置を得る
  m_my->getPosition(m_pos);

  // カメラの位置からロボットを結ぶベクトル
  Vector3d tmpp = r_pos;
  tmpp -= m_pos;
  
  // y方向は考えない
  tmpp.y(0);
  
  // カメラの回転角度を得る
  Rotation myRot;
  m_my->getRotation(myRot);
     
  // カメラのy軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();      
  double theta = 2*acos(fabs(qw));

  if(qw*qy < 0)
    theta = -1*theta;

  // ロボットまでの回転角度を得る
  double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
  double targetAngle = acos(tmp);
  if(tmpp.x() > 0) targetAngle = -1*targetAngle;

  // 角度差から回転量を得る
  targetAngle += theta;
  m_my->setAxisAndAngle(0, 1, 0, -targetAngle, 0);
}

extern "C"  Controller * createController ()
{
  return new CameraController;
}

