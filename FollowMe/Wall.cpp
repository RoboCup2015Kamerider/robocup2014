#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>      

using namespace std;

//ControllerのサブクラスMyController作成  
class MyController : public Controller {  
  public:  
      
  // シミュレーション開始時に一度だけ呼出される関数onInit  
   void onInit(InitEvent &evt);  
      
   // 定期的な処理を行うonAction  
   double onAction(ActionEvent&);  
      
   // メッセージ受信時に呼び出されます  
   void onRecvMsg(RecvMsgEvent &evt);  
      
   // 衝突時に呼び出されます  
   void onCollision(CollisionEvent &evt);  
};  
      
void MyController::onInit(InitEvent &evt) {  
}  
      
double MyController::onAction(ActionEvent &evt) {  

  return 0.1;     //次にonActionが呼ばれるまでの時間を返します  
}  
      
void MyController::onRecvMsg(RecvMsgEvent &evt) {  
    SimObj *obj = getObj(myname());
    Vector3d pos;
    string msg = evt.getMsg();
    if(msg == "elevator_close"){
        obj->getPosition(pos);
        obj->setPosition( pos.x(), pos.y() , pos.z()+150);
    }else if(msg == "elevator_open"){
        obj->getPosition(pos);
        obj->setPosition( pos.x(), pos.y() , pos.z()-150);
    }
}  
   
void MyController::onCollision(CollisionEvent &evt) {  
}  
      
 //自身のインスタンスをSIGVerseに返します  
extern "C" Controller * createController() {  
  return new MyController;  
}  
