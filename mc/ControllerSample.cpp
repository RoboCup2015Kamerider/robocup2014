#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
  
class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 
};  
  
void MyController::onInit(InitEvent &evt) {  
}  
  
double MyController::onAction(ActionEvent &evt) {  
  return 1.0;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) {  
}  

void MyController::onCollision(CollisionEvent &evt) { 
}
  
extern "C" Controller * createController() {  
  return new MyController;  
}  

