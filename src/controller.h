#include <Control/Controller.h>

class RobotControllerInterface : public RobotController
{
public:
  RobotControllerInterface(Robot& robot) : RobotController(robot) {}
  virtual ~RobotControllerInterface() {}
  virtual const char* Type() const { return "RobotControllerInterface"; }
  virtual void Reset() { 
    //put any initialization code here
    RobotController::Reset(); 
  } 
  virtual void Update(Real dt) {
    //We'll put our code here: read from this->sensors, and write to this->command.
    //See Sensor.h and Command.h for details on these structures
    printf("Whee, the control loop is being called!\n");
    std::cout << "CONTROLLER LOOPIN'" << std::endl;
    //call the base class's Update method
    RobotController::Update(dt);

    Vector qcmd,vcmd;
    Vector qactual,vactual;
    GetCommandedConfig(qcmd);  //convenience function in RobotController
    GetCommandedVelocity(vcmd);  //convenience function in RobotController
    GetSensedConfig(qactual);  //convenience function in RobotController
    GetSensedVelocity(vactual);  //convenience function in RobotController
    int link = 7;  //the movement link
    if(time >= 1.0 && time < 2.0)
    {
      Real speed = -1.5;
      vcmd[link] = speed;
    }else
    {
      vcmd[link] = 0;
    }
    SetPIDCommand(qcmd,vcmd); //convenience function in RobotController

  }
};

