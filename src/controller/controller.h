#pragma once
#include <Control/Controller.h>

struct ControllerState{
  public:
    std::vector<Vector3> predicted_com;
    std::vector<Vector3> predicted_com_dir;
    std::vector<Vector3> com_window;
    std::vector<Vector3> linmomentum_window;
    std::vector<Vector3> angmomentum_window;

    double mass;

    double maximumWindowLength = 2.0;

    void AddCOM( Vector3 &com, Vector3 &linmom, Vector3 &angmom ){
      com_window.push_back(com);
      linmomentum_window.push_back(linmom);
      angmomentum_window.push_back(angmom);

      if(getLength(com_window) > maximumWindowLength){
        com_window.erase(com_window.begin());
        linmomentum_window.erase(linmomentum_window.begin());
        angmomentum_window.erase(angmomentum_window.begin());
      }
    }

    void SetMass( double _mass ){
      mass = _mass;
    }

    void PredictCOM( double tstep, uint Nsteps){
      if(com_window.empty()){
        return;
      }
      //compute change in momentum to get force estimate
      Vector3 force;
      if(linmomentum_window.size()>1){
        uint N = linmomentum_window.size()-1;
        Vector3 l1 = linmomentum_window.at(N-1);
        Vector3 l2 = linmomentum_window.at(N);
        force = (l2 - l1)/0.1;
      }else{
        force = Vector3(0,0,0);
      }
      Vector3 torque;
      if(angmomentum_window.size()>1){
        uint N = angmomentum_window.size()-1;
        Vector3 a1 = angmomentum_window.at(N-1);
        Vector3 a2 = angmomentum_window.at(N);
        torque = (a2 - a1)/tstep;
      }else{
        torque = Vector3(0,0,0);
      }

      //predict COM forward assuming that we are in uniform force field
      Vector3 com = com_window.back();
      Vector3 LM = linmomentum_window.back();
      Vector3 AM = angmomentum_window.back();

      double dt = tstep;
      double dt2 = tstep*tstep*0.5;

      Vector3 dcom = LM/mass;

      //std::cout << "force " << force << std::endl;

      predicted_com.clear();
      //if(predicted_com.size() > 3*Nsteps){
      //  predicted_com.erase (predicted_com.begin(),predicted_com.begin()+Nsteps);
      //}

      for(int i = 0; i < Nsteps; i++){
        predicted_com.push_back(com);
        com = com + dt * dcom + dt2 * force/mass;
        dcom = dcom + dt * force/mass;
      }

    }

  private:
    double getLength(std::vector<Vector3> &path){
      double length = 0.0;
      for(int i = 0; i < path.size()-1; i++){
        length += (path.at(i) - path.at(i+1)).norm();
      }
      return length;
    }
};


class ContactStabilityController: public RobotController
{
public:
  ContactStabilityController(Robot& robot) : RobotController(robot) {}
  virtual ~ContactStabilityController() {}
  virtual const char* Type() const { return "ContactStabilityController"; }
  virtual void Reset() { 
    //put any initialization code here
    RobotController::Reset(); 
  } 

  ControllerState output;

  const ControllerState& GetControllerState() const {
    return output;
  }

  virtual void Update(Real dt) {
    //We'll put our code here: read from this->sensors, and write to this->command.
    //See Sensor.h and Command.h for details on these structures
    std::cout << "controller time " << time << std::endl;

    Vector qcmd,vcmd;
    Vector qactual,vactual;
    GetCommandedConfig(qcmd);  //convenience function in RobotController
    GetCommandedVelocity(vcmd);  //convenience function in RobotController
    GetSensedConfig(qactual);  //convenience function in RobotController
    GetSensedVelocity(vactual);  //convenience function in RobotController

    //uint Nsensors =  sensors->sensors.size();
    //for(int i = 0; i < Nsensors; i++){
    //  std::cout << sensors->sensors.at(i)->name << std::endl;
    //}

    Vector3 com = robot.GetCOM();
    Vector3 LM = robot.GetLinearMomentum();
    Vector3 AM = robot.GetAngularMomentum();

    output.SetMass( robot.GetTotalMass() );

    output.AddCOM(com, LM, AM);

    output.PredictCOM(0.001, 1000);

    //int link = 15;  //the movement link
    //vcmd.setZero();
    //if(time >= 1.0 && time < 2.0)
    //{
    //  Real speed = 10;
    //  vcmd[link] = speed;
    //}else
    //{
    //  vcmd[link] = 0;
    //}

    SetPIDCommand(qcmd,vcmd); //convenience function in RobotController
    //Vector torques;
    //torques.resize(qcmd.size());
    //torques.setZero();

    //std::cout << "drivers: " << robot.drivers.size() << std::endl;
    //std::cout << "torques: " << torques.size() << std::endl;
    //torques[15] = 1;
    //torques[16] = 1;
    //torques[17] = 1;

    //SetTorqueCommand(torques);

    RobotController::Update(dt);
  }
};
