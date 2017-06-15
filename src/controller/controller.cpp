#include "controller.h"

void ControllerState::AddCOM( Vector3 &com, Vector3 &linmom, Vector3 &angmom ){
  com_window.push_back(com);
  linmomentum_window.push_back(linmom);
  angmomentum_window.push_back(angmom);

  if(getLength(com_window) > maximumWindowLength){
    com_window.erase(com_window.begin());
    linmomentum_window.erase(linmomentum_window.begin());
    angmomentum_window.erase(angmomentum_window.begin());
  }
}

void ControllerState::SetMass( double _mass ){
  mass = _mass;
}

void ControllerState::PredictCOM( double tstep, uint Nsteps){
  if(com_window.empty()){
    return;
  }
  //compute change in momentum to get force estimate
  Vector3 force;
  if(linmomentum_window.size()>1){
    double dt = 0.01;
    //uint N = linmomentum_window.size()-1;

    //average over last elements with diminishing return
    uint N= min(10, int(linmomentum_window.size()-1));
    for(int i = 0; i < N; i++){
      Vector3 l1 = linmomentum_window.at(i);
      Vector3 l2 = linmomentum_window.at(i+1);
      force += ((N-i)/N)*(l2 - l1)/dt;
    }
    force /= N;
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

  predicted_com.clear();
  for(int i = 0; i < Nsteps; i++){
    predicted_com.push_back(com);
    com = com + dt * dcom + dt2 * force/mass;
    dcom = dcom + dt * force/mass;
  }

}

double ControllerState::getLength(std::vector<Vector3> &path){
  double length = 0.0;
  for(int i = 0; i < path.size()-1; i++){
    length += (path.at(i) - path.at(i+1)).norm();
  }
  return length;
}
//################################################################################
//################################################################################
//################################################################################

ContactStabilityController::ContactStabilityController(Robot& robot) : RobotController(robot) {}
ContactStabilityController::~ContactStabilityController() {}
const char* ContactStabilityController::Type() const { return "ContactStabilityController"; }
void ContactStabilityController::Reset() { 
  //put any initialization code here
  std::cout << "["<<Type() << "] Reset" << std::endl;
  RobotController::Reset(); 
} 

const ControllerState& ContactStabilityController::GetControllerState() const {
  return output;
}

void ContactStabilityController::Update(Real dt) {
  //We'll put our code here: read from this->sensors, and write to this->command.
  //See Sensor.h and Command.h for details on these structures
  std::cout << "controller time " << time << std::endl;

  Vector qcmd,vcmd;
  Vector qactual,vactual;

  //GetCommandedConfig(qcmd);  //convenience function in RobotController
  //GetCommandedVelocity(vcmd);  //convenience function in RobotController
  //GetSensedConfig(qactual);  //convenience function in RobotController
  //GetSensedVelocity(vactual);  //convenience function in RobotController

  //std::cout << "config: " << qcmd << std::endl;
  //std::cout << "velocity: " << vcmd << std::endl;

  //uint Nsensors = sensors->sensors.size();
  //for(int i = 0; i < Nsensors; i++){
  //  std::cout << sensors->sensors.at(i)->name << std::endl;
  //}

  //uint Nlinks = robot.links.size();
  ////const Terrain* terrain = sim.odesim.terrain(0);
  //const Geometry::AnyCollisionGeometry3D tgeom = (*terrain->geometry);
  //Geometry::AnyCollisionGeometry3D tt(tgeom);
  //robot->robot.CleanupCollisions();
  //robot->robot.InitMeshCollision(tt);

  //for(int i = 0; i < Nlinks; i++){
  //  dBodyID bodyid = robot->body(i);
  //  if(bodyid){
  //    if(!robot->robot.IsGeometryEmpty(i)){
  //      RobotLink3D *link = &robot->robot.links[i];
  //      Geometry::AnyCollisionQuery *query = robot->robot.envCollisions[i];
  //      double d = query->Distance(0,0.1);
  //      std::vector<Vector3> vp1,vp2;
  //      query->InteractingPoints(vp1,vp2);
  //      if(vp1.size()!=1){
  //        std::cout << "Warning: got " << vp1.size() << " contact points for single rigid body" << std::endl;
  //      }
  //      Matrix4 mat = link->T_World;
  //      Vector3 p1 = link->T_World*vp1.front();
  //      Vector3 p2 = vp2.front();
  //    }
  //  }
  //}

  Vector3 com = robot.GetCOM();
  Vector3 LM = robot.GetLinearMomentum();
  Vector3 AM = robot.GetAngularMomentum();

  output.SetMass( robot.GetTotalMass() );
  output.AddCOM(com, LM, AM);
  output.PredictCOM(0.001, 1000);

  if(torques.size()>0){
    uint N = torques.at(0).size();

    uint ictr = 0;
    double t = 0;
    while(t < time && ictr<times.size()){
      t+= times.at(ictr++);
      //std::cout << t << "/" << time << "(" << ictr << "/" << torques.size() << ")" << std::endl;
    }
    if(t>time){
      Vector torque = torques.at(ictr-1);
      for(int k = 0; k < torque.size(); k++){
        torque(k) += Rand(-1,1);
      }
      SetTorqueCommand(torque);
      std::cout << "Setting torque: " << torque << std::endl;
    }else{
      SetTorqueCommand(ZeroTorque);
    }
  }

  //SetPIDCommand(qcmd,vcmd);
  //SetTorqueCommand(const Vector& torques);

  //Vector torques;
  //torques.resize(qcmd.size());
  //torques.setZero();

  //std::cout << "drivers: " << robot.drivers.size() << std::endl;
  //for(int i = 0; i < robot.drivers.size(); i++){
  //  RobotJointDriver driver = robot.drivers.at(i);
  //  if(driver.type == RobotJointDriver::Normal) {
  //    std::cout << "[" << i << "/" << robot.drivers.size() << "] : normal" << std::endl;
  //  }
  //}
  //exit(0);

  RobotController::Update(dt);
}
bool ContactStabilityController::SendCommand(const string& name,const string& str){

  stringstream ss(str);
  Vector torque_and_time;
  if(name == "set_q") {
    fprintf(stderr,"not done yet\n");
    return false;
  }else if(name == "set_torque_control") {
    ss >> torque_and_time;
    torques.clear();
    AppendTorqueAndTime(torque_and_time);
    ZeroTorque = torques.at(0);
    ZeroTorque.setZero();
    return true;
  }else if(name == "append_torque_control") {
    ss >> torque_and_time;
    std::cout << "Controller: append_torque_control : #torques " << torques.size() << " last item " << torques.back() << std::endl;
    AppendTorqueAndTime(torque_and_time);
    return true;
  }else if(name == "brake") {
    fprintf(stderr,"Brake is not done yet\n");
    return false;
  }
  return false;
}

void ContactStabilityController::AppendTorqueAndTime( Vector &torque_and_time )
{
  std::vector<Real> torque_tmp(torque_and_time.begin(),torque_and_time.end()-1);
  Vector torque(torque_tmp);
  torques.push_back(torque);
  times.push_back(torque_and_time(torque_and_time.size()-1));
  // std::cout << torque_and_time << std::endl;
  // std::cout << torque << std::endl;
  // std::cout << "time:" << times.back() << std::endl;
  // exit(0);
}

vector<string> ContactStabilityController::Commands() const
{
  vector<string> res;
  res.push_back("set_q");
  res.push_back("set_torque_control");
  res.push_back("append_torque_control");
  res.push_back("brake");
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Controller Commands:" << std::endl;
  for(int i = 0; i < res.size(); i++){
    std::cout << res.at(i) << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  return res;
}
