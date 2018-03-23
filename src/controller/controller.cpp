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
    for(uint i = 0; i < N; i++){
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
  //Vector3 AM = angmomentum_window.back();

  double dt = tstep;
  double dt2 = tstep*tstep*0.5;

  Vector3 dcom = LM/mass;

  predicted_com.clear();
  for(uint i = 0; i < Nsteps; i++){
    predicted_com.push_back(com);
    com = com + dt * dcom + dt2 * force/mass;
    dcom = dcom + dt * force/mass;
  }

}

double ControllerState::getLength(std::vector<Vector3> &path){
  double length = 0.0;
  for(uint i = 0; i < path.size()-1; i++){
    length += (path.at(i) - path.at(i+1)).norm();
  }
  return length;
}
void ControllerState::Reset(){
  predicted_com.clear();
  predicted_com_dir.clear();
  com_window.clear();
  linmomentum_window.clear();
  angmomentum_window.clear();
}
//################################################################################
//################################################################################
//################################################################################

ContactStabilityController::ContactStabilityController(Robot& robot) : RobotController(robot) 
{
}
ContactStabilityController::~ContactStabilityController() {}
const char* ContactStabilityController::Type() const { return "SE3Controller"; }

void ContactStabilityController::Reset() { 
  if(command){
    uint N = command->actuators.size();
    if(N>0){
      ZeroTorque.resize(N); 
      ZeroTorque.setZero();
      SetTorqueCommand(ZeroTorque);
    }
  }
  output.Reset();
  RobotController::Reset(); 

  //std::cout << std::string(80, '-') << std::endl;
  //std::cout << "["<<Type() << "] Reset" << std::endl;
  //std::cout << std::string(80, '-') << std::endl;

  //uint Nsensors = sensors->sensors.size();
  //std::cout << "Sensors : " << std::endl;
  //for(uint i = 0; i < Nsensors; i++){
  //  std::cout << "         " << sensors->sensors.at(i)->name << std::endl;
  //}
  //std::cout << std::string(80, '-') << std::endl;
} 

const ControllerState& ContactStabilityController::GetControllerState() const {
  return output;
}

void ContactStabilityController::Update(Real dt) {
  //We'll put our code here: read from this->sensors, and write to this->command.
  //See Sensor.h and Command.h for details on these structures
  std::cout << "controller time " << time << " dt=" << dt << std::endl;

  Vector q_sensed,dq_sensed;
  GetSensedConfig(q_sensed);
  GetSensedVelocity(dq_sensed);

  std::cout << std::string(80, '-') << std::endl;
  std::cout << q_sensed << std::endl;
  std::cout << robot.q << std::endl;

  robot.q = q_sensed;
  robot.dq = dq_sensed;
  robot.UpdateDynamics();

  Vector3 com = robot.GetCOM();
  Vector3 LM = robot.GetLinearMomentum();
  Vector3 AM = robot.GetAngularMomentum();

  output.SetMass( robot.GetTotalMass() );
  output.AddCOM(com, LM, AM);
  output.PredictCOM(0.001, 1000);

  if(torques.size()>0){
    output.current_torque = ZeroTorque;

    uint ictr = 0;
    double t = 0;
    while(t <= time*0.1 && ictr<times.size()){
      t+= times.at(ictr++);
    }
    //std::cout << time << "::" << t << "::" << ictr << std::endl;
    if(t>time*0.1){
      Vector torque = torques.at(ictr-1);
      output.current_torque = torque;
    }else{
      output.current_torque = ZeroTorque;
    }
    SetWrenchCommand(output.current_torque);
  }

  RobotController::Update(dt);

}

void ContactStabilityController::SetWrenchCommand(const Vector& wrenches)
{
  if((uint)wrenches.size()!=robot.drivers.size()) {
    std::cout << "wrenches are size " << wrenches.size() << " but drivers are size " << robot.drivers.size() << std::endl;
    exit(0);
  }

  Vector T_wrenches(wrenches); T_wrenches.setZero();

  for(uint i = 0; i < robot.drivers.size(); i++){
    const RobotJointDriver& driver = robot.drivers[i];
    //############################################################################
    if(driver.type == RobotJointDriver::Rotation){
      T_wrenches[i] = wrenches[i];
    }
  //############################################################################
    if(driver.type == RobotJointDriver::Translation){
      uint didx = driver.linkIndices[0]; //actual driver
      uint lidx = driver.linkIndices[1]; //link affected by driver
      Frame3D Tw = robot.links[lidx].T_World;
      Vector3 pos = Tw*robot.links[lidx].com;
      Vector3 dir = Tw*robot.links[didx].w - pos;
      dir = wrenches(i)*dir/dir.norm();
      for(uint k = 0; k < 3; k++){
        T_wrenches[k]+=dir[k];
      }

    }
  }
  //std::cout << "torques: " << wrenches << std::endl;
  std::cout << "torques: " << T_wrenches << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  //SetTorqueCommand(T_wrenches);
  for(size_t i=0;i<robot.drivers.size();i++)
      command->actuators[i].SetTorque(T_wrenches[i]);


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
    ZeroTorque = torques.back();
    ZeroTorque.setZero();
    SetTorqueCommand(ZeroTorque);
    overall_time = times.back();
    return true;
  }else if(name == "append_torque_control") {
    ss >> torque_and_time;
    AppendTorqueAndTime(torque_and_time);
    std::cout << "Controller: append_torque_control " << torques.size() << " last item " << torques.back();
    overall_time += times.back();
    std::cout << " t " << times.back() << " overall time " << overall_time << std::endl;
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
}

vector<string> ContactStabilityController::Commands() const
{
  vector<string> res;
  res.push_back("set_q");
  res.push_back("set_torque_control");
  res.push_back("append_torque_control");
  res.push_back("brake");
  return res;
}
