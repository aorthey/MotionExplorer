#include "environment_loader.h"
#include "controller/controller.h"
#include "loader.h"
#include <boost/filesystem.hpp>

RobotWorld& EnvironmentLoader::GetWorld(){
  return world;
}
RobotWorld* EnvironmentLoader::GetWorldPtr(){
  return &world;
}
Robot* EnvironmentLoader::GetRobotPtr(){
  return world.robots[0];
}
PlannerInput EnvironmentLoader::GetPlannerInput(){
  return pin;
}
ForceFieldBackendPtr EnvironmentLoader::GetBackendPtr(){
  return _backend;
}

EnvironmentLoader EnvironmentLoader::from_args(int argc,const char** argv){
  std::string exec = argv[0];
  std::string file;
  std::vector<std::string> all_args;

  if (argc > 1) {
    file = argv[1];
    all_args.assign(argv + 1, argv + argc);
  }else{
    std::cout << "Usage: <xml world file>" << std::endl;
    exit(0);
  }
  using namespace boost::filesystem;
  path cur = current_path();
  file = cur.string()+"/"+file;
  return EnvironmentLoader(file.c_str());
}
EnvironmentLoader::EnvironmentLoader(const char *file_name_){
  file_name = file_name_;

  std::cout << "[EnvironmentLoader] loading from file " << file_name << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  world.background = GLColor(1,1,1);

  _backend = new ForceFieldBackend(&world);
  if(!_backend->LoadAndInitSim(file_name.c_str())){
    std::cout << std::string(80, '-') << std::endl;
    std::cout << std::endl;
    std::cout << "ERROR:" << std::endl;
    std::cout << std::endl;
    std::cout << "XML file does not exists or corrupted: "<< file_name << std::endl;
    std::cout << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }


  uint Nrobots = world.robots.size();
  if(Nrobots<1){
    std::cout << "Current planner only supports one robot! selected " << Nrobots << " robots." << std::endl;
    for(uint i = 0; i < Nrobots; i++){
      std::cout << world.robots[i]->name << std::endl;
    }
    exit(0);
  }
  name_robot = world.robots[0]->name;
//################################################################################
//################################################################################

  if(!(world.robots[0]->joints[0].type == RobotJoint::Floating)){
    std::cout << "First joint of robot should be a free floating joint" << std::endl;
    std::cout << "But actual type is: " << world.robots[0]->joints[0].type << std::endl;
    exit(0);
  }

  Robot *robot = world.robots[0];


  bool addSE3drivers = false;
  if(addSE3drivers){
    std::cout << "Adding free float driver to robot " << name_robot << std::endl;
    vector<string>* driverNames = &robot->driverNames;
    vector<RobotJointDriver>* drivers = &robot->drivers;

    RobotJointDriver translation[3], rotation[3];

    //TODO read any jet propulsion mechanisms directly from XML (i.e. add jet
    //propulsion while specifying robot urdf)
    // Rotation is eulerangles ZYX

    for(int i = 2; i >= 0; i--){
      rotation[i].type = RobotJointDriver::Rotation;
      rotation[i].linkIndices.push_back(i+3);
      rotation[i].linkIndices.push_back(5);
      rotation[i].qmin = -dInf;
      rotation[i].qmax = dInf;
      rotation[i].vmin = -dInf;
      rotation[i].vmax = dInf;
      rotation[i].tmin = -dInf;
      rotation[i].tmax = dInf;
      rotation[i].amin = -dInf;
      rotation[i].amax = dInf;
      rotation[i].servoP = 0;
      rotation[i].servoI = 0;
      rotation[i].servoD = 0;
      rotation[i].dryFriction = 0;
      rotation[i].viscousFriction = 0;
      drivers->insert(drivers->begin(), rotation[i]);
      std::string dName = "rotation e"+to_string(i);
      driverNames->insert(driverNames->begin(), dName.c_str());
    }
    for(int i = 2; i >= 0; i--){
      translation[i].type = RobotJointDriver::Translation;
      translation[i].linkIndices.push_back(i);
      translation[i].linkIndices.push_back(5);
      translation[i].qmin = -dInf;
      translation[i].qmax = dInf;
      translation[i].vmin = -dInf;
      translation[i].vmax = dInf;
      translation[i].tmin = -dInf;
      translation[i].tmax = dInf;
      translation[i].amin = -dInf;
      translation[i].amax = dInf;
      translation[i].servoP = 0;
      translation[i].servoI = 0;
      translation[i].servoD = 0;
      translation[i].dryFriction = 0;
      translation[i].viscousFriction = 0;
      drivers->insert(drivers->begin(), translation[i]);
      std::string dName = "translation e"+to_string(i);
      driverNames->insert(driverNames->begin(), dName.c_str());
    }

    //  nd = (int) drivers.size();
    SmartPointer<RobotController> controller = new ContactStabilityController(*robot);

    RobotControllerFactory::Register("ContactStabilityController", controller);
    _backend->sim.SetController(0, controller);

    //reinit robot such that drivers are copied to actuators (so we can use them in Command)
    _backend->sim.controlSimulators[0].Init(robot, _backend->sim.odesim.robot(0), _backend->sim.robotControllers[0]);

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "LOADED CONTACTSTABILITYCONTROLLER" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
  }
  info(&world);

  if(pin.load(file_name.c_str())){

    uint ridx = pin.robot_idxs.at(0);
    Robot *robot = world.robots[ridx];

    for(int i = 0; i < 6; i++){
      robot->qMin[i] = pin.se3min[i];
      robot->qMax[i] = pin.se3max[i];
    }

    pin.qMin = robot->qMin;
    pin.qMax = robot->qMax;
    robot->q = pin.q_init;
    robot->UpdateFrames();

    //set oderobot to planner start pos
    ODERobot *simrobot = _backend->sim.odesim.robot(ridx);
    simrobot->SetConfig(pin.q_init);
    Vector dq;
    _backend->sim.odesim.robot(0)->GetVelocities(dq);
    dq.setZero();
    _backend->sim.odesim.robot(0)->SetVelocities(dq);

    robot->q = pin.q_goal;
    robot->UpdateFrames();

    //set other nested robots
    for(uint k = 0; k < pin.robot_idxs.size(); k++){
      uint ik = pin.robot_idxs.at(k);
      Robot *rk= world.robots[ik];

      for(int i = 0; i < 6; i++){
        rk->qMin[i] = pin.se3min[i];
        rk->qMax[i] = pin.se3max[i];
      }
    }
  }else{
    std::cout << "No Planner Settings. No Planning" << std::endl;
  }
  //_backend->sim.simStep = 0.001;
  //WorldSimulation sim = _backend->sim;
  for(uint i = 0; i < _backend->sim.odesim.numRobots(); i++){
    ODERobot *simrobot = _backend->sim.odesim.robot(i);
    simrobot->EnableSelfCollisions(true);
    std::cout << "SelfCollisionsEnabled" << std::endl;
  }

  _backend->wrenchfield.LoadFromWorldFile(file_name.c_str());
  _backend->wrenchfield.print();

}
bool EnvironmentLoader::LoadPath(const char* file)
{
  std::string file_name = util::GetApplicationFolder()+file;
  TiXmlDocument doc(file_name.c_str());
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  LoadPath(root);
  return true;
}
bool EnvironmentLoader::LoadPath(TiXmlElement *node)
{
  CheckNodeName(node, "path");

  TiXmlElement* q = FindFirstSubNode(node, "q");
  std::vector<Config> keyframes;
  while(q!=NULL){
    //std::cout << q->GetText() << std::endl;
    Config qconfig;
    GetStreamText(q) >> qconfig;
    keyframes.push_back(qconfig);
    q = FindNextSiblingNode(q, "q");
  }
  _backend->AddPath(keyframes);

  return true;
}
std::vector<Config> EnvironmentLoader::GetKeyframesFromFile(const char* file)
{
  std::string file_name = util::GetApplicationFolder()+file;
  TiXmlDocument doc(file_name.c_str());
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  CheckNodeName(root, "path");

  TiXmlElement* q = FindFirstSubNode(root, "q");
  std::vector<Config> keyframes;
  while(q!=NULL){
    //std::cout << q->GetText() << std::endl;
    Config qconfig;
    GetStreamText(q) >> qconfig;
    keyframes.push_back(qconfig);
    q = FindNextSiblingNode(q, "q");
  }

  return keyframes;
}
