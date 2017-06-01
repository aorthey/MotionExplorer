#include "controller/controller.h"
#include "loader.h"
#include "environment_loader.h"

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

EnvironmentLoader::EnvironmentLoader(const char *xml_file){
  file_name = util::GetApplicationFolder()+xml_file;

  std::cout << "[EnvironmentLoader] loading from file " << file_name << std::endl;

  world.background = GLColor(1,1,1);

  _backend = new ForceFieldBackend(&world);
  if(!_backend->LoadAndInitSim(file_name.c_str())){
    std::cout << std::string(80, '-') << std::endl;
    std::cout << std::endl;
    std::cout << "ERROR:" << std::endl;
    std::cout << std::endl;
    std::cout << "XML file does not exists or corrupted: "<<xml_file << std::endl;
    std::cout << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }


  uint Nrobots = world.robots.size();
  if(Nrobots<1){
    std::cout << "Current planner only supports one robot! selected " << Nrobots << " robots." << std::endl;
    for(int i = 0; i < Nrobots; i++){
      std::cout << world.robots[i]->name << std::endl;
    }
    exit(0);
  }
  name_robot = world.robots[0]->name;

  //name_environment = world.rigidObjects[0]->name;
  //name = name_robot + "_" + name_environment;
  //std::cout << name << std::endl;

  info(&world);
  Robot *robot = world.robots[0];

  SmartPointer<RobotController> controller = new ContactStabilityController(*robot);
  RobotControllerFactory::Register("ContactStabilityController", controller);
  _backend->sim.SetController(0, controller);


  std::cout << std::string(80, '-') << std::endl;
  std::cout << "LOADED CONTACTSTABILITYCONTROLLER" << std::endl;
  std::cout << std::string(80, '-') << std::endl;



  if(LoadPlannerSettings(file_name.c_str())){

    for(int i = 0; i < 6; i++){
      robot->qMin[i] = pin.se3min[i];
      robot->qMax[i] = pin.se3max[i];
    }
    pin.qMin = robot->qMin;
    pin.qMax = robot->qMax;
    robot->q = pin.q_init;
    robot->UpdateFrames();
    ODERobot *simrobot = _backend->sim.odesim.robot(0);
    simrobot->SetConfig(pin.q_init);
    robot->q = pin.q_goal;
    robot->UpdateFrames();
  }

  _backend->wrenchfield.LoadFromWorldFile(file_name.c_str());
  _backend->wrenchfield.print();

}
bool EnvironmentLoader::LoadPlannerSettings(TiXmlElement *node)
{
  CheckNodeName(node, "world");

  TiXmlElement* plannersettings = FindSubNode(node, "plannersettings");

  if(!plannersettings){
    std::cout << "world xml file has no plannersettings" << std::endl;
    return false;
  }

  TiXmlElement* qinit = FindSubNode(plannersettings, "qinit");
  TiXmlElement* qgoal = FindSubNode(plannersettings, "qgoal");
  TiXmlElement* se3min = FindSubNode(plannersettings, "se3min");
  TiXmlElement* se3max = FindSubNode(plannersettings, "se3max");
  TiXmlElement* algorithm = FindSubNode(plannersettings, "algorithm");

  GetStreamAttribute(qinit,"config") >> pin.q_init;
  GetStreamAttribute(qgoal,"config") >> pin.q_goal;
  GetStreamAttribute(se3min,"config")  >> pin.se3min;
  GetStreamAttribute(se3max,"config")  >> pin.se3max;
  GetStreamAttribute(algorithm, "name") >> pin.name_algorithm;

  return true;
}

bool EnvironmentLoader::LoadPlannerSettings(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return LoadPlannerSettings(root);
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
