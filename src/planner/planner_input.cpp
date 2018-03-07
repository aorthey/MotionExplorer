#include "util.h"
#include "planner/planner_input.h"
#include <KrisLibrary/math/VectorTemplate.h>

bool PlannerMultiInput::Load(const char* file){
  TiXmlDocument doc(file);
  return Load(GetRootNodeFromDocument(doc));
}

std::vector<std::string> PlannerMultiInput::GetAlgorithms()
{
  std::string pidef = util::GetDataFolder()+"/../settings/planner.xml";
  TiXmlDocument doc(pidef);
  TiXmlElement *node = GetRootNodeFromDocument(doc);

  CheckNodeName(node, "planner");

  TiXmlElement* node_algorithm = FindFirstSubNode(node, "algorithm");
  std::vector<std::string> algorithms;
  while(node_algorithm!=NULL){
    std::string a = GetAttribute<std::string>(node_algorithm, "name");
    algorithms.push_back(a);
    node_algorithm = FindNextSiblingNode(node_algorithm);
  }
  return algorithms;
}

bool PlannerMultiInput::Load(TiXmlElement *node){
  CheckNodeName(node, "world");
  TiXmlElement* node_plannerinput = FindSubNode(node, "plannerinput");

  if(!node_plannerinput){
    std::cout << "world xml file has no plannerinput" << std::endl;
    return false;
  }

  std::vector<std::string> algorithms = GetAlgorithms();
  for(uint k = 0; k < algorithms.size(); k++){

    PlannerInput* input = new PlannerInput();
    if(!input->Load(node_plannerinput)) return false;

    input->name_algorithm = algorithms.at(k);
    inputs.push_back(input);
  }

  // for(uint k = 0; k < inputs.size(); k++){
  //   std::cout << *inputs.at(k) << std::endl;
  // }
  return true;
}

void PlannerInput::SetDefault()
{
  std::string pidef = util::GetDataFolder()+"/../settings/planner.xml";
  TiXmlDocument doc(pidef);
  TiXmlElement *node = GetRootNodeFromDocument(doc);

  CheckNodeName(node, "planner");

  max_planning_time = GetSubNodeText<double>(node, "maxplanningtime");
  freeFloating = GetSubNodeText<int>(node, "freeFloating");
  timestep_min = GetSubNodeAttribute<double>(node, "timestep", "min");
  timestep_max = GetSubNodeAttribute<double>(node, "timestep", "max");
  max_planning_time = GetSubNodeText<double>(node, "maxplanningtime");
  epsilon_goalregion = GetSubNodeText<double>(node, "epsilongoalregion");
  pathSpeed = GetSubNodeText<double>(node, "pathSpeed");
  smoothPath = GetSubNodeText<int>(node, "smoothPath");
  enableSufficiency = GetSubNodeText<int>(node, "enableSufficiency");
  name_sampler = GetSubNodeAttribute<std::string>(node, "sampler", "name");
}

bool PlannerInput::Load(TiXmlElement *node)
{
  SetDefault();
  CheckNodeName(node, "plannerinput");

  //necessary arguments

  q_init = GetSubNodeAttribute<Config>(node, "qinit", "config");
  q_goal = GetSubNodeAttribute<Config>(node, "qgoal", "config");

  Config dq; dq.resize(q_init.size()); dq.setZero();
  dq_init = GetSubNodeAttributeDefault<Config>(node, "dqinit", "config", dq);
  dq_goal = GetSubNodeAttributeDefault<Config>(node, "dqgoal", "config", dq);
  se3min = GetSubNodeAttribute<Config>(node, "se3min", "config");
  se3max = GetSubNodeAttribute<Config>(node, "se3max", "config");

  TiXmlElement* node_hierarchy = FindSubNode(node, "hierarchy");
  uint level = 0;

  if(node_hierarchy){
    TiXmlElement* lindex = FindFirstSubNode(node_hierarchy, "level");
    robot_idxs.clear();
    layers.clear();
    while(lindex!=NULL){
      Layer layer;

      layer.level = level++;
      layer.inner_index = GetAttribute<int>(lindex, "inner_index");
      layer.outer_index = GetAttributeDefault<int>(lindex, "outer_index", layer.inner_index);
      layer.type = GetAttribute<std::string>(lindex, "type");

      robot_idxs.push_back(layer.inner_index);
      layers.push_back(layer);

      lindex = FindNextSiblingNode(lindex);
    }
  }else{
    //std::cout << "[WARNING] Did not specify robot hierarchy. Assuming one layer SE3RN" << std::endl;
    Layer layer;
    layer.level = 0;
    layer.inner_index = 0;
    layer.outer_index = 0;
    layer.type = "SE3RN";
    robot_idxs.push_back(layer.inner_index);
    layers.push_back(layer);
  }

  //optional arguments

  freeFloating = GetSubNodeTextDefault(node, "freeFloating", freeFloating);
  robot_idx = GetSubNodeTextDefault(node, "robot", 0);
  timestep_min = GetSubNodeAttributeDefault(node, "timestep", "min", timestep_min);
  timestep_max = GetSubNodeAttributeDefault(node, "timestep", "max", timestep_max);
  max_planning_time = GetSubNodeTextDefault(node, "maxplanningtime", max_planning_time);
  epsilon_goalregion = GetSubNodeTextDefault(node, "epsilongoalregion", epsilon_goalregion);
  pathSpeed = GetSubNodeTextDefault(node, "pathSpeed", pathSpeed);
  smoothPath = GetSubNodeTextDefault(node, "smoothPath", smoothPath);
  enableSufficiency = GetSubNodeTextDefault(node, "enableSufficiency", enableSufficiency);
  name_sampler = GetSubNodeAttributeDefault<std::string>(node, "sampler", "name", name_sampler);

  return true;
}

bool PlannerInput::Load(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return Load(root);
}
std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) 
{
  out << std::string(80, '-') << std::endl;
  out << "[PlannerInput] " << pin.name_algorithm << std::endl;
  out << std::string(80, '-') << std::endl;
  out << "q_init             : " << pin.q_init << std::endl;
  out << "  dq_init          : " << pin.dq_init << std::endl;
  out << "q_goal             : " << pin.q_goal << std::endl;
  out << "  dq_goal          : " << pin.dq_goal << std::endl;
  out << "SE3_min            : " << pin.se3min << std::endl;
  out << "SE3_max            : " << pin.se3max << std::endl;
  out << "algorithm          : " << pin.name_algorithm << std::endl;
  out << "sampler            : " << pin.name_sampler << std::endl;
  out << "discr timestep     : [" << pin.timestep_min << "," << pin.timestep_max << "]" << std::endl;
  out << "max planning time  : " << pin.max_planning_time << " (seconds)" << std::endl;
  out << "epsilon_goalregion : " << pin.epsilon_goalregion << std::endl;
  out << "robot              : " << pin.robot_idx << std::endl;
  out << "robot indices      : ";
  for(uint k = 0; k < pin.robot_idxs.size(); k++){
    out << " " << pin.robot_idxs.at(k);
  }
  out << std::endl;
    
  out << std::string(80, '-') << std::endl;
  return out;
}
