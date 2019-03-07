#include "util.h"
#include "common.h"
#include "planner/planner_input.h"
#include "cspace/cspace_input.h"
#include "planner/strategy/strategy_input.h"
#include <KrisLibrary/math/VectorTemplate.h>
#include <boost/filesystem.hpp>

bool PlannerMultiInput::Load(const char* file){
  TiXmlDocument doc(file);
  Load(GetRootNodeFromDocument(doc));
  boost::filesystem::path p(file);

  for(uint k = 0; k < inputs.size(); k++){

    inputs.at(k)->environment_name = p.filename().string();
  }
  return (inputs.size()>0);
}

std::vector<std::string> PlannerMultiInput::GetAlgorithms(bool kinodynamic)
{
  std::string pidef = util::GetDataFolder()+"/../settings/planner.xml";
  TiXmlDocument doc(pidef);
  TiXmlElement *node = GetRootNodeFromDocument(doc);

  CheckNodeName(node, "planner");

  TiXmlElement* node_algorithm = FindFirstSubNode(node, "algorithm");
  std::vector<std::string> algorithms;
  while(node_algorithm){
    std::string a = GetAttribute<std::string>(node_algorithm, "name");
    bool isDynamic = GetAttributeDefault<int>(node_algorithm, "dynamic", false);
    if(kinodynamic == isDynamic){
      algorithms.push_back(a);
    }
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

  bool kinodynamic = GetSubNodeTextDefault<int>(node_plannerinput, "kinodynamic", false);
  std::vector<std::string> algorithms = GetAlgorithms(kinodynamic);
  const int i_hierarchy = CountNumberOfSubNodes(node_plannerinput, "hierarchy");

  for(uint k_algorithm = 0; k_algorithm < algorithms.size(); k_algorithm++){
    std::string name_algorithm = algorithms.at(k_algorithm);
    std::cout << name_algorithm << std::endl;
    if(util::StartsWith(name_algorithm, "benchmark")){
      PlannerInput* input = new PlannerInput();
      input->name_algorithm = algorithms.at(k_algorithm);
      if(!input->Load(node_plannerinput)) return false;

      for(uint k_hierarchy = 0; k_hierarchy < (uint)i_hierarchy; k_hierarchy++){
        std::cout << "hierarchy: " << k_hierarchy << std::endl;
        input->ExtractHierarchy(node_plannerinput, k_hierarchy);
      }
      inputs.push_back(input);
    }else{
      for(uint k_hierarchy = 0; k_hierarchy < (uint)i_hierarchy; k_hierarchy++){

        PlannerInput* input = new PlannerInput();
        input->name_algorithm = algorithms.at(k_algorithm);

        if(!input->Load(node_plannerinput)) return false;
        input->ExtractHierarchy(node_plannerinput, k_hierarchy);

        inputs.push_back(input);
      }
    }
  }
  for(uint k = 0; k < inputs.size(); k++){
    std::cout << *(inputs.at(k)) << std::endl;
  }
  // exit(0);
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
  kinodynamic = GetSubNodeText<int>(node, "kinodynamic");
  name_sampler = GetSubNodeAttribute<std::string>(node, "sampler", "name");
}

bool PlannerInput::Load(TiXmlElement *node, int hierarchy_index)
{
  SetDefault();
  CheckNodeName(node, "plannerinput");

  //optional arguments

  freeFloating = GetSubNodeTextDefault(node, "freeFloating", freeFloating);
  robot_idx = GetSubNodeTextDefault(node, "robot", 0);
  timestep_min = GetSubNodeAttributeDefault(node, "timestep", "min", timestep_min);
  timestep_max = GetSubNodeAttributeDefault(node, "timestep", "max", timestep_max);
  max_planning_time = GetSubNodeTextDefault(node, "maxplanningtime", max_planning_time);
  epsilon_goalregion = GetSubNodeTextDefault(node, "epsilongoalregion", epsilon_goalregion);
  pathSpeed = GetSubNodeTextDefault(node, "pathSpeed", pathSpeed);
  smoothPath = GetSubNodeTextDefault(node, "smoothPath", smoothPath);
  name_sampler = GetSubNodeAttributeDefault<std::string>(node, "sampler", "name", name_sampler);
  kinodynamic = GetSubNodeTextDefault(node, "kinodynamic", kinodynamic);
  if(kinodynamic)
  {
    uMin = GetSubNodeAttribute<Config>(node, "control_min", "config");
    uMax = GetSubNodeAttribute<Config>(node, "control_max", "config");
  }

  //necessary arguments

  q_init = GetSubNodeAttribute<Config>(node, "qinit", "config");
  q_goal = GetSubNodeAttribute<Config>(node, "qgoal", "config");

  Config dq; dq.resize(q_init.size()); dq.setZero();
  dq_init = GetSubNodeAttributeDefault<Config>(node, "dqinit", "config", dq);
  dq_goal = GetSubNodeAttributeDefault<Config>(node, "dqgoal", "config", dq);

  se3min.resize(6); se3min.setZero();
  se3max.resize(6); se3max.setZero();
  se3min = GetSubNodeAttributeDefault<Config>(node, "se3min", "config", se3min);
  se3max = GetSubNodeAttributeDefault<Config>(node, "se3max", "config", se3max);

  return true;
}

void PlannerInput::ExtractHierarchy(TiXmlElement *node, int hierarchy_index)
{
  int ctr = 0;
  TiXmlElement* node_hierarchy = FindSubNode(node, "hierarchy");
  while(ctr < hierarchy_index){
    std::cout << node_hierarchy->Value() << std::endl;
    node_hierarchy = FindNextSiblingNode(node_hierarchy);
    ctr++;
  }
  uint level = 0;
  Stratification stratification;

  if(node_hierarchy){
    TiXmlElement* lindex = FindFirstSubNode(node_hierarchy, "level");
    //layers.clear();
    while(lindex!=NULL){
      Layer layer;

      layer.level = level++;
      layer.inner_index = GetAttribute<int>(lindex, "inner_index");
      layer.outer_index = GetAttributeDefault<int>(lindex, "outer_index", layer.inner_index);
      layer.cspace_constant = GetAttributeDefault<double>(lindex, "cspace_constant", 0);

      layer.type = GetAttribute<std::string>(lindex, "type");

      stratification.layers.push_back(layer);

      lindex = FindNextSiblingNode(lindex);
    }
  }else{
    std::cout << "[WARNING] Did not specify robot hierarchy. Assuming one layer SE3RN" << std::endl;
    Layer layer;
    layer.level = 0;
    layer.inner_index = 0;
    layer.outer_index = 0;
    layer.type = "SE3RN";
    stratification.layers.push_back(layer);
  }
  stratifications.push_back(stratification);

}

const CSpaceInput& PlannerInput::GetCSpaceInput()
{
  cin = new CSpaceInput();
  cin->timestep_max = timestep_max;
  cin->timestep_min = timestep_min;
  cin->fixedBase = !freeFloating;
  cin->uMin = uMin;
  cin->uMax = uMax;
  cin->kinodynamic = kinodynamic;
  return *cin;
}
const StrategyInput& PlannerInput::GetStrategyInput()
{
  sin = new StrategyInput();
  sin->q_init = q_init;
  sin->q_goal = q_goal;
  sin->dq_init = dq_init;
  sin->dq_goal = dq_goal;
  sin->name_sampler = name_sampler;
  sin->name_algorithm = name_algorithm;
  sin->epsilon_goalregion = epsilon_goalregion;
  sin->max_planning_time = max_planning_time;
  sin->environment_name = environment_name;
  return *sin;
}

bool PlannerInput::Load(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  environment_name = file;
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
  out << "environment        : " << pin.environment_name << std::endl;
  out << "stratifications    : " << pin.stratifications.size() << std::endl;
  for(uint k = 0; k < pin.stratifications.size(); k++){
    int nr_of_layers = pin.stratifications.at(k).layers.size();
    out << " - strat " << k << " has " << nr_of_layers << " layers." << std::endl;
  }
  out << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
