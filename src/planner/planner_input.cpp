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

std::vector<std::string> PlannerMultiInput::GetAlgorithms(TiXmlElement *node, bool kinodynamic)
{
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
std::vector<std::string> PlannerMultiInput::GetAlgorithmsDefault(bool kinodynamic)
{
  std::string pidef = util::GetDataFolder()+"/../settings/planner.xml";
  TiXmlDocument doc(pidef);
  TiXmlElement *node = GetRootNodeFromDocument(doc);

  CheckNodeName(node, "planner");
  return GetAlgorithms(node, kinodynamic);
}
std::vector<std::string> PlannerMultiInput::GetAlgorithmsCustom(TiXmlElement *node, bool kinodynamic)
{
  CheckNodeName(node, "plannerinput");
  return GetAlgorithms(node, kinodynamic);
}

bool PlannerMultiInput::Load(TiXmlElement *node){
  CheckNodeName(node, "world");
  TiXmlElement* node_plannerinput = FindSubNode(node, "plannerinput");

  if(!node_plannerinput){
    std::cout << "world xml file has no plannerinput" << std::endl;
    return false;
  }

  bool kinodynamic = GetSubNodeTextDefault<int>(node_plannerinput, "kinodynamic", false);

  TiXmlElement* node_algorithms = FindSubNode(node_plannerinput, "algorithm");
  bool hasCustomAlgorithms = (node_algorithms != nullptr);
  
  std::vector<std::string> algorithms;

  if(hasCustomAlgorithms){
      algorithms = GetAlgorithmsCustom(node_plannerinput, kinodynamic);
  }else{
      algorithms = GetAlgorithmsDefault(kinodynamic);
  }
  const int i_hierarchy = CountNumberOfSubNodes(node_plannerinput, "hierarchy");

  for(uint k_algorithm = 0; k_algorithm < algorithms.size(); k_algorithm++){
    std::string name_algorithm = algorithms.at(k_algorithm);
    if(util::StartsWith(name_algorithm, "benchmark") || util::StartsWith(name_algorithm, "fiberoptimizer")){
      PlannerInput* input = new PlannerInput();
      input->name_algorithm = algorithms.at(k_algorithm);
      if(!input->Load(node_plannerinput)) return false;

      std::cout << "HIERARCHIES:" << i_hierarchy << std::endl;
      for(uint k_hierarchy = 0; k_hierarchy < (uint)i_hierarchy; k_hierarchy++){
        input->ExtractHierarchy(node_plannerinput, k_hierarchy);
      }
      inputs.push_back(input);
    }else{
      uint number_of_hierarchies = 1;
      if(util::StartsWith(name_algorithm, "hierarchy")){
        number_of_hierarchies = i_hierarchy;
      }
      for(uint k_hierarchy = 0; k_hierarchy < number_of_hierarchies; k_hierarchy++){

        PlannerInput* input = new PlannerInput();
        input->name_algorithm = algorithms.at(k_algorithm);

        if(!input->Load(node_plannerinput)) return false;
        input->ExtractHierarchy(node_plannerinput, k_hierarchy);

        inputs.push_back(input);
      }
    }
  }

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
  contactPlanner = GetSubNodeText<int>(node, "contactPlanner");
  timestep_min = GetSubNodeAttribute<double>(node, "timestep", "min");
  timestep_max = GetSubNodeAttribute<double>(node, "timestep", "max");
  max_planning_time = GetSubNodeText<double>(node, "maxplanningtime");
  epsilon_goalregion = GetSubNodeText<double>(node, "epsilongoalregion");
  pathSpeed = GetSubNodeText<double>(node, "pathSpeed");
  pathWidth = GetSubNodeText<double>(node, "pathWidth");
  pathBorderWidth = GetSubNodeText<double>(node, "pathBorderWidth");
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
  contactPlanner = GetSubNodeTextDefault(node, "contactPlanner", contactPlanner);
  robot_idx = GetSubNodeTextDefault(node, "robot", 0);
  timestep_min = GetSubNodeAttributeDefault(node, "timestep", "min", timestep_min);
  timestep_max = GetSubNodeAttributeDefault(node, "timestep", "max", timestep_max);
  max_planning_time = GetSubNodeTextDefault(node, "maxplanningtime", max_planning_time);
  epsilon_goalregion = GetSubNodeTextDefault(node, "epsilongoalregion", epsilon_goalregion);
  pathSpeed = GetSubNodeTextDefault(node, "pathSpeed", pathSpeed);
  pathWidth = GetSubNodeTextDefault(node, "pathWidth", pathWidth);
  pathBorderWidth = GetSubNodeTextDefault(node, "pathBorderWidth", pathBorderWidth);
  smoothPath = GetSubNodeTextDefault(node, "smoothPath", smoothPath);
  name_sampler = GetSubNodeAttributeDefault<std::string>(node, "sampler", "name", name_sampler);
  kinodynamic = GetSubNodeTextDefault(node, "kinodynamic", kinodynamic);
  name_loadPath = GetSubNodeAttributeDefault<std::string>(node, "loadPath", "file", name_loadPath);
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

    contact_links.clear();
    TiXmlElement* node_contacts = FindSubNode(node, "contacts");
    if(node_contacts != nullptr)
    {
        TiXmlElement* node_contact = FindFirstSubNode(node_contacts, "contact");
        while(node_contact != nullptr)
        {
            ContactInformation c_link;

            std::string robot_name = GetAttribute<std::string>(node_contact, "robot_name");
            c_link.robot_name = robot_name;

            std::string link = GetAttribute<std::string>(node_contact, "robot_link");
            c_link.robot_link = link;

            std::string mode = GetAttribute<std::string>(node_contact, "mode");
            c_link.mode = mode;

            if(mode=="fixed")
            {
                std::string mesh = GetAttribute<std::string>(node_contact, "mesh");
                int tri = GetAttributeDefault<int>(node_contact, "tri", -1);
                c_link.meshFrom = mesh;
                c_link.triFrom = tri;
            }else if(mode=="transition")
            {
                std::string meshFrom = GetAttribute<std::string>(node_contact, "meshFrom");
                int triFrom = GetAttributeDefault<int>(node_contact, "triFrom", -1);
                std::string meshTo = GetAttribute<std::string>(node_contact, "meshTo");
                int triTo = GetAttributeDefault<int>(node_contact, "triTo", -1);
                c_link.meshFrom = meshFrom;
                c_link.triFrom = triFrom;
                c_link.meshTo = meshTo;
                c_link.triTo = triTo;
            }else{
                std::cout << "ERROR: mode " << mode << " unknown." << std::endl;
                throw "MODE";
            }
            contact_links.push_back(c_link);
            node_contact = FindNextSiblingNode(node_contact);
        }
    }

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
    node_hierarchy = FindNextSiblingNode(node_hierarchy);
    ctr++;
  }
  uint level = 0;
  Stratification stratification;

  if(node_hierarchy){
    TiXmlElement* lindex = FindFirstSubNode(node_hierarchy, "level");
    //layers.clear();
    while(lindex!=nullptr){
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
  cin->contact_links = contact_links;
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
  sin->name_loadPath = name_loadPath;
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
  out << "loadPath           : " << pin.name_loadPath << std::endl;
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
