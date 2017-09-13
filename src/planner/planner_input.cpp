#include <KrisLibrary/math/VectorTemplate.h>

#include "planner_input.h"

bool PlannerInput::load(TiXmlElement *node)
{
  exists = false;
  CheckNodeName(node, "world");
  TiXmlElement* plannersettings = FindSubNode(node, "plannersettings");

  if(!plannersettings){
    std::cout << "world xml file has no plannersettings" << std::endl;
    return false;
  }

  TiXmlElement* node_qinit = FindSubNode(plannersettings, "qinit");
  TiXmlElement* node_qgoal = FindSubNode(plannersettings, "qgoal");
  TiXmlElement* node_se3min = FindSubNode(plannersettings, "se3min");
  TiXmlElement* node_se3max = FindSubNode(plannersettings, "se3max");
  TiXmlElement* node_algorithm = FindSubNode(plannersettings, "algorithm");

  TiXmlElement* node_dqinit = FindSubNode(plannersettings, "dqinit");
  TiXmlElement* node_dqgoal = FindSubNode(plannersettings, "dqgoal");
  TiXmlElement* node_timestep = FindSubNode(plannersettings, "timestep");
  TiXmlElement* node_max_planning_time = FindSubNode(plannersettings, "maxplanningtime");
  TiXmlElement* node_epsilon_goalregion = FindSubNode(plannersettings, "epsilongoalregion");
  TiXmlElement* node_freeFloating = FindSubNode(plannersettings, "freeFloating");

  TiXmlElement* node_drawTree = FindSubNode(plannersettings, "drawTree");
  TiXmlElement* node_drawSimplicialComplex = FindSubNode(plannersettings, "drawSimplicialComplex");
  TiXmlElement* node_drawSweptVolume = FindSubNode(plannersettings, "drawSweptVolume");
  TiXmlElement* node_drawMilestones = FindSubNode(plannersettings, "drawMilestones");
  TiXmlElement* node_drawStartGoal = FindSubNode(plannersettings, "drawStartGoal");
  TiXmlElement* node_drawShortestPath = FindSubNode(plannersettings, "drawShortestPath");

  GetStreamText(node_drawTree) >> drawTree;
  GetStreamText(node_drawSimplicialComplex) >> drawSimplicialComplex;
  GetStreamText(node_drawSweptVolume) >> drawSweptVolume;
  GetStreamText(node_drawMilestones) >> drawMilestones;
  GetStreamText(node_drawStartGoal) >> drawStartGoal;
  GetStreamText(node_drawShortestPath) >> drawShortestPath;

  GetStreamAttribute(node_qinit,"config") >> q_init;
  GetStreamAttribute(node_qgoal,"config") >> q_goal;
  GetStreamAttribute(node_dqinit,"config") >> dq_init;
  GetStreamAttribute(node_dqgoal,"config") >> dq_goal;

  if(node_timestep){
    GetStreamAttribute(node_timestep,"min") >> timestep_min;
    GetStreamAttribute(node_timestep,"max") >> timestep_max;
  }else{
    timestep_min= 0.01;
    timestep_max= 0.1;
  }
  GetStreamText(node_freeFloating) >> freeFloating;

  GetStreamText(node_max_planning_time) >> max_planning_time;
  GetStreamText(node_epsilon_goalregion) >> epsilon_goalregion;

  GetStreamAttribute(node_se3min,"config")  >> se3min;
  GetStreamAttribute(node_se3max,"config")  >> se3max;
  GetStreamText(node_algorithm) >> name_algorithm;

  TiXmlElement* node_robot = FindSubNode(plannersettings, "robot");
  if(node_robot){
    TiXmlElement* rindex = FindFirstSubNode(node_robot, "index");
    while(rindex!=NULL){
      int tmp;
      GetStreamText(rindex) >> tmp;
      if(tmp<0){
        std::cout << "Robot Index " << tmp << " is negative. Not valid." << std::endl;
        exit(0);
      }
      robot_idxs.push_back(tmp);
      rindex = FindNextSiblingNode(rindex, "index");
    }
    std::cout << "Loading Robots: ";
    for(uint k = 0; k < robot_idxs.size(); k++){
      std::cout << " " << robot_idxs.at(k);
    }
    std::cout << std::endl;
    if(robot_idxs.size()>0){
      robot_idx = robot_idxs.at(0);
    }else{
      robot_idx = 0;
    }

  }else{
    robot_idx = 0;
  }

  exists = true;
  return true;
}

bool PlannerInput::load(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return load(root);
}
std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) 
{
  out << std::string(80, '-') << std::endl;
  out << "[PlannerInput]" << std::endl;
  out << std::string(80, '-') << std::endl;
  out << "q_init             : " << pin.q_init << std::endl;
  out << "  dq_init          : " << pin.dq_init << std::endl;
  out << "q_goal             : " << pin.q_goal << std::endl;
  out << "  dq_goal          : " << pin.dq_goal << std::endl;
  out << "SE3_min            : " << pin.se3min << std::endl;
  out << "SE3_max            : " << pin.se3max << std::endl;
  out << "algorithm          : " << pin.name_algorithm << std::endl;
  out << "discr timestep     : [" << pin.timestep_min << "," << pin.timestep_max << "]" << std::endl;
  out << "max planning time  : " << pin.max_planning_time << " (seconds)" << std::endl;
  out << "epsilon_goalregion : " << pin.epsilon_goalregion<< std::endl;
  out << "robot indices      : ";
  for(uint k = 0; k < pin.robot_idxs.size(); k++){
    out << " " << pin.robot_idxs.at(k);
  }
  out << std::endl;
    
  out << std::string(80, '-') << std::endl;
  return out;
}
