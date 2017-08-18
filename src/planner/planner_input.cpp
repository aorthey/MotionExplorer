#include "planner_input.h"

bool PlannerInput::load(TiXmlElement *node)
{
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

  TiXmlElement* node_drawTree = FindSubNode(plannersettings, "drawTree");
  TiXmlElement* node_drawSimplicialComplex = FindSubNode(plannersettings, "drawSimplicialComplex");
  TiXmlElement* node_drawSweptVolume = FindSubNode(plannersettings, "drawSweptVolume");
  TiXmlElement* node_drawMilestones = FindSubNode(plannersettings, "drawMilestones");
  TiXmlElement* node_drawStartGoal = FindSubNode(plannersettings, "drawStartGoal");

  GetStreamText(node_drawTree) >> drawTree;
  GetStreamText(node_drawSimplicialComplex) >> drawSimplicialComplex;
  GetStreamText(node_drawSweptVolume) >> drawSweptVolume;
  GetStreamText(node_drawMilestones) >> drawMilestones;
  GetStreamText(node_drawStartGoal) >> drawStartGoal;

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

  GetStreamText(node_max_planning_time) >> max_planning_time;
  GetStreamText(node_epsilon_goalregion) >> epsilon_goalregion;

  GetStreamAttribute(node_se3min,"config")  >> se3min;
  GetStreamAttribute(node_se3max,"config")  >> se3max;
  GetStreamText(node_algorithm) >> name_algorithm;

  TiXmlElement* robot = FindSubNode(plannersettings, "robot");
  if(robot){
    TiXmlElement* rindex = FindSubNode(robot, "index");
    if(rindex) GetStreamText(rindex) >> robot_idx;
    else robot_idx = 0;

    TiXmlElement* rindexos = FindSubNode(robot, "indexoutershell");
    if(rindexos) GetStreamText(rindexos) >> robot_idx_outer_shell;
    else robot_idx_outer_shell = -1;
  }else{
    robot_idx = 0;
    robot_idx_outer_shell = -1;
  }

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
  out << "q_init            : " << pin.q_init << std::endl;
  out << "  dq_init         : " << pin.dq_init << std::endl;
  out << "q_goal            : " << pin.q_goal << std::endl;
  out << "  dq_goal         : " << pin.dq_goal << std::endl;
  out << "SE3_min           : " << pin.se3min << std::endl;
  out << "SE3_max           : " << pin.se3max << std::endl;
  out << "algorithm         : " << pin.name_algorithm << std::endl;
  out << "discr timestep    : [" << pin.timestep_min << "," << pin.timestep_max << "]" << std::endl;
  out << "max planning time : " << pin.max_planning_time << " (seconds)" << std::endl;
  out << "epsilon_goalregion: " << pin.epsilon_goalregion<< std::endl;
  out << "robot index       : " << pin.robot_idx << std::endl;
  out << "robot index       : " << pin.robot_idx_outer_shell << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
