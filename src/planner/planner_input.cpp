#include <KrisLibrary/math/VectorTemplate.h>
#include "planner/planner_input.h"

bool PlannerMultiInput::Load(const char* file){
  TiXmlDocument doc(file);
  return Load(GetRootNodeFromDocument(doc));
}
bool PlannerMultiInput::Load(TiXmlElement *node){
  CheckNodeName(node, "world");
  TiXmlElement* node_plannerinput = FindSubNode(node, "plannerinput");

  if(!node_plannerinput){
    std::cout << "world xml file has no plannerinput" << std::endl;
    return false;
  }

  //################################################################################
  // obtain default settings for all algorithms
  //################################################################################
  double max_planning_time;
  double epsilon_goalregion;
  double timestep_min;
  double timestep_max;
  std::string name_sampler;
  bool smoothPath;
  TiXmlElement* node_timestep = FindSubNode(node_plannerinput, "timestep");
  if(node_timestep){
    GetStreamAttribute(node_timestep,"min") >> timestep_min;
    GetStreamAttribute(node_timestep,"max") >> timestep_max;
  }else{
    timestep_min= 0.01;
    timestep_max= 0.1;
  }
  TiXmlElement* node_max_planning_time = FindSubNode(node_plannerinput, "maxplanningtime");
  TiXmlElement* node_epsilon_goalregion = FindSubNode(node_plannerinput, "epsilongoalregion");

  GetStreamText(node_max_planning_time) >> max_planning_time;
  GetStreamText(node_epsilon_goalregion) >> epsilon_goalregion;

  TiXmlElement* node_smooth = FindSubNode(node_plannerinput, "smoothPath");
  GetStreamTextDefault<double>(node_smooth, 1) >> smoothPath;

  TiXmlElement* node_sampler = FindSubNode(node_plannerinput, "sampler");
  GetStreamAttributeDefault<std::string>(node_sampler,"name", "uniform") >> name_sampler;

  int isSE2;
  TiXmlElement* node_se2 = FindSubNode(node_plannerinput, "se2");
  GetStreamTextDefault<int>(node_se2, 0) >> isSE2;

  //################################################################################
  // loop through all algorithms, search for individual settings; if not found
  // apply default settings 
  //################################################################################
  TiXmlElement* node_algorithm = FindFirstSubNode(node_plannerinput, "algorithm");

  while(node_algorithm!=NULL){
    PlannerInput* input = new PlannerInput();
    input->smoothPath = smoothPath;
    input->isSE2 = isSE2;
    input->name_sampler = name_sampler;

    if(!input->Load(node_plannerinput)) return false;

    GetStreamAttribute(node_algorithm,"name") >> input->name_algorithm;

    TiXmlElement* node_algorithm_max_planning_time = FindSubNode(node_algorithm, "maxplanningtime");
    TiXmlElement* node_algorithm_epsilon_goalregion = FindSubNode(node_algorithm, "epsilongoalregion");

    GetStreamTextDefault<double>(node_algorithm_max_planning_time, max_planning_time) >> input->max_planning_time;
    GetStreamTextDefault<double>(node_algorithm_epsilon_goalregion, epsilon_goalregion) >> input->epsilon_goalregion;

    TiXmlElement* node_algorithm_timestep = FindSubNode(node_algorithm, "timestep");
    if(node_algorithm_timestep){
      GetStreamAttributeDefault<double>(node_algorithm_timestep,"min", timestep_min) >> input->timestep_min;
      GetStreamAttributeDefault<double>(node_algorithm_timestep,"max", timestep_max) >> input->timestep_max;
    }else{
      input->timestep_min = timestep_min;
      input->timestep_max = timestep_max;
    }

    TiXmlElement* node_hierarchy = FindSubNode(node_algorithm, "hierarchy");
    uint level = 0;
    if(node_hierarchy){
      TiXmlElement* lindex = FindFirstSubNode(node_hierarchy, "level");
      while(lindex!=NULL){

        Layer layer;
        layer.level = level++;

        GetStreamAttribute(lindex, "inner_index") >> layer.inner_index;

        if(ExistStreamAttribute(lindex, "outer_index")){
          GetStreamAttribute(lindex, "outer_index") >> layer.outer_index;
          layer.isInnerOuter =true;
        }else{
          layer.outer_index = layer.inner_index;
          layer.isInnerOuter =false;
        }
        GetStreamAttribute(lindex, "type") >> layer.type;

        input->robot_idxs.push_back(layer.inner_index);
        input->layers.push_back(layer);

        lindex = FindNextSiblingNode(lindex, "level");
      }
    }

    inputs.push_back(input);
    node_algorithm = FindNextSiblingNode(node_algorithm, "algorithm");
  }

  //for(uint k = 0; k < inputs.size(); k++){
  //  std::cout << inputs.at(k) << std::endl;
  //}

  //################################################################################
  // check for benchmarks
  //################################################################################

  TiXmlElement* node_benchmark = FindFirstSubNode(node_plannerinput, "benchmark");
  if(node_benchmark!=NULL){
    benchmark.isInitialized=true;
    TiXmlElement* node_max_planning_time = FindSubNode(node_benchmark, "maxplanningtime");
    TiXmlElement* node_memory_mb = FindSubNode(node_benchmark, "maxmemoryMB");
    TiXmlElement* node_runs = FindSubNode(node_benchmark, "runs");
    TiXmlElement* node_display_progress = FindSubNode(node_benchmark, "displayProgress");
    TiXmlElement* node_filename = FindSubNode(node_benchmark, "filename");

    if(!node_max_planning_time){
      std::cout << "Benchmark needs maxplanningtime variable" << std::endl;
      exit(0);
    }
    if(!node_memory_mb){
      std::cout << "Benchmark needs maxmemoryMB variable" << std::endl;
      exit(0);
    }
    if(!node_runs){
      std::cout << "Benchmark needs runs variable" << std::endl;
      exit(0);
    }

    GetStreamText(node_max_planning_time) >> benchmark.max_planning_time;
    GetStreamText(node_memory_mb) >> benchmark.maxmemoryMB;
    GetStreamText(node_runs) >> benchmark.Nruns;
    GetStreamText(node_display_progress) >> benchmark.displayProgress;
    GetStreamAttribute(node_filename,"name") >> benchmark.filename;
    GetStreamAttribute(node_benchmark,"name") >> benchmark.name;

    // std::cout << "BEnchmark:" << std::endl;
    // std::cout << benchmark.name << std::endl;
    // std::cout << benchmark.filename << std::endl;
    // std::cout << benchmark.Nruns << std::endl;
    // std::cout << benchmark.max_planning_time << std::endl;
    // std::cout << benchmark.maxmemoryMB << std::endl;
    // std::cout << benchmark.displayProgress << std::endl;
    // exit(0);

  }
  return true;
}

bool PlannerInput::GetConfig(const TiXmlElement* node, const char *name, Config &q){
  if(!node){
    q.resize(0);
    return false;
  }
  const char *na = node->Attribute(name);
  if(na){
    //safety check for config
    std::stringstream ss = stringstream(na);
    int n;
    ss >> n;
    double tt;

    int ictr = 0;
    while(!ss.fail()){
      ss>>tt;
      ictr++;
    }
    ictr--;
    if(n!=ictr){
      std::cout << node->Value() << " Config is not correctly formatted (elements " << ictr << " but expected " << n << ")" << std::endl;
      exit(0);
      q.resize(0);
      return false;
    }

    ss.clear();
    ss.seekg(0, std::ios::beg);
    //std::cout << ss.str() << std::endl;
    //std::cout << std::string(80, '-') << std::endl;
    ss >> q;
    return true;
  }else{
    q.resize(0);
    return false;
  }
}
//################################################################################
///@brief get fixed general settings 
//################################################################################

bool PlannerInput::Load(TiXmlElement *node)
{
  CheckNodeName(node, "plannerinput");
  TiXmlElement* node_plannerinput = node;

  if(!node_plannerinput){
    std::cout << "world xml file has no plannerinput" << std::endl;
    return false;
  }

  TiXmlElement* node_qinit = FindSubNode(node_plannerinput, "qinit");
  TiXmlElement* node_qgoal = FindSubNode(node_plannerinput, "qgoal");
  TiXmlElement* node_se3min = FindSubNode(node_plannerinput, "se3min");
  TiXmlElement* node_se3max = FindSubNode(node_plannerinput, "se3max");

  TiXmlElement* node_dqinit = FindSubNode(node_plannerinput, "dqinit");
  TiXmlElement* node_dqgoal = FindSubNode(node_plannerinput, "dqgoal");
  TiXmlElement* node_freeFloating = FindSubNode(node_plannerinput, "freeFloating");

  GetConfig(node_qinit , "config", q_init);
  GetConfig(node_qgoal , "config", q_goal);
  GetConfig(node_dqinit, "config", dq_init);
  GetConfig(node_dqgoal, "config", dq_goal);
  GetConfig(node_se3min, "config", se3min);
  GetConfig(node_se3max, "config", se3max);

  GetStreamText(node_freeFloating) >> freeFloating;

  TiXmlElement* node_robot = FindSubNode(node_plannerinput, "robot");
  if(node_robot){
    GetStreamText(node_robot) >> robot_idx;
  }else{
    robot_idx = 0;
  }

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
