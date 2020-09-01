#pragma once
#include "file_io.h"
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include <KrisLibrary/math3d/Triangle3D.h>

struct Layer{
  int level;
  int robot_index;
  int outer_index;
  // double cspace_constant;
  double finite_horizon_relaxation{0.0};
  Config q_init;
  Config q_goal;
  Config dq_init;
  Config dq_goal;
  std::string type;
  bool isTimeDependent{false};

  std::string path_fname;

  //multiagent
  bool isMultiAgent{false};

  int maxRobots{0};
  std::vector<int> ids;
  std::vector<Config> q_inits;
  std::vector<Config> dq_inits;
  std::vector<Config> q_goals;
  std::vector<Config> dq_goals;
  std::vector<int> ptr_to_next_level_ids;
  std::vector<std::string> types;
  std::vector<int> freeFloating;
  std::vector<int> controllable;
};

struct ContactInformation
{
  std::string robot_name;
  std::string robot_link;
  int robot_link_idx{-1};
  std::string mode;

  std::string meshFrom;
  std::string meshTo;
  int meshFromIdx{-1};
  int meshToIdx{-1};
  int triFrom{-1};
  int triTo{-1};
};


struct AgentInformation{
  Config q_init;
  Config q_goal;
  Config dq_init;
  Config dq_goal;
  int id;
  Config qMin;
  Config qMax;
  Config dqMin;
  Config dqMax;
  Config uMin;
  Config uMax;
  std::vector<ContactInformation> contact_links;

  bool isTimeDependent{false};
  std::string timePathFile;
};

struct Stratification{
  std::vector<Layer> layers;
};

class CSpaceInput;
class StrategyInput;

class PlannerInput{
  public:
    //general input for any planner method (fixed)
    Config q_init;
    Config q_goal;
    Config dq_init;
    Config dq_goal;

    Config qMin;
    Config qMax;
    Config dqMin;
    Config dqMax;
    Config uMin;
    Config uMax;

    Config se3min;
    Config se3max;
    uint robot_idx;
    std::vector<ContactInformation> contact_links;

    //multiagents
    std::vector<AgentInformation> agent_information;

    //contact-planning
    int freeFloating;
    bool contactPlanner{false};
    bool threading{false};
    
    // access to surface triangles of objects/obstacles
    std::vector<Triangle3D> tris;

    std::string name_algorithm;
    std::string name_sampler;
    std::string environment_name;
    std::string name_loadPath;

    double epsilon_goalregion{0.0};
    double max_planning_time{0.0};
    double timestep_min{0.0};
    double timestep_max{0.0};

    bool smoothPath{false};
    double pathSpeed{1};
    double pathWidth{1};
    double pathBorderWidth{0.01};

    bool kinodynamic{false};
    bool multiAgent{false};

    std::vector<Stratification> stratifications;

    bool Load(const char* file);
    bool Load(TiXmlElement *node, int hierarchy = 0);
    void SetDefault();
    void ExtractHierarchy(TiXmlElement *node, int hierarchy_index);
    void ExtractMultiHierarchy(TiXmlElement *node, int hierarchy_index);
    const CSpaceInput& GetCSpaceInput(int robot_idx = -1);
    const StrategyInput& GetStrategyInput();

    friend std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) ;
    const AgentInformation& GetAgentAtID(int id);
    void AddConfigToConfig(Config &q, const Config &qadd);
    void AddConfigToConfig(Config &q, const Config &qadd, int Nclip);
    bool ExistsAgentAtID(int id);

  private:

    CSpaceInput* cin;
    StrategyInput* sin;
};

/// @brief multiple plannerinputs (to use several algorithms inside GUI)
struct PlannerMultiInput{
  std::vector<PlannerInput*> inputs;

  bool Load(const char* file);
  bool Load(TiXmlElement *node);
  std::vector<std::string> GetAlgorithms(TiXmlElement *node, bool kinodynamic);
  std::vector<std::string> GetAlgorithmsDefault(bool kinodynamic);
  std::vector<std::string> GetAlgorithmsCustom(TiXmlElement *node, bool kinodynamic);
};

