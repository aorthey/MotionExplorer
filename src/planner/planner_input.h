#pragma once
#include "loader.h"
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct Layer{
  int level;
  int inner_index;
  int outer_index;
  Config q_init;
  Config q_goal;
  bool isInnerOuter;
  std::string type;
};

struct Benchmark{
  bool isInitialized{false};
  std::string name;
  std::string filename;
  double max_planning_time;
  double maxmemoryMB;
  uint Nruns;
  bool displayProgress;
};

class PlannerInput{
  public:
    //general input for any planner method (fixed)

    Config q_init;
    Config q_goal;
    Config dq_init;
    Config dq_goal;

    Config qMin;
    Config qMax;

    Config se3min;
    Config se3max;

    uint robot_idx;

    int isSE2;

    int freeFloating;

    std::string name_algorithm;
    std::string name_sampler;

    //specific input for planner methods
    double epsilon_goalregion;
    double max_planning_time;
    double timestep_min;
    double timestep_max;

    //input for hierarchical planner methods
    std::vector<int> robot_idxs;
    std::vector<Layer> layers;

    bool load(const char* file);
    bool load(TiXmlElement *node);
    bool GetConfig(const TiXmlElement* node, const char *name, Config &q);

    friend std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) ;
};

/// @brief multiple plannerinputs (to use several algorithms inside GUI)
struct PlannerMultiInput{
  std::vector<PlannerInput*> inputs;
  bool load(const char* file);
  bool load(TiXmlElement *node);
  Benchmark benchmark;
};

