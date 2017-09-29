#pragma once
#include "loader.h"
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct Layer{
  int level;
  int inner_index;
  int outer_index;
  bool isInnerOuter;
};
struct PlannerInputAlgorithm{
  double epsilon_goalregion;
  double max_planning_time;
  double timestep_min;
  double timestep_max;
  std::string name;
};

class PlannerInput{
  public:
    //general input for any planner method (fixed)
    bool exists;

    Config q_init;
    Config q_goal;
    Config dq_init;
    Config dq_goal;

    Config qMin;
    Config qMax;

    uint robot_idx;

    int freeFloating;

    Config se3min;
    Config se3max;

    //std::string name_robot;
    std::string name_algorithm;

    //specific input for planner methods
    double epsilon_goalregion;
    double max_planning_time;
    double timestep_min, timestep_max;
    //std::vector<std::string> algorithms;

    std::vector<PlannerInputAlgorithm> algorithms;

    //input for hierarchical planner methods
    std::vector<int> robot_idxs;
    std::vector<Layer> layers;

    //to <gui>
    int drawShortestPath;
    int drawSweptVolume;
    int drawMilestones;
    int drawStartGoal;
    int drawTree;
    int drawSimplicialComplex;

    bool load(const char* file);
    bool load(TiXmlElement *node);

    friend std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) ;
};

