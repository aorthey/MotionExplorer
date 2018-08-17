#pragma once
#include "file_io.h"
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct Layer{
  int level;
  int inner_index;
  int outer_index;
  Config q_init;
  Config q_goal;
  std::string type;
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

    Config se3min;
    Config se3max;

    uint robot_idx;
    int freeFloating;

    std::string name_algorithm;
    std::string name_sampler;

    double epsilon_goalregion;
    double max_planning_time;
    double timestep_min;
    double timestep_max;

    bool smoothPath{false};
    double pathSpeed{1};

    bool kinodynamic{false};
    Config uMin;
    Config uMax;

    //input for hierarchical planner methods
    std::vector<int> robot_idxs;
    std::vector<Layer> layers;

    bool Load(const char* file);
    bool Load(TiXmlElement *node);
    void SetDefault();
    const CSpaceInput& GetCSpaceInput();
    const StrategyInput& GetStrategyInput();

    friend std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) ;
  private:
    CSpaceInput* cin;
    StrategyInput* sin;
};

/// @brief multiple plannerinputs (to use several algorithms inside GUI)
struct PlannerMultiInput{
  std::vector<PlannerInput*> inputs;
  bool Load(const char* file);
  bool Load(TiXmlElement *node);
  std::vector<std::string> GetAlgorithms(bool kinodynamic);
};

