#include "planner.h"
class MotionPlannerKinodynamic: public MotionPlanner{
  public:
    using MotionPlanner::MotionPlanner;
    //MotionPlannerKinodynamic();

    bool solve(Config &p_init, Config &p_goal, double timelimit=100.0, bool shortcutting=true);
  //KinodynamicCSpaceSentinel cspace_kinodynamic;
    void CheckFeasibility(Robot *robot, SingleRobotCSpace &cspace, Config &q);
};

