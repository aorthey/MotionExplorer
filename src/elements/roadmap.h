#pragma once
#include "planner/cspace.h"
#include <ompl/base/PlannerData.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;

class Roadmap{
  public:
    Roadmap();

    void CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);

    void GLDraw();

  private:
    CSpaceOMPL *cspace;
    std::vector<Config> V;
    std::vector<std::pair<Config,Config>> E;
};

