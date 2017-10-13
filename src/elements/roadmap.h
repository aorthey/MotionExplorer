#pragma once
#include "planner/cspace.h"
#include "gui_state.h"
#include <ompl/base/PlannerData.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;

class Roadmap{
  public:
    Roadmap();

    void CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);

    virtual void DrawGL(GUIState&);

  private:
    CSpaceOMPL *cspace;
    std::vector<Config> V;
    std::vector<bool> Vsufficient;
    std::vector<std::pair<Config,Config>> E;
    std::vector<bool> Esufficient;
};

