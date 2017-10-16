#pragma once
#include "planner/cspace.h"
#include "gui/gui_state.h"
#include "algorithms/lemon_interface.h"
#include <ompl/base/PlannerData.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;

class Roadmap{

  public:

    Roadmap();

    void CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);
    void CreateFromPlannerDataOnlySufficient(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);

    virtual void DrawGL(GUIState&);

    std::vector<Config> GetSufficientVertices();
    std::vector<std::pair<Config,Config>> GetSufficientEdges();
    void SetVertices(std::vector<Config>);
    void SetEdges(std::vector<std::pair<Config,Config>>);

  private:

    CSpaceOMPL *cspace;

    std::vector<Config> V;
    std::vector<std::pair<Config,Config>> E;
    int startVertex;
    int goalVertex;

    std::vector<bool> Vsufficient;
    std::vector<bool> Esufficient;

    ob::PlannerDataPtr pds;
    LemonInterface* lemon;
};
