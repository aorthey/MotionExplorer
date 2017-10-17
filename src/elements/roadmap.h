#pragma once
#include "planner/cspace/cspace.h"
#include "gui/gui_state.h"
#include "algorithms/lemon_interface.h"
#include <ompl/base/PlannerData.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;

class Roadmap{

  public:

    Roadmap();

    void CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);
    void CreateFromPlannerDataOnlySufficient(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);
    void CreateFromPlannerDataOnlyNecessary(const ob::PlannerDataPtr pd, CSpaceOMPL *cspace_);

    virtual void DrawGL(GUIState&);

    std::vector<Config> GetVertices();
    std::vector<std::pair<Config,Config>> GetEdges();
    void SetVertices(const std::vector<Config>&);
    void SetEdges(const std::vector<std::pair<Config,Config>>&);

    GLDraw::GLColor cVertex, cEdge;

  private:

    CSpaceOMPL *cspace;

    std::vector<Config> V;
    std::vector<std::pair<Config,Config>> E;

    ob::PlannerDataPtr pds;
    LemonInterface* lemon;
};
