#pragma once
#include "algorithms/lemon_interface.h"
#include "planner/cspace/cspace.h"
#include "gui/gui_state.h"

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

    void SetVertices(const std::vector<Config>&);
    void SetEdges(const std::vector<std::pair<Config,Config>>&);

    std::vector<Config> GetVertices();
    std::vector<std::pair<Config,Config>> GetEdges();
    std::vector<Config> GetShortestPath();

    GLDraw::GLColor cVertex, cEdge;

  private:
    std::vector<Config> VertexPathToConfigPath( const std::vector<ob::PlannerData::Graph::Vertex> &path);

    CSpaceOMPL *cspace;

    std::vector<Config> V;
    std::vector<std::pair<Config,Config>> E;

    ob::PlannerDataPtr pds;
    LemonInterface* lemon;
};
