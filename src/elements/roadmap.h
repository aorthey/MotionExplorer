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
    void DrawSingleLevelGL(GUIState &, uint);

    void SetVertices(const std::vector<Config>&);
    void SetEdges(const std::vector<std::pair<Config,Config>>&);
    void SetShortestPath(const std::vector<Config>&);

    std::vector<Config> GetVertices();
    std::vector<std::pair<Config,Config>> GetEdges();
    std::vector<Config> GetShortestPath();
    ob::PlannerDataPtr GetPlannerDataPtr();
    CSpaceOMPL* GetCSpacePtr();

    GLDraw::GLColor cVertex, cEdge, cEdgeRemoved;

    int numEdges();
    int numVertices();
    void removeInfeasibleEdgeAlongShortestPath(uint index);

  private:
    std::vector<ob::PlannerDataVertex> shortest_path_vertex;

    std::vector<Config> shortest_path;
    std::vector<ob::PlannerData::Graph::Vertex> shortest_path_idxs;

    std::vector<Config> VertexPathToConfigPath( const std::vector<ob::PlannerData::Graph::Vertex> &path);

    CSpaceOMPL *cspace;

    std::vector<Config> V;
    std::vector<std::pair<Config,Config>> E;
    std::vector<std::pair<Config,Config>> E_removed;

    uint Nvertices;
    uint Nedges;

    ob::PlannerDataPtr pds;
    LemonInterface* lemon;

    std::vector<double> distance_vertex_environment;
};
typedef std::shared_ptr<Roadmap> RoadmapPtr;
