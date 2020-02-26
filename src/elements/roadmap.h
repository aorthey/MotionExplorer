#pragma once
#include "algorithms/lemon_interface.h"
#include "planner/cspace/cspace.h"
#include "elements/path_pwl.h"
#include "gui/gui_state.h"

#include <ompl/base/PlannerData.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;

class Roadmap{

  public:
    Roadmap();
    Roadmap(const ob::PlannerDataPtr, CSpaceOMPL* cspace_);
    Roadmap(const ob::PlannerDataPtr, CSpaceOMPL* cspace_, CSpaceOMPL* quotient_space_);

    PathPiecewiseLinear* GetShortestPath();
    void SetShortestPathOMPL(ob::PathPtr&);

    virtual void DrawGL(GUIState&);

    bool Save(const char* fn);
    bool Save(TiXmlElement *node);

    GLDraw::GLColor cEdge{grey};
    GLDraw::GLColor cVertex{grey};
    GLDraw::GLColor cVertexStart{green};
    GLDraw::GLColor cVertexGoal{red};
    GLDraw::GLColor cPath{darkMagenta};

    GLDraw::GLColor cVertexComponentOut{gray}; //a vertex not in the same component as the designated start vertex
    GLDraw::GLColor cVertexComponentGoal{cyan}; //goal component vertex

    uint numEdges();
    uint numVertices();

  private:

    void DrawGLPlannerData(GUIState&);
    //void DrawGLShortestPath(GUIState&);
    void DrawGLRoadmapVertices(GUIState&, int ridx = -1);
    void DrawGLRoadmapEdges(GUIState&, int ridx = -1);

    ob::PlannerDataPtr pd{nullptr};
    CSpaceOMPL *cspace{nullptr};
    CSpaceOMPL *quotient_space{nullptr};
    PathPiecewiseLinear *path_ompl{nullptr};

    std::vector<Vector3> shortest_path;
    bool draw_planar{false};
};
typedef std::shared_ptr<Roadmap> RoadmapPtr;
