#pragma once
#include "algorithms/lemon_interface.h"
#include "planner/cspace/cspace.h"
#include "elements/swath_volume.h"
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

    //void CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);
    PathPiecewiseLinear* GetShortestPath();

    virtual void DrawGL(GUIState&);

    GLDraw::GLColor cVertex{green};
    GLDraw::GLColor cEdge{green};
    GLDraw::GLColor cPath{magenta};
    GLDraw::GLColor cVertexOut{gray}; //a vertex not in the same component as the designated start vertex
    GLDraw::GLColor cVertexGoal{cyan}; //goal component vertex
    GLDraw::GLColor cNeighborhoodVolume{GLColor(0.1,0.8,0.1,0.2)}; //goal component vertex
    double sizeVertex{8};
    double widthEdge{5};
    double widthPath{15};

    uint numEdges();
    uint numVertices();

  private:

    //void DrawSingleLevelGL(GUIState &, ob::PlannerDataPtr);
    void DrawPlannerData(GUIState&);
    void DrawShortestPath(GUIState&);

    ob::PlannerDataPtr pd{nullptr};
    CSpaceOMPL *cspace{nullptr};
    CSpaceOMPL *quotient_space{nullptr};
    SwathVolume *swv{nullptr};
    PathPiecewiseLinear *path_ompl{nullptr};

    std::vector<Vector3> shortest_path;
};
typedef std::shared_ptr<Roadmap> RoadmapPtr;
