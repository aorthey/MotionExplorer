#pragma once
#include "algorithms/lemon_interface.h"
#include "planner/cspace/cspace.h"
#include "elements/swath_volume.h"
#include "gui/gui_state.h"

#include <ompl/base/PlannerData.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;

class Roadmap{

  public:
    Roadmap();

    void CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);

    virtual void DrawGL(GUIState&);

    GLDraw::GLColor cVertex{green};
    GLDraw::GLColor cEdge{magenta};
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

    void DrawSingleLevelGL(GUIState &, ob::PlannerDataPtr);
    void DrawPathGL(GUIState &state, std::vector<Vector3> &q);

    CSpaceOMPL *cspace;
    SwathVolume *swv{nullptr};

    std::vector<ob::PlannerDataPtr> roadmaps_level;
    std::vector<std::vector<Vector3>> shortest_path_level;
};
typedef std::shared_ptr<Roadmap> RoadmapPtr;
