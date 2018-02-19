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

  friend class RoadmapDecoratorSE2;
  public:
    Roadmap();

    void CreateFromPlannerData(const ob::PlannerDataPtr pd, CSpaceOMPL* cspace_);

    virtual void DrawGL(GUIState&);

    GLDraw::GLColor cVertex, cEdge, cPath;
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
