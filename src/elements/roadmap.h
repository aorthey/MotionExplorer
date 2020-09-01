#pragma once
#include "planner/cspace/cspace.h"
#include "elements/path_pwl.h"
#include "gui/gui_state.h"

#include <ompl/base/PlannerData.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;

class Roadmap{

  public:
    Roadmap() = delete;
    Roadmap(const ob::PlannerDataPtr, std::vector<CSpaceOMPL*> cspace_levels);

    virtual void DrawGL(GUIState&, int level=0);

    bool Save(const char* fn);
    bool Save(TiXmlElement *node);

    GLDraw::GLColor cEdge{green};
    GLDraw::GLColor cVertex{green};
    GLDraw::GLColor cVertexStart{green};
    GLDraw::GLColor cVertexGoal{yellow};
    GLDraw::GLColor cPath{darkMagenta};

    GLDraw::GLColor cVertexComponentOut{gray}; //a vertex not in the same component as the designated start vertex
    GLDraw::GLColor cVertexComponentGoal{cyan}; //goal component vertex

    uint numEdges();
    uint numVertices();

  private:

    void DrawGLPlannerData(GUIState&);
    void DrawGLRoadmapVertices(GUIState&, int ridx = -1);
    void DrawGLRoadmapEdges(GUIState&, int ridx = -1);
    void DrawGLEdge(const ob::PlannerDataVertex *v, const ob::PlannerDataVertex *w, int ridx);
    void DrawGLEdge(CSpaceOMPL *space, const ob::State *s, const ob::State *t, int ridx);
    void DrawGLEdgeStateToState(CSpaceOMPL *space, const ob::State *s, const ob::State *t, int ridx);
    Vector3 VectorFromVertex(const ob::PlannerDataVertex *v, int ridx);

    ob::PlannerDataPtr pd_{nullptr};

    std::vector<CSpaceOMPL*> cspace_levels_;
    bool draw_planar{false};
    int current_level_{0};

    ob::State *stateTmpCur{nullptr};
    ob::State *stateTmpOld{nullptr};
};
typedef std::shared_ptr<Roadmap> RoadmapPtr;
