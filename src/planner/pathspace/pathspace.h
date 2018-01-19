#pragma once

#include "elements/roadmap.h"
#include "elements/path_pwl.h"
#include "gui/gui_state.h"
#include "planner/pathspace/pathspace_input.h"
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

class RobotWorld;
class SweptVolume;

using namespace GLDraw;

class PathSpace{
  public:

    PathSpace(RobotWorld *world_, PathSpaceInput* input_);

    const std::vector<Config>& GetVertices();
    const std::vector<std::pair<Config,Config>>& GetEdges();
    const RoadmapPtr GetRoadmap();
    PathPiecewiseLinear* getShortestPathOMPL();

    void SetVertices(const std::vector<Config>&);
    void SetEdges(const std::vector<std::pair<Config,Config>>&);
    //void SetPaths(const std::vector<std::vector<Config>>&);
    void SetRoadmap(const RoadmapPtr);
    void SetShortestPath(const ob::PathPtr p, CSpaceOMPL *cspace);

    virtual std::vector<PathSpace*> Decompose() = 0;
    virtual void DrawGL(GUIState&) = 0;
    virtual bool isAtomic() const = 0;

    RobotWorld* GetWorldPtr();
    PathSpaceInput* GetPathSpaceInput();

    SweptVolume& GetSweptVolume(Robot *robot);

    friend std::ostream& operator<< (std::ostream& out, const PathSpace& space);
  protected:

    std::vector<Config> vertices;
    std::vector<std::pair<Config,Config>> edges;
    //std::vector<std::vector<Config>> paths;

    PathPiecewiseLinear *path_ompl{nullptr};

    RobotWorld *world;
    PathSpaceInput *input;
    SweptVolume *sv;
    RoadmapPtr roadmap;
    std::vector<CSpaceOMPL*> cspace_levels;
};

