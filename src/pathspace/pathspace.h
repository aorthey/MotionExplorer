#pragma once

#include "planner/cspace.h"
#include "pathspace/pathspace_input.h"
#include "gui_state.h"
#include "elements/swept_volume.h"
#include "elements/roadmap.h"
#include <ompl/base/PlannerData.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

class PathSpace{
  public:

    PathSpace(RobotWorld *world_, PathSpaceInput* input_);

    std::vector<Config> GetShortestPath();
    std::vector<Config> GetVertices();
    std::vector<std::pair<Config,Config>> GetEdges();
    std::vector<std::vector<Config>> GetPaths();
    Roadmap GetRoadmap();

    void SetShortestPath(const std::vector<Config>&);
    void SetVertices(const std::vector<Config>&);
    void SetEdges(const std::vector<std::pair<Config,Config>>&);
    void SetPaths(const std::vector<std::vector<Config>>&);
    void SetRoadmap(const Roadmap&);
    //split the pathspace up into smaller pieces.
    //Note: this decomposition does not need to be a partition, but could also
    //be a covering.
    virtual std::vector<PathSpace*> Decompose() = 0;
    virtual void DrawGL(GUIState&) = 0;
    virtual bool isAtomic() const = 0;

    RobotWorld* GetWorldPtr();
    PathSpaceInput* GetPathSpaceInput();

    SweptVolume& GetSweptVolume(Robot *robot);

    friend std::ostream& operator<< (std::ostream& out, const PathSpace& space);
  protected:

    std::vector<Config> vantage_path;
    std::vector<Config> vertices;
    std::vector<std::pair<Config,Config>> edges;
    std::vector<std::vector<Config>> paths;

    RobotWorld *world;
    PathSpaceInput *input;
    SweptVolume *sv;

    Roadmap roadmap;
};

