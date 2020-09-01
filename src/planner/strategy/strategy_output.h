#pragma once
#include "elements/path_pwl.h"
#include "planner/cspace/cspace.h"
#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;
namespace og = ompl::geometric;

OMPL_CLASS_FORWARD(Roadmap);

struct StrategyOutput
{

  public:

    StrategyOutput() = delete;
    StrategyOutput(std::vector<CSpaceOMPL*>);

    void SetShortestPath( std::vector<Config> );

    void SetPlannerData( ob::PlannerDataPtr pd_ );
    void SetProblemDefinition( ob::ProblemDefinitionPtr pdef_ );
    void SetProblemDefinition( std::vector<ob::ProblemDefinitionPtr> pdefVec );
    ob::PlannerDataPtr GetPlannerDataPtr();
    PathPiecewiseLinear* getSolutionPath(int level = 0);

    bool hasSolution(int level = 0);

    void Clear();

    bool hasExactSolution();
    bool hasApproximateSolution();

    friend std::ostream& operator<< (std::ostream&, const StrategyOutput&);

    double planner_time{-1};
    double max_planner_time{-1};

    void DrawGL(GUIState&, int);

    void DrawGLPath(GUIState&, int);

    RoadmapPtr getRoadmap();

  private:

    std::vector<Config> PathGeometricToConfigPath(og::PathGeometric &path);

    std::vector<Config> shortest_path;

    ob::PlannerDataPtr plannerData_;

    std::vector<PathPiecewiseLinear*> pathVec_;

    std::vector<ob::ProblemDefinitionPtr> pdefVec_;

    std::vector<CSpaceOMPL*> cspace_levels_;

    RoadmapPtr roadmap_{nullptr};

};
