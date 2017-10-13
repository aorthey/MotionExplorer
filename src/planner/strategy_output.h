#pragma once
#include "elements/roadmap.h"
#include <omplapp/config.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct StrategyOutput{

  public:
    StrategyOutput(CSpaceOMPL*);

    std::vector<Config> GetShortestPath();
    void SetShortestPath( std::vector<Config> );

    Roadmap GetRoadmap();
    void SetRoadmap(Roadmap);

    void SetPlannerData( ob::PlannerDataPtr pd_ );

    bool success;

    ob::PlannerDataPtr GetPlannerDataPtr(){
      return pd;
    }

  private:

    std::vector<Config> shortest_path;

    ob::PlannerDataPtr pd;

    Roadmap roadmap;

    CSpaceOMPL *cspace;

};
