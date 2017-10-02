#pragma once

#include "planner/cspace.h"
#include <ompl/base/PlannerData.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

//this should shadow all those onetopic covers etcetera. I.e. the best would be
//to just have one abstract pathspace, and then build a onetopicPathspace
//etcetera.
//
// pathspaces should differ in the way they are decomposed (i.e. which criteria
// will we use for a decomposition?). 

class PathSpace{
  public:

    PathSpace();
    PathSpace( ob::PlannerDataPtr pd_, CSpaceOMPL *cspace_);

    std::vector<Config> GetShortestPath();
    std::vector<Config> GetVertices();

    void SetShortestPath(std::vector<Config>);
    void SetVertices(std::vector<Config>);

    //split the pathspace up into smaller pieces.
    //Note: this decomposition does not need to be a partition, but could also
    //be a covering.
    virtual std::vector<PathSpace> Decompose();

  protected:

    ob::PlannerDataPtr pd; //contains local copy of PD

    CSpaceOMPL *cspace; //contains only ptr, but should also contain deep copy

    std::vector<Config> vantage_path;

    std::vector<Config> vertices;
};

