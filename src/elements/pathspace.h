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

    virtual void Decompose();

    int GetNumberOfDecompositions() const;
    const std::vector<std::vector<Config> > GetDecompositionVantagePaths() const;
    const std::vector<Config> GetDecompositionVantagePath(uint i) const;
    const std::vector<Config> GetDecompositionVertices() const;

  protected:

    ob::PlannerDataPtr pd; //contains local copy of PD

    CSpaceOMPL *cspace; //contains only ptr, but should also contain deep copy

    std::vector< std::vector<Config> > vantage_paths;
};

